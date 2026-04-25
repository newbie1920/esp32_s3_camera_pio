/**
 * ================================================================
 *   HỆ THỐNG QUẢN LÝ SÂN BÓNG RỔ - ESP32-S3 N16R8
 * ================================================================
 *  Board: ESP32-S3-WROOM-1-N16R8 (16MB Flash, 8MB OPI PSRAM)
 *  Camera tích hợp sẵn trên board (OV3660)
 *
 *  CHỨC NĂNG:
 *    1. Camera stream (MJPEG) qua HTTP tại /stream
 *    2. WiFiManager - tự tạo AP "SanBongRo_Setup" khi chưa có WiFi
 *    3. MQTT - lắng nghe lệnh điều khiển sân từ backend
 *       Topics lắng nghe:
 *         court/1/open   -> "OPEN" / "CLOSE" (Servo cửa sân 1)
 *         court/2/open   -> "OPEN" / "CLOSE" (Servo cửa sân 2)
 *         court/1/light  -> "ON" / "OFF"      (Relay đèn)
 *         court/1/fan    -> "ON" / "OFF"      (Relay quạt)
 *         court/+/status -> "BOOKED" / "EMPTY" (Trạng thái sân)
 *       Topics publish:
 *         camera/stream/url  -> URL xem camera khi khởi động
 *
 *  SƠ ĐỒ CHÂN (ESP32-S3 N16R8 board camera tích hợp):
 *    Camera: (xem phần CAMERA_PIN bên dưới - Freenove S3 WROOM)
 *    *** GPIO 33-37: RESERVED cho OPI PSRAM - KHÔNG DÙNG! ***
 *    Relay đèn 1: GPIO 39    Relay đèn 2: GPIO 21
 *    Relay quạt 1: GPIO 40   Relay quạt 2: GPIO 41
 *    Servo sân 1: GPIO 42    Servo sân 2: GPIO 38
 *    PIR sân 1  : GPIO 1     PIR sân 2  : GPIO 2
 *    Buzzer sân 1: GPIO 6    Buzzer sân 2: GPIO 7
 *
 *  LƯU Ý: Các chân relay/servo/PIR bạn CÓ THỂ THAY ĐỔI tùy dây nối thực tế.
 *          Chỉ TRÁNH các chân camera đã được dùng (xem CAMERA_PIN).
 * ================================================================
 */

#include <Arduino.h>
// Wire.h khong dung (I2C scan da xoa)
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_task_wdt.h"
#include "esp_log.h"

// ================================================================
//   CAMERA PINOUT - Freenove / Generic ESP32-S3-WROOM CAM board
//   (Board màu đen, 2 cổng USB-C, có khe microSD ở mặt sau)
// ================================================================
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM   4
#define SIOC_GPIO_NUM   5

#define Y9_GPIO_NUM    16
#define Y8_GPIO_NUM    17
#define Y7_GPIO_NUM    18
#define Y6_GPIO_NUM    12
#define Y5_GPIO_NUM    10
#define Y4_GPIO_NUM     8   // Freenove official (cu: 11)
#define Y3_GPIO_NUM     9   // Freenove official (cu: 14)
#define Y2_GPIO_NUM    11   // Freenove official (cu: 47)
#define VSYNC_GPIO_NUM  6   // Freenove official (cu: 8)
#define HREF_GPIO_NUM   7   // Freenove official (cu: 9)
#define PCLK_GPIO_NUM  13

// ================================================================
//   CẤU HÌNH CHÂN NGOẠI VI
//   (Các chân này nối với module mở rộng bên ngoài)
//
//   *** CHÚ Ý: ESP32-S3 N16R8 dùng OPI PSRAM ***
//   GPIO 33-37 bị PSRAM chiếm (SPIIO4-7 + SPIDQS)
//   KHÔNG ĐƯỢC dùng GPIO 33, 34, 35, 36, 37 cho ngoại vi!
// ================================================================
#define RELAY_LIGHT_PIN_1  39  // Relay điều khiển đèn sân 1 (cũ: 35 - bị PSRAM chiếm)
#define RELAY_FAN_PIN_1    40  // Relay điều khiển quạt sân 1 (cũ: 36 - bị PSRAM chiếm)
#define RELAY_LIGHT_PIN_2  21  // Relay điều khiển đèn sân 2
#define RELAY_FAN_PIN_2    41  // Relay điều khiển quạt sân 2
#define SERVO_PIN_1      42  // Servo mở cửa sân 1 (cũ: 37 - bị PSRAM chiếm)
#define SERVO_PIN_2      38  // Servo mở cửa sân 2
#define PIR_PIN_1         1  // Cảm biến chuyển động sân 1 (cũ: 33 - bị PSRAM chiếm)
#define PIR_PIN_2         2  // Cảm biến chuyển động sân 2 (cũ: 34 - bị PSRAM chiếm)
#define BUZZER_PIN_1     14  // Còi báo động sân 1 (cu: 6 - xung dot VSYNC)
#define BUZZER_PIN_2     47  // Còi báo động sân 2 (cu: 7 - xung dot HREF)

// ================================================================
//   MQTT
// ================================================================
const char* MQTT_SERVER = "broker.emqx.io";
const int   MQTT_PORT   = 1883;
// Topics
#define TOPIC_COURT1_OPEN   "court/1/open"
#define TOPIC_COURT2_OPEN   "court/2/open"
#define TOPIC_COURT1_LIGHT  "court/1/light"
#define TOPIC_COURT1_FAN    "court/1/fan"
#define TOPIC_COURT2_LIGHT  "court/2/light"
#define TOPIC_COURT2_FAN    "court/2/fan"
#define TOPIC_COURT_STATUS  "court/+/status"
#define TOPIC_CAM_URL       "camera/stream/url"

// ================================================================
//   TOÀN CỤC
// ================================================================
WiFiClient   espClient;
PubSubClient mqtt(espClient);

bool isCourt1Booked = false;
bool isCourt2Booked = false;
bool lightOn        = false;
bool fanOn          = false;

// ================================================================
//   SERVO (không dùng thư viện, dùng PWM thô để tránh xung đột
//          với camera LEDC channel)
// ================================================================
#define SERVO1_LEDC_CH  2
#define SERVO2_LEDC_CH  3
#define SERVO_FREQ      50    // 50Hz
#define SERVO_RES       14    // ESP32-S3 chỉ hỗ trợ tối đa 14-bit

// Tính duty cycle từ góc độ (0-180 deg)
// Pulse: 1ms (0°) → 2ms (180°) trên chu kỳ 20ms
uint32_t angleToDuty(int angle) {
  float minDuty = (1.0f / 20.0f) * 16383;   // 1ms / 20ms
  float maxDuty = (2.0f / 20.0f) * 16383;   // 2ms / 20ms
  return (uint32_t)(minDuty + (maxDuty - minDuty) * angle / 180.0f);
}

void servoSetup() {
  ledcSetup(SERVO1_LEDC_CH, SERVO_FREQ, SERVO_RES);
  ledcSetup(SERVO2_LEDC_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN_1, SERVO1_LEDC_CH);
  ledcAttachPin(SERVO_PIN_2, SERVO2_LEDC_CH);
  // Khóa cửa ban đầu (0 độ)
  ledcWrite(SERVO1_LEDC_CH, angleToDuty(0));
  ledcWrite(SERVO2_LEDC_CH, angleToDuty(0));
}

void servoWrite(int channel, int angle) {
  ledcWrite(channel, angleToDuty(angle));
}

// ================================================================
//   CAMERA - KHỞI TẠO
// ================================================================

bool initCamera() {
  Serial.println("[CAM] Khoi tao camera...");
  Serial.printf("[CAM] XCLK=%d SDA=%d SCL=%d\n", XCLK_GPIO_NUM, SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Serial.flush();

  delay(500);

  if (!psramFound()) {
    Serial.println("[CAM] FATAL: PSRAM khong co! ESP32-S3 GDMA can PSRAM cho camera buffer.");
    return false;
  }

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  // === ESP32-S3 + OV3660: Buffer PHAI nam trong PSRAM ===
  // GDMA tren ESP32-S3 khong hoat dong dung voi DRAM buffer
  config.xclk_freq_hz = 22000000;            // 22MHz - Muc can bang giua toc do (FPS) va chong nhieu (soc)
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_HVGA;      // HVGA (480x320)
  config.jpeg_quality = 13;                  // 14 = Nen anh manh hon 1 chut de truyen WiFi le hon (giam lag)
  config.fb_count     = 2;                   // 2 buffer - double buffering cho stream muot
  config.fb_location  = CAMERA_FB_IN_PSRAM;  // BAT BUOC tren ESP32-S3!
  config.grab_mode    = CAMERA_GRAB_LATEST;  // LUON lay frame moi nhat -> giam do tre (lag)

  Serial.println("[CAM] Config: HVGA 480x320, 2 buf, PSRAM, 20MHz (OV3660)");
  Serial.println("[CAM] Goi esp_camera_init()...");
  Serial.flush();

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init FAILED: 0x%x\n", err);
    return false;
  }

  Serial.println("[CAM] esp_camera_init() THANH CONG!");

  sensor_t *s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("[CAM] Khong lay duoc sensor handle!");
    return false;
  }
  Serial.printf("[CAM] Sensor PID: 0x%04x\n", s->id.PID);

  // === SENSOR TUNING cho OV3660 - fix artifact + toi uu FPS ===
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);

  // OV3660 can set framesize SAU init (khac OV2640)
  // Dat lai framesize o day de dam bao sensor nhan dung
  s->set_framesize(s, FRAMESIZE_HVGA);  // 480x320 - max FPS

  s->set_brightness(s, 1);     // Tang sang nhe
  s->set_contrast(s, 1);       // Tang tuong phan nhe  
  s->set_saturation(s, 0);     // Mau sac binh thuong
  s->set_whitebal(s, 1);       // Auto white balance ON
  s->set_awb_gain(s, 1);       // AWB gain ON
  s->set_wb_mode(s, 0);        // Auto WB mode
  s->set_aec2(s, 1);           // Auto exposure (DSP) ON
  s->set_ae_level(s, 0);       // AE level binh thuong
  s->set_gain_ctrl(s, 1);      // Auto gain ON
  s->set_agc_gain(s, 0);       // AGC gain auto
  s->set_gainceiling(s, (gainceiling_t)6); // Max gain ceiling
  s->set_bpc(s, 1);            // Black pixel correction ON
  s->set_wpc(s, 1);            // White pixel correction ON
  s->set_raw_gma(s, 1);        // Gamma correction ON
  s->set_lenc(s, 1);           // Lens correction ON
  s->set_dcw(s, 1);            // Downsize EN - giup scale nhanh hon
  Serial.println("[CAM] Sensor tuning applied (OV3660)");

  // Cho camera on dinh
  Serial.println("[CAM] Cho camera on dinh 1s...");
  delay(1000);

  // === WARM-UP: Bo 5 frame dau tien (thuong bi loi/artifact) ===
  Serial.println("[CAM] Flush 5 warm-up frames...");
  for (int i = 0; i < 5; i++) {
    camera_fb_t *warmup = esp_camera_fb_get();
    if (warmup) {
      esp_camera_fb_return(warmup);
      Serial.printf("[CAM] Warmup frame %d flushed (%u bytes)\n", i + 1, (unsigned)warmup->len);
    } else {
      Serial.printf("[CAM] Warmup frame %d: NULL\n", i + 1);
    }
    delay(100);
  }

  // Test lay frame that su
  Serial.println("[CAM] Thu lay frame...");
  camera_fb_t *fb = NULL;
  for (int i = 0; i < 10; i++) {
    fb = esp_camera_fb_get();
    if (fb) {
      Serial.printf("[CAM] Frame OK! %u bytes, %dx%d, fmt=%d\n",
        (unsigned)fb->len, fb->width, fb->height, fb->format);
      esp_camera_fb_return(fb);
      Serial.println("[CAM] Camera san sang!");
      return true;
    }
    Serial.printf("[CAM] fb_get lan %d: NULL\n", i + 1);
    delay(1000);
  }

  // Khong lay duoc frame nhung van giu camera init
  Serial.println("[CAM] CANH BAO: fb_get fail 10 lan!");
  Serial.println("[CAM] Camera van init - co the do XCLK can nhieu.");
  return true;  // Van return true de he thong chay tiep
}

// ================================================================
//   CAMERA - HTTP STREAM SERVER (MJPEG) - TOI UU TOC DO
// ================================================================
#define STREAM_BOUNDARY "mjpeg-boundary-esp32s3"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY;
static const char* STREAM_PART = "\r\n--" STREAM_BOUNDARY "\r\n"
                                  "Content-Type: image/jpeg\r\n"
                                  "Content-Length: %u\r\n\r\n";

httpd_handle_t camera_httpd  = NULL;  // Port 80: snapshot, status
httpd_handle_t stream_httpd2 = NULL;  // Port 81: stream rieng (CPU core 1)

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb   = NULL;
  esp_err_t    res  = ESP_OK;
  char         part[128];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");
  // Tat cache de trinh duyet hien thi frame moi nhat
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");

  Serial.println("[STREAM] Client ket noi stream!");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[STREAM] fb_get NULL!");
      res = ESP_FAIL;
      break;
    }

    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)fb->len);

    res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) break;

    // Yield nhe de he thong khong bi treo
    taskYIELD();
  }

  Serial.println("[STREAM] Client ngat ket noi.");
  return res;
}

// Endpoint /snapshot: chup 1 anh tinh JPEG (co retry)
static esp_err_t snapshot_handler(httpd_req_t *req) {
  Serial.println("[HTTP] Snapshot requested...");
  camera_fb_t *fb = NULL;

  // Thu toi da 3 lan
  for (int i = 0; i < 3; i++) {
    fb = esp_camera_fb_get();
    if (fb) break;
    Serial.printf("[HTTP] fb_get fail, retry %d/3\n", i + 1);
    delay(100);
  }

  if (!fb) {
    Serial.println("[HTTP] Snapshot FAILED - fb NULL sau 3 lan!");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  Serial.printf("[HTTP] Snapshot OK: %u bytes, %dx%d\n",
    (unsigned)fb->len, fb->width, fb->height);

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return ESP_OK;
}

// Endpoint /status: kiem tra trang thai camera
static esp_err_t status_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  char buf[256];
  if (fb) {
    snprintf(buf, sizeof(buf),
      "Camera: OK\nFrame: %ux%u\nSize: %u bytes\nFormat: %d\nPSRAM: %s",
      fb->width, fb->height, (unsigned)fb->len, fb->format,
      psramFound() ? "Yes" : "No");
    esp_camera_fb_return(fb);
  } else {
    snprintf(buf, sizeof(buf),
      "Camera: FAILED (fb_get returned NULL)\nPSRAM: %s\nFree heap: %u",
      psramFound() ? "Yes" : "No", (unsigned)esp_get_free_heap_size());
  }
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_send(req, buf, strlen(buf));
  return ESP_OK;
}

void startCameraServer() {
  // === SERVER 1: Port 80 - Snapshot + Status (nhẹ) ===
  httpd_config_t cfg1 = HTTPD_DEFAULT_CONFIG();
  cfg1.server_port    = 80;
  cfg1.stack_size     = 8192;

  if (httpd_start(&camera_httpd, &cfg1) == ESP_OK) {
    httpd_uri_t snap_uri = { "/snapshot", HTTP_GET, snapshot_handler, NULL };
    httpd_uri_t stat_uri = { "/status",   HTTP_GET, status_handler,   NULL };
    httpd_register_uri_handler(camera_httpd, &snap_uri);
    httpd_register_uri_handler(camera_httpd, &stat_uri);
  }

  // === SERVER 2: Port 81 - Stream MJPEG (nang, chay CPU core 1) ===
  httpd_config_t cfg2 = HTTPD_DEFAULT_CONFIG();
  cfg2.server_port    = 81;
  cfg2.stack_size     = 16384;   // Stack lon cho xu ly frame
  cfg2.core_id        = 1;      // Chay tren CPU Core 1 (Core 0 cho WiFi/MQTT)
  cfg2.ctrl_port      = 32769;  // Control port khac port 80

  if (httpd_start(&stream_httpd2, &cfg2) == ESP_OK) {
    httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
    httpd_register_uri_handler(stream_httpd2, &stream_uri);
  }

  Serial.println("[HTTP] Stream: port 81 /stream | Snapshot: port 80 /snapshot");
}

// ================================================================
//   MQTT CALLBACK
// ================================================================
void openDoor(int courtId) {
  int ch = (courtId == 1) ? SERVO1_LEDC_CH : SERVO2_LEDC_CH;
  Serial.printf("[DOOR] Mở cửa sân %d\n", courtId);
  servoWrite(ch, 90);   // Mở
  delay(5000);
  Serial.printf("[DOOR] Đóng cửa sân %d\n", courtId);
  servoWrite(ch, 0);    // Đóng lại sau 5 giây
}

void closeDoor(int courtId) {
  int ch = (courtId == 1) ? SERVO1_LEDC_CH : SERVO2_LEDC_CH;
  Serial.printf("[DOOR] Đóng cửa sân %d\n", courtId);
  servoWrite(ch, 0);
}

void mqttCallback(char *topic, byte *payload, unsigned int len) {
  String msg = "";
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  String t = String(topic);
  Serial.printf("[MQTT] %s -> %s\n", topic, msg.c_str());

  // --- Cửa ---
  if (t == TOPIC_COURT1_OPEN) {
    if      (msg == "OPEN")  openDoor(1);
    else if (msg == "CLOSE") closeDoor(1);
  } else if (t == TOPIC_COURT2_OPEN) {
    if      (msg == "OPEN")  openDoor(2);
    else if (msg == "CLOSE") closeDoor(2);
  }

  // --- Đèn Sân 1 ---
  if (t == TOPIC_COURT1_LIGHT) {
    if (msg == "ON") {
      digitalWrite(RELAY_LIGHT_PIN_1, HIGH); lightOn = true;
      Serial.println("[RELAY] Den San 1 ON");
    } else if (msg == "OFF") {
      digitalWrite(RELAY_LIGHT_PIN_1, LOW); lightOn = false;
      Serial.println("[RELAY] Den San 1 OFF");
    }
  }

  // --- Quạt Sân 1 ---
  if (t == TOPIC_COURT1_FAN) {
    if (msg == "ON") {
      digitalWrite(RELAY_FAN_PIN_1, HIGH); fanOn = true;
      Serial.println("[RELAY] Quat San 1 ON");
    } else if (msg == "OFF") {
      digitalWrite(RELAY_FAN_PIN_1, LOW); fanOn = false;
      Serial.println("[RELAY] Quat San 1 OFF");
    }
  }

  // --- Đèn Sân 2 ---
  if (t == TOPIC_COURT2_LIGHT) {
    if (msg == "ON") {
      digitalWrite(RELAY_LIGHT_PIN_2, HIGH);
      Serial.println("[RELAY] Den San 2 ON");
    } else if (msg == "OFF") {
      digitalWrite(RELAY_LIGHT_PIN_2, LOW);
      Serial.println("[RELAY] Den San 2 OFF");
    }
  }

  // --- Quạt Sân 2 ---
  if (t == TOPIC_COURT2_FAN) {
    if (msg == "ON") {
      digitalWrite(RELAY_FAN_PIN_2, HIGH);
      Serial.println("[RELAY] Quat San 2 ON");
    } else if (msg == "OFF") {
      digitalWrite(RELAY_FAN_PIN_2, LOW);
      Serial.println("[RELAY] Quat San 2 OFF");
    }
  }

  // --- Trạng thái sân ---
  if (t == "court/1/status") {
    isCourt1Booked = (msg == "BOOKED");
    Serial.printf("[STATUS] San 1: %s\n", isCourt1Booked ? "BOOKED" : "EMPTY");
  } else if (t == "court/2/status") {
    isCourt2Booked = (msg == "BOOKED");
    Serial.printf("[STATUS] San 2: %s\n", isCourt2Booked ? "BOOKED" : "EMPTY");
  }
}

// ================================================================
//   MQTT KẾT NỐI LẠI
// ================================================================
void reconnectMQTT() {
  int attempts = 0;
  while (!mqtt.connected() && attempts < 3) { // Giảm xuống 3 lần để tránh WDT
    attempts++;
    String clientId = "ESP32S3_Court_" + String(random(0xFFFF), HEX);
    Serial.printf("[MQTT] Ket noi... (lan %d)\n", attempts);
    esp_task_wdt_reset(); // Reset WDT trước mỗi lần thử kết nối

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("[MQTT] OK!");
      mqtt.subscribe(TOPIC_COURT1_OPEN);
      mqtt.subscribe(TOPIC_COURT2_OPEN);
      mqtt.subscribe(TOPIC_COURT1_LIGHT);
      mqtt.subscribe(TOPIC_COURT1_FAN);
      mqtt.subscribe(TOPIC_COURT2_LIGHT);
      mqtt.subscribe(TOPIC_COURT2_FAN);
      mqtt.subscribe(TOPIC_COURT_STATUS);
      Serial.println("[MQTT] Da subscribe tat ca topics");

      // Publish URL camera lên MQTT để backend biết
      String url = "http://" + WiFi.localIP().toString() + "/stream";
      mqtt.publish(TOPIC_CAM_URL, url.c_str(), true); // retain = true
      Serial.printf("[MQTT] Camera URL: %s\n", url.c_str());

    } else {
      Serial.printf("[MQTT] That bai (rc=%d), thu lai sau 2s\n", mqtt.state());
      for (int i = 0; i < 4; i++) { // 4 x 500ms = 2s, mỗi 500ms reset WDT
        esp_task_wdt_reset();
        delay(500);
      }
    }
  }
  if (!mqtt.connected()) {
    Serial.println("[MQTT] Khong ket duoc MQTT - tiep tuc khong co MQTT");
  }
}

// ================================================================
//   SETUP
// ================================================================
void setup() {
  // ===== TAT BROWNOUT =====
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // ===== TAT TASK WATCHDOG (API chinh thuc) =====
  // Dung API esp_task_wdt thay vi ghi truc tiep vao register
  // (ghi register bi IDF ghi de lai ngay lap tuc tren ESP32-S3)
  esp_task_wdt_deinit();

  // Cho setup() co thoi gian chay - khong co WDT nao can tro

  Serial.begin(115200);

  // Tat spam "cam_hal: EV-VSYNC-OVF" - chi hien loi nghiem trong
  esp_log_level_set("cam_hal", ESP_LOG_ERROR);
  unsigned long serialStart = millis();
  while (!Serial && (millis() - serialStart) < 3000) {
    delay(10);
  }
  delay(500);
  Serial.println("\n=== HE THONG QUAN LY SAN BONG RO - ESP32-S3 N16R8 ===");
  Serial.printf("[SYS] PSRAM: %s | Flash: %dMB\n",
    psramFound() ? "OK" : "KHONG TIM THAY",
    spi_flash_get_chip_size() / (1024 * 1024));

  // ============================================================
  // BUOC 1: CAMERA INIT TRUOC - truoc khi bat ky peripheral nao
  //         de dam bao PSRAM chua bi can thiep
  // ============================================================
  if (!initCamera()) {
    Serial.println("[FATAL] Camera loi! Kiem tra chan hoac nguon.");
    // Khong restart - van cho chay cac chuc nang khac
  }

  // ============================================================
  // BUOC 2: PERIPHERAL INIT (sau camera)
  // LUU Y: GPIO 33-37 DA BI PSRAM CHIEM - KHONG DUOC DUNG!
  // ============================================================

  // --- Relay ---
  pinMode(RELAY_LIGHT_PIN_1, OUTPUT);  // GPIO 39 (an toan)
  pinMode(RELAY_FAN_PIN_1,   OUTPUT);  // GPIO 40 (an toan)
  pinMode(RELAY_LIGHT_PIN_2, OUTPUT);  // GPIO 21 (an toan)
  pinMode(RELAY_FAN_PIN_2,   OUTPUT);  // GPIO 41 (an toan)
  digitalWrite(RELAY_LIGHT_PIN_1, LOW);
  digitalWrite(RELAY_FAN_PIN_1,   LOW);
  digitalWrite(RELAY_LIGHT_PIN_2, LOW);
  digitalWrite(RELAY_FAN_PIN_2,   LOW);

  // --- PIR & Buzzer ---
  pinMode(PIR_PIN_1,    INPUT);   // GPIO 1 (an toan)
  pinMode(PIR_PIN_2,    INPUT);   // GPIO 2 (an toan)
  pinMode(BUZZER_PIN_1, OUTPUT);  // GPIO 6 (an toan)
  pinMode(BUZZER_PIN_2, OUTPUT);  // GPIO 7 (an toan)
  digitalWrite(BUZZER_PIN_1, LOW);
  digitalWrite(BUZZER_PIN_2, LOW);

  // --- Servo ---
  servoSetup();  // GPIO 42, 38 (an toan)

  // --- WiFiManager ---
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  Serial.println("[WIFI] Khởi động WiFiManager...");
  Serial.println("[WIFI] Nếu chưa có WiFi, kết nối AP 'SanBongRo_Setup' (pass: 12345678)");
  Serial.println("[WIFI] Rồi vào 192.168.4.1 để nhập thông tin WiFi");

  bool connected = wm.autoConnect("SanBongRo_Setup", "12345678");

  if (!connected) {
    Serial.println("[WIFI] Kết nối thất bại! Khởi động lại sau 5s...");
    delay(5000);
    ESP.restart();
  }

  // TAT CHE DO TIET KIEM PIN CUA WIFI - GIUP TANG TOC DO TRUYEN VIDEO (FPS)
  WiFi.setSleep(false);

  Serial.printf("[WIFI] Kết nối thành công! IP: %s\n",
    WiFi.localIP().toString().c_str());

  // --- HTTP Camera Server ---
  startCameraServer();
  Serial.printf("[HTTP] Stream  : http://%s:81/stream\n",
    WiFi.localIP().toString().c_str());
  Serial.printf("[HTTP] Snapshot: http://%s/snapshot\n",
    WiFi.localIP().toString().c_str());

  // --- MQTT ---
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  reconnectMQTT();

  Serial.println("\n=== HE THONG SAN SANG ===");

  // ===== BẬT LẠI WATCHDOG VỚI TIMEOUT DÀI CHO LOOP =====
  esp_task_wdt_init(60, true); // 60 giây timeout (MQTT reconnect co the lau)
  esp_task_wdt_add(NULL);      // Thêm task hiện tại vào WDT
  Serial.println("[WDT] Watchdog bat lai voi timeout 60s");
}

// ================================================================
//   LOOP
// ================================================================
void loop() {
  esp_task_wdt_reset(); // Reset WDT mỗi vòng loop

  // Giữ kết nối MQTT
  if (!mqtt.connected()) {
    reconnectMQTT();
  }
  mqtt.loop();

  // Báo động Sân 1 - Nếu phát hiện người khi sân chưa đặt
  if (digitalRead(PIR_PIN_1) == HIGH && !isCourt1Booked) {
    digitalWrite(BUZZER_PIN_1, HIGH);
  } else {
    digitalWrite(BUZZER_PIN_1, LOW);
  }

  // Báo động Sân 2
  if (digitalRead(PIR_PIN_2) == HIGH && !isCourt2Booked) {
    digitalWrite(BUZZER_PIN_2, HIGH);
  } else {
    digitalWrite(BUZZER_PIN_2, LOW);
  }

  // Delay ngắn để không chặn WiFi/MQTT stack
  delay(50);
}

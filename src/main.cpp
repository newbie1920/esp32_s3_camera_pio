/**
 * ================================================================
 *   HỆ THỐNG QUẢN LÝ SÂN BÓNG RỔ - ESP32-S3 N16R8
 * ================================================================
 *  Board: ESP32-S3-WROOM-1-N16R8 (16MB Flash, 8MB OPI PSRAM)
 *  Camera tích hợp sẵn trên board (OV2640)
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
 *    Relay đèn  : GPIO 35
 *    Relay quạt : GPIO 36
 *    Servo sân 1: GPIO 37
 *    Servo sân 2: GPIO 38  (nếu board cho phép dùng, kiểm tra trước)
 *    PIR sân 1  : GPIO 33
 *    PIR sân 2  : GPIO 34
 *    Buzzer sân 1: GPIO 6
 *    Buzzer sân 2: GPIO 7
 *
 *  LƯU Ý: Các chân relay/servo/PIR bạn CÓ THỂ THAY ĐỔI tùy dây nối thực tế.
 *          Chỉ TRÁNH các chân camera đã được dùng (xem CAMERA_PIN).
 * ================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

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
#define Y4_GPIO_NUM    11
#define Y3_GPIO_NUM    14
#define Y2_GPIO_NUM    47
#define VSYNC_GPIO_NUM  8
#define HREF_GPIO_NUM   9
#define PCLK_GPIO_NUM  13

// ================================================================
//   CẤU HÌNH CHÂN NGOẠI VI
//   (Các chân này nối với module mở rộng bên ngoài)
// ================================================================
#define RELAY_LIGHT_PIN_1  35  // Relay điều khiển đèn sân 1
#define RELAY_FAN_PIN_1    36  // Relay điều khiển quạt sân 1
#define RELAY_LIGHT_PIN_2  21  // Relay điều khiển đèn sân 2
#define RELAY_FAN_PIN_2    41  // Relay điều khiển quạt sân 2
#define SERVO_PIN_1      37  // Servo mở cửa sân 1
#define SERVO_PIN_2      38  // Servo mở cửa sân 2
#define PIR_PIN_1        33  // Cảm biến chuyển động sân 1
#define PIR_PIN_2        34  // Cảm biến chuyển động sân 2
#define BUZZER_PIN_1      6  // Còi báo động sân 1
#define BUZZER_PIN_2      7  // Còi báo động sân 2

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
httpd_handle_t stream_httpd = NULL;

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
// Quét I2C để tìm camera trước khi gọi esp_camera_init
// LƯU Ý: camera cần XCLK chạy trước mới phản hồi I2C
void scanI2CForCamera(int sda, int scl) {
  // Validate pin (GPIO 35-42 bị dùng cho OPI PSRAM, không dùng được)
  if (scl >= 35 && scl <= 42) {
    Serial.printf("[I2C] Scan SDA=%d SCL=%d: Bo qua - SCL nam trong vung PSRAM\n", sda, scl);
    return;
  }
  Wire.end();
  delay(10);
  if (!Wire.begin(sda, scl)) {
    Serial.printf("[I2C] Scan SDA=%d SCL=%d: Khong khoi duoc I2C\n", sda, scl);
    return;
  }
  Serial.printf("[I2C] Scanning SDA=%d SCL=%d...\n", sda, scl);
  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("[I2C] >>> THIET BI TIM THAY tai: 0x%02X", addr);
      if (addr == 0x30 || addr == 0x31 || addr == 0x21) Serial.print(" << OV2640/OV3660 !!!");
      if (addr == 0x3C) Serial.print(" << OV5640 !!!");
      Serial.println();
      found = true;
    }
  }
  Wire.end();
  if (!found) Serial.printf("[I2C] SDA=%d SCL=%d: Khong tim thay\n", sda, scl);
}

bool initCamera() {
  Serial.println("[CAM] Khoi tao camera...");
  Serial.printf("[CAM] XCLK=%d SDA=%d SCL=%d\n", XCLK_GPIO_NUM, SIOD_GPIO_NUM, SIOC_GPIO_NUM);

  // Bat XCLK truoc de camera co clock, moi phan hoi I2C
  Serial.printf("[CAM] Bat XCLK (GPIO%d @ 20MHz)...\n", XCLK_GPIO_NUM);
  ledcSetup(0, 20000000, 1);        // Channel 0, 20MHz, 1-bit
  ledcAttachPin(XCLK_GPIO_NUM, 0);  // GPIO 15
  ledcWrite(0, 1);                  // 50% duty cycle
  delay(200);                       // Cho camera khoi dong (OV2640 can ~100ms)

  // Quet I2C - gio camera da co clock nen se phan hoi
  Serial.println("[I2C] === Quet I2C (XCLK da bat) ===");
  scanI2CForCamera(4,  5);   // Freenove / ESP32-S3-EYE
  scanI2CForCamera(1,  2);   // Bien the
  scanI2CForCamera(41, 42);  // Bien the cao
  Serial.println("[I2C] === Ket thuc quet ===");

  // Tat LEDC thu cong (esp_camera_init se tu khoi dong lai)
  ledcDetachPin(XCLK_GPIO_NUM);
  delay(50);

  camera_config_t config;
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  // Bắt đầu bằng VGA + DRAM để test - đơn giản nhất, ít lỗi nhất
  config.frame_size   = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.fb_location  = CAMERA_FB_IN_DRAM;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init FAILED: 0x%x\n", err);
    if (err == 0x105) {
      Serial.println("[CAM] >> Loi 0x105: SAI CHAN! Kiem tra SIOD/SIOC/XCLK");
    } else if (err == 0x101) {
      Serial.println("[CAM] >> Loi 0x101: Camera khong phan hoi (nguon yeu?)");
    }
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("[CAM] OK! Sensor PID: 0x%04x\n", s->id.PID);

  // Nếu có PSRAM thì nâng lên UXGA
  if (psramFound()) {
    s->set_framesize(s, FRAMESIZE_UXGA);
    Serial.println("[CAM] Nang len UXGA (PSRAM)");
  }

  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  return true;
}

// ================================================================
//   CAMERA - HTTP STREAM SERVER (MJPEG)
// ================================================================
#define STREAM_BOUNDARY "mjpeg-boundary-esp32s3"

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb        = NULL;
  esp_err_t    res        = ESP_OK;
  uint8_t     *jpg_buf   = NULL;
  size_t       jpg_len   = 0;
  char         part[128];

  res = httpd_resp_set_type(req,
    "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    if (fb->format != PIXFORMAT_JPEG) {
      bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!ok) { res = ESP_FAIL; break; }
    } else {
      jpg_len = fb->len;
      jpg_buf = fb->buf;
    }

    size_t hlen = snprintf(part, sizeof(part),
      "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
      STREAM_BOUNDARY, (unsigned)jpg_len);

    res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);

    if (fb) {
      esp_camera_fb_return(fb); fb = NULL;
    } else if (jpg_buf) {
      free(jpg_buf); jpg_buf = NULL;
    }

    if (res != ESP_OK) break;
  }
  return res;
}

// Endpoint /snapshot: chụp 1 ảnh tĩnh JPEG
static esp_err_t snapshot_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return ESP_OK;
}

void startCameraServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port    = 80;
  cfg.stack_size     = 8192;

  if (httpd_start(&stream_httpd, &cfg) != ESP_OK) {
    Serial.println("[HTTP] Không thể khởi động HTTP server!");
    return;
  }

  httpd_uri_t stream_uri  = { "/stream",   HTTP_GET, stream_handler,   NULL };
  httpd_uri_t snap_uri    = { "/snapshot", HTTP_GET, snapshot_handler, NULL };
  httpd_register_uri_handler(stream_httpd, &stream_uri);
  httpd_register_uri_handler(stream_httpd, &snap_uri);

  Serial.println("[HTTP] Stream: /stream | Snapshot: /snapshot");
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
  while (!mqtt.connected() && attempts < 5) {
    attempts++;
    String clientId = "ESP32S3_Court_" + String(random(0xFFFF), HEX);
    Serial.printf("[MQTT] Kết nối... (lần %d)\n", attempts);

    if (mqtt.connect(clientId.c_str())) {
      Serial.println("[MQTT] OK!");
      mqtt.subscribe(TOPIC_COURT1_OPEN);
      mqtt.subscribe(TOPIC_COURT2_OPEN);
      mqtt.subscribe(TOPIC_COURT1_LIGHT);
      mqtt.subscribe(TOPIC_COURT1_FAN);
      mqtt.subscribe(TOPIC_COURT2_LIGHT);
      mqtt.subscribe(TOPIC_COURT2_FAN);
      mqtt.subscribe(TOPIC_COURT_STATUS);
      Serial.println("[MQTT] Đã subscribe tất cả topics");

      // Publish URL camera lên MQTT để backend biết
      String url = "http://" + WiFi.localIP().toString() + "/stream";
      mqtt.publish(TOPIC_CAM_URL, url.c_str(), true); // retain = true
      Serial.printf("[MQTT] Camera URL: %s\n", url.c_str());

    } else {
      Serial.printf("[MQTT] Thất bại (rc=%d), thử lại sau 3s\n", mqtt.state());
      delay(3000);
    }
  }
}

// ================================================================
//   SETUP
// ================================================================
void setup() {
  // Tắt brownout detector để tránh reset bất ngờ
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HE THONG QUAN LY SAN BONG RO - ESP32-S3 N16R8 ===");
  Serial.printf("[SYS] PSRAM: %s | Flash: %dMB\n",
    psramFound() ? "OK" : "KHONG TIM THAY",
    spi_flash_get_chip_size() / (1024 * 1024));

  // --- Relay ---
  pinMode(RELAY_LIGHT_PIN_1, OUTPUT);
  pinMode(RELAY_FAN_PIN_1,   OUTPUT);
  pinMode(RELAY_LIGHT_PIN_2, OUTPUT);
  pinMode(RELAY_FAN_PIN_2,   OUTPUT);
  digitalWrite(RELAY_LIGHT_PIN_1, LOW);
  digitalWrite(RELAY_FAN_PIN_1,   LOW);
  digitalWrite(RELAY_LIGHT_PIN_2, LOW);
  digitalWrite(RELAY_FAN_PIN_2,   LOW);

  // --- PIR & Buzzer ---
  pinMode(PIR_PIN_1,    INPUT);
  pinMode(PIR_PIN_2,    INPUT);
  pinMode(BUZZER_PIN_1, OUTPUT);
  pinMode(BUZZER_PIN_2, OUTPUT);
  digitalWrite(BUZZER_PIN_1, LOW);
  digitalWrite(BUZZER_PIN_2, LOW);

  // --- Servo ---
  servoSetup();

  // --- Khởi tạo Camera ---
  if (!initCamera()) {
    Serial.println("[FATAL] Camera lỗi! Kiểm tra chân hoặc nguồn.");
    // Không restart - vẫn cho chạy các chức năng khác
  }

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

  Serial.printf("[WIFI] Kết nối thành công! IP: %s\n",
    WiFi.localIP().toString().c_str());

  // --- HTTP Camera Server ---
  startCameraServer();
  Serial.printf("[HTTP] Stream  : http://%s/stream\n",
    WiFi.localIP().toString().c_str());
  Serial.printf("[HTTP] Snapshot: http://%s/snapshot\n",
    WiFi.localIP().toString().c_str());

  // --- MQTT ---
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  reconnectMQTT();

  Serial.println("\n=== HỆ THỐNG SẴN SÀNG ===");
}

// ================================================================
//   LOOP
// ================================================================
void loop() {
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

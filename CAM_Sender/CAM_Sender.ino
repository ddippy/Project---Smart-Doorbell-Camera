
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "esp_camera.h"
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

using namespace websockets;

const char* WIFI_SSID = "CAM_TFT_LINK";
const char* WIFI_PASS = "0641204734";

WebsocketsClient client;
volatile bool captureRequested = false;
volatile bool captureWithFlash = false;
bool cameraReady = false;
bool flashEnabled = false;
volatile bool captureInProgress = false;
volatile bool tftStreamPaused = false;
volatile bool mobileMaxFpsMode = false;
uint32_t lastTftStreamAt = 0;
constexpr uint32_t TFT_STREAM_INTERVAL_MS = 100;
volatile bool rebootRequested = false;
volatile bool sosEnabled = false;
bool sosWasEnabled = false;
uint8_t sosSegment = 0;
uint32_t sosSegmentUntil = 0;
SemaphoreHandle_t cameraFrameMutex = nullptr;
httpd_handle_t mobileStreamServer = nullptr;

// Repeating Morse SOS: three short, three long, three short, then a pause.
const uint16_t SOS_SEGMENT_MS[] = {
  200, 200, 200, 200, 200, 600,
  600, 200, 600, 200, 600, 600,
  200, 200, 200, 200, 200, 1500
};
const uint8_t SOS_SEGMENT_COUNT = sizeof(SOS_SEGMENT_MS) / sizeof(SOS_SEGMENT_MS[0]);

esp_err_t cameraWhoAmIHandler(httpd_req_t* request) {
  httpd_resp_set_type(request, "text/plain; charset=utf-8");
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  return httpd_resp_send(request, "HOME_CAM_CAMERA", HTTPD_RESP_USE_STRLEN);
}

esp_err_t mobilePageHandler(httpd_req_t* request) {
  static const char page[] =
      "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<meta name='theme-color' content='#111'><title>HOME CAM</title>"
      "<style>body{margin:0;background:#111;color:#eee;font:16px system-ui;text-align:center}"
      "header{height:42px;display:flex;align-items:center;justify-content:center;letter-spacing:.12em}"
      "img{display:block;width:100vw;height:calc(100dvh - 42px);object-fit:contain;background:#000}</style>"
      "</head><body><header>HOME CAM · LIVE</header><img src='/stream' alt='Live camera'></body></html>";
  httpd_resp_set_type(request, "text/html; charset=utf-8");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store");
  return httpd_resp_send(request, page, HTTPD_RESP_USE_STRLEN);
}

esp_err_t mobileStreamHandler(httpd_req_t* request) {
  httpd_resp_set_type(request, "multipart/x-mixed-replace;boundary=homecam-frame");
  httpd_resp_set_hdr(request, "Cache-Control", "no-store, no-cache, must-revalidate");
  httpd_resp_set_hdr(request, "Access-Control-Allow-Origin", "*");

  bool firstFrame = true;
  esp_err_t result = ESP_OK;
  while (true) {
    if (captureInProgress) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    if (!cameraFrameMutex ||
        xSemaphoreTake(cameraFrameMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      continue;
    }
    if (captureInProgress) {
      xSemaphoreGive(cameraFrameMutex);
      continue;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      xSemaphoreGive(cameraFrameMutex);
      result = ESP_FAIL;
      break;
    }

    char partHeader[80];
    const char* boundary = firstFrame ? "--homecam-frame\r\n"
                                      : "\r\n--homecam-frame\r\n";
    int headerLength = snprintf(partHeader, sizeof(partHeader),
                                "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                (unsigned int)fb->len);
    result = httpd_resp_send_chunk(request, boundary, strlen(boundary));
    if (result == ESP_OK)
      result = httpd_resp_send_chunk(request, partHeader, headerLength);
    if (result == ESP_OK && fb->format == PIXFORMAT_JPEG)
      result = httpd_resp_send_chunk(request, (const char*)fb->buf, fb->len);
    else if (fb->format != PIXFORMAT_JPEG)
      result = ESP_FAIL;

    esp_camera_fb_return(fb);
    xSemaphoreGive(cameraFrameMutex);
    if (result != ESP_OK) break;
    firstFrame = false;
    // Shared mode yields camera/Wi-Fi time to the TFT; max mode removes this cap.
    if (!mobileMaxFpsMode) vTaskDelay(pdMS_TO_TICKS(100));
  }
  httpd_resp_send_chunk(request, nullptr, 0);
  return result;
}

void startMobileStreamServer() {
  if (!cameraReady || !cameraFrameMutex) return;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32769;
  config.max_open_sockets = 2;
  config.stack_size = 6144;

  if (httpd_start(&mobileStreamServer, &config) != ESP_OK) {
    Serial.println("Mobile stream server failed to start");
    return;
  }

  httpd_uri_t page = {};
  page.uri = "/";
  page.method = HTTP_GET;
  page.handler = mobilePageHandler;
  httpd_register_uri_handler(mobileStreamServer, &page);

  httpd_uri_t identity = {};
  identity.uri = "/whoami"; identity.method = HTTP_GET; identity.handler = cameraWhoAmIHandler;
  httpd_register_uri_handler(mobileStreamServer, &identity);

  httpd_uri_t stream = {};
  stream.uri = "/stream";
  stream.method = HTTP_GET;
  stream.handler = mobileStreamHandler;
  httpd_register_uri_handler(mobileStreamServer, &stream);

  Serial.print("Mobile live view: http://");
  Serial.print(WiFi.localIP());
  Serial.println(":81/");
}

void setFlash(bool enabled) {
#ifdef LED_GPIO_NUM
  flashEnabled = enabled;
  digitalWrite(LED_GPIO_NUM, flashEnabled ? HIGH : LOW);
  Serial.printf("Flash %s\n", flashEnabled ? "ON" : "OFF");
#else
  (void)enabled;
  Serial.println("Flash LED pin is not defined for this camera model");
#endif
}

void serviceSosFlash() {
  if (!sosEnabled) {
    if (sosWasEnabled || flashEnabled) setFlash(false);
    sosWasEnabled = false;
    sosSegment = 0;
    return;
  }

  if (!sosWasEnabled) {
    sosWasEnabled = true;
    sosSegment = 0;
    sosSegmentUntil = millis();
  }
  if ((int32_t)(millis() - sosSegmentUntil) < 0) return;

  setFlash((sosSegment & 1U) == 0);
  sosSegmentUntil = millis() + SOS_SEGMENT_MS[sosSegment];
  sosSegment = (sosSegment + 1) % SOS_SEGMENT_COUNT;
}

void connect_receiver() {
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost; reconnecting...");
    WiFi.reconnect();
    delay(1000);
  }

  IPAddress receiverIP = WiFi.gatewayIP();
  while (!client.connect(receiverIP.toString(), 80, "/")) {
    Serial.printf("WebSocket connect to %s failed; retrying...\n",
                  receiverIP.toString().c_str());
    delay(1000);
  }
  captureRequested = false;
  client.onMessage([](WebsocketsMessage message) {
    if (message.isText()) {
      const String command = message.data();
      if (command == "CAPTURE") {
        captureWithFlash = false;
        captureRequested = true;
      }
      else if (command == "CAPTURE_FLASH") {
        captureWithFlash = true;
        captureRequested = true;
      }
      else if (command == "PAUSE_STREAM") tftStreamPaused = true;
      else if (command == "RESUME_STREAM") tftStreamPaused = mobileMaxFpsMode;
      else if (command == "MOBILE_MAX_ON") { mobileMaxFpsMode = true; tftStreamPaused = true; }
      else if (command == "MOBILE_SHARED") { mobileMaxFpsMode = false; tftStreamPaused = false; }
      else if (command == "FLASH_ON") setFlash(true);
      else if (command == "FLASH_OFF") setFlash(false);
      else if (command == "SOS_ON") {
        sosEnabled = true;
      }
      else if (command == "SOS_OFF") {
        sosEnabled = false;
        setFlash(false);
      }
      else if (command == "REBOOT") rebootRequested = true;
    }
  });
  Serial.println("WebSocket connected");
}

void init_camera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Use VGA and two JPEG buffers when PSRAM is present. Without PSRAM, QVGA
  // and one buffer are the stable high-frame-rate setting for this WROOM setup.
  config.jpeg_quality = 14; // Smaller JPEGs shorten Wi-Fi and Telegram upload time.
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA; // Fast TFT feed; mobile uses same camera feed.
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    Serial.println("PSRAM found; using VGA live stream");
  } else {
    config.frame_size = FRAMESIZE_QVGA; // 320x240 for reliable FPS without PSRAM.
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    Serial.println("PSRAM not found; using QVGA live stream for stable FPS");
  }

  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    Serial.println("WiFi/WebSocket will still start; camera frames are unavailable.");
    return;
  }

  cameraReady = true;
  Serial.println("Camera initialized");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
#ifdef LED_GPIO_NUM
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);
#endif
  cameraFrameMutex = xSemaphoreCreateMutex();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to TFT hotspot");

  uint32_t wifiWaitStart = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - wifiWaitStart > 20000) {
      Serial.println("\nHotspot not found yet; continuing to retry.");
      wifiWaitStart = millis();
    }
  }

  Serial.println("\nWiFi connected");
  Serial.print("Camera IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Connected AP: ");
  Serial.println(WiFi.BSSIDstr());
  Serial.print("Receiver gateway: ");
  Serial.println(WiFi.gatewayIP());

  // Join the TFT AP first, so a camera initialization error cannot prevent
  // the board from appearing as a connected client of the hotspot.
  init_camera();
  startMobileStreamServer();

  // The TFT is the access point; use its gateway IP as the WebSocket host.
  connect_receiver();
}

void loop() {
  if (!client.available()) {
    Serial.println("WebSocket disconnected");
    connect_receiver();
  }

  client.poll();
  if (rebootRequested) {
    Serial.println("Reboot requested by TFT");
    delay(100);
    ESP.restart();
  }
  serviceSosFlash();
  if (!cameraReady || !client.available()) { delay(2); return; }
  if (!captureRequested) {
    if (!tftStreamPaused && millis() - lastTftStreamAt >= TFT_STREAM_INTERVAL_MS && cameraFrameMutex &&
        xSemaphoreTake(cameraFrameMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      camera_fb_t* live = esp_camera_fb_get();
      xSemaphoreGive(cameraFrameMutex);
      if (live) { if (live->format == PIXFORMAT_JPEG) { client.send("LIVE_FRAME"); client.sendBinary((const char*)live->buf, live->len); } esp_camera_fb_return(live); }
      lastTftStreamAt = millis();
    }
    delay(1); return;
  }
  captureRequested = false;
  const bool useFlash = captureWithFlash;
  captureWithFlash = false;
  captureInProgress = true;
  if (cameraFrameMutex) xSemaphoreTake(cameraFrameMutex, portMAX_DELAY);
  Serial.println("Taking one photo...");
  if (useFlash) {
    setFlash(true);
    delay(120); // Let the flash illuminate the scene before exposing the frame.
  }
  // Drain the camera's already-buffered frame first. The frame sent below is
  // captured after the shutter request, avoiding the one-photo-behind effect.
  camera_fb_t* stale = esp_camera_fb_get();
  if (stale) esp_camera_fb_return(stale);
  delay(35);

  camera_fb_t* fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("Camera capture failed");
    if (useFlash) setFlash(false);
    if (cameraFrameMutex) xSemaphoreGive(cameraFrameMutex);
    captureInProgress = false;
    client.send("CAPTURE_FAILED");
    return;
  }

  // The image has been exposed; turn the flash off before network transfer.
  if (useFlash) setFlash(false);
  bool sent = fb->format == PIXFORMAT_JPEG && client.send("PHOTO_FRAME") &&
              client.sendBinary((const char*)fb->buf, fb->len);
  Serial.printf("Photo %s: %u bytes\n", sent ? "sent" : "send failed",
                (unsigned int)fb->len);
  esp_camera_fb_return(fb);
  if (cameraFrameMutex) xSemaphoreGive(cameraFrameMutex);
  captureInProgress = false;
  if (!sent) client.send("CAPTURE_FAILED");
}

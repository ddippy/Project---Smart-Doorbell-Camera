
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <time.h>
#include <sys/time.h>
#include "TFT_Secrets.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

using namespace websockets;

// TFT_eSPI pin/driver settings are configured in its User_Setup.h.
// Use ILI9488_DRIVER for the PCB3501S 3.5-inch SPI TFT; MOSI=23, MISO=19,
// SCLK=18, TFT CS=2, DC=21, RST=4.

const char* AP_SSID = "CAM_TFT_LINK";
const char* AP_PASS = "0641204734";
WebServer timeServer(81);
volatile uint8_t webCommand = 0; // 1 photo, 2 flash, 3 SOS on, 4 SOS off, 5 reboot, 6 max FPS, 7 shared FPS
bool mobileMaxFpsMode = false;
bool cameraModeSyncPending = false;
volatile bool telegramRebootCommand = false;
volatile bool telegramReplyPending = false;
char telegramReplyChatId[24] = "";
char telegramReplyText[320] = "";

QueueHandle_t tftFrameQueue = nullptr;
SemaphoreHandle_t tftMutex = nullptr;
SemaphoreHandle_t photoMutex = nullptr;

TFT_eSPI tft = TFT_eSPI();
WebsocketsServer server;
WebsocketsClient client;
volatile bool expectedPhotoFrame = false;
uint32_t liveFramesDecoded = 0, lastFpsReportAt = 0, displayedFps = 0, lastUiRefreshAt = 0;

bool connected = false;
volatile bool photoReady = false;
volatile bool capturePending = false;
bool decoderStatusPrinted = false;
bool ntpStarted = false;
bool sosEnabled = false;
uint32_t previewExpiresAt = 0;
volatile uint8_t telegramCaptureCommand = 0; // 1 = photo, 2 = flash photo
volatile uint8_t telegramSosCommand = 0; // 1 = start SOS, 2 = stop SOS
volatile bool telegramAutoSendPending = false;
volatile bool telegramUploadInProgress = false;
volatile bool autoSendThisCapture = false;
char telegramRequestedChatId[24] = "";
char telegramActiveChatId[24] = "";
int64_t lastTelegramUpdateId = 0;
bool telegramUpdatesInitialized = false;
char photoTimestamp[24] = "TIME NOT SET";
char uiNotice[32] = "";
uint32_t uiNoticeUntil = 0;
bool tftTaskRunning = false;

// A still photo fills the landscape screen; controls stay overlaid at the bottom.
const int16_t IMAGE_X = 40;
const int16_t IMAGE_Y = 48;
const uint16_t IMAGE_W = 400;
const uint16_t IMAGE_H = 225;
const int16_t UI_TOP_H = 26;
const int16_t UI_BOTTOM_Y = 294;
const int16_t UI_BOTTOM_H = 26;

// Muted charcoal / violet palette inspired by the provided reference.
const uint16_t UI_BG = 0x0861;       // deep near-black
const uint16_t UI_PANEL = 0x2803;    // dark red-charcoal
const uint16_t UI_SURFACE = 0x4105;  // muted red surface
const uint16_t UI_ACCENT = 0xC166;   // crimson accent
const uint16_t UI_TEXT = 0xF77D;     // warm white
const uint16_t UI_MUTED = 0xA534;    // muted rose-gray
const uint16_t UI_LIVE = 0xF9AA;     // bright red highlight
uint16_t frameWidth = 320;
uint16_t frameHeight = 240;
uint8_t* lastPhoto = nullptr;
uint8_t* lastPhotoAllocation = nullptr;
size_t lastPhotoLength = 0;
volatile uint32_t photoGeneration = 0;

void drawOverlay() {
 tft.fillRect(0,0,480,UI_TOP_H,UI_PANEL); tft.setTextColor(UI_TEXT,UI_PANEL); tft.drawString("HOME CAM",10,5,2);
 char status[48];
 if(uiNoticeUntil && (int32_t)(millis()-uiNoticeUntil)<0){snprintf(status,sizeof(status),"%s",uiNotice);tft.setTextColor(UI_LIVE,UI_PANEL);}
 else if(capturePending){snprintf(status,sizeof(status),"CAPTURING");tft.setTextColor(UI_LIVE,UI_PANEL);}
 else if(telegramUploadInProgress){snprintf(status,sizeof(status),"SENDING TELEGRAM");tft.setTextColor(UI_LIVE,UI_PANEL);}
 else if(photoReady){snprintf(status,sizeof(status),"PHOTO %s",photoTimestamp);tft.setTextColor(UI_TEXT,UI_PANEL);}
 else if(sosEnabled){snprintf(status,sizeof(status),"SOS ACTIVE");tft.setTextColor(TFT_RED,UI_PANEL);}
 else if(connected){snprintf(status,sizeof(status),"LIVE");tft.setTextColor(UI_LIVE,UI_PANEL);}
 else {snprintf(status,sizeof(status),"CAM OFFLINE");tft.setTextColor(UI_MUTED,UI_PANEL);}
 tft.drawRightString(status,470,5,2); tft.fillRect(0,UI_BOTTOM_Y,480,UI_BOTTOM_H,UI_PANEL);
 char footer[56]; snprintf(footer,sizeof(footer),"%s  |  %lu FPS  |  TG READY",photoReady?(telegramUploadInProgress?"UPLOADING":"PHOTO"):(capturePending?"HOLD":"CAM_TFT_LINK"),(unsigned long)displayedFps);
 tft.setTextColor(UI_MUTED,UI_PANEL);tft.drawCentreString(footer,240,UI_BOTTOM_Y+6,2);
}

void restoreScreenAfterSos() {
  if (photoReady && lastPhoto && lastPhotoLength) {
    TJpgDec.drawJpg(0, 0, lastPhoto, lastPhotoLength);
  } else if (connected && client.available()) {
    client.send("RESUME_STREAM");
  }
  if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
  drawOverlay();
  if (tftMutex) xSemaphoreGive(tftMutex);
}

void setNotice(const char* message, uint32_t durationMs = 2200) {
  snprintf(uiNotice, sizeof(uiNotice), "%s", message);
  uiNoticeUntil = millis() + durationMs;
  if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
  drawOverlay();
  if (tftMutex) xSemaphoreGive(tftMutex);
}

void stampCurrentTime() {
  const time_t now = time(nullptr);
  if (now < 1700000000) {
    snprintf(photoTimestamp, sizeof(photoTimestamp), "TIME NOT SET");
    return;
  }
  struct tm localTime;
  localtime_r(&now, &localTime);
  strftime(photoTimestamp, sizeof(photoTimestamp), "%d/%m/%Y %H:%M:%S", &localTime);
}

bool isTelegramChatAllowed(const String& chatId) {
  String allowed = TELEGRAM_ALLOWED_CHAT_IDS;
  int start = 0;
  while (start < allowed.length()) {
    int end = allowed.indexOf(',', start);
    if (end < 0) end = allowed.length();
    String candidate = allowed.substring(start, end);
    candidate.trim();
    if (candidate == chatId) return true;
    start = end + 1;
  }
  return false;
}

// Send an owned JPEG snapshot from a worker so the WebSocket/UI loop stays responsive.
struct TelegramPhotoJob {
  uint8_t* jpeg;
  size_t length;
  char chatId[24];
  char timestamp[24];
};

void telegramPhotoUploadTask(void* parameter) {
  TelegramPhotoJob* job = (TelegramPhotoJob*)parameter;
  const String boundary = "----HomeCamBoundary7MA4YWxkTrZu0gW";
  String prefix = "--" + boundary + "\r\n"
                  "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
  prefix += job->chatId;
  prefix += "\r\n--" + boundary + "\r\n"
            "Content-Disposition: form-data; name=\"caption\"\r\n\r\n";
  prefix += "HOME CAM "; prefix += job->timestamp;
  prefix += "\r\n--" + boundary + "\r\n"
            "Content-Disposition: form-data; name=\"photo\"; filename=\"home.jpg\"\r\n"
            "Content-Type: image/jpeg\r\n\r\n";
  const String suffix = "\r\n--" + boundary + "--\r\n";
  const size_t totalLength = prefix.length() + job->length + suffix.length();
  uint8_t* body = (uint8_t*)malloc(totalLength);
  bool sent = false;
  if (body) {
    size_t offset = 0;
    memcpy(body + offset, prefix.c_str(), prefix.length()); offset += prefix.length();
    memcpy(body + offset, job->jpeg, job->length); offset += job->length;
    memcpy(body + offset, suffix.c_str(), suffix.length());
    WiFiClientSecure tls; tls.useBuiltinCACertBundle();
    HTTPClient http;
    String url = "https://api.telegram.org/bot"; url += TELEGRAM_BOT_TOKEN; url += "/sendPhoto";
    if (http.begin(tls, url)) {
      http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
      http.setTimeout(15000);
      const int code = http.POST(body, totalLength);
      const String response = http.getString();
      sent = code == 200 && response.indexOf("\"ok\":true") >= 0;
      Serial.printf("Telegram send HTTP status: %d\n", code);
      http.end();
    } else Serial.println("Could not open Telegram HTTPS connection");
    tls.stop(); free(body);
  } else Serial.println("Not enough memory to build Telegram request");
  free(job);
  if (photoReady) previewExpiresAt = millis() + 10000;
  telegramUploadInProgress = false;
  setNotice(sent ? "TELEGRAM SENT" : "TELEGRAM FAILED");
  vTaskDelete(nullptr);
}

// Snapshot the displayed photo and start the upload without blocking live control.
void sendPreviewPhotoToTelegram() {
  if (telegramUploadInProgress) { setNotice("TELEGRAM BUSY"); return; }
  if (capturePending) { setNotice("WAIT FOR PREVIEW"); return; }
  const char* destination = telegramActiveChatId[0] ? telegramActiveChatId : TELEGRAM_CHAT_ID;
  if (strlen(TELEGRAM_BOT_TOKEN) < 20 || strlen(destination) == 0) { setNotice("TELEGRAM NOT SET"); return; }
  if (WiFi.status() != WL_CONNECTED) { setNotice("HOME WIFI OFFLINE"); return; }
  if (!photoMutex || xSemaphoreTake(photoMutex, pdMS_TO_TICKS(1000)) != pdTRUE) { setNotice("PHOTO BUSY"); return; }
  if (!photoReady || !lastPhoto || lastPhotoLength == 0) {
    xSemaphoreGive(photoMutex); setNotice("NO PHOTO PREVIEW"); return;
  }
  TelegramPhotoJob* job = (TelegramPhotoJob*)malloc(sizeof(TelegramPhotoJob));
  if (!job) { xSemaphoreGive(photoMutex); setNotice("NOT ENOUGH MEMORY"); return; }
  // Keep the retained preview alive until the worker finishes; no second JPEG copy.
  job->length = lastPhotoLength; job->jpeg = lastPhoto;
  snprintf(job->chatId, sizeof(job->chatId), "%s", destination);
  snprintf(job->timestamp, sizeof(job->timestamp), "%s", photoTimestamp);
  xSemaphoreGive(photoMutex);
  telegramUploadInProgress = true;
  setNotice("SENDING TELEGRAM", 30000);
  if (xTaskCreatePinnedToCore(telegramPhotoUploadTask, "telegram-photo", 8192,
                              job, 1, nullptr, 0) != pdPASS) {
    telegramUploadInProgress = false; free(job); setNotice("UPLOAD TASK FAILED");
  }
}

void queueTelegramReply(const char* chatId, const char* text) {
  if (telegramReplyPending) return;
  snprintf(telegramReplyChatId, sizeof(telegramReplyChatId), "%s", chatId);
  snprintf(telegramReplyText, sizeof(telegramReplyText), "%s", text);
  telegramReplyPending = true;
}

void sendQueuedTelegramReply() {
  if (!telegramReplyPending || WiFi.status() != WL_CONNECTED) return;
  WiFiClientSecure tls; tls.useBuiltinCACertBundle(); HTTPClient http;
  String url = "https://api.telegram.org/bot"; url += TELEGRAM_BOT_TOKEN; url += "/sendMessage";
  String body = "chat_id="; body += telegramReplyChatId; body += "&text=";
  const char* hex = "0123456789ABCDEF";
  for (size_t i=0; telegramReplyText[i]; ++i) {
    const uint8_t ch = (uint8_t)telegramReplyText[i];
    if ((ch>='a'&&ch<='z')||(ch>='A'&&ch<='Z')||(ch>='0'&&ch<='9')||ch=='-'||ch=='_'||ch=='.'||ch=='~') body += (char)ch;
    else if (ch==' ') body += '+';
    else { body += '%'; body += hex[ch>>4]; body += hex[ch&0x0f]; }
  }
  String contentType="application/x-www-form-urlencoded";
  bool sent=false;
  if(http.begin(tls,url)){http.addHeader("Content-Type",contentType);http.setTimeout(10000);sent=http.POST((uint8_t*)body.c_str(),body.length())==200;http.end();}
  tls.stop(); telegramReplyPending=false;
  Serial.printf("Telegram reply %s\n", sent?"sent":"failed");
}

void processTelegramUpdates(const String& json, bool executeCommands) {
  int searchFrom = 0;
  while (true) {
    const int updateAt = json.indexOf("\"update_id\":", searchFrom);
    if (updateAt < 0) break;
    const int valueAt = updateAt + 12;
    const int nextUpdate = json.indexOf("\"update_id\":", valueAt);
    const int blockEnd = nextUpdate < 0 ? json.length() : nextUpdate;
    const String update = json.substring(updateAt, blockEnd);
    const long long updateId = atoll(json.c_str() + valueAt);
    if (updateId >= lastTelegramUpdateId) lastTelegramUpdateId = updateId + 1;

    const int chatAt = update.indexOf("\"chat\"");
    const int chatIdAt = chatAt < 0 ? -1 : update.indexOf("\"id\"", chatAt);
    String chatId;
    if (chatIdAt >= 0) {
      const int colon = update.indexOf(':', chatIdAt);
      if (colon >= 0) {
        int start = colon + 1;
        while (start < update.length() && update[start] == ' ') ++start;
        int end = start;
        if (end < update.length() && update[end] == '-') ++end;
        while (end < update.length() && isDigit(update[end])) ++end;
        chatId = update.substring(start, end);
      }
    }

    const int textAt = update.indexOf("\"text\":\"");
    if (executeCommands && isTelegramChatAllowed(chatId) && textAt >= 0) {
      const int start = textAt + 8;
      const int end = update.indexOf('"', start);
      if (end > start) {
        String command = update.substring(start, end);
        const int mention = command.indexOf('@');
        if (mention >= 0) command.remove(mention);
        if (command == "/photo") {
          if (telegramCaptureCommand == 0) {
            snprintf(telegramRequestedChatId, sizeof(telegramRequestedChatId),
                     "%s", chatId.c_str());
            telegramCaptureCommand = 1;
            Serial.printf("Telegram command /photo from %s\n", chatId.c_str());
          }
        } else if (command == "/flash") {
          if (telegramCaptureCommand == 0) {
            snprintf(telegramRequestedChatId, sizeof(telegramRequestedChatId),
                     "%s", chatId.c_str());
            telegramCaptureCommand = 2;
            Serial.printf("Telegram command /flash from %s\n", chatId.c_str());
          }
        } else if (command == "/startsos") {
          telegramSosCommand = 1;
          Serial.printf("Telegram command /startsos from %s\n", chatId.c_str());
        } else if (command == "/stopsos") {
          telegramSosCommand = 2;
          Serial.printf("Telegram command /stopsos from %s\n", chatId.c_str());
        } else if (command == "/help") {
          queueTelegramReply(chatId.c_str(), "/photo - take photo\n/flash - flash photo\n/startsos /stopsos - SOS flash\n/status - camera status\n/stream - open live view\n/reboot - restart camera");
        } else if (command == "/status") {
          char statusText[180];
          snprintf(statusText, sizeof(statusText), "Camera: %s\nTFT stream: %s\nDisplay FPS: %lu\nTelegram photo: %s",
                   connected ? "ONLINE" : "OFFLINE", (photoReady || capturePending || telegramUploadInProgress) ? "PAUSED" : "LIVE",
                   (unsigned long)displayedFps, telegramUploadInProgress ? "SENDING" : "READY");
          queueTelegramReply(chatId.c_str(), statusText);
        } else if (command == "/stream") {
          queueTelegramReply(chatId.c_str(), "Live view page: http://192.168.4.1:81/\nConnect your phone to CAM_TFT_LINK first.");
        } else if (command == "/reboot") {
          telegramRebootCommand = true;
          Serial.printf("Telegram command /reboot from %s\n", chatId.c_str());
        }
      }
    }
    searchFrom = blockEnd;
    if (nextUpdate < 0) break;
  }
}

void telegramCommandTask(void* parameter) {
  for (;;) {
    if (HOME_WIFI_SSID[0] == '\0' || WiFi.status() != WL_CONNECTED ||
        strlen(TELEGRAM_BOT_TOKEN) < 20 || strlen(TELEGRAM_CHAT_ID) == 0) {
      vTaskDelay(pdMS_TO_TICKS(3000));
      continue;
    }

    WiFiClientSecure tls;
    tls.useBuiltinCACertBundle();
    HTTPClient http;
    String url = "https://api.telegram.org/bot";
    url += TELEGRAM_BOT_TOKEN;
    url += "/getUpdates?limit=5&timeout=10&offset=";
    char offsetText[24];
    snprintf(offsetText, sizeof(offsetText), "%lld", (long long)lastTelegramUpdateId);
    url += offsetText;
    if (http.begin(tls, url)) {
      http.setTimeout(15000);
      const int code = http.GET();
      if (code == 200) {
        const String updates = http.getString();
        const bool hadUpdates = updates.indexOf("\"update_id\":") >= 0;
        processTelegramUpdates(updates, telegramUpdatesInitialized);
        if (!hadUpdates) telegramUpdatesInitialized = true;
      }
      else if (code > 0) Serial.printf("Telegram polling HTTP status: %d\n", code);
      http.end();
    }
    tls.stop();
    sendQueuedTelegramReply();
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

bool requestPhotoCapture(bool withFlash, bool sendToTelegram,
                         bool preserveTelegramChat = false) {
  if (sosEnabled) {
    setNotice("STOP SOS TO TAKE PHOTO");
    return false;
  }
  if (capturePending) return false;
  if (!connected || !client.available()) {
    setNotice("CAM OFFLINE");
    return false;
  }
  stampCurrentTime();
  capturePending = true;
  autoSendThisCapture = sendToTelegram;
  if (sendToTelegram && !preserveTelegramChat) telegramActiveChatId[0] = '\0';
  if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
  drawOverlay();
  if (tftMutex) xSemaphoreGive(tftMutex);
  if (!client.send("PAUSE_STREAM")) { capturePending=false; autoSendThisCapture=false; setNotice("STREAM PAUSE FAILED"); return false; }
  const char* command = withFlash ? "CAPTURE_FLASH" : "CAPTURE";
  if (!client.send(command)) {
    client.send("RESUME_STREAM");
    capturePending = false;
    autoSendThisCapture = false;
    setNotice("CAPTURE SEND FAILED");
    return false;
  }
  Serial.printf("%s requested at %s\n", command, photoTimestamp);
  return true;
}

void toggleSos() {
  const bool enable = !sosEnabled;
  if (!connected || !client.available()) {
    setNotice("CAM OFFLINE");
    return;
  }
  if (!client.send(enable ? "SOS_ON" : "SOS_OFF")) {
    setNotice("SOS COMMAND FAILED");
    return;
  }
  sosEnabled = enable;
  if (enable) {
    setNotice("SOS SIGNAL ACTIVE");
  } else {
    restoreScreenAfterSos();
    setNotice("SOS STOPPED");
  }
  if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
  drawOverlay();
  if (tftMutex) xSemaphoreGive(tftMutex);
}

bool tft_output(int16_t x, int16_t y,
                uint16_t w, uint16_t h,
                uint16_t* bitmap) {
  if (x >= frameWidth || y >= frameHeight || w == 0 || h == 0) return true;

  // Center-crop the QVGA still slightly, then scale it to fill the 480x320 LCD.
  uint16_t cropWidth = frameWidth;
  uint16_t cropHeight = frameHeight;
  if ((uint32_t)frameWidth * IMAGE_H > (uint32_t)frameHeight * IMAGE_W) {
    cropWidth = (uint32_t)frameHeight * IMAGE_W / IMAGE_H;
  } else {
    cropHeight = (uint32_t)frameWidth * IMAGE_H / IMAGE_W;
  }
  const uint16_t cropX = (frameWidth - cropWidth) / 2;
  const uint16_t cropY = (frameHeight - cropHeight) / 2;
  const uint16_t clippedX0 = max((uint16_t)x, cropX);
  const uint16_t clippedY0 = max((uint16_t)y, cropY);
  const uint16_t clippedX1 = min((uint16_t)(x + w), (uint16_t)(cropX + cropWidth));
  const uint16_t clippedY1 = min((uint16_t)(y + h), (uint16_t)(cropY + cropHeight));
  if (clippedX0 >= clippedX1 || clippedY0 >= clippedY1) return true;

  const uint16_t x0 = (uint32_t)(clippedX0 - cropX) * IMAGE_W / cropWidth;
  const uint16_t x1 = ((uint32_t)(clippedX1 - cropX) * IMAGE_W + cropWidth - 1) / cropWidth;
  const uint16_t y0 = (uint32_t)(clippedY0 - cropY) * IMAGE_H / cropHeight;
  const uint16_t y1 = ((uint32_t)(clippedY1 - cropY) * IMAGE_H + cropHeight - 1) / cropHeight;
  const uint16_t outWidth = x1 - x0;
  const uint16_t outHeight = y1 - y0;
  if ((uint32_t)outWidth * outHeight > 32U * 32U) return true;
  static uint16_t scaled[32 * 32];

  for (uint16_t outY = 0; outY < outHeight; ++outY) {
    uint16_t srcY = cropY + (uint32_t)(y0 + outY) * cropHeight / IMAGE_H;
    if (srcY < clippedY0) srcY = clippedY0;
    if (srcY >= clippedY1) srcY = clippedY1 - 1;
    uint16_t* out = scaled + outY * outWidth;
    const uint16_t* src = bitmap + (srcY - y) * w;
    for (uint16_t outX = 0; outX < outWidth; ++outX) {
      uint16_t srcX = cropX + (uint32_t)(x0 + outX) * cropWidth / IMAGE_W;
      if (srcX < clippedX0) srcX = clippedX0;
      if (srcX >= clippedX1) srcX = clippedX1 - 1;
      out[outX] = src[srcX - x];
    }
  }
  if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
  if (!sosEnabled) tft.pushImage(IMAGE_X + x0, IMAGE_Y + y0, outWidth, outHeight, scaled);
  if (tftMutex) xSemaphoreGive(tftMutex);
  return true;
}

void tftFrameTask(void* parameter) {
 uint8_t* allocation=nullptr;
 for(;;){
  if(xQueueReceive(tftFrameQueue,&allocation,portMAX_DELAY)!=pdTRUE||!allocation)continue;
  size_t length=*((size_t*)allocation); bool still=allocation[sizeof(size_t)]!=0; uint8_t* jpg=allocation+sizeof(size_t)+1;
  if(!still&&(photoReady||capturePending)){free(allocation);allocation=nullptr;continue;}
  TJpgDec.getJpgSize(&frameWidth,&frameHeight,jpg,length); JRESULT result=TJpgDec.drawJpg(0,0,jpg,length);
  if(result==JDR_OK&&still){
   if(photoMutex)xSemaphoreTake(photoMutex,portMAX_DELAY); uint8_t* old=lastPhotoAllocation;
   lastPhotoAllocation=allocation;lastPhoto=jpg;lastPhotoLength=length;allocation=nullptr;photoReady=true;++photoGeneration;previewExpiresAt=millis()+10000;
   if(autoSendThisCapture){autoSendThisCapture=false;telegramAutoSendPending=true;}
   if(photoMutex)xSemaphoreGive(photoMutex);free(old);capturePending=false;
   if(tftMutex)xSemaphoreTake(tftMutex,portMAX_DELAY);drawOverlay();if(tftMutex)xSemaphoreGive(tftMutex);
  }else if(result==JDR_OK){++liveFramesDecoded;}
  else if(still){capturePending=false;if(connected&&client.available())client.send("RESUME_STREAM");}
  if(result!=JDR_OK)Serial.printf("JPEG decode error: %d\n",(int)result);
  free(allocation);allocation=nullptr;
 }
}
bool queueFrameForTft(const char* data,size_t length,bool still){
 if(!tftFrameQueue)return false;uint8_t* a=(uint8_t*)malloc(sizeof(size_t)+1+length);if(!a)return false;
 *((size_t*)a)=length;a[sizeof(size_t)]=still?1:0;memcpy(a+sizeof(size_t)+1,data,length);
 uint8_t* old=nullptr;if(xQueueReceive(tftFrameQueue,&old,0)==pdTRUE)free(old);xQueueOverwrite(tftFrameQueue,&a);return true;
}

void webPhotoHandler() {
  if (!photoMutex || xSemaphoreTake(photoMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    timeServer.send(503, "text/plain", "Photo is busy");
    return;
  }
  if (!photoReady || !lastPhoto || lastPhotoLength == 0) {
    xSemaphoreGive(photoMutex);
    timeServer.sendHeader("Cache-Control", "no-store");
    timeServer.send(404, "text/plain", "No captured photo available");
    return;
  }

  const uint8_t* jpeg = lastPhoto;
  const size_t jpegLength = lastPhotoLength;
  timeServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  timeServer.sendHeader("Content-Disposition", "inline; filename=homecam.jpg");
  timeServer.setContentLength(jpegLength);
  timeServer.send(200, "image/jpeg", "");
  size_t sent = 0;
  while (sent < jpegLength && timeServer.client().connected()) {
    const size_t chunk = (jpegLength - sent > 1024) ? 1024 : jpegLength - sent;
    const size_t written = timeServer.client().write(jpeg + sent, chunk);
    if (written == 0) break;
    sent += written;
  }
  xSemaphoreGive(photoMutex);
}

void webCommandHandler() {
  const String name = timeServer.arg("name");
  if (name == "photo") webCommand = 1;
  else if (name == "flash") webCommand = 2;
  else if (name == "sos_on") webCommand = 3;
  else if (name == "sos_off") webCommand = 4;
  else if (name == "reboot") webCommand = 5;
  else if (name == "maxfps") webCommand = 6;
  else if (name == "sharedfps") webCommand = 7;
  else { timeServer.send(400, "application/json", "{\"ok\":false,\"error\":\"unknown command\"}"); return; }
  timeServer.send(200, "application/json", "{\"ok\":true,\"queued\":true}");
}

void webStatusHandler() {
  char json[192];
  snprintf(json, sizeof(json), "{\"camera\":\"%s\",\"live\":%s,\"maxFps\":%s,\"fps\":%lu,\"photo\":%s,\"photoId\":%lu,\"uploading\":%s}",
    connected ? "ONLINE" : "OFFLINE", (!mobileMaxFpsMode && !photoReady && !capturePending && !telegramUploadInProgress) ? "true" : "false",
    mobileMaxFpsMode ? "true" : "false",
    (unsigned long)displayedFps, photoReady ? "true" : "false", (unsigned long)photoGeneration,
    telegramUploadInProgress ? "true" : "false");
  timeServer.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);

  // Configure TFT_eSPI's User_Setup.h for ILI9488 and the ESP32 SPI pins.
  tft.init();
  tft.setRotation(1); // Flip the landscape UI 180 degrees for the opposite mounting direction.
  tft.setSwapBytes(false);
  tftMutex = xSemaphoreCreateMutex();
  photoMutex = xSemaphoreCreateMutex();

  // Bring up the camera access point.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  if (HOME_WIFI_SSID[0] != '\0') {
    WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASSWORD);
    Serial.println("Connecting to home Wi-Fi for NTP and Telegram...");
  }
  setenv("TZ", "ICT-7", 1); // Thailand time, UTC+7
  tzset();
  Serial.print("Receiver AP: ");
  Serial.println(AP_SSID);
  Serial.print("Receiver IP: ");
  Serial.println(WiFi.softAPIP());
  server.listen(80);
  Serial.println("Live video + command WebSocket listening on port 80");

  timeServer.on("/", HTTP_GET, []() {
    static const char page[] = R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#12090c"><title>HOME CAM LIVE</title>
<style>*{box-sizing:border-box}body{margin:0;background:#100b0d;color:#f7e9eb;font:15px system-ui}header{height:54px;padding:0 16px;display:flex;align-items:center;justify-content:space-between;background:#291017;border-bottom:1px solid #c51f36}header b{letter-spacing:.12em;color:#ff5365}.pill{font-size:12px;color:#ff9aa5}main{max-width:850px;margin:auto;padding:14px}.video,.photo{width:100%;aspect-ratio:4/3;background:#050405;border:1px solid #64202c;border-radius:12px;object-fit:contain}.status{padding:9px 4px;color:#d5b8bd;font-size:13px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:9px;margin:10px 0}.grid button{border:1px solid #772632;background:#34131b;color:#fff;padding:14px 7px;border-radius:10px;font-weight:700;font-size:14px}.grid button:active{background:#c51f36}.grid .danger{background:#941c30}.note{color:#a98b91;font-size:12px;text-align:center;padding:8px}.photo-panel{margin-top:14px;padding:12px;border:1px solid #64202c;border-radius:12px;background:#1a1013}.photo-panel[hidden]{display:none}.photo-title{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;color:#ff9aa5;font-weight:700}.download{display:inline-block;margin-top:10px;padding:10px 16px;background:#941c30;color:#fff;text-decoration:none;border-radius:8px;font-weight:700}</style></head><body>
<header><b>HOME CAM</b><span class="pill" id="badge">CONNECTING</span></header><main><img id="video" class="video" alt="Connecting to camera stream"><div class="status" id="status">Finding camera stream…</div>
<div class="grid"><button onclick="cmd('photo')">PHOTO</button><button onclick="cmd('flash')">FLASH PHOTO</button><button id="fpsModeButton" onclick="toggleFpsMode()">10 FPS · SHARE TFT</button><button class="danger" onclick="cmd('sos_on')">START SOS</button><button onclick="cmd('sos_off')">STOP SOS</button><button onclick="cmd('reboot')">REBOOT CAM</button></div><section class="photo-panel" id="photoPanel" hidden><div class="photo-title"><span>CAPTURED PHOTO</span><span id="photoTime"></span></div><img id="photoPreview" class="photo" alt="Captured photo"><a id="downloadPhoto" class="download" download="homecam.jpg">DOWNLOAD PHOTO</a></section><div class="note">10 FPS shares camera time with TFT. MAX FPS pauses the live picture on TFT while the phone stream runs uncapped.</div></main>
<script>
fetch('/set-time?epoch='+Math.floor(Date.now()/1000)).catch(()=>{});
let cameraFound=false;const video=document.getElementById('video');video.onerror=()=>{cameraFound=false;video.removeAttribute('src');document.getElementById('status').textContent='Camera stream failed. Reconnecting…';};async function findCamera(){if(cameraFound)return;const img=document.getElementById('video'),s=document.getElementById('status');for(let n=2;n<=10;n++){const ip='192.168.4.'+n,ctl=new AbortController(),timer=setTimeout(()=>ctl.abort(),700);try{const r=await fetch('http://'+ip+':81/whoami',{cache:'no-store',signal:ctl.signal});if(r.ok&&(await r.text()).includes('HOME_CAM_CAMERA')){cameraFound=true;img.src='http://'+ip+':81/stream';s.textContent='Live video · '+ip+' · QVGA / up to 10 FPS';document.getElementById('badge').textContent='LIVE';clearTimeout(timer);return;}}catch(e){}clearTimeout(timer);}s.textContent='Camera not found. Retrying… Check CAM_TFT_LINK.';document.getElementById('badge').textContent='CAM OFFLINE';}
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));async function waitForPhoto(previousId){for(let i=0;i<50;i++){await wait(400);try{const st=await(await fetch('/api/status?ts='+Date.now(),{cache:'no-store'})).json();if(st.photo&&st.photoId!==previousId){const r=await fetch('/api/photo?ts='+Date.now(),{cache:'no-store'});if(!r.ok)continue;const blob=await r.blob(),url=URL.createObjectURL(blob);const p=document.getElementById('photoPreview'),a=document.getElementById('downloadPhoto');if(p.dataset.url)URL.revokeObjectURL(p.dataset.url);p.dataset.url=url;p.src=url;a.href=url;a.download='homecam-'+new Date().toISOString().replace(/[:.]/g,'-')+'.jpg';document.getElementById('photoTime').textContent=new Date().toLocaleTimeString();document.getElementById('photoPanel').hidden=false;return true;}}catch(e){}}return false;}
async function cmd(name){const s=document.getElementById('status');try{let oldId=0;if(name==='photo'||name==='flash'){oldId=(await(await fetch('/api/status?ts='+Date.now(),{cache:'no-store'})).json()).photoId||0;s.textContent='Taking photo…';}const r=await fetch('/api/cmd?name='+encodeURIComponent(name),{cache:'no-store'}),j=await r.json();if(!j.ok){s.textContent='Command failed';return;}if(name==='photo'||name==='flash'){s.textContent='Waiting for captured photo…';s.textContent=await waitForPhoto(oldId)?'Photo ready. Tap Download Photo to save it.':'Capture timed out. Check camera connection.';}else{s.textContent='Command sent: '+name;}}catch(e){s.textContent='Command request failed';}}
let currentMaxFps=false;async function toggleFpsMode(){const target=!currentMaxFps,name=target?'maxfps':'sharedfps',s=document.getElementById('status');try{const r=await fetch('/api/cmd?name='+name,{cache:'no-store'}),j=await r.json();if(j.ok){currentMaxFps=target;document.getElementById('fpsModeButton').textContent=target?'MAX FPS · TFT PAUSED':'10 FPS · SHARE TFT';s.textContent=target?'Switching to MAX FPS. TFT live picture will pause…':'Switching to shared 10 FPS…';}}catch(e){s.textContent='FPS mode command failed';}}
async function status(){try{const j=await(await fetch('/api/status',{cache:'no-store'})).json();currentMaxFps=!!j.maxFps;document.getElementById('fpsModeButton').textContent=currentMaxFps?'MAX FPS · TFT PAUSED':'10 FPS · SHARE TFT';document.getElementById('badge').textContent=j.camera+(currentMaxFps?' · MAX FPS / TFT PAUSED':j.live?' · LIVE':' · PAUSED');}catch(e){}}
findCamera();setInterval(()=>{if(!cameraFound)findCamera();status();},4000);
</script></body></html>)HTML";
    timeServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    timeServer.send(200, "text/html; charset=utf-8", page);
  });
  timeServer.on("/api/cmd", HTTP_GET, webCommandHandler);
  timeServer.on("/api/photo", HTTP_GET, webPhotoHandler);
  timeServer.on("/api/status", HTTP_GET, webStatusHandler);
  timeServer.on("/set-time", HTTP_GET, []() {
    const long epoch = timeServer.arg("epoch").toInt();
    if (epoch < 1700000000L) {
      timeServer.send(400, "text/plain", "Invalid phone time");
      return;
    }
    struct timeval tv = {};
    tv.tv_sec = epoch;
    settimeofday(&tv, nullptr);
    timeServer.send(200, "text/plain", "Clock synced. You can return to the TFT.");
    Serial.printf("Clock synced from phone: %ld\n", epoch);
  });
  timeServer.begin();
  Serial.println("To set date/time: connect phone to CAM_TFT_LINK, open http://192.168.4.1:81/");

  Serial.println("Touchless live video mode");

  tft.fillScreen(UI_BG);
  tft.setTextColor(UI_TEXT, UI_BG);
  tft.drawCentreString("WAITING FOR CAMERA", 240, 145, 2);
  drawOverlay();

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);
  Serial.println("Latest-frame renderer enabled");
  tftFrameQueue = xQueueCreate(1, sizeof(uint8_t*));
  if (tftMutex && tftFrameQueue &&
      xTaskCreatePinnedToCore(tftFrameTask, "tft-frame", 4096, nullptr,
                              1, nullptr, 1) == pdPASS) {
    tftTaskRunning = true;
    Serial.println("TFT renderer running independently; latest-frame queue enabled");
  } else {
    Serial.println("TFT background renderer could not start");
  }
  if (HOME_WIFI_SSID[0] != '\0' && strlen(TELEGRAM_BOT_TOKEN) >= 20 &&
      strlen(TELEGRAM_CHAT_ID) > 0) {
    if (xTaskCreatePinnedToCore(telegramCommandTask, "telegram-poll", 8192,
                                nullptr, 1, nullptr, 0) == pdPASS) {
      Serial.println("Telegram commands enabled: /photo /flash /startsos /stopsos /help /status /stream /reboot");
    } else {
      Serial.println("Telegram polling task could not start");
    }
  }
}

void loop() {
  timeServer.handleClient();
  const uint8_t action = webCommand; webCommand = 0;
  if (action == 1 || action == 2) { requestPhotoCapture(action==2, false); }
  else if (action == 3) telegramSosCommand = 1;
  else if (action == 4) telegramSosCommand = 2;
  else if (action == 5) telegramRebootCommand = true;
  else if (action == 6) { mobileMaxFpsMode = true; cameraModeSyncPending = true; setNotice("MAX FPS: TFT STREAM PAUSED"); }
  else if (action == 7) { mobileMaxFpsMode = false; cameraModeSyncPending = true; setNotice("SHARED 10 FPS MODE"); }
  if (cameraModeSyncPending && connected && client.available()) {
    if (client.send(mobileMaxFpsMode ? "MOBILE_MAX_ON" : "MOBILE_SHARED"))
      cameraModeSyncPending = false;
  }
  if (telegramRebootCommand && connected && client.available()) {
    telegramRebootCommand = false;
    if (client.send("REBOOT")) setNotice("REBOOTING CAMERA");
    else setNotice("REBOOT COMMAND FAILED");
  }
  if(millis()-lastFpsReportAt>=1000){displayedFps=liveFramesDecoded;liveFramesDecoded=0;lastFpsReportAt=millis();}
  if(millis()-lastUiRefreshAt>=1000){lastUiRefreshAt=millis();if(tftMutex)xSemaphoreTake(tftMutex,portMAX_DELAY);drawOverlay();if(tftMutex)xSemaphoreGive(tftMutex);}
  if (!ntpStarted && WiFi.status() == WL_CONNECTED) {
    configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
    ntpStarted = true;
    Serial.println("NTP time sync requested");
  }
  if (uiNoticeUntil && (int32_t)(millis() - uiNoticeUntil) >= 0) {
    uiNoticeUntil = 0;
    if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
    drawOverlay();
    if (tftMutex) xSemaphoreGive(tftMutex);
  }
  if (photoReady && !capturePending && !telegramUploadInProgress && previewExpiresAt &&
      (int32_t)(millis() - previewExpiresAt) >= 0) {
    uint8_t* expiredPhoto = nullptr;
    if (photoMutex) xSemaphoreTake(photoMutex, portMAX_DELAY);
    if (photoReady && (int32_t)(millis() - previewExpiresAt) >= 0) {
      photoReady = false;
      previewExpiresAt = 0;
      expiredPhoto = lastPhotoAllocation;
      lastPhotoAllocation = nullptr;
      lastPhoto = nullptr;
      lastPhotoLength = 0;
    }
    if (photoMutex) xSemaphoreGive(photoMutex);
    free(expiredPhoto);
    if (expiredPhoto) {
      if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
      tft.fillRect(IMAGE_X, IMAGE_Y, IMAGE_W, IMAGE_H, UI_BG);
      tft.setTextColor(UI_MUTED, UI_BG);
      tft.drawCentreString("RESUMING LIVE VIDEO", 240, 150, 2);
      drawOverlay();
      if (connected && client.available()) client.send("RESUME_STREAM");
      if (tftMutex) xSemaphoreGive(tftMutex);
      Serial.println("Preview expired after 10 seconds");
    }
  }

  uint8_t botCommand = telegramCaptureCommand;
  if (botCommand != 0) {
    telegramCaptureCommand = 0;
    if (capturePending || telegramUploadInProgress) {
      Serial.println("Telegram capture ignored: another photo operation is in progress");
    } else if (!connected || !client.available()) {
      Serial.println("Telegram capture ignored: CAM is offline");
    } else {
      snprintf(telegramActiveChatId, sizeof(telegramActiveChatId), "%s",
               telegramRequestedChatId);
      telegramRequestedChatId[0] = '\0';
      requestPhotoCapture(botCommand == 2, true, true);
    }
  }
  if (telegramAutoSendPending && !capturePending) {
    telegramAutoSendPending = false;
    sendPreviewPhotoToTelegram();
  }

  if (telegramSosCommand != 0 && connected && client.available()) {
    const uint8_t command = telegramSosCommand;
    if (client.send(command == 1 ? "SOS_ON" : "SOS_OFF")) {
      telegramSosCommand = 0;
      sosEnabled = command == 1;
      if (command == 1) {
        setNotice("SOS SIGNAL ACTIVE");
      } else {
        restoreScreenAfterSos();
        setNotice("SOS STOPPED");
      }
      Serial.println(command == 1 ? "Telegram started SOS flash" :
                                    "Telegram stopped SOS flash");
      if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
      drawOverlay();
      if (tftMutex) xSemaphoreGive(tftMutex);
    }
  }

  if (!connected) {
    client = server.accept();

    if (client.available()) {
      connected = true;
      cameraModeSyncPending = true;
      Serial.println("CAM connected");

      client.onMessage([](WebsocketsMessage message) {
        if(message.isText()){
          String status=message.data();
          if(status=="PHOTO_FRAME")expectedPhotoFrame=true;else if(status=="LIVE_FRAME")expectedPhotoFrame=false;
          else if(status=="CAPTURE_FAILED"){expectedPhotoFrame=false;capturePending=false;autoSendThisCapture=false;if(connected&&client.available())client.send("RESUME_STREAM");if(tftMutex)xSemaphoreTake(tftMutex,portMAX_DELAY);drawOverlay();if(tftMutex)xSemaphoreGive(tftMutex);}
          return;
        }
        if(!message.isBinary())return;auto jpg=message.rawData();bool still=expectedPhotoFrame;expectedPhotoFrame=false;if(jpg.empty())return;
        if(still)photoReady=false;
        if(tftTaskRunning){if(!queueFrameForTft(jpg.data(),jpg.size(),still)){if(still){capturePending=false;if(connected&&client.available())client.send("RESUME_STREAM");}}}
        else if(still||(!photoReady&&!capturePending)){TJpgDec.getJpgSize(&frameWidth,&frameHeight,(const uint8_t*)jpg.data(),jpg.size());JRESULT r=TJpgDec.drawJpg(0,0,(const uint8_t*)jpg.data(),jpg.size());if(r==JDR_OK&&still){photoReady=true;previewExpiresAt=millis()+10000;capturePending=false;drawOverlay();}else if(r==JDR_OK)++liveFramesDecoded;}
      });

      client.onEvent([](WebsocketsEvent event, String data) {
        if (event == WebsocketsEvent::ConnectionClosed) {
          connected = false;
          capturePending = false;
          sosEnabled = false;
          Serial.println("CAM disconnected");
          if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
          drawOverlay();
          if (tftMutex) xSemaphoreGive(tftMutex);
        }
      });
    }
  }

  if (connected) {
    client.poll();

    if (!client.available()) {
      connected = false;
      capturePending = false;
      sosEnabled = false;
      Serial.println("Waiting for reconnect");
      if (tftMutex) xSemaphoreTake(tftMutex, portMAX_DELAY);
      drawOverlay();
      if (tftMutex) xSemaphoreGive(tftMutex);
    }
  }

}




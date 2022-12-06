// Original Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <esp_timer.h>
#include <esp_camera.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <Arduino.h>
#include <WiFi.h>
#include "storage.h"

// Functions from the main .ino
extern void flashLED(int flashtime);
extern void setLamp(int newVal);
extern void printLocalTime(bool extraData);

// External variables declared in the main .ino
extern char myName[];
extern char myVer[];
extern char baseVersion[];
extern IPAddress ip;
extern IPAddress net;
extern IPAddress gw;
extern char apName[];
extern bool captivePortal;
extern int udpPort;
extern int streamPort;
extern char default_index[];
extern int8_t streamCount;
extern unsigned long streamsServed;
extern unsigned long imagesServed;
extern int myRotation;
extern int minFrameTime;
extern int lampVal;
extern bool autoLamp;
extern bool filesystem;
extern String critERR;
extern bool debugData;
extern bool haveTime;
extern unsigned long xclk;
extern int sensorPID;

// Flag that can be set to kill all active streams
bool streamKill;

#ifdef __cplusplus
extern "C" {
#endif
  uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

void serialDump() {
  Serial.println();
  // Module
  Serial.printf("Name: %s\r\n", myName);
  if (haveTime) {
    Serial.print("Time: ");
    printLocalTime(true);
  }
  Serial.printf("Firmware: %s (base: %s)\r\n", myVer, baseVersion);
  Serial.printf("ESP sdk: %s\r\n", ESP.getSdkVersion());
  // Network

  Serial.printf("WiFi Mode: Client\r\n");
  String ssidName = WiFi.SSID();
  Serial.printf("WiFi Ssid: %s\r\n", ssidName.c_str());
  Serial.printf("WiFi Rssi: %i\r\n", WiFi.RSSI());
  String bssid = WiFi.BSSIDstr();
  Serial.printf("WiFi BSSID: %s\r\n", bssid.c_str());
  Serial.printf("WiFi IP address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
  Serial.printf("WiFi Udp port: %i, Stream port: %i\r\n", udpPort, streamPort);
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.printf("WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  // System
  int64_t sec = esp_timer_get_time() / 1000000;
  int64_t upDays = int64_t(floor(sec / 86400));
  int upHours = int64_t(floor(sec / 3600)) % 24;
  int upMin = int64_t(floor(sec / 60)) % 60;
  int upSec = sec % 60;
  int McuTc = (temprature_sens_read() - 32) / 1.8;  // celsius
  int McuTf = temprature_sens_read();               // fahrenheit
  Serial.printf("System up: %" PRId64 ":%02i:%02i:%02i (d:h:m:s)\r\n", upDays, upHours, upMin, upSec);
  Serial.printf("Active streams: %i, Previous streams: %lu, Images captured: %lu\r\n", streamCount, streamsServed, imagesServed);
  Serial.printf("CPU Freq: %i MHz, Xclk Freq: %i MHz\r\n", ESP.getCpuFreqMHz(), xclk);
  Serial.printf("MCU temperature : %i C, %i F  (approximate)\r\n", McuTc, McuTf);
  Serial.printf("Heap: %i, free: %i, min free: %i, max block: %i\r\n", ESP.getHeapSize(), ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
  if (psramFound()) {
    Serial.printf("Psram: %i, free: %i, min free: %i, max block: %i\r\n", ESP.getPsramSize(), ESP.getFreePsram(), ESP.getMinFreePsram(), ESP.getMaxAllocPsram());
  } else {
    Serial.printf("Psram: Not found; please check your board configuration.\r\n");
    Serial.printf("- High resolution/quality settings will show incomplete frames to low memory.\r\n");
  }
  // Filesystems
  if (filesystem && (SPIFFS.totalBytes() > 0)) {
    Serial.printf("Spiffs: %i, used: %i\r\n", SPIFFS.totalBytes(), SPIFFS.usedBytes());
  } else {
    Serial.printf("Spiffs: No filesystem found, please check your board configuration.\r\n");
    Serial.printf("- Saving and restoring camera settings will not function without this.\r\n");
  }
  Serial.println("Preferences file: ");
  dumpPrefs(SPIFFS);
  if (critERR.length() > 0) {
    Serial.printf("\r\n\r\nAn error or halt has occurred with Camera Hardware, see previous messages.\r\n");
    Serial.printf("A reboot is required to recover from this.\r\nError message: (html)\r\n %s\r\n\r\n", critERR.c_str());
  }
  Serial.println();
  return;
}

static esp_err_t capture_handler() {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;

  Serial.println("Capture Requested");
  if (autoLamp && (lampVal != -1)) {
    setLamp(lampVal);
    delay(75);  // coupled with the status led flash this gives ~150ms for lamp to settle.
  }
  flashLED(75);  // little flash of status LED

  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("CAPTURE: failed to acquire frame");
    if (autoLamp && (lampVal != -1)) setLamp(0);
    return ESP_FAIL;
  }

  size_t fb_len = 0;
  if (fb->format == PIXFORMAT_JPEG) {
    fb_len = fb->len;
    //res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    res = ESP_FAIL;
    Serial.println("Capture Error: Non-JPEG image returned by camera module");
  }
  esp_camera_fb_return(fb);
  fb = NULL;

  int64_t fr_end = esp_timer_get_time();
  if (debugData) {
    Serial.printf("JPG: %uB %ums\r\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
  }
  imagesServed++;
  if (autoLamp && (lampVal != -1)) {
    setLamp(0);
  }
  return res;
}

static esp_err_t stream_handler() {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  streamKill = false;

  Serial.println("Stream requested");
  if (autoLamp && (lampVal != -1)) setLamp(lampVal);
  streamCount = 1;  // at present we only have one stream handler, so values are 0 or 1..
  flashLED(75);     // double flash of status LED
  delay(75);
  flashLED(75);

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }


  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("STREAM: failed to acquire frame");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        Serial.println("STREAM: Non-JPEG frame returned by camera module");
        res = ESP_FAIL;
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, "", _jpg_buf_len);
      //res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      //res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      // This is the error exit point from the stream loop.
      // We end the stream here only if a Hard failure has been encountered or the connection has been interrupted.
      Serial.printf("Stream failed, code = %i : %s\r\n", res, esp_err_to_name(res));
      break;
    }
    if ((res != ESP_OK) || streamKill) {
      // We end the stream here when a kill is signalled.
      Serial.printf("Stream killed\r\n");
      break;
    }
    int64_t frame_time = esp_timer_get_time() - last_frame;
    frame_time /= 1000;
    int32_t frame_delay = (minFrameTime > frame_time) ? minFrameTime - frame_time : 0;
    delay(frame_delay);

    if (debugData) {
      Serial.printf("MJPG: %uB %ums, delay: %ums, framerate (%.1ffps)\r\n",
                    (uint32_t)(_jpg_buf_len),
                    (uint32_t)frame_time, frame_delay, 1000.0 / (uint32_t)(frame_time + frame_delay));
    }
    last_frame = esp_timer_get_time();
  }

  streamsServed++;
  streamCount = 0;
  if (autoLamp && (lampVal != -1)) setLamp(0);
  Serial.println("Stream ended");
  last_frame = 0;
  return res;
}

static esp_err_t stop_handler() {
  flashLED(75);
  Serial.println("\r\nStream stop requested via Web");
  streamKill = true;
  //httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return ESP_OK; //httpd_resp_send(req, NULL, 0);
}

static esp_err_t error_handler() {
  flashLED(75);
  Serial.println("Sending error page");
  std::string s("");
  size_t index;
  return ESP_OK; //httpd_resp_send(req, (const char *)s.c_str(), s.length());
}

// this is not called by the main program
// The UDP server controls the creation of the stream
void startCameraStream(IPAddress remoteIp,int remoteTcpPort) {
  Serial.printf("Starting stream to: '%d'.'%d'.'%d'.'%d' on port: '%d'\r\n",remoteIp[0],remoteIp[1],remoteIp[2],remoteIp[3],remoteTcpPort);
}

void startUdpServer(int localUdpPort, int remoteTcpPort) {
  Serial.printf("Starting udp server on port: '%d'\r\n", localUdpPort);
    
}

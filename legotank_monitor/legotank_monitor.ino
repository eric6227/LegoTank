// Based on Espressif Systems ESP32 CameraWebServer example.
//
// Copyright (c) 2026 eric6227
// Released under the MIT License. See LICENSE file in the project root for full text.
//
// 需要修改 line 162 的 IP 地址

#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "8-2-102";
const char *password = "go192837";

// ===========================
// MQTT 配置
// ===========================
const char *mqttHost = "192.168.2.45";
const int   mqttPort = 1883;
const char *mqttClientId = "tank-cam-001";
const char *telemetryTopic = "tank/telemetry/monitor";
const char *cmdTopic = "tank/cmd/camera";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void startCameraServer();
void setupLedFlash();
void ensureWiFi();
void reconnectMQTT();
void publishTelemetry();
void mqttCallback(char *topic, byte *payload, unsigned int length);
framesize_t stringToFrameSize(const char *str);
void applyCameraConfig(const char *resolution, int quality, int fps);

unsigned long lastWiFiCheckMs = 0;
unsigned long lastTelemetryMs = 0;

// RSSI 持续过低重连配置（可通过 MQTT tank/cmd/camera 下发修改）
int rssiReconnectThreshold = -70;      // 默认 -70dBm
unsigned long rssiReconnectDelayMs = 10000; // 默认持续 10 秒才触发重连
unsigned long rssiBelowThresholdSince = 0;  // RSSI 首次低于阈值的时间戳

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_VGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // ========== 分辨率对照表 ==========
  // FRAMESIZE_96X96   = 96x96
  // FRAMESIZE_QQVGA   = 160x120
  // FRAMESIZE_QCIF    = 176x144
  // FRAMESIZE_HQVGA   = 240x176
  // FRAMESIZE_240X240 = 240x240
  // FRAMESIZE_QVGA    = 320x240
  // FRAMESIZE_CIF     = 400x296
  // FRAMESIZE_HVGA    = 480x320
  // FRAMESIZE_VGA     = 640x480
  // FRAMESIZE_SVGA    = 800x600
  // FRAMESIZE_XGA     = 1024x768
  // FRAMESIZE_HD      = 1280x720
  // FRAMESIZE_SXGA    = 1280x1024
  // FRAMESIZE_UXGA    = 1600x1200
  // ==================================
  if (config.pixel_format == PIXFORMAT_JPEG) {
    // 默认分辨率，后续可通过 MQTT 实时调整
    s->set_framesize(s, FRAMESIZE_HD);
    s->set_quality(s, 10);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif
  //{AI生成
  IPAddress local_IP(192, 168, 2, 188);  // 改成你想固定的 IP，末尾避开 1-100 一般不会被占用 改成188到255之间（包括188和255）
  IPAddress gateway(192, 168, 2, 1);     // 你的网关
  IPAddress subnet(255, 255, 255, 0);    // 你的掩码

  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
  //AI生成}
  Serial.print("WiFi connecting");
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    delay(100);
    Serial.print(".");
    attempt++;
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // MQTT
  mqttClient.setServer(mqttHost, mqttPort);
  mqttClient.setCallback(mqttCallback);
  reconnectMQTT();
}

void ensureWiFi() {
  bool needReconnect = false;

  if (WiFi.status() != WL_CONNECTED) {
    needReconnect = true;
    rssiBelowThresholdSince = 0;  // 重置计时
  } else {
    int rssi = WiFi.RSSI();
    if (rssi < rssiReconnectThreshold) {
      if (rssiBelowThresholdSince == 0) {
        rssiBelowThresholdSince = millis();
        Serial.printf("[WiFi] 信号强度 %d dBm 低于阈值 %d dBm，开始计时 %lu ms\n",
                      rssi, rssiReconnectThreshold, rssiReconnectDelayMs);
      } else if (millis() - rssiBelowThresholdSince >= rssiReconnectDelayMs) {
        needReconnect = true;
        Serial.printf("[WiFi] 信号强度持续低于阈值 %lu ms，触发重连\n",
                      millis() - rssiBelowThresholdSince);
      }
    } else {
      rssiBelowThresholdSince = 0;  // 信号恢复，重置计时
    }
  }

  if (!needReconnect) {
    return;
  }

  rssiBelowThresholdSince = 0;
  Serial.println("[WiFi] 连接断开或信号过弱，正在重连...");
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 15) {
    delay(100);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.print("[WiFi] 已恢复，IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("[WiFi] 重连失败，下次继续尝试");
  }
}

void reconnectMQTT() {
  if (mqttClient.connected()) {
    return;
  }
  Serial.print("[MQTT] 尝试连接服务器...");
  bool ok = mqttClient.connect(mqttClientId);
  if (ok) {
    Serial.println("已连接");
    mqttClient.subscribe(cmdTopic);
    Serial.print("[MQTT] 已订阅: ");
    Serial.println(cmdTopic);
  } else {
    Serial.print("失败，状态码=");
    Serial.println(mqttClient.state());
  }
}

void publishTelemetry() {
  if (!mqttClient.connected()) {
    return;
  }
  if (millis() - lastTelemetryMs < 500) {
    return;
  }
  lastTelemetryMs = millis();

  JsonDocument doc;
  doc["monrssi"] = WiFi.RSSI();
  doc["uptime"]  = millis();

  char buf[256];
  size_t n = serializeJson(doc, buf);
  bool ok = mqttClient.publish(telemetryTopic, buf, n);
  if (ok) {
    Serial.print("[Telemetry] ");
    Serial.println(buf);
  } else {
    Serial.println("[Telemetry] 发送失败");
  }
}

framesize_t stringToFrameSize(const char *str) {
  if (strcmp(str, "QQVGA") == 0) return FRAMESIZE_QQVGA;
  if (strcmp(str, "QVGA") == 0) return FRAMESIZE_QVGA;
  if (strcmp(str, "VGA") == 0) return FRAMESIZE_VGA;
  if (strcmp(str, "SVGA") == 0) return FRAMESIZE_SVGA;
  if (strcmp(str, "XGA") == 0) return FRAMESIZE_XGA;
  if (strcmp(str, "HD") == 0) return FRAMESIZE_HD;
  if (strcmp(str, "SXGA") == 0) return FRAMESIZE_SXGA;
  if (strcmp(str, "UXGA") == 0) return FRAMESIZE_UXGA;
  return FRAMESIZE_HD;  // 默认回退
}

void applyCameraConfig(const char *resolution, int quality, int fps) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("[Camera] 获取传感器失败，无法应用配置");
    return;
  }
  framesize_t fs = stringToFrameSize(resolution);
  s->set_framesize(s, fs);
  if (quality >= 0 && quality <= 63) {
    s->set_quality(s, quality);
  }
  // 注意：当前 esp32-camera 版本没有 set_frame_duration，帧率通过分辨率/质量间接控制
  (void)fps;
  Serial.printf("[Camera] 已应用配置: resolution=%s, quality=%d (目标fps=%d，实际由分辨率决定)\n", resolution, quality, fps);
}

void mqttCallback(char *topic, byte *payload, unsigned int length) {
  (void)topic;
  char buf[256];
  if (length >= sizeof(buf)) length = sizeof(buf) - 1;
  memcpy(buf, payload, length);
  buf[length] = '\0';

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, buf);
  if (err) {
    Serial.print("[MQTT] 命令解析失败: ");
    Serial.println(err.c_str());
    return;
  }

  const char *resolution = doc["resolution"] | "";
  int quality = doc["quality"] | -1;
  int fps = doc["fps"] | -1;
  int rssiThresh = doc["rssi_threshold"] | -999;  // -999 表示未设置
  int rssiDelay = doc["rssi_delay"] | -1;

  if (rssiThresh != -999) {
    rssiReconnectThreshold = rssiThresh;
    Serial.printf("[WiFi] RSSI 重连阈值已更新为 %d dBm\n", rssiReconnectThreshold);
  }
  if (rssiDelay >= 0) {
    rssiReconnectDelayMs = (unsigned long)rssiDelay;
    rssiBelowThresholdSince = 0;  // 重置计时，避免旧计时触发误重连
    Serial.printf("[WiFi] RSSI 重连延迟已更新为 %lu ms\n", rssiReconnectDelayMs);
  }

  if (strlen(resolution) > 0 || quality >= 0 || fps >= 0) {
    if (strlen(resolution) == 0) resolution = "HD";
    if (quality < 0) quality = 10;
    if (fps < 0) fps = 30;
    applyCameraConfig(resolution, quality, fps);
  }
}

void loop() {
  unsigned long now = millis();
  if (now - lastWiFiCheckMs > 500) {
    ensureWiFi();
    lastWiFiCheckMs = now;
  }

  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  publishTelemetry();

  delay(50);
}
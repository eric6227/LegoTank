/*
 * Copyright (c) 2026 eric6227
 * Released under the MIT License. See LICENSE file in the project root for full text.
 *
 * 玩具坦克 ESP32 控制板主程序
 *
 * 功能：
 *   1. 通过 MQTT 接收上位机发来的原始控制量
 *   2. 直接驱动左/右电机和三个舵机 M1/M2/M3
 *   3. 定时回传电池电压、当前输出、WiFi 信号等遥测数据
 *   4. 指令超时自动停车（failsafe）
 *
 * 依赖库（请通过 Arduino IDE 库管理器安装）：
 *   - PubSubClient（by Nick O'Leary）
 *   - ArduinoJson（by Benoit Blanchon）
 *
 * 硬件：ESP32 + TB6612FNG 双路电机驱动 + 3 路舵机 + 电池分压检测
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

// ============================================================
// 兼容 ESP32 Arduino Core 2.x 与 3.x 的 LEDC API 差异
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define LEDC_NEW_API 1
#else
  #define LEDC_NEW_API 0
#endif

// 电机 / 舵机 PWM 引脚
const uint8_t MOTOR_PINS[2] = { LEFT_PWM, RIGHT_PWM };
const uint8_t SERVO_PINS[3] = { SERVO1_PIN, SERVO2_PIN, SERVO3_PIN };

#if !LEDC_NEW_API
// Core 2.x 需要手动分配通道号
const uint8_t MOTOR_CH[2] = { 0, 1 };
const uint8_t SERVO_CH[3] = { 2, 3, 4 };
const uint8_t SERVO23_CH[2] = { 5, 6 };  // M2/M3 直接PWM通道
#endif

// ============================================================
// 全局对象与状态
// ============================================================
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// 当前控制量
int16_t motorL = 0;   // 左电机：-255 ~ 255
int16_t motorR = 0;   // 右电机：-255 ~ 255
int16_t servoM1 = 90; // M1 舵机角度：0 ~ 180
uint8_t servoM2 = 0;   // M2 电平：0 ~ 255（直接 PWM）
uint8_t servoM3 = 0;   // M3 电平：0 ~ 255（直接 PWM）

uint32_t lastCmdMs    = 0;  // 上次收到指令的时间
uint32_t telemetryMs  = 0;  // 上次遥测发送时间
bool     wifiWasConnected = false; // 用于检测 WiFi 断线重连

// RSSI 持续过低重连配置（可通过 MQTT tank/cmd 下发修改）
int rssiReconnectThreshold = -75;         // 默认 -75dBm
unsigned long rssiReconnectDelayMs = 10000; // 默认持续 10 秒才触发重连
unsigned long rssiBelowThresholdSince = 0;  // RSSI 首次低于阈值的时间戳

// 电机换向保护状态
int16_t  appliedMotorL = 0;          // 最近一次实际输出到驱动的左电机值
int16_t  appliedMotorR = 0;          // 最近一次实际输出到驱动的右电机值
uint32_t directionChangeHoldEnd = 0; // 换向保持结束时间
bool     directionChangeHolding = false;

// ============================================================
// LEDC 辅助函数
// ============================================================
bool pwmAttach(uint8_t pin, uint32_t freq, uint8_t resolution, uint8_t channel) {
#if LEDC_NEW_API
  (void)channel;
  bool ok = ledcAttach(pin, freq, resolution);
  if (!ok) {
    Serial.print("[PWM] 引脚 ");
    Serial.print(pin);
    Serial.println(" 初始化失败");
  }
  return ok;
#else
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
  return true;
#endif
}

void pwmWrite(uint8_t pin, uint8_t channel, uint32_t duty) {
#if LEDC_NEW_API
  (void)channel;
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

// ============================================================
// 电机控制
// ============================================================
void setMotor(uint8_t side, int speed) {
  if (side > 1) return;

  speed = constrain(speed, -255, 255);

  uint8_t in1, in2;
  if (side == 0) {
    in1 = LEFT_AIN1;
    in2 = LEFT_AIN2;
  } else {
    in1 = RIGHT_BIN1;
    in2 = RIGHT_BIN2;
  }

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // 刹车/停止：两片输入都拉低
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  uint32_t duty = abs(speed);  // 8 位分辨率，0-255
#if !LEDC_NEW_API
  uint8_t ch = MOTOR_CH[side];
#else
  uint8_t ch = 0;
#endif
  pwmWrite(MOTOR_PINS[side], ch, duty);
}

// ============================================================
// 舵机控制（角度 -> 脉宽 -> 16 位占空比）
// ============================================================
void setServo(uint8_t idx, int angle) {
  if (idx > 2) return;

  angle = constrain(angle, 0, 180);

  // 把角度映射为脉宽（微秒）
  int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);

  // 50Hz 周期 = 20000us，16 位最大值为 65535
  uint32_t maxDuty = (1UL << SERVO_RES) - 1;
  uint32_t duty = (uint32_t)((pulseUs / 20000.0) * maxDuty);
  duty = constrain(duty, 0, maxDuty);

#if !LEDC_NEW_API
  uint8_t ch = SERVO_CH[idx];
#else
  uint8_t ch = 0;
#endif
  pwmWrite(SERVO_PINS[idx], ch, duty);

  Serial.print("[Servo] M");
  Serial.print(idx + 1);
  Serial.print(" angle=");
  Serial.print(angle);
  Serial.print(" pulse=");
  Serial.print(pulseUs);
  Serial.print("us duty=");
  Serial.println(duty);
}

// M2/M3 直接 PWM 输出（电平值 0-255）
void setServoLevel(uint8_t idx, uint8_t level) {
  // idx: 0=M2, 1=M3
  uint32_t maxDuty = (1UL << SERVO23_RES) - 1;
  uint32_t duty = (uint32_t)level * maxDuty / 255;
  duty = constrain(duty, 0, maxDuty);

  uint8_t pin = (idx == 0) ? SERVO2_PIN : SERVO3_PIN;
#if !LEDC_NEW_API
  uint8_t ch = SERVO23_CH[idx];
#else
  uint8_t ch = 0;
#endif
  pwmWrite(pin, ch, duty);
}

// ============================================================
// 电池电压检测
// ============================================================
float readBatteryVoltage() {
  const int samples = 20;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(BAT_PIN);
    delayMicroseconds(100);
  }
  float adcAvg = sum / (float)samples;
  float vAdc = adcAvg * ADC_REF / ADC_MAX;
  float divider = BAT_R2 / (BAT_R1 + BAT_R2);
  return vAdc / divider;
}

// 保留两位小数（用于遥测 JSON）
float round2(float value) {
  return round(value * 100.0) / 100.0;
}

// ============================================================
// 输出同步
// ============================================================
// 电机输出，带换向保护：检测到方向切换时先强制输出 0 一段时间
void applyMotorOutputs() {
  int signL = (motorL > 0) ? 1 : (motorL < 0) ? -1 : 0;
  int signR = (motorR > 0) ? 1 : (motorR < 0) ? -1 : 0;
  int appliedSignL = (appliedMotorL > 0) ? 1 : (appliedMotorL < 0) ? -1 : 0;
  int appliedSignR = (appliedMotorR > 0) ? 1 : (appliedMotorR < 0) ? -1 : 0;

  bool dirChanged =
    (signL != 0 && appliedSignL != 0 && signL != appliedSignL) ||
    (signR != 0 && appliedSignR != 0 && signR != appliedSignR);

  if (dirChanged && !directionChangeHolding) {
    directionChangeHolding = true;
    directionChangeHoldEnd = millis() + DIRECTION_CHANGE_HOLD_MS;
    Serial.println("[MotorProtect] 检测到电机换向，强制停转保护");
  }

  if (directionChangeHolding) {
    setMotor(0, 0);
    setMotor(1, 0);
    appliedMotorL = 0;
    appliedMotorR = 0;
    if ((int32_t)(millis() - directionChangeHoldEnd) >= 0) {
      directionChangeHolding = false;
    }
    return;
  }

  setMotor(0, motorL);
  setMotor(1, motorR);
  appliedMotorL = motorL;
  appliedMotorR = motorR;
}

// 输出同步
void applyOutputs() {
  applyMotorOutputs();
  setServo(0, servoM1);
  setServoLevel(0, servoM2);
  setServoLevel(1, servoM3);
}

void stopAllMotors() {
  motorL = 0;
  motorR = 0;
  setMotor(0, 0);
  setMotor(1, 0);
  appliedMotorL = 0;
  appliedMotorR = 0;
  directionChangeHolding = false;
}

// ============================================================
// MQTT 指令解析
// ============================================================
void parseCommand(const char* json) {
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, json);

  if (error) {
    Serial.print("[MQTT] JSON 解析失败: ");
    Serial.println(error.c_str());
    return;
  }

  // 所有字段均为可选；缺失字段保持上一次值
  if (doc.containsKey("L"))  motorL  = (int16_t)doc["L"];
  if (doc.containsKey("R"))  motorR  = (int16_t)doc["R"];
  if (doc.containsKey("M1")) servoM1 = (int16_t)doc["M1"];
  if (doc.containsKey("M2")) servoM2 = (uint8_t)doc["M2"];
  if (doc.containsKey("M3")) servoM3 = (uint8_t)doc["M3"];

  // RSSI 重连参数（可通过上位机实时调整）
  if (doc.containsKey("rssi_threshold")) {
    rssiReconnectThreshold = (int)doc["rssi_threshold"];
    rssiBelowThresholdSince = 0;
    Serial.printf("[WiFi] RSSI 重连阈值已更新为 %d dBm\n", rssiReconnectThreshold);
  }
  if (doc.containsKey("rssi_delay")) {
    rssiReconnectDelayMs = (unsigned long)doc["rssi_delay"];
    rssiBelowThresholdSince = 0;
    Serial.printf("[WiFi] RSSI 重连延迟已更新为 %lu ms\n", rssiReconnectDelayMs);
  }

  applyOutputs();
  lastCmdMs = millis();

  Serial.println("[MQTT] 已执行控制指令");
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_CMD) != 0) return;

  char buf[MQTT_BUFFER_SIZE];
  if (length >= sizeof(buf)) length = sizeof(buf) - 1;
  memcpy(buf, payload, length);
  buf[length] = '\0';

  Serial.print("[MQTT] 收到指令: ");
  Serial.println(buf);

  parseCommand(buf);
}

// ============================================================
// 网络与 MQTT 连接
// ============================================================
void setupWiFi() {
  Serial.print("[WiFi] 连接 ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    delay(100);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[WiFi] 已连接，IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("[WiFi] 连接失败，稍后重试");
  }
}

bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    int32_t rssi = WiFi.RSSI();
    if (rssi < rssiReconnectThreshold) {
      if (rssiBelowThresholdSince == 0) {
        rssiBelowThresholdSince = millis();
        Serial.printf("[WiFi] 控制板信号强度 %d dBm 低于阈值 %d dBm，开始计时 %lu ms\n",
                      rssi, rssiReconnectThreshold, rssiReconnectDelayMs);
      } else if (millis() - rssiBelowThresholdSince >= rssiReconnectDelayMs) {
        Serial.printf("[WiFi] 信号强度持续低于阈值 %lu ms，触发重连\n",
                      millis() - rssiBelowThresholdSince);
        rssiBelowThresholdSince = 0;
        WiFi.disconnect();
        delay(100);
        setupWiFi();
        wifiWasConnected = false;
        return (WiFi.status() == WL_CONNECTED);
      }
    } else {
      rssiBelowThresholdSince = 0;  // 信号恢复，重置计时
    }

    if (!wifiWasConnected) {
      Serial.println("[WiFi] 连接已恢复");
      mqttClient.disconnect();
    }
    wifiWasConnected = true;
    return true;
  }

  rssiBelowThresholdSince = 0;
  if (wifiWasConnected) {
    Serial.println("[WiFi] 连接断开，尝试重连...");
    wifiWasConnected = false;
  }
  WiFi.disconnect();
  setupWiFi();
  return (WiFi.status() == WL_CONNECTED);
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("[MQTT] 尝试连接服务器...");
    bool ok;

    if (strlen(MQTT_USER) > 0) {
      ok = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS);
    } else {
      ok = mqttClient.connect(MQTT_CLIENT_ID);
    }

    if (ok) {
      Serial.println("已连接");
      mqttClient.subscribe(TOPIC_CMD);
      Serial.print("[MQTT] 已订阅: ");
      Serial.println(TOPIC_CMD);
    } else {
      Serial.print("失败，状态码=");
      Serial.print(mqttClient.state());
      Serial.println("，0.3秒后重试");
      delay(300);
    }
  }
}

// ============================================================
// 遥测上报
// ============================================================
void publishTelemetry() {
  JsonDocument doc;

  doc["vbat"]   = round2(readBatteryVoltage());
  doc["L"]      = appliedMotorL;  // 上报实际输出到驱动芯片的值
  doc["R"]      = appliedMotorR;
  doc["M1"]     = servoM1;
  doc["M2"]     = servoM2;
  doc["M3"]     = servoM3;
  doc["ctrlrssi"]   = WiFi.RSSI();
  doc["uptime"] = millis();

  char buf[384];
  size_t n = serializeJson(doc, buf);

  bool ok = mqttClient.publish(TOPIC_TELEMETRY, buf, n);

  if (ok) {
    Serial.print("[Telemetry] ");
    Serial.println(buf);
  } else {
    Serial.println("[Telemetry] 发送失败，数据过大或网络断开");
  }
}

// ============================================================
// 初始化
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[Init] 玩具坦克 ESP32 控制板启动");

  // 配置 GPIO
  pinMode(LEFT_AIN1, OUTPUT);
  pinMode(LEFT_AIN2, OUTPUT);
  pinMode(RIGHT_BIN1, OUTPUT);
  pinMode(RIGHT_BIN2, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  // 使能电机驱动板
  digitalWrite(STBY_PIN, HIGH);

  // 电机 PWM
  bool pwmOk = true;
  pwmOk &= pwmAttach(LEFT_PWM,  MOTOR_FREQ, MOTOR_RES, 0);
  pwmOk &= pwmAttach(RIGHT_PWM, MOTOR_FREQ, MOTOR_RES, 1);

  // 舵机 PWM
  pwmOk &= pwmAttach(SERVO1_PIN, SERVO_FREQ, SERVO_RES, 2);
  pwmOk &= pwmAttach(SERVO2_PIN, SERVO23_FREQ, SERVO23_RES, 5);
  pwmOk &= pwmAttach(SERVO3_PIN, SERVO23_FREQ, SERVO23_RES, 6);

  if (!pwmOk) {
    Serial.println("[Init] 部分 PWM 初始化失败，请检查引脚或 LEDC 通道");
  } else {
    Serial.println("[Init] PWM 初始化完成");
  }

  // 电池 ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // 初始状态：停车、舵机居中
  applyOutputs();

  // 舵机自检：M1 摆动，M2/M3 电平闪烁，便于确认硬件接线正常
  Serial.println("[Init] 舵机自检开始");
  setServo(0, 60);
  delay(150);
  setServo(0, 120);
  delay(150);
  setServo(0, 90);
  delay(100);
  setServoLevel(0, 128);
  setServoLevel(1, 128);
  delay(300);
  setServoLevel(0, 0);
  setServoLevel(1, 0);
  Serial.println("[Init] 舵机自检结束");

  // 网络
  setupWiFi();

  // MQTT
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);

  lastCmdMs   = millis();
  telemetryMs = millis();
}

// ============================================================
// 主循环
// ============================================================
void loop() {
  bool wifiOk = ensureWiFi();

  if (wifiOk) {
    if (!mqttClient.connected()) {
      reconnectMQTT();
    }
    mqttClient.loop();
  }

  // 看门狗：超时未收到指令则停车，伺服电机保持原位
  if (millis() - lastCmdMs > WATCHDOG_MS) {
    stopAllMotors();
    setServo(0, servoM1);
    setServoLevel(0, servoM2);
    setServoLevel(1, servoM3);
    // 只打印一次，避免刷屏
    static uint32_t lastWarn = 0;
    if (millis() - lastWarn > 1000) {
      Serial.println("[Watchdog] 指令超时，已自动停车，伺服保持原位");
      lastWarn = millis();
    }
  }

  // 定时遥测
  if (millis() - telemetryMs >= TELEMETRY_MS) {
    publishTelemetry();
    telemetryMs = millis();
  }

  delay(1);  // 喂 ESP32 看门狗
}
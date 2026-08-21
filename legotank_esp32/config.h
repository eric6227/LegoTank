/*
 * Copyright (c) 2026 eric6227
 * Released under the MIT License. See LICENSE file in the project root for full text.
 *
 * 玩具坦克 ESP32 控制板配置文件
 * 根据你给出的引脚定义、PWM 参数和电池检测参数编写
 */

#ifndef TANK_CONFIG_H
#define TANK_CONFIG_H

// ========== 引脚定义 ==========
// 左电机 (TB6612FNG 或类似驱动)
#define LEFT_AIN1   4
#define LEFT_AIN2   5
#define LEFT_PWM    18

// 右电机
#define RIGHT_BIN1  19
#define RIGHT_BIN2  21
#define RIGHT_PWM   22

// STBY (驱动板使能)
#define STBY_PIN    23

// 舵机
#define SERVO1_PIN  25   // M1：换挡
#define SERVO2_PIN  26   // M2：大灯
#define SERVO3_PIN  27   // M3：晃动逗猫棒

// 电池检测 (GPIO34，ADC1_CH6)
#define BAT_PIN     34

// ========== PWM 参数 ==========
#define MOTOR_FREQ  20000   // 电机 PWM 频率 20kHz
#define MOTOR_RES   8       // 8 位分辨率 (0-255)

#define SERVO_FREQ  50      // 舵机 PWM 频率 50Hz
#define SERVO_RES   16      // 16 位分辨率 (0-65535)

#define SERVO23_FREQ  1200   // M2/M3 直接 PWM 频率 (4×300Hz，无频闪)
#define SERVO23_RES   8      // 8 位分辨率 (0-255)

// 舵机角度对应的脉宽（微秒），可按实际舵机微调
#define SERVO_MIN_US  1000   // 0°
#define SERVO_MAX_US  2000  // 180°

// ========== 电池检测参数 ==========
// 分压电阻：BAT+ ---[100k]--- ADC ---[30k]--- GND
// ADC 引脚电压 = Vbat * R2 / (R1 + R2)
#define BAT_R1      95100.0    // 实际值，电阻有误差
#define BAT_R2      26500.0      // 实际值，电阻有误差
#define ADC_REF     3.3
#define ADC_MAX     4095.0   // 12 位 ADC

// 3S LiPo 参考值（用于可选的电量百分比估算）
#define BAT_3S_MAX  12.6
#define BAT_3S_MIN  9.9

// ========== 网络 / MQTT 配置 ==========
// 请修改成你自己的 WiFi 和 MQTT 服务器信息
#define WIFI_SSID   "8-2-102"
#define WIFI_PASS   "go192837"

#define MQTT_HOST   "192.168.2.45"
#define MQTT_PORT   1883
#define MQTT_USER   ""        // 无认证时留空
#define MQTT_PASS   ""

// 主题定义
#define TOPIC_CMD        "tank/cmd"              // 上位机 -> ESP32 控制指令
#define TOPIC_TELEMETRY  "tank/telemetry/control" // ESP32 -> 上位机遥测数据

// MQTT 客户端 ID（建议每台设备唯一）
#define MQTT_CLIENT_ID   "tank-esp32-001"

// ========== 运行时参数 ==========
#define TELEMETRY_MS     50       // 遥测上报间隔（毫秒）
#define WATCHDOG_MS      500      // 指令超时保护（毫秒），超时自动停车
#define DIRECTION_CHANGE_HOLD_MS 50  // 电机换向时强制停转保持时间（毫秒）
#define MQTT_BUFFER_SIZE 1024     // MQTT 收发缓冲区

#endif // TANK_CONFIG_H
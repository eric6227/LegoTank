# Copyright (c) 2026 eric6227
# Released under the MIT License. See LICENSE file in the project root for full text.
"""
无限遥控车控制器

架构：
    - 内置本地 HTTP 服务器，提供一个带 HUD 叠加的网页
    - 网页直接加载 ESP32-CAM 视频流，并用 JS 轮询 /telemetry 显示遥测
    - 上位机主窗口使用 QWebEngineView 显示该网页
    - OBS 浏览器源也能拉取同一个网页，画面完全一致

依赖：
    pip install PyQt6 PyQt6-WebEngine paho-mqtt opencv-python numpy pyyaml keyring sounddevice

用法：
    python infinite_rc_controller.py
"""

import json
import os
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional, Union

import cv2
import keyring
import numpy as np
import paho.mqtt.client as mqtt
import requests
import sounddevice as sd
import yaml

from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal, QUrl, QEvent
from PyQt6.QtGui import QColor, QFont, QKeySequence
from PyQt6.QtWebEngineWidgets import QWebEngineView
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QKeySequenceEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

# 这是控制软件的默认预设配置文件：默认与控制软件入口同目录。
# 运行时优先从这里读取 yaml，并在缺失时落回 DEFAULT_CONFIG。
CONFIG_FILE = os.path.join(os.path.dirname(__file__), "config.yaml")
APP_NAME = "无限遥控车控制器"
KEYRING_SERVICE = "infinite_rc_controller"
KEYRING_USERNAME = "mqtt_password"

# 摄像头断流时用于保持 OBS 流不卡住的占位黑帧
_, _no_signal_buf = cv2.imencode(".jpg", np.zeros((480, 640, 3), dtype=np.uint8))
NO_SIGNAL_JPEG: bytes = _no_signal_buf.tobytes()

DEFAULT_CONTROL_TELEMETRY_TOPIC = "tank/telemetry/control"
DEFAULT_MONITOR_TELEMETRY_TOPIC = "tank/telemetry/monitor"
LEGACY_CONTROL_TELEMETRY_TOPIC = "tank/telemetry"

DEFAULT_CONFIG: Dict[str, Any] = {
    "mqtt_host": "192.168.2.45",
    "mqtt_port": 1883,
    "mqtt_topic": DEFAULT_CONTROL_TELEMETRY_TOPIC,
    "mqtt_monitor_topic": DEFAULT_MONITOR_TELEMETRY_TOPIC,
    "mqtt_username": "",
    "stream_url": "http://192.168.2.188:81/stream",
    "cam_resolution": "1280x720 (HD)",
    "cam_quality": 15,
    "cam_fps": 60,
    "audio_port": 5004,
    "stream_output_port": 8080,
    "volume": 71,
    "audio_enabled": True,
    # 控制
    "control_key_forward": 16777235,
    "control_key_backward": 16777237,
    "control_key_left": 16777234,
    "control_key_right": 16777236,
    "control_key_boost": 66,
    "control_key_shift_up": 81,
    "control_key_shift_down": 65,
    "control_key_mode_toggle": 77,
    "control_max_output_pct": 70,
    "control_boost_output_pct": 95,
    "control_boost_max_ms": 15000,
    "control_accel_rate_pct": 8,
    "control_brake_strength_pct": 50,
    "control_steering_factor": 0.4,
    "control_steering_factor_high": 0.4,
    "control_steering_factor_low": 0.4,
    "control_steering_factor_reverse": 0.4,
    "control_steering_factor_inplace": 0.4,
    "control_reverse_steering_invert": True,
    "control_wifi_cam_rssi_reconnect_threshold_dbm": -70.0,
    "control_wifi_cam_rssi_reconnect_delay_ms": 2000,
    "control_wifi_ctrl_rssi_reconnect_threshold_dbm": -75.0,
    "control_wifi_ctrl_rssi_reconnect_delay_ms": 2000,
    "control_upshift_threshold": 0.6,
    "control_downshift_threshold": 0.25,
    "control_shift_blip_pct": 5,
    "control_low_ratio": -0.8,
    "control_high_ratio": 1.846,
    "control_m1_low": 0,
    "control_m1_high": 180,
    "control_shift_time_s": 0.02,
    "control_direction_change_hold_ms": 50,
    "control_m1_speed_deg_per_tick": 3,
    # 伺服电机控制（默认2个，M1为换挡舵机保留）
    # 使用电平值(0-255)，直接输出到M2/M3引脚的PWM
    # 按下某段按键：当前为该段→回到不触发，否则→切换到该段
    "servo_count": 2,
    "servo_1_name": "逗猫棒",
    "servo_1_seg_count": 1,
    "servo_1_idle_level": 0,
    "servo_1_seg1_key": 0,
    "servo_1_seg1_level": 128,
    "servo_1_seg1_time_s": 0.5,
    "servo_1_seg1_mode": "hold",
    "servo_1_seg2_key": 0,
    "servo_1_seg2_level": 255,
    "servo_1_seg2_time_s": 0.5,
    "servo_1_seg2_mode": "toggle",
    "servo_2_name": "大灯",
    "servo_2_seg_count": 2,
    "servo_2_idle_level": 0,
    "servo_2_seg1_key": 68,
    "servo_2_seg1_level": 128,
    "servo_2_seg1_time_s": 0.5,
    "servo_2_seg1_mode": "toggle",
    "servo_2_seg2_key": 83,
    "servo_2_seg2_level": 255,
    "servo_2_seg2_time_s": 0.5,
    "servo_2_seg2_mode": "hold",
    "servo_3_name": "伺服3",
    "servo_3_seg_count": 2,
    "servo_3_idle_level": 0,
    "servo_3_seg1_key": 0,
    "servo_3_seg1_level": 0,
    "servo_3_seg1_time_s": 0.5,
    "servo_3_seg1_mode": "toggle",
    "servo_3_seg2_key": 0,
    "servo_3_seg2_level": 0,
    "servo_3_seg2_time_s": 0.5,
    "servo_3_seg2_mode": "toggle",
    "video_flip_h": False,
    "video_flip_v": False,
    "video_rotation": 0,
    "auto_quality_enabled": False,
    "auto_quality_low_rssi": -80,
    "auto_quality_recover_rssi": -70,
    "auto_quality_low_resolution": "320x240 (QVGA)",
    "auto_quality_low_quality": 30,
    "auto_quality_low_fps": 10,
}


def load_config() -> Dict[str, Any]:
    cfg = dict(DEFAULT_CONFIG)
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                loaded = yaml.safe_load(f)
                if isinstance(loaded, dict):
                    cfg.update(loaded)
        except Exception:
            pass
    try:
        password = keyring.get_password(KEYRING_SERVICE, KEYRING_USERNAME)
    except Exception:
        password = None
    cfg["mqtt_password"] = password if password else ""
    return cfg


def save_config_silent(cfg: Dict[str, Any]) -> None:
    try:
        cfg_copy = dict(cfg)
        password = cfg_copy.pop("mqtt_password", "")
        if password:
            keyring.set_password(KEYRING_SERVICE, KEYRING_USERNAME, password)
        else:
            try:
                keyring.delete_password(KEYRING_SERVICE, KEYRING_USERNAME)
            except Exception:
                pass
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg_copy, f, allow_unicode=True, sort_keys=False)
    except Exception:
        pass


def create_mqtt_client() -> mqtt.Client:
    try:
        return mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
    except AttributeError:
        return mqtt.Client()


def resolve_mqtt_subscription_topics(
    configured_topic: Union[str, None], monitor_topic: Union[str, None]
) -> list[str]:
    """把旧主题别名兼容到实际可订阅主题集合，避免控制板遥测消息被遗失。"""
    topics: list[str] = []

    configured = (configured_topic or DEFAULT_CONTROL_TELEMETRY_TOPIC).strip()
    if not configured:
        configured = DEFAULT_CONTROL_TELEMETRY_TOPIC

    if configured == LEGACY_CONTROL_TELEMETRY_TOPIC:
        # 兼容旧版 UI 配置里已经落盘成通用 telemetry 主题的场景。
        topics.extend([LEGACY_CONTROL_TELEMETRY_TOPIC, DEFAULT_CONTROL_TELEMETRY_TOPIC])
    elif configured == DEFAULT_CONTROL_TELEMETRY_TOPIC:
        topics.append(DEFAULT_CONTROL_TELEMETRY_TOPIC)
    else:
        # 保留用户显式配置；如果它不等于默认主题，仍然让它作为主订阅值。
        topics.append(configured)

    if monitor_topic and monitor_topic.strip():
        topics.append(monitor_topic.strip())

    # 避免重复订阅同一主题。
    seen = set()
    deduped = []
    for topic in topics:
        if topic not in seen:
            seen.add(topic)
            deduped.append(topic)
    return deduped


def generate_overlay_html(stream_url: str) -> str:
    return f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>{APP_NAME}</title>
<style>
  body {{ margin: 0; background: #000; overflow: hidden; }}
  #video {{
    position: absolute; top: 0; left: 0;
    width: 100vw; height: 100vh;
    object-fit: contain;
    background: #000;
  }}
  #crosshair {{
    position: absolute; top: 50%; left: 50%;
    transform: translate(-50%, -50%);
    width: 60px; height: 60px;
    pointer-events: none;
  }}
  #crosshair::before, #crosshair::after {{
    content: ''; position: absolute; background: #0f0;
  }}
  #crosshair::before {{ top: 50%; left: 0; width: 100%; height: 2px; transform: translateY(-50%); }}
  #crosshair::after {{ left: 50%; top: 0; height: 100%; width: 2px; transform: translateX(-50%); }}
  #hud {{
    position: absolute; bottom: 0; left: 0;
    width: 100%; padding: 10px 16px; box-sizing: border-box;
    color: #0f0; font-family: Consolas, "Courier New", monospace; font-size: 18px;
    text-shadow: 0 0 4px rgba(0,0,0,0.7);
    pointer-events: none; white-space: nowrap; overflow: hidden; text-overflow: ellipsis;
  }}
</style>
</head>
<body>
<img id="video" src="/cam" alt="video stream">
<div id="crosshair"></div>
<div id="hud">等待遥测数据...</div>
<script>
const hud = document.getElementById('hud');
const video = document.getElementById('video');

function formatUptime(ms) {{
  if (ms === undefined || ms === null) return '--';
  const total = Math.floor(ms / 1000);
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  if (h) return `${{h}}:${{String(m).padStart(2,'0')}}:${{String(s).padStart(2,'0')}}`;
  return `${{m}}:${{String(s).padStart(2,'0')}}`;
}}

async function updateTelemetry() {{
  try {{
    const r = await fetch('/telemetry');
    if (!r.ok) return;
    const d = await r.json();
    const vbat = (d.vbat !== undefined) ? d.vbat.toFixed(2) + 'V' : '--V';
    const l = (d.L !== undefined) ? d.L : '--';
    const rv = (d.R !== undefined) ? d.R : '--';
    const m1 = (d.M1 !== undefined) ? d.M1 : '--';
    const m2 = (d.M2 !== undefined) ? d.M2 : '--';
    const m3 = (d.M3 !== undefined) ? d.M3 : '--';
    const ctrlrssi = (d.ctrlrssi !== undefined) ? d.ctrlrssi : '--';
    const monrssi = (d.monrssi !== undefined) ? d.monrssi : '--';
    const up = formatUptime(d.uptime);
    const now = new Date().toLocaleTimeString('zh-CN', {{hour12:false}});
    const gear = d.gear ? d.gear.toUpperCase() : 'LOW';
    const mode = d.mode ? d.mode.toUpperCase() : 'MAN';
    const limit = (d.limit_pct !== undefined) ? d.limit_pct + '%' : '--';
    const boostSec = (d.boost_remaining_ms > 0)
      ? Math.ceil(d.boost_remaining_ms / 1000)
      : 0;
    const boostRem = (d.boost_remaining_ms !== undefined) ? `B:${{boostSec}}s` : '';
    const cd = (d.cooldown_ms > 0)
      ? `CD:${{Math.floor(d.cooldown_ms / 1000) + 1}}s`
      : '';
    hud.textContent = (
      `BAT ${{vbat}}  L:${{l}} R:${{rv}}  M1:${{m1}} M2:${{m2}} M3:${{m3}}  ` +
      `CR:${{ctrlrssi}} MR:${{monrssi}}  ${{gear}} ${{mode}} 限${{limit}} ${{boostRem}} ${{cd}}  UP ${{up}}  ${{now}}`
    );
  }} catch (e) {{
    console.log('updateTelemetry failed:', e);
  }}
}}

video.onerror = () => {{
  setTimeout(() => {{ video.src = '/cam?r=' + Date.now(); }}, 200);
}};

setInterval(updateTelemetry, 300);
updateTelemetry();
</script>
</body>
</html>"""


class MqttThread(QThread):
    telemetry = pyqtSignal(str, dict)
    status = pyqtSignal(str)
    connected = pyqtSignal(bool)

    def __init__(
        self,
        host: str,
        port: int,
        topic: Union[str, list],
        username: str = "",
        password: str = "",
    ):
        super().__init__()
        self.host = host
        self.port = port
        self.topics = [topic] if isinstance(topic, str) else list(topic)
        self.username = username
        self.password = password
        self._running = True
        self.client: Optional[mqtt.Client] = None

    def stop(self) -> None:
        self._running = False
        if self.client:
            try:
                self.client.disconnect()
            except Exception:
                pass
        self.wait(1500)

    def publish(self, topic: str, payload: str) -> bool:
        if self.client and self.client.is_connected():
            try:
                self.client.publish(topic, payload, qos=0)
                return True
            except Exception:
                pass
        return False

    def _on_connect(self, client, userdata, flags, rc):
        print(f"[MQTT] _on_connect rc={rc}, topics={self.topics}")
        if rc == 0:
            for t in self.topics:
                client.subscribe(t)
                print(f"[MQTT] 已订阅: {t}")
            self.connected.emit(True)
            self.status.emit("MQTT 已连接")
        else:
            self.status.emit(f"MQTT 连接失败: {rc}")

    def _on_disconnect(self, client, userdata, rc):
        print(f"[MQTT] _on_disconnect rc={rc}")
        self.connected.emit(False)
        if rc != 0:
            self.status.emit("MQTT 断开，自动重连中...")

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
            self.telemetry.emit(msg.topic, data)
        except Exception as e:
            self.status.emit(f"遥测解析失败: {e}")

    def run(self) -> None:
        while self._running:
            try:
                self.client = create_mqtt_client()
                if self.username:
                    self.client.username_pw_set(self.username, self.password)
                self.client.on_connect = self._on_connect
                self.client.on_disconnect = self._on_disconnect
                self.client.on_message = self._on_message
                print(f"[MQTT] 连接 {self.host}:{self.port} ...")
                self.client.connect(self.host, self.port, keepalive=30)
                # 等待 MQTT 握手完成
                for _ in range(40):
                    if not self._running:
                        break
                    self.client.loop(timeout=0.05)
                    if self.client.is_connected():
                        break
                # 维持连接，断开后外层循环重连
                while self._running and self.client.is_connected():
                    self.client.loop(timeout=0.1)
                    self.msleep(10)
            except Exception as e:
                print(f"[MQTT] 错误: {e}")
                self.status.emit(f"MQTT 错误: {e}")
                self.msleep(500)


class AudioUdpThread(QThread):
    error = pyqtSignal(str)

    def __init__(self, port: int):
        super().__init__()
        self.port = port
        self._running = True
        self.queue = queue.Queue(maxsize=100)
        self.sock: Optional[socket.socket] = None

    def stop(self) -> None:
        self._running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.wait(2000)

    def run(self) -> None:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("", self.port))
            self.sock.settimeout(1.0)
        except Exception as e:
            self.error.emit(f"音频端口绑定失败: {e}")
            return

        while self._running:
            try:
                data, _ = self.sock.recvfrom(2048)
                if data:
                    try:
                        self.queue.put_nowait(data)
                    except queue.Full:
                        try:
                            self.queue.get_nowait()
                            self.queue.put_nowait(data)
                        except Exception:
                            pass
            except socket.timeout:
                continue
            except Exception as e:
                self.error.emit(f"音频接收错误: {e}")


class AudioPlayer:
    SAMPLE_RATE = 16000
    CHANNELS = 1

    def __init__(self, pkt_queue: queue.Queue):
        self.queue = pkt_queue
        self.enabled = True
        self.volume = 0.8
        self._stream: Optional[sd.OutputStream] = None
        try:
            self._stream = sd.OutputStream(
                samplerate=self.SAMPLE_RATE,
                channels=self.CHANNELS,
                dtype=np.int16,
                blocksize=1024,
                callback=self._callback,
            )
            self._stream.start()
        except Exception as e:
            print(f"音频输出初始化失败: {e}")
            self._stream = None

    def set_volume(self, value: int) -> None:
        self.volume = max(0.0, min(2.0, value / 100.0))

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled

    def close(self) -> None:
        if self._stream:
            try:
                self._stream.stop()
                self._stream.close()
            except Exception:
                pass
            self._stream = None

    def _callback(self, outdata, frames, time_info, status):
        if not self.enabled or self.volume <= 0 or self._stream is None:
            outdata[:] = 0
            return

        needed = frames
        chunks = []
        total = 0
        while total < needed:
            try:
                packet = self.queue.get_nowait()
                samples = np.frombuffer(packet, dtype=np.int16)
                chunks.append(samples)
                total += len(samples)
            except queue.Empty:
                break

        if chunks:
            data = np.concatenate(chunks)
            if len(data) >= needed:
                chunk = data[:needed]
            else:
                chunk = np.concatenate(
                    [data, np.zeros(needed - len(data), dtype=np.int16)]
                )
        else:
            chunk = np.zeros(needed, dtype=np.int16)

        scaled = chunk.astype(np.float32) * self.volume
        np.clip(scaled, -32768, 32767, out=scaled)
        outdata[:] = scaled.astype(np.int16).reshape(-1, 1)


class FrameBuffer:
    """带条件变量的视频帧缓冲：写入最新 JPEG 后通知所有等待的客户端。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._jpeg: bytes = b""
        self._frame_id = 0

    def update(self, jpeg: bytes):
        with self._lock:
            self._jpeg = jpeg
            self._frame_id += 1
            self._cond.notify_all()

    def wait_next(self, last_id: int, timeout: float = 1.0) -> tuple[bytes, int]:
        with self._lock:
            if self._frame_id != last_id:
                return self._jpeg, self._frame_id
            self._cond.wait(timeout=timeout)
            return self._jpeg, self._frame_id


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _move_toward(current: float, target: float, max_delta: float) -> float:
    if target > current:
        return min(target, current + max_delta)
    if target < current:
        return max(target, current - max_delta)
    return current


class TankController:
    """计算左右电机 PWM、档位舵机角度，全部由上位机完成。"""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.gear = "low"
        self.mode = "manual"
        self.motor_left = 0.0
        self.motor_right = 0.0
        self.pressed_keys: set[int] = set()

        self.boost_active = False
        self.boost_remaining_ms = float(
            self.config.get("control_boost_max_ms", 15000)
        )
        self.last_boost_update_ms = 0

        self.brake_phase = "none"  # none / ramp_to_zero / reverse
        self.last_throttle_sign = 0  # 用于检测行进方向切换
        self.last_shift_ms = 0       # 换挡冷却计时
        self.shift_coast_end = 0     # 反向换挡后的强制滑行截止时间
        self.direction_change_state = "none"  # none / ramp_to_zero / hold
        self.direction_change_end = 0         # 低档方向切换保持结束时间

        self.target_shift_servo = int(self.config.get("control_m1_low", 0))
        self.current_shift_servo = float(self.target_shift_servo)

        # 伺服电机状态（索引1..N 对应 MQTT 的 M2..M(N+1)）
        # 使用电平值(0-255)，内部转换为舵机角度
        self._servo_levels: Dict[int, float] = {}
        self._servo_targets: Dict[int, int] = {}
        self._servo_transition_start: Dict[int, int] = {}
        self._servo_transition_duration: Dict[int, int] = {}
        self._servo_active_segment: Dict[int, int] = {}  # 0=不触发, 1..N=段N
        self._servo_prev_segment: Dict[int, int] = {}   # 长按前的状态，松开时恢复
        servo_count = int(self.config.get("servo_count", 2))
        for idx in range(1, servo_count + 1):
            idle_level = int(self.config.get(f"servo_{idx}_idle_level", 0))
            self._servo_levels[idx] = float(idle_level)
            self._servo_targets[idx] = idle_level
            self._servo_transition_start[idx] = 0
            self._servo_transition_duration[idx] = 500
            self._servo_active_segment[idx] = 0
            self._servo_prev_segment[idx] = 0

        self.control_state: Dict[str, Any] = {
            "gear": self.gear,
            "mode": self.mode,
            "limit_pct": config.get("control_max_output_pct", 70),
            "boost": False,
            "boost_remaining_ms": int(self.boost_remaining_ms),
            "cooldown_ms": 0,
            "l_cmd": 0,
            "r_cmd": 0,
            "m1": config.get("control_m1_low", 0),
            "m2": int(self.config.get("servo_1_idle_level", 0)) if servo_count >= 1 else 0,
            "m3": int(self.config.get("servo_2_idle_level", 0)) if servo_count >= 2 else 0,
            "brake_phase": self.brake_phase,
        }

    # ---- key / action APIs ----

    def set_key(self, key: int, pressed: bool):
        if pressed:
            self.pressed_keys.add(key)
        else:
            self.pressed_keys.discard(key)

    def is_pressed(self, key: int) -> bool:
        return key in self.pressed_keys

    def shift_up(self):
        if self.gear != "high":
            self._do_shift("high")

    def shift_down(self):
        if self.gear != "low":
            self._do_shift("low")

    def toggle_mode(self):
        self.mode = "auto" if self.mode == "manual" else "manual"

    def toggle_boost(self, now_ms: int, throttle: float = 0.0):
        if self.boost_active:
            self._stop_boost(now_ms)
        elif self.boost_remaining_ms > 0 and abs(throttle) > 0.5:
            self._start_boost(now_ms)

    def _start_boost(self, now_ms: int):
        self._update_boost_pool(now_ms)
        if self.boost_remaining_ms > 0:
            self.boost_active = True
            self.last_boost_update_ms = now_ms

    def _stop_boost(self, now_ms: int):
        if not self.boost_active:
            return
        self._update_boost_pool(now_ms)
        self.boost_active = False
        self.last_boost_update_ms = now_ms

    # ---- 伺服电机控制 ----

    def handle_servo_segment(self, servo_index: int, segment_index: int, now_ms: int, pressed: bool = True):
        """伺服电机分段控制。
        pressed=True: 按键按下
        pressed=False: 按键释放
        按住触发：按下→该段，释放→不触发
        按键切换：按下→当前为该段则回不触发，否则切到该段
        """
        mode = str(self.config.get(f"servo_{servo_index}_seg{segment_index}_mode", "toggle"))
        active = self._servo_active_segment.get(servo_index, 0)

        if mode == "hold":
            if pressed:
                self._servo_prev_segment[servo_index] = active
                target = int(self.config.get(f"servo_{servo_index}_seg{segment_index}_level", 0))
                self._servo_active_segment[servo_index] = segment_index
            else:
                if active == segment_index:
                    prev = self._servo_prev_segment.get(servo_index, 0)
                    if prev == 0:
                        target = int(self.config.get(f"servo_{servo_index}_idle_level", 0))
                    else:
                        target = int(self.config.get(f"servo_{servo_index}_seg{prev}_level", 0))
                    self._servo_active_segment[servo_index] = prev
                else:
                    return
            time_key = f"servo_{servo_index}_seg{segment_index}_time_s"
        else:
            if not pressed:
                return
            if active == segment_index:
                target = int(self.config.get(f"servo_{servo_index}_idle_level", 0))
                self._servo_active_segment[servo_index] = 0
            else:
                target = int(self.config.get(f"servo_{servo_index}_seg{segment_index}_level", 0))
                self._servo_active_segment[servo_index] = segment_index
            time_key = f"servo_{servo_index}_seg{segment_index}_time_s"

        duration_s = float(self.config.get(time_key, 0.5))
        if servo_index not in self._servo_levels:
            self._servo_levels[servo_index] = float(target)
        self._servo_targets[servo_index] = target
        self._servo_transition_start[servo_index] = now_ms
        self._servo_transition_duration[servo_index] = max(1, int(duration_s * 1000))

    def _update_boost_pool(self, now_ms: int):
        if self.last_boost_update_ms == 0:
            self.last_boost_update_ms = now_ms
        elapsed = now_ms - self.last_boost_update_ms
        if elapsed <= 0:
            return
        max_ms = float(self.config.get("control_boost_max_ms", 15000))
        if self.boost_active:
            self.boost_remaining_ms = max(0.0, self.boost_remaining_ms - elapsed)
            if self.boost_remaining_ms <= 0:
                self.boost_active = False
        else:
            if self.boost_remaining_ms < max_ms:
                self.boost_remaining_ms = min(
                    max_ms, self.boost_remaining_ms + elapsed / 4
                )
        self.last_boost_update_ms = now_ms

    # ---- helpers ----

    def _ratio(self) -> float:
        return (
            self.config.get("control_low_ratio", 0.8)
            if self.gear == "low"
            else self.config.get("control_high_ratio", 1.84615)
        )

    def _m1(self) -> int:
        """M1 为换挡舵机，返回当前平滑后的角度。"""
        return int(round(self.current_shift_servo))

    def _m2(self) -> int:
        """M2 为伺服电机1，返回当前电平值（0-255）。"""
        return int(round(self._servo_levels.get(1, 0)))

    def _m3(self) -> int:
        """M3 为伺服电机2，返回当前电平值（0-255）。"""
        return int(round(self._servo_levels.get(2, 0)))

    def _update_shift_servo(self):
        """M1 换挡舵机平滑移动，换挡时间可配置。"""
        low = int(self.config.get("control_m1_low", 0))
        high = int(self.config.get("control_m1_high", 90))
        target = low if self.gear == "low" else high
        self.target_shift_servo = target
        angle_range = abs(high - low)
        shift_time_s = self.config.get("control_shift_time_s", 0.08)
        # 控制周期 50ms，按目标换挡时间计算每帧最大移动角度
        max_delta = max(0.5, angle_range / shift_time_s * 0.05)
        self.current_shift_servo = _move_toward(
            self.current_shift_servo, float(target), max_delta
        )

    def _current_limit(self) -> float:
        if self.boost_active:
            return self.config.get("control_boost_output_pct", 90) / 100.0
        return self.config.get("control_max_output_pct", 70) / 100.0

    def _do_shift(self, new_gear: str):
        if new_gear == self.gear:
            return
        ratio_old = self._ratio()
        self.gear = new_gear
        ratio_new = self._ratio()

        wheel_left = self.motor_left * ratio_old
        wheel_right = self.motor_right * ratio_old

        if ratio_new == 0:
            return

        new_left = wheel_left / ratio_new
        new_right = wheel_right / ratio_new
        avg_old = (self.motor_left + self.motor_right) / 2.0
        avg_new = (new_left + new_right) / 2.0

        # 安全保护：换挡如果会导致电机瞬间反向，先把电机归零，
        # 否则反向电压/大电流可能烧毁 H 桥驱动芯片。
        if avg_old * avg_new < 0:
            self.motor_left = 0.0
            self.motor_right = 0.0
            self.shift_coast_end = int(time.time() * 1000) + 200
        else:
            blip = 0.0
            if abs(avg_new) > 0.02:
                blip_dir = 1.0 if avg_new >= 0 else -1.0
                blip = self.config.get("control_shift_blip_pct", 5) / 100.0 * blip_dir
            self.motor_left = _clamp(new_left + blip, -1.0, 1.0)
            self.motor_right = _clamp(new_right + blip, -1.0, 1.0)

        self.last_shift_ms = int(time.time() * 1000)

    # ---- main tick ----

    def tick(self, now_ms: int) -> Dict[str, int]:
        # 更新 boost 资源池：开启时消耗，关闭后以 4 倍速度恢复
        self._update_boost_pool(now_ms)

        key_fwd = int(self.config.get("control_key_forward", Qt.Key.Key_Up))
        key_bwd = int(self.config.get("control_key_backward", Qt.Key.Key_Down))
        key_left = int(self.config.get("control_key_left", Qt.Key.Key_Left))
        key_right = int(self.config.get("control_key_right", Qt.Key.Key_Right))

        forward = self.is_pressed(key_fwd)
        backward = self.is_pressed(key_bwd)
        left = self.is_pressed(key_left)
        right = self.is_pressed(key_right)

        # 目标油门
        if forward and backward:
            throttle = 0.0
        elif forward:
            throttle = 1.0
        elif backward:
            throttle = -self.config.get("control_brake_strength_pct", 50) / 100.0
        else:
            throttle = 0.0

        # 油门开度小于 90% 自动退出 boost
        if self.boost_active and abs(throttle) < 0.9:
            self._stop_boost(now_ms)

        # 目标转向
        steer = 0.0
        if left:
            steer -= 1.0
        if right:
            steer += 1.0

        # 根据档位和行驶方向选择转向系数，倒车时自动反转以保证操控直觉
        if throttle > 0:
            steering_factor = (
                self.config.get(
                    "control_steering_factor_high" if self.gear == "high" else "control_steering_factor_low",
                    0.4,
                )
                if self.gear in ("high", "low")
                else self.config.get("control_steering_factor", 0.5)
            )
        elif throttle < 0:
            steering_factor = self.config.get("control_steering_factor_reverse", 0.5)
            if self.config.get("control_reverse_steering_invert", True):
                steer = -steer
        else:
            steering_factor = self.config.get("control_steering_factor_inplace", 1.5)

        # 目标轮速（-1 ~ 1），与传动比无关
        left_wheel = throttle + steer * steering_factor
        right_wheel = throttle - steer * steering_factor
        max_val = max(abs(left_wheel), abs(right_wheel), 1.0)
        left_wheel /= max_val
        right_wheel /= max_val

        # 行进方向切换时，即使是在手动档，也先降到低速档
        # 用目标油门方向判断，而不是用瞬时轮速，避免负传动比换挡过程中被误判为倒车
        ratio = self._ratio()
        throttle_sign = 0
        if throttle > 0:
            throttle_sign = 1
        elif throttle < 0:
            throttle_sign = -1
        if (
            self.gear == "high"
            and throttle_sign != 0
            and throttle_sign != self.last_throttle_sign
        ):
            self._do_shift("low")
            # 方向切换时把电机归零，避免换挡瞬间继续向原方向冲击
            self.motor_left = 0.0
            self.motor_right = 0.0
            ratio = self._ratio()

        # 低档前进/后退直接切换保护：先减速到 0，再保持一段时间，防止 H 桥反向冲击
        if self.gear == "low" and throttle_sign == 0:
            self.direction_change_state = "none"
        elif (
            self.gear == "low"
            and throttle_sign != 0
            and self.last_throttle_sign != 0
            and throttle_sign != self.last_throttle_sign
            and self.direction_change_state == "none"
        ):
            self.direction_change_state = "ramp_to_zero"

        if self.direction_change_state == "ramp_to_zero":
            left_wheel = 0.0
            right_wheel = 0.0
            if max(abs(self.motor_left), abs(self.motor_right)) < 0.02:
                self.direction_change_state = "hold"
                self.direction_change_end = (
                    now_ms
                    + self.config.get("control_direction_change_hold_ms", 150)
                )
        elif self.direction_change_state == "hold":
            left_wheel = 0.0
            right_wheel = 0.0
            if now_ms >= self.direction_change_end:
                self.direction_change_state = "none"

        wheel_speed = (self.motor_left * ratio + self.motor_right * ratio) / 2.0

        # 高速档前进 + 刹车：先降到 0，降到低速档，再倒车
        if backward and self.gear == "high" and wheel_speed > 0.05:
            self.brake_phase = "ramp_to_zero"
        elif not backward:
            self.brake_phase = "none"

        if self.brake_phase == "ramp_to_zero":
            left_wheel = 0.0
            right_wheel = 0.0
            if max(abs(self.motor_left), abs(self.motor_right)) < 0.02:
                self._do_shift("low")
                self.brake_phase = "reverse"
        elif self.brake_phase == "reverse":
            rev = -self.config.get("control_brake_strength_pct", 50) / 100.0
            left_wheel = rev
            right_wheel = rev

        # 自动挡换挡（基于目标轮速），加入冷却防止抖动
        shift_cooldown = 500  # ms
        can_auto_shift = now_ms - self.last_shift_ms >= shift_cooldown
        if (
            self.mode == "auto"
            and self.brake_phase == "none"
            and can_auto_shift
        ):
            speed_mag = max(abs(left_wheel), abs(right_wheel))
            if self.gear == "low" and speed_mag > self.config.get(
                "control_upshift_threshold", 0.6
            ):
                self._do_shift("high")
            elif self.gear == "high" and speed_mag < self.config.get(
                "control_downshift_threshold", 0.35
            ):
                self._do_shift("low")

        # 目标轮速 -> 目标电机转速
        # 用满电机输出限制，让高档位能真正跑得更快
        ratio = self._ratio()
        limit = self._current_limit()
        ratio_sign = 1.0 if ratio >= 0 else -1.0
        left_motor_target = left_wheel * limit * ratio_sign
        right_motor_target = right_wheel * limit * ratio_sign

        # 反向换挡后的 200ms 强制滑行，避免同一 tick 内又从 0  ramp 到反向目标
        if now_ms < self.shift_coast_end:
            left_motor_target = 0.0
            right_motor_target = 0.0

        # 转速变化速率限制
        max_delta = self.config.get("control_accel_rate_pct", 8) / 100.0

        self.motor_left = _move_toward(
            self.motor_left, left_motor_target, max_delta
        )
        self.motor_right = _move_toward(
            self.motor_right, right_motor_target, max_delta
        )

        # 防止换挡瞬间计算值越界
        self.motor_left = _clamp(self.motor_left, -limit, limit)
        self.motor_right = _clamp(self.motor_right, -limit, limit)

        # M1 换挡舵机平滑跟随目标角度
        self._update_shift_servo()

        # 伺服电机平滑插值（电平值 0-255）
        for idx in range(1, int(self.config.get("servo_count", 2)) + 1):
            if idx not in self._servo_levels:
                self._servo_levels[idx] = float(self._servo_targets.get(idx, 0))
            target = self._servo_targets.get(idx, 0)
            current = self._servo_levels[idx]
            duration = self._servo_transition_duration.get(idx, 500)
            elapsed = now_ms - self._servo_transition_start.get(idx, 0)
            if elapsed >= duration:
                self._servo_levels[idx] = float(target)
            else:
                progress = elapsed / max(duration, 1)
                self._servo_levels[idx] = current + (target - current) * progress

        l_pwm = int(round(self.motor_left * 255))
        r_pwm = int(round(self.motor_right * 255))

        boost_max = float(self.config.get("control_boost_max_ms", 15000))
        boost_remaining_ms = int(self.boost_remaining_ms)
        # 冷却时间 = 从当前剩余量恢复到满所需的真实时间（恢复速度 1/4）
        cooldown_ms = 0
        if not self.boost_active and self.boost_remaining_ms < boost_max:
            cooldown_ms = int((boost_max - self.boost_remaining_ms) * 4)
        self.control_state = {
            "gear": self.gear,
            "mode": self.mode,
            "limit_pct": int(round(limit * 100)),
            "boost": self.boost_active,
            "boost_remaining_ms": boost_remaining_ms,
            "cooldown_ms": cooldown_ms,
            "l_cmd": l_pwm,
            "r_cmd": r_pwm,
            "m1": self._m1(),
            "m2": self._m2(),
            "m3": self._m3(),
            "brake_phase": self.brake_phase,
            "direction_change_state": self.direction_change_state,
        }

        self.last_throttle_sign = throttle_sign
        return {"L": l_pwm, "R": r_pwm, "M1": self._m1(), "M2": self._m2(), "M3": self._m3()}


class CamProxyThread(QThread):
    """使用 requests 解析 MJPEG 流，把最新帧写入 FrameBuffer，更稳定。"""

    error = pyqtSignal(str)

    def __init__(self, stream_url: str, buffer: FrameBuffer, quality: int = 80):
        super().__init__()
        self.stream_url = stream_url
        self.buffer = buffer
        self.quality = quality
        self._running = True
        self.flip_h = False
        self.flip_v = False
        self.rotation = 0

    def stop(self) -> None:
        self._running = False
        self.wait(1500)

    def set_transform(self, flip_h: bool, flip_v: bool, rotation: int):
        self.flip_h = flip_h
        self.flip_v = flip_v
        self.rotation = rotation

    def _apply_transform(self, frame):
        if self.flip_h:
            frame = cv2.flip(frame, 1)
        if self.flip_v:
            frame = cv2.flip(frame, 0)
        if self.rotation == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        return frame

    def _reencode(self, jpeg: bytes) -> bytes:
        if self.quality >= 95 and not self.flip_h and not self.flip_v and self.rotation == 0:
            return jpeg
        try:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                return jpeg
            frame = self._apply_transform(frame)
            ok, enc = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            )
            return enc.tobytes() if ok else jpeg
        except Exception:
            return jpeg

    def _parse_boundary(self, content_type: str) -> Optional[bytes]:
        for part in content_type.split(";"):
            part = part.strip()
            if part.lower().startswith("boundary="):
                b = part.split("=", 1)[1]
                if b.startswith('"') and b.endswith('"'):
                    b = b[1:-1]
                return b.encode()
        return None

    @staticmethod
    def _parse_content_length(headers: bytes) -> Optional[int]:
        for line in headers.split(b"\r\n"):
            if line.lower().startswith(b"content-length:"):
                try:
                    return int(line.split(b":", 1)[1].strip())
                except Exception:
                    return None
        return None

    def run(self) -> None:
        session = requests.Session()
        session.headers.update({
            "User-Agent": "Mozilla/5.0 (TankRC)",
            "Cache-Control": "no-cache",
            "Connection": "close",
        })
        while self._running:
            try:
                with session.get(
                    self.stream_url, stream=True, timeout=(2, 2)
                ) as resp:
                    if resp.status_code != 200:
                        self.error.emit(
                            f"摄像头返回 {resp.status_code}，{self._retry_msg()}"
                        )
                        self.msleep(200)
                        continue

                    boundary = self._parse_boundary(
                        resp.headers.get("content-type", "")
                    )
                    if not boundary:
                        boundary = b"frame"
                    marker = b"--" + boundary

                    buf = b""
                    for chunk in resp.iter_content(chunk_size=4096):
                        if not self._running:
                            break
                        buf += chunk
                        while True:
                            header_end = buf.find(b"\r\n\r\n")
                            if header_end == -1:
                                break
                            data_start = header_end + 4
                            headers = buf[:header_end]
                            content_length = self._parse_content_length(headers)

                            if content_length is not None and len(buf) >= data_start + content_length:
                                jpeg = buf[data_start:data_start + content_length]
                                buf = buf[data_start + content_length:]
                                if buf.startswith(b"\r\n"):
                                    buf = buf[2:]
                            else:
                                next_marker = buf.find(marker, data_start)
                                if next_marker == -1:
                                    break
                                jpeg = buf[data_start:next_marker]
                                buf = buf[next_marker:]

                            if jpeg:
                                # 去掉尾部可能的 CRLF
                                if jpeg.endswith(b"\r\n"):
                                    jpeg = jpeg[:-2]
                                self.buffer.update(self._reencode(jpeg))
            except Exception as e:
                if self._running:
                    self.error.emit(f"摄像头流中断: {e}，{self._retry_msg()}")
                    self.msleep(200)

    def _retry_msg(self) -> str:
        return "0.2秒后重连"


class OverlayHandler(BaseHTTPRequestHandler):
    """为上位机窗口和 OBS 提供带 HUD 的网页、视频代理流与遥测数据。"""

    MJPEG_BOUNDARY = "frame"

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._serve_html()
        elif self.path == "/cam":
            self._serve_cam()
        elif self.path == "/telemetry":
            self._serve_telemetry()
        else:
            self.send_response(404)
            self.end_headers()

    def _serve_html(self):
        html = generate_overlay_html(self.server.stream_url)
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _serve_cam(self):
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={self.MJPEG_BOUNDARY}",
        )
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()

        server = self.server
        last_id = -1
        while not server.shutdown_flag:
            jpeg, last_id = server.buffer.wait_next(last_id, timeout=0.5)
            if server.shutdown_flag:
                break
            if not jpeg:
                jpeg = NO_SIGNAL_JPEG
            try:
                packet = (
                    f"--{self.MJPEG_BOUNDARY}\r\n".encode()
                    + b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                    + jpeg
                    + b"\r\n"
                )
                self.wfile.write(packet)
            except (BrokenPipeError, ConnectionResetError):
                break

    def _serve_telemetry(self):
        with self.server.lock:
            data = dict(self.server.latest_telemetry)
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class HttpOverlayServer(ThreadingHTTPServer):
    allow_reuse_address = True

    def __init__(self, server_address, handler_class, stream_url: str):
        super().__init__(server_address, handler_class)
        self.stream_url = stream_url
        self.latest_telemetry: Dict[str, Any] = {}
        self.lock = threading.Lock()
        self.buffer = FrameBuffer()
        self.shutdown_flag = False

    def set_telemetry(self, data: Dict[str, Any]):
        with self.lock:
            self.latest_telemetry = dict(data)

    def stop(self):
        self.shutdown_flag = True
        self.shutdown()


class OverlayServerThread(QThread):
    def __init__(self, port: int, stream_url: str):
        super().__init__()
        self.port = port
        self.server = HttpOverlayServer(("", port), OverlayHandler, stream_url)

    def run(self):
        self.server.serve_forever()

    def set_telemetry(self, data: Dict[str, Any]):
        self.server.set_telemetry(data)

    def stop(self):
        self.server.stop()
        self.wait(2000)


class SettingsOverlay(QWidget):
    apply_requested = pyqtSignal()
    camera_config_changed = pyqtSignal(str, int, int)

    def __init__(self, parent: QWidget, config: Dict[str, Any]):
        super().__init__(parent)
        self.config = config
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAutoFillBackground(False)

        self._build_ui()
        self._apply_values()
        self.hide()

    @staticmethod
    def _key_from_seq_edit(edit) -> int:
        seq = edit.keySequence()
        if seq.count() > 0:
            return int(seq[0].key())
        return 0

    @staticmethod
    def _set_seq_edit(edit, key: int):
        edit.setKeySequence(QKeySequence(Qt.Key(key)))

    def _build_ui(self):
        self.container = QFrame(self)
        self.container.setStyleSheet(
            "QFrame { background-color: rgba(0, 0, 0, 64); border-radius: 10px; }"
            "QLabel { color: #00ff00; }"
            "QLineEdit, QSpinBox, QDoubleSpinBox { background-color: rgba(34, 34, 34, 200); color: #00ff00; border: 1px solid #00ff00; }"
            "QPushButton { background-color: rgba(0, 51, 0, 200); color: #00ff00; border: 1px solid #00ff00; padding: 4px 12px; }"
            "QPushButton:hover { background-color: rgba(0, 85, 0, 200); }"
            "QTabWidget::pane { border: 1px solid #00ff00; background: transparent; }"
            "QTabBar::tab { background: rgba(0, 51, 0, 200); color: #00ff00; padding: 6px 14px; }"
            "QTabBar::tab:selected { background: rgba(0, 85, 0, 220); }"
        )

        layout = QVBoxLayout(self.container)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        header = QHBoxLayout()
        header.addWidget(QLabel(f"<b>{APP_NAME} - 设置</b>"))
        header.addStretch()
        self.btn_close = QPushButton("✕")
        self.btn_close.setFixedSize(28, 28)
        self.btn_close.clicked.connect(self.hide)
        header.addWidget(self.btn_close)
        layout.addLayout(header)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, stretch=1)

        self._build_connection_tab()
        self._build_control_tab()
        self._build_servo_tab()
        self._build_camera_tab()

        bottom = QHBoxLayout()
        bottom.addStretch()
        self.btn_ok = QPushButton("确定")
        self.btn_ok.setFixedWidth(100)
        self.btn_ok.clicked.connect(self._on_ok)
        bottom.addWidget(self.btn_ok)
        layout.addLayout(bottom)

        self._save_timer = QTimer(self)
        self._save_timer.setSingleShot(True)
        self._save_timer.timeout.connect(self._save_from_inputs)

    def _build_connection_tab(self):
        tab = QWidget()
        form = QFormLayout(tab)
        form.setSpacing(10)

        self.inp_mqtt_host = QLineEdit()
        self.inp_mqtt_port = QSpinBox()
        self.inp_mqtt_port.setRange(1, 65535)
        self.inp_mqtt_topic = QLineEdit()
        self.inp_mqtt_user = QLineEdit()
        self.inp_mqtt_pass = QLineEdit()
        self.inp_mqtt_pass.setEchoMode(QLineEdit.EchoMode.Password)
        self.inp_stream_url = QLineEdit()
        self.inp_audio_port = QSpinBox()
        self.inp_audio_port.setRange(1, 65535)
        self.inp_stream_port = QSpinBox()
        self.inp_stream_port.setRange(1, 65535)
        self.inp_cam_rssi_threshold = QDoubleSpinBox()
        self.inp_cam_rssi_threshold.setRange(-100, 0)
        self.inp_cam_rssi_threshold.setSingleStep(1)
        self.inp_cam_rssi_threshold.setDecimals(0)
        self.inp_cam_rssi_delay = QSpinBox()
        self.inp_cam_rssi_delay.setRange(1000, 3600000)
        self.inp_cam_rssi_delay.setSuffix("ms")
        self.inp_ctrl_rssi_threshold = QDoubleSpinBox()
        self.inp_ctrl_rssi_threshold.setRange(-100, 0)
        self.inp_ctrl_rssi_threshold.setSingleStep(1)
        self.inp_ctrl_rssi_threshold.setDecimals(0)
        self.inp_ctrl_rssi_delay = QSpinBox()
        self.inp_ctrl_rssi_delay.setRange(1000, 3600000)
        self.inp_ctrl_rssi_delay.setSuffix("ms")

        form.addRow("MQTT 服务器:", self.inp_mqtt_host)
        form.addRow("MQTT 端口:", self.inp_mqtt_port)
        form.addRow("遥测主题:", self.inp_mqtt_topic)
        form.addRow("MQTT 用户名:", self.inp_mqtt_user)
        form.addRow("MQTT 密码:", self.inp_mqtt_pass)
        form.addRow("视频流 URL:", self.inp_stream_url)
        form.addRow("音频接收端口:", self.inp_audio_port)
        form.addRow("推流输出端口:", self.inp_stream_port)
        form.addRow("摄像头低 RSSI 阈值(dBm):", self.inp_cam_rssi_threshold)
        form.addRow("摄像头持续低 RSSI 进入重连(ms):", self.inp_cam_rssi_delay)
        form.addRow("控制板低 RSSI 阈值(dBm):", self.inp_ctrl_rssi_threshold)
        form.addRow("控制板持续低 RSSI 进入重连(ms):", self.inp_ctrl_rssi_delay)

        self.tabs.addTab(tab, "连接")

        for w in (
            self.inp_mqtt_host,
            self.inp_mqtt_port,
            self.inp_mqtt_topic,
            self.inp_mqtt_user,
            self.inp_mqtt_pass,
            self.inp_stream_url,
            self.inp_audio_port,
            self.inp_stream_port,
            self.inp_cam_rssi_delay,
            self.inp_ctrl_rssi_delay,
        ):
            if isinstance(w, QLineEdit):
                w.textChanged.connect(self._schedule_save)
            elif isinstance(w, QSpinBox):
                w.valueChanged.connect(self._schedule_save)
        for w in (
            self.inp_cam_rssi_threshold,
            self.inp_ctrl_rssi_threshold,
        ):
            w.valueChanged.connect(self._schedule_save)

    def _build_control_tab(self):
        tab = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        content = QWidget()
        form = QFormLayout(content)
        form.setSpacing(10)
        scroll.setWidget(content)

        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(scroll)

        # 按键绑定
        self.inp_key_forward = QKeySequenceEdit()
        self.inp_key_backward = QKeySequenceEdit()
        self.inp_key_left = QKeySequenceEdit()
        self.inp_key_right = QKeySequenceEdit()
        self.inp_key_boost = QKeySequenceEdit()
        self.inp_key_shift_up = QKeySequenceEdit()
        self.inp_key_shift_down = QKeySequenceEdit()
        self.inp_key_mode = QKeySequenceEdit()
        for edit in (
            self.inp_key_forward,
            self.inp_key_backward,
            self.inp_key_left,
            self.inp_key_right,
            self.inp_key_boost,
            self.inp_key_shift_up,
            self.inp_key_shift_down,
            self.inp_key_mode,
        ):
            edit.setMaximumSequenceLength(1)
            edit.keySequenceChanged.connect(self._schedule_save)

        form.addRow("前进:", self.inp_key_forward)
        form.addRow("后退/刹车:", self.inp_key_backward)
        form.addRow("左转:", self.inp_key_left)
        form.addRow("右转:", self.inp_key_right)
        form.addRow("Boost:", self.inp_key_boost)
        form.addRow("升档:", self.inp_key_shift_up)
        form.addRow("降档:", self.inp_key_shift_down)
        form.addRow("自动/手动切换:", self.inp_key_mode)

        # 参数
        self.inp_max_output = QSpinBox()
        self.inp_max_output.setRange(1, 100)
        self.inp_max_output.setSuffix("%")
        self.inp_boost_output = QSpinBox()
        self.inp_boost_output.setRange(1, 100)
        self.inp_boost_output.setSuffix("%")
        self.inp_boost_ms = QSpinBox()
        self.inp_boost_ms.setRange(100, 30000)
        self.inp_boost_ms.setSuffix("ms")
        self.inp_accel_rate = QSpinBox()
        self.inp_accel_rate.setRange(1, 100)
        self.inp_accel_rate.setSuffix("%/tick")
        self.inp_brake_strength = QSpinBox()
        self.inp_brake_strength.setRange(1, 100)
        self.inp_brake_strength.setSuffix("%")
        self.inp_steering = QDoubleSpinBox()
        self.inp_steering.setRange(0.0, 2.0)
        self.inp_steering.setSingleStep(0.05)
        self.inp_steering.setDecimals(2)
        self.inp_steering_high = QDoubleSpinBox()
        self.inp_steering_high.setRange(0.0, 2.0)
        self.inp_steering_high.setSingleStep(0.05)
        self.inp_steering_high.setDecimals(2)
        self.inp_steering_low = QDoubleSpinBox()
        self.inp_steering_low.setRange(0.0, 2.0)
        self.inp_steering_low.setSingleStep(0.05)
        self.inp_steering_low.setDecimals(2)
        self.inp_steering_reverse = QDoubleSpinBox()
        self.inp_steering_reverse.setRange(0.0, 2.0)
        self.inp_steering_reverse.setSingleStep(0.05)
        self.inp_steering_reverse.setDecimals(2)
        self.inp_steering_inplace = QDoubleSpinBox()
        self.inp_steering_inplace.setRange(0.0, 2.0)
        self.inp_steering_inplace.setSingleStep(0.05)
        self.inp_steering_inplace.setDecimals(2)
        self.inp_upshift = QDoubleSpinBox()
        self.inp_upshift.setRange(0.0, 1.0)
        self.inp_upshift.setSingleStep(0.05)
        self.inp_downshift = QDoubleSpinBox()
        self.inp_downshift.setRange(0.0, 1.0)
        self.inp_downshift.setSingleStep(0.05)
        self.inp_shift_blip = QSpinBox()
        self.inp_shift_blip.setRange(0, 50)
        self.inp_shift_blip.setSuffix("%")
        self.inp_low_ratio = QDoubleSpinBox()
        self.inp_low_ratio.setRange(-5.0, 5.0)
        self.inp_low_ratio.setSingleStep(0.1)
        self.inp_low_ratio.setDecimals(3)
        self.inp_high_ratio = QDoubleSpinBox()
        self.inp_high_ratio.setRange(-5.0, 5.0)
        self.inp_high_ratio.setSingleStep(0.1)
        self.inp_high_ratio.setDecimals(3)
        self.inp_m1_low = QSpinBox()
        self.inp_m1_low.setRange(0, 180)
        self.inp_m1_high = QSpinBox()
        self.inp_m1_high.setRange(0, 180)
        self.inp_shift_time = QDoubleSpinBox()
        self.inp_shift_time.setRange(0.02, 2.0)
        self.inp_shift_time.setSingleStep(0.01)
        self.inp_shift_time.setDecimals(2)
        self.inp_shift_time.setSuffix("s")
        self.inp_direction_change_hold = QSpinBox()
        self.inp_direction_change_hold.setRange(0, 1000)
        self.inp_direction_change_hold.setSingleStep(10)
        self.inp_direction_change_hold.setSuffix("ms")

        form.addRow("最大输出限制:", self.inp_max_output)
        form.addRow("Boost 输出限制:", self.inp_boost_output)
        form.addRow("Boost 最长时间:", self.inp_boost_ms)
        form.addRow("转速变化速率:", self.inp_accel_rate)
        form.addRow("刹车/倒车力度:", self.inp_brake_strength)
        form.addRow("前进转向系数:", self.inp_steering)
        form.addRow("高速档前进转向系数:", self.inp_steering_high)
        form.addRow("低速档前进转向系数:", self.inp_steering_low)
        form.addRow("倒车转向系数:", self.inp_steering_reverse)
        form.addRow("原地转向系数:", self.inp_steering_inplace)
        form.addRow("自动升档阈值:", self.inp_upshift)
        form.addRow("自动降档阈值:", self.inp_downshift)
        form.addRow("换挡补油百分比:", self.inp_shift_blip)
        form.addRow("低速档传动比:", self.inp_low_ratio)
        form.addRow("高速档传动比:", self.inp_high_ratio)
        form.addRow("M1 低速档换挡角度:", self.inp_m1_low)
        form.addRow("M1 高速档换挡角度:", self.inp_m1_high)
        form.addRow("换挡时间:", self.inp_shift_time)
        form.addRow("低档换向保持时间:", self.inp_direction_change_hold)

        self.tabs.addTab(tab, "控制")

        for w in (
            self.inp_max_output,
            self.inp_boost_output,
            self.inp_boost_ms,
            self.inp_accel_rate,
            self.inp_brake_strength,
            self.inp_shift_blip,
            self.inp_m1_low,
            self.inp_m1_high,
            self.inp_direction_change_hold,
        ):
            w.valueChanged.connect(self._schedule_save)
        for w in (
            self.inp_steering,
            self.inp_steering_high,
            self.inp_steering_low,
            self.inp_steering_reverse,
            self.inp_steering_inplace,
            self.inp_upshift,
            self.inp_downshift,
            self.inp_low_ratio,
            self.inp_high_ratio,
            self.inp_shift_time,
        ):
            w.valueChanged.connect(self._schedule_save)

    def _build_servo_tab(self):
        tab = QWidget()
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: transparent; }")
        content = QWidget()
        form = QFormLayout(content)
        form.setSpacing(10)
        scroll.setWidget(content)

        layout = QVBoxLayout(tab)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(scroll)

        self.inp_servo_count = QSpinBox()
        self.inp_servo_count.setRange(1, 8)
        self.inp_servo_count.blockSignals(True)
        self.inp_servo_count.setValue(int(self.config.get("servo_count", 2)))
        self.inp_servo_count.blockSignals(False)
        self.inp_servo_count.valueChanged.connect(self._on_servo_count_changed)
        form.addRow("伺服电机数量:", self.inp_servo_count)

        self._servo_group_widgets: list[QWidget] = []
        self._servo_inputs: dict[str, Any] = {}
        self._servo_form = form

        self._rebuild_servo_sections()
        self.tabs.addTab(tab, "伺服电机")

    def _on_servo_count_changed(self, value: int):
        self._rebuild_servo_sections()
        self._schedule_save()

    def _on_servo_seg_count_changed(self, servo_idx: int, value: int):
        self.config[f"servo_{servo_idx}_seg_count"] = value
        self._schedule_save()
        QTimer.singleShot(0, self._rebuild_servo_sections)

    def _rebuild_servo_sections(self):
        for w in self._servo_group_widgets:
            self._servo_form.removeRow(w)
        self._servo_group_widgets.clear()
        self._servo_inputs.clear()

        count = self.inp_servo_count.value()
        for idx in range(1, count + 1):
            label = QLabel(f"<hr><b>伺服电机 {idx}</b>")
            self._servo_form.addRow(label)
            self._servo_group_widgets.append(label)

            inp_name = QLineEdit()
            inp_name.setText(str(self.config.get(f"servo_{idx}_name", f"伺服{idx}")))
            inp_name.textChanged.connect(self._schedule_save)
            self._servo_form.addRow("名称:", inp_name)
            self._servo_group_widgets.append(inp_name)
            self._servo_inputs[f"servo_{idx}_name"] = inp_name

            inp_seg_count = QSpinBox()
            inp_seg_count.setRange(1, 8)
            inp_seg_count.blockSignals(True)
            inp_seg_count.setValue(int(self.config.get(f"servo_{idx}_seg_count", 2)))
            inp_seg_count.blockSignals(False)
            inp_seg_count.valueChanged.connect(self._schedule_save)
            inp_seg_count.valueChanged.connect(lambda val, i=idx: self._on_servo_seg_count_changed(i, val))
            self._servo_form.addRow("段数:", inp_seg_count)
            self._servo_group_widgets.append(inp_seg_count)
            self._servo_inputs[f"servo_{idx}_seg_count"] = inp_seg_count

            inp_idle = QSpinBox()
            inp_idle.setRange(0, 255)
            inp_idle.blockSignals(True)
            inp_idle.setValue(int(self.config.get(f"servo_{idx}_idle_level", 0)))
            inp_idle.blockSignals(False)
            inp_idle.valueChanged.connect(self._schedule_save)
            self._servo_form.addRow("不触发电平:", inp_idle)
            self._servo_group_widgets.append(inp_idle)
            self._servo_inputs[f"servo_{idx}_idle_level"] = inp_idle

            seg_count = inp_seg_count.value()
            for seg in range(1, seg_count + 1):
                seg_label = QLabel(f"段 {seg}")
                self._servo_form.addRow(seg_label)
                self._servo_group_widgets.append(seg_label)

                inp_key = QKeySequenceEdit()
                inp_key.setMaximumSequenceLength(1)
                key_val = int(self.config.get(f"servo_{idx}_seg{seg}_key", 0))
                self._set_seq_edit(inp_key, key_val)
                inp_key.keySequenceChanged.connect(self._schedule_save)
                self._servo_form.addRow("  按键:", inp_key)
                self._servo_group_widgets.append(inp_key)
                self._servo_inputs[f"servo_{idx}_seg{seg}_key"] = inp_key

                inp_level = QSpinBox()
                inp_level.setRange(0, 255)
                inp_level.blockSignals(True)
                inp_level.setValue(int(self.config.get(f"servo_{idx}_seg{seg}_level", 0)))
                inp_level.blockSignals(False)
                inp_level.valueChanged.connect(self._schedule_save)
                self._servo_form.addRow("  电平:", inp_level)
                self._servo_group_widgets.append(inp_level)
                self._servo_inputs[f"servo_{idx}_seg{seg}_level"] = inp_level

                inp_time = QDoubleSpinBox()
                inp_time.setRange(0.01, 10.0)
                inp_time.setSingleStep(0.1)
                inp_time.setDecimals(2)
                inp_time.setSuffix("s")
                inp_time.blockSignals(True)
                inp_time.setValue(float(self.config.get(f"servo_{idx}_seg{seg}_time_s", 0.5)))
                inp_time.blockSignals(False)
                inp_time.valueChanged.connect(self._schedule_save)
                self._servo_form.addRow("  平滑过渡时间:", inp_time)
                self._servo_group_widgets.append(inp_time)
                self._servo_inputs[f"servo_{idx}_seg{seg}_time_s"] = inp_time

                inp_mode = QComboBox()
                inp_mode.addItems(["按住触发", "按键切换"])
                mode_val = str(self.config.get(f"servo_{idx}_seg{seg}_mode", "toggle"))
                inp_mode.blockSignals(True)
                inp_mode.setCurrentIndex(0 if mode_val == "hold" else 1)
                inp_mode.blockSignals(False)
                inp_mode.currentIndexChanged.connect(self._schedule_save)
                self._servo_form.addRow("  触发方式:", inp_mode)
                self._servo_group_widgets.append(inp_mode)
                self._servo_inputs[f"servo_{idx}_seg{seg}_mode"] = inp_mode

    # 摄像头分辨率显示名称 -> 下发给 ESP32 的 FRAMESIZE_ 名称
    CAM_RESOLUTION_ITEMS: Dict[str, str] = {
        "320x240 (QVGA)": "QVGA",
        "640x480 (VGA)": "VGA",
        "800x600 (SVGA)": "SVGA",
        "1024x768 (XGA)": "XGA",
        "1280x720 (HD)": "HD",
        "1280x1024 (SXGA)": "SXGA",
        "1600x1200 (UXGA)": "UXGA",
    }

    def _build_camera_tab(self):
        tab = QWidget()
        form = QFormLayout(tab)
        form.setSpacing(10)

        self.inp_cam_resolution = QComboBox()
        self.inp_cam_resolution.addItems(list(self.CAM_RESOLUTION_ITEMS.keys()))
        self.inp_cam_quality = QSpinBox()
        self.inp_cam_quality.setRange(0, 63)
        self.inp_cam_quality.setSuffix(" (越低越清晰)")
        self.inp_cam_fps = QSpinBox()
        self.inp_cam_fps.setRange(1, 60)
        self.inp_cam_fps.setSuffix(" fps")

        self.lbl_cam_config_status = QLabel("修改后点击“确定”即可通过 MQTT 实时下发到摄像头")

        form.addRow("分辨率:", self.inp_cam_resolution)
        form.addRow("JPEG 质量:", self.inp_cam_quality)
        form.addRow("目标帧率:", self.inp_cam_fps)
        form.addRow(self.lbl_cam_config_status)

        hline = QFrame()
        hline.setFrameShape(QFrame.Shape.HLine)
        hline.setStyleSheet("QFrame { color: #00ff00; }")
        form.addRow(hline)

        self.inp_flip_h = QCheckBox("左右镜像")
        self.inp_flip_v = QCheckBox("上下镜像")
        self.inp_rotation = QComboBox()
        self.inp_rotation.addItems(["0°", "90°", "180°", "270°"])

        form.addRow("画面翻转:", self.inp_flip_h)
        form.addRow("", self.inp_flip_v)
        form.addRow("旋转:", self.inp_rotation)

        hline2 = QFrame()
        hline2.setFrameShape(QFrame.Shape.HLine)
        hline2.setStyleSheet("QFrame { color: #00ff00; }")
        form.addRow(hline2)

        self.inp_auto_quality = QCheckBox("启用自动降画质")
        self.inp_aq_low_rssi = QSpinBox()
        self.inp_aq_low_rssi.setRange(-100, -1)
        self.inp_aq_low_rssi.setSuffix(" dBm")
        self.inp_aq_recover_rssi = QSpinBox()
        self.inp_aq_recover_rssi.setRange(-100, -1)
        self.inp_aq_recover_rssi.setSuffix(" dBm")
        self.inp_aq_low_resolution = QComboBox()
        self.inp_aq_low_resolution.addItems(list(self.CAM_RESOLUTION_ITEMS.keys()))
        self.inp_aq_low_quality = QSpinBox()
        self.inp_aq_low_quality.setRange(0, 63)
        self.inp_aq_low_quality.setSuffix(" (越低越清晰)")
        self.inp_aq_low_fps = QSpinBox()
        self.inp_aq_low_fps.setRange(1, 30)
        self.inp_aq_low_fps.setSuffix(" fps")

        form.addRow(self.inp_auto_quality)
        form.addRow("低于此 RSSI 降画质:", self.inp_aq_low_rssi)
        form.addRow("高于此 RSSI 恢复:", self.inp_aq_recover_rssi)
        form.addRow("降级分辨率:", self.inp_aq_low_resolution)
        form.addRow("降级 JPEG 质量:", self.inp_aq_low_quality)
        form.addRow("降级目标帧率:", self.inp_aq_low_fps)

        self.tabs.addTab(tab, "摄像头")

        self.inp_cam_resolution.currentTextChanged.connect(self._schedule_save)
        self.inp_cam_quality.valueChanged.connect(self._schedule_save)
        self.inp_cam_fps.valueChanged.connect(self._schedule_save)
        self.inp_flip_h.toggled.connect(self._schedule_save)
        self.inp_flip_v.toggled.connect(self._schedule_save)
        self.inp_rotation.currentTextChanged.connect(self._schedule_save)
        self.inp_auto_quality.toggled.connect(self._schedule_save)
        self.inp_aq_low_rssi.valueChanged.connect(self._schedule_save)
        self.inp_aq_recover_rssi.valueChanged.connect(self._schedule_save)
        self.inp_aq_low_resolution.currentTextChanged.connect(self._schedule_save)
        self.inp_aq_low_quality.valueChanged.connect(self._schedule_save)
        self.inp_aq_low_fps.valueChanged.connect(self._schedule_save)

    def _cam_resolution_code(self) -> str:
        return self.CAM_RESOLUTION_ITEMS[self.inp_cam_resolution.currentText()]

    def _apply_values(self):
        # 连接
        self.inp_mqtt_host.setText(str(self.config.get("mqtt_host", "")))
        self.inp_mqtt_port.setValue(int(self.config.get("mqtt_port", 1883)))
        self.inp_mqtt_topic.setText(str(self.config.get("mqtt_topic", "")))
        self.inp_mqtt_user.setText(str(self.config.get("mqtt_username", "")))
        self.inp_mqtt_pass.setText(str(self.config.get("mqtt_password", "")))
        self.inp_stream_url.setText(str(self.config.get("stream_url", "")))
        self.inp_audio_port.setValue(int(self.config.get("audio_port", 5004)))
        self.inp_stream_port.setValue(
            int(self.config.get("stream_output_port", 8080))
        )
        self.inp_cam_rssi_threshold.setValue(
            float(self.config.get("control_wifi_cam_rssi_reconnect_threshold_dbm", -60))
        )
        self.inp_cam_rssi_delay.setValue(
            int(self.config.get("control_wifi_cam_rssi_reconnect_delay_ms", 10000))
        )
        self.inp_ctrl_rssi_threshold.setValue(
            float(self.config.get("control_wifi_ctrl_rssi_reconnect_threshold_dbm", -75))
        )
        self.inp_ctrl_rssi_delay.setValue(
            int(self.config.get("control_wifi_ctrl_rssi_reconnect_delay_ms", 10000))
        )
        # 控制按键
        self._set_seq_edit(
            self.inp_key_forward, int(self.config.get("control_key_forward", Qt.Key.Key_Up))
        )
        self._set_seq_edit(
            self.inp_key_backward, int(self.config.get("control_key_backward", Qt.Key.Key_Down))
        )
        self._set_seq_edit(
            self.inp_key_left, int(self.config.get("control_key_left", Qt.Key.Key_Left))
        )
        self._set_seq_edit(
            self.inp_key_right, int(self.config.get("control_key_right", Qt.Key.Key_Right))
        )
        self._set_seq_edit(
            self.inp_key_boost, int(self.config.get("control_key_boost", Qt.Key.Key_B))
        )
        self._set_seq_edit(
            self.inp_key_shift_up, int(self.config.get("control_key_shift_up", Qt.Key.Key_Q))
        )
        self._set_seq_edit(
            self.inp_key_shift_down, int(self.config.get("control_key_shift_down", Qt.Key.Key_A))
        )
        self._set_seq_edit(
            self.inp_key_mode, int(self.config.get("control_key_mode_toggle", Qt.Key.Key_M))
        )
        # 控制参数
        self.inp_max_output.setValue(int(self.config.get("control_max_output_pct", 70)))
        self.inp_boost_output.setValue(int(self.config.get("control_boost_output_pct", 90)))
        self.inp_boost_ms.setValue(int(self.config.get("control_boost_max_ms", 8000)))
        self.inp_accel_rate.setValue(int(self.config.get("control_accel_rate_pct", 8)))
        self.inp_brake_strength.setValue(
            int(self.config.get("control_brake_strength_pct", 50))
        )
        self.inp_steering.setValue(float(self.config.get("control_steering_factor", 0.5)))
        self.inp_steering_high.setValue(
            float(self.config.get("control_steering_factor_high", 0.4))
        )
        self.inp_steering_low.setValue(
            float(self.config.get("control_steering_factor_low", 0.4))
        )
        self.inp_steering_reverse.setValue(
            float(self.config.get("control_steering_factor_reverse", 0.5))
        )
        self.inp_steering_inplace.setValue(
            float(self.config.get("control_steering_factor_inplace", 1.5))
        )
        self.inp_upshift.setValue(float(self.config.get("control_upshift_threshold", 0.6)))
        self.inp_downshift.setValue(
            float(self.config.get("control_downshift_threshold", 0.35))
        )
        self.inp_shift_blip.setValue(int(self.config.get("control_shift_blip_pct", 5)))
        self.inp_low_ratio.setValue(float(self.config.get("control_low_ratio", 0.8)))
        self.inp_high_ratio.setValue(float(self.config.get("control_high_ratio", -1.84615)))
        self.inp_m1_low.setValue(int(self.config.get("control_m1_low", 0)))
        self.inp_m1_high.setValue(int(self.config.get("control_m1_high", 90)))
        self.inp_shift_time.setValue(float(self.config.get("control_shift_time_s", 0.08)))
        self.inp_direction_change_hold.setValue(
            int(self.config.get("control_direction_change_hold_ms", 150))
        )
        # 摄像头参数
        res = str(self.config.get("cam_resolution", "1280x720 (HD)"))
        # 兼容旧配置里保存的短名称如 "HD"
        display = res if res in self.CAM_RESOLUTION_ITEMS else None
        if display is None:
            for k, v in self.CAM_RESOLUTION_ITEMS.items():
                if v == res:
                    display = k
                    break
        if display is None:
            display = "1280x720 (HD)"
        idx = self.inp_cam_resolution.findText(display)
        if idx >= 0:
            self.inp_cam_resolution.setCurrentIndex(idx)
        self.inp_cam_quality.setValue(int(self.config.get("cam_quality", 10)))
        self.inp_cam_fps.setValue(int(self.config.get("cam_fps", 30)))
        self.inp_flip_h.setChecked(bool(self.config.get("video_flip_h", False)))
        self.inp_flip_v.setChecked(bool(self.config.get("video_flip_v", False)))
        rot = int(self.config.get("video_rotation", 0))
        rot_map = {0: 0, 90: 1, 180: 2, 270: 3}
        self.inp_rotation.setCurrentIndex(rot_map.get(rot, 0))
        # 自动降画质
        self.inp_auto_quality.setChecked(bool(self.config.get("auto_quality_enabled", False)))
        self.inp_aq_low_rssi.setValue(int(self.config.get("auto_quality_low_rssi", -80)))
        self.inp_aq_recover_rssi.setValue(int(self.config.get("auto_quality_recover_rssi", -70)))
        aq_res = str(self.config.get("auto_quality_low_resolution", "320x240 (QVGA)"))
        aq_idx = self.inp_aq_low_resolution.findText(aq_res)
        if aq_idx >= 0:
            self.inp_aq_low_resolution.setCurrentIndex(aq_idx)
        self.inp_aq_low_quality.setValue(int(self.config.get("auto_quality_low_quality", 30)))
        self.inp_aq_low_fps.setValue(int(self.config.get("auto_quality_low_fps", 10)))

    def _on_ok(self):
        self._save_from_inputs()
        self.apply_requested.emit()
        self.camera_config_changed.emit(
            self._cam_resolution_code(),
            self.inp_cam_quality.value(),
            self.inp_cam_fps.value(),
        )
        self.hide()

    def _schedule_save(self):
        self._save_timer.start(800)

    def _save_from_inputs(self):
        # 连接
        self.config["mqtt_host"] = self.inp_mqtt_host.text().strip()
        self.config["mqtt_port"] = self.inp_mqtt_port.value()
        self.config["mqtt_topic"] = self.inp_mqtt_topic.text().strip()
        self.config["mqtt_username"] = self.inp_mqtt_user.text().strip()
        self.config["mqtt_password"] = self.inp_mqtt_pass.text().strip()
        self.config["stream_url"] = self.inp_stream_url.text().strip()
        self.config["audio_port"] = self.inp_audio_port.value()
        self.config["stream_output_port"] = self.inp_stream_port.value()
        self.config["control_wifi_cam_rssi_reconnect_threshold_dbm"] = self.inp_cam_rssi_threshold.value()
        self.config["control_wifi_cam_rssi_reconnect_delay_ms"] = self.inp_cam_rssi_delay.value()
        self.config["control_wifi_ctrl_rssi_reconnect_threshold_dbm"] = self.inp_ctrl_rssi_threshold.value()
        self.config["control_wifi_ctrl_rssi_reconnect_delay_ms"] = self.inp_ctrl_rssi_delay.value()
        # 控制按键
        self.config["control_key_forward"] = self._key_from_seq_edit(self.inp_key_forward)
        self.config["control_key_backward"] = self._key_from_seq_edit(self.inp_key_backward)
        self.config["control_key_left"] = self._key_from_seq_edit(self.inp_key_left)
        self.config["control_key_right"] = self._key_from_seq_edit(self.inp_key_right)
        self.config["control_key_boost"] = self._key_from_seq_edit(self.inp_key_boost)
        self.config["control_key_shift_up"] = self._key_from_seq_edit(self.inp_key_shift_up)
        self.config["control_key_shift_down"] = self._key_from_seq_edit(
            self.inp_key_shift_down
        )
        self.config["control_key_mode_toggle"] = self._key_from_seq_edit(self.inp_key_mode)
        # 控制参数
        self.config["control_max_output_pct"] = self.inp_max_output.value()
        self.config["control_boost_output_pct"] = self.inp_boost_output.value()
        self.config["control_boost_max_ms"] = self.inp_boost_ms.value()
        self.config["control_accel_rate_pct"] = self.inp_accel_rate.value()
        self.config["control_brake_strength_pct"] = self.inp_brake_strength.value()
        self.config["control_steering_factor"] = self.inp_steering.value()
        self.config["control_steering_factor_high"] = self.inp_steering_high.value()
        self.config["control_steering_factor_low"] = self.inp_steering_low.value()
        self.config["control_steering_factor_reverse"] = self.inp_steering_reverse.value()
        self.config["control_steering_factor_inplace"] = self.inp_steering_inplace.value()
        self.config["control_upshift_threshold"] = self.inp_upshift.value()
        self.config["control_downshift_threshold"] = self.inp_downshift.value()
        self.config["control_shift_blip_pct"] = self.inp_shift_blip.value()
        self.config["control_low_ratio"] = self.inp_low_ratio.value()
        self.config["control_high_ratio"] = self.inp_high_ratio.value()
        self.config["control_m1_low"] = self.inp_m1_low.value()
        self.config["control_m1_high"] = self.inp_m1_high.value()
        self.config["control_shift_time_s"] = self.inp_shift_time.value()
        self.config["control_direction_change_hold_ms"] = self.inp_direction_change_hold.value()
        # 伺服电机
        self.config["servo_count"] = self.inp_servo_count.value()
        count = self.inp_servo_count.value()
        for idx in range(1, count + 1):
            name_key = f"servo_{idx}_name"
            name_widget = self._servo_inputs.get(name_key)
            if name_widget:
                self.config[name_key] = name_widget.text().strip()
            seg_count_widget = self._servo_inputs.get(f"servo_{idx}_seg_count")
            seg_count = seg_count_widget.value() if seg_count_widget else 2
            self.config[f"servo_{idx}_seg_count"] = seg_count
            idle_widget = self._servo_inputs.get(f"servo_{idx}_idle_level")
            self.config[f"servo_{idx}_idle_level"] = idle_widget.value() if idle_widget else 0
            for seg in range(1, seg_count + 1):
                key_widget = self._servo_inputs.get(f"servo_{idx}_seg{seg}_key")
                if key_widget:
                    self.config[f"servo_{idx}_seg{seg}_key"] = self._key_from_seq_edit(key_widget)
                level_widget = self._servo_inputs.get(f"servo_{idx}_seg{seg}_level")
                if level_widget:
                    self.config[f"servo_{idx}_seg{seg}_level"] = level_widget.value()
                time_widget = self._servo_inputs.get(f"servo_{idx}_seg{seg}_time_s")
                if time_widget:
                    self.config[f"servo_{idx}_seg{seg}_time_s"] = time_widget.value()
                mode_widget = self._servo_inputs.get(f"servo_{idx}_seg{seg}_mode")
                if mode_widget:
                    self.config[f"servo_{idx}_seg{seg}_mode"] = "hold" if mode_widget.currentIndex() == 0 else "toggle"
        # 摄像头参数
        self.config["cam_resolution"] = self.inp_cam_resolution.currentText()
        self.config["cam_quality"] = self.inp_cam_quality.value()
        self.config["cam_fps"] = self.inp_cam_fps.value()
        self.config["video_flip_h"] = self.inp_flip_h.isChecked()
        self.config["video_flip_v"] = self.inp_flip_v.isChecked()
        rot_text = self.inp_rotation.currentText()
        self.config["video_rotation"] = int(rot_text.replace("°", ""))
        # 自动降画质
        self.config["auto_quality_enabled"] = self.inp_auto_quality.isChecked()
        self.config["auto_quality_low_rssi"] = self.inp_aq_low_rssi.value()
        self.config["auto_quality_recover_rssi"] = self.inp_aq_recover_rssi.value()
        self.config["auto_quality_low_resolution"] = self.inp_aq_low_resolution.currentText()
        self.config["auto_quality_low_quality"] = self.inp_aq_low_quality.value()
        self.config["auto_quality_low_fps"] = self.inp_aq_low_fps.value()
        save_config_silent(self.config)

    def resizeEvent(self, event):
        parent = self.parentWidget()
        if parent:
            self.setGeometry(parent.rect())
            cw = int(parent.width() * 0.7)
            ch = int(parent.height() * 0.7)
            cx = (parent.width() - cw) // 2
            cy = (parent.height() - ch) // 2
            self.container.setGeometry(cx, cy, cw, ch)
        super().resizeEvent(event)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_NAME)
        self.resize(1100, 800)

        self.config = load_config()

        self.mqtt_thread: Optional[MqttThread] = None
        self.audio_thread: Optional[AudioUdpThread] = None
        self.audio_player: Optional[AudioPlayer] = None
        self.overlay_server: Optional[OverlayServerThread] = None
        self.cam_thread: Optional[CamProxyThread] = None

        self._mqtt_connected = False
        self.latest_telemetry: Dict[str, Any] = {}
        self.car_telemetry: Dict[str, Any] = {}
        self.camera_telemetry: Dict[str, Any] = {}
        self._auto_quality_degraded = False
        self._auto_quality_low_since = 0
        self._auto_quality_ok_since = 0
        self.controller = TankController(self.config)
        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self._control_tick)
        self._action_keys_handled: set[int] = set()

        self._build_ui()
        self.statusBar().showMessage("就绪")

        QApplication.instance().installEventFilter(self)
        QTimer.singleShot(500, self._start_connections)
        self.control_timer.start(50)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 顶部工具栏
        top = QHBoxLayout()
        top.setContentsMargins(4, 4, 4, 4)
        top.setSpacing(6)

        self.btn_settings = QPushButton("⚙ 设置")
        self.btn_settings.setFixedWidth(90)
        self.btn_settings.clicked.connect(self._toggle_settings)
        top.addWidget(self.btn_settings)

        self.lbl_status = QLabel("MQTT: 未连接")
        top.addWidget(self.lbl_status)
        top.addStretch()

        self.btn_audio = QPushButton("🔊 音频开")
        self.btn_audio.setCheckable(True)
        self.btn_audio.setChecked(bool(self.config.get("audio_enabled", True)))
        self.btn_audio.clicked.connect(self._on_audio_toggle)
        top.addWidget(self.btn_audio)

        self.slider_volume = QSlider(Qt.Orientation.Horizontal)
        self.slider_volume.setRange(0, 100)
        self.slider_volume.setValue(int(self.config.get("volume", 80)))
        self.slider_volume.setFixedWidth(120)
        self.slider_volume.valueChanged.connect(self._on_volume_changed)
        top.addWidget(QLabel("音量"))
        top.addWidget(self.slider_volume)

        layout.addLayout(top)

        # 网页显示区域
        self.web_view = QWebEngineView()
        self.web_view.setStyleSheet("background-color: black;")
        layout.addWidget(self.web_view, stretch=1)

        # 半透明设置界面
        self.settings_overlay = SettingsOverlay(central, self.config)
        self.settings_overlay.apply_requested.connect(self._apply_config_and_reload)
        self.settings_overlay.camera_config_changed.connect(self._publish_camera_config)

    def _toggle_settings(self):
        if self.settings_overlay.isVisible():
            self.settings_overlay.hide()
        else:
            self.settings_overlay.show()
            self.settings_overlay.raise_()
            self.settings_overlay.setFocus()

    def _apply_config_and_reload(self):
        old_config = self.config
        self.config = load_config()
        self.controller.config = self.config
        self.settings_overlay.config = self.config
        self.settings_overlay._apply_values()

        conn_keys = [
            "mqtt_host", "mqtt_port", "mqtt_topic", "mqtt_username",
            "stream_url", "audio_port", "stream_output_port",
        ]
        old_conn = {k: old_config.get(k) for k in conn_keys}
        new_conn = {k: self.config.get(k) for k in conn_keys}
        if old_conn != new_conn:
            self._start_connections()
        elif self.cam_thread:
            self.cam_thread.set_transform(
                bool(self.config.get("video_flip_h", False)),
                bool(self.config.get("video_flip_v", False)),
                int(self.config.get("video_rotation", 0)),
            )
        if not self.config.get("auto_quality_enabled", False):
            self._auto_quality_degraded = False
        self._publish_control_rssi_settings()

    def _publish_control_rssi_settings(self):
        if not self.mqtt_thread:
            return
        payload = json.dumps(
            {
                "rssi_threshold": int(
                    self.config.get(
                        "control_wifi_ctrl_rssi_reconnect_threshold_dbm", -75
                    )
                ),
                "rssi_delay": int(
                    self.config.get(
                        "control_wifi_ctrl_rssi_reconnect_delay_ms", 10000
                    )
                ),
            }
        )
        self.mqtt_thread.publish("tank/cmd", payload)

    def _publish_camera_config(self, resolution: str, quality: int, fps: int):
        if not self.mqtt_thread:
            return
        payload = json.dumps(
            {
                "resolution": resolution,
                "quality": quality,
                "fps": fps,
                "rssi_threshold": int(
                    self.config.get(
                        "control_wifi_cam_rssi_reconnect_threshold_dbm", -70
                    )
                ),
                "rssi_delay": int(
                    self.config.get("control_wifi_cam_rssi_reconnect_delay_ms", 10000)
                ),
            }
        )
        self.mqtt_thread.publish("tank/cmd/camera", payload)

    def _check_auto_quality(self):
        if not self.config.get("auto_quality_enabled", False):
            return
        monrssi = self.camera_telemetry.get("monrssi")
        if monrssi is None:
            return
        now = int(time.time() * 1000)
        low_threshold = int(self.config.get("auto_quality_low_rssi", -80))
        recover_threshold = int(self.config.get("auto_quality_recover_rssi", -70))
        debounce_ms = 3000

        if not self._auto_quality_degraded:
            if monrssi < low_threshold:
                if self._auto_quality_low_since == 0:
                    self._auto_quality_low_since = now
                elif now - self._auto_quality_low_since >= debounce_ms:
                    self._apply_auto_quality(degrade=True)
                    self._auto_quality_low_since = 0
            else:
                self._auto_quality_low_since = 0
        else:
            if monrssi >= recover_threshold:
                if self._auto_quality_ok_since == 0:
                    self._auto_quality_ok_since = now
                elif now - self._auto_quality_ok_since >= debounce_ms:
                    self._apply_auto_quality(degrade=False)
                    self._auto_quality_ok_since = 0
            else:
                self._auto_quality_ok_since = 0

    def _apply_auto_quality(self, degrade: bool):
        if degrade:
            self._auto_quality_degraded = True
            resolution = self.config.get("auto_quality_low_resolution", "320x240 (QVGA)")
            quality = int(self.config.get("auto_quality_low_quality", 30))
            fps = int(self.config.get("auto_quality_low_fps", 10))
            self.statusBar().showMessage(
                f"[自动降画质] 信号弱，已降至 {resolution} Q{quality} F{fps}"
            )
        else:
            self._auto_quality_degraded = False
            resolution = self.config.get("cam_resolution", "1280x720 (HD)")
            quality = int(self.config.get("cam_quality", 10))
            fps = int(self.config.get("cam_fps", 30))
            self.statusBar().showMessage("[自动降画质] 信号恢复，已恢复原画质")
        self._publish_camera_config(resolution, quality, fps)

    def _reload_web_view(self):
        port = int(
            self.config.get("stream_output_port", DEFAULT_CONFIG["stream_output_port"])
        )
        url = QUrl(f"http://localhost:{port}/")
        self.web_view.setUrl(url)

    def _on_volume_changed(self, value: int):
        self.config["volume"] = value
        save_config_silent(self.config)
        if self.audio_player:
            self.audio_player.set_volume(value)

    def _on_audio_toggle(self, checked: bool):
        self.config["audio_enabled"] = checked
        save_config_silent(self.config)
        self.btn_audio.setText("🔊 音频开" if checked else "🔇 音频关")
        if self.audio_player:
            self.audio_player.set_enabled(checked)

    def _start_connections(self):
        self._stop_connections()

        host = self.config.get("mqtt_host", DEFAULT_CONFIG["mqtt_host"])
        port = int(self.config.get("mqtt_port", DEFAULT_CONFIG["mqtt_port"]))
        topic = self.config.get("mqtt_topic", DEFAULT_CONFIG["mqtt_topic"])
        monitor_topic = self.config.get(
            "mqtt_monitor_topic", DEFAULT_CONFIG["mqtt_monitor_topic"]
        )
        user = self.config.get("mqtt_username", "")
        pwd = self.config.get("mqtt_password", "")
        topics = resolve_mqtt_subscription_topics(topic, monitor_topic)
        self.mqtt_thread = MqttThread(host, port, topics, user, pwd)
        self.mqtt_thread.telemetry.connect(self._on_telemetry)
        self.mqtt_thread.status.connect(self.statusBar().showMessage)
        self.mqtt_thread.connected.connect(self._on_mqtt_connected)
        self.mqtt_thread.start()

        audio_port = int(self.config.get("audio_port", DEFAULT_CONFIG["audio_port"]))
        self.audio_thread = AudioUdpThread(audio_port)
        self.audio_thread.error.connect(self.statusBar().showMessage)
        self.audio_thread.start()

        self.audio_player = AudioPlayer(self.audio_thread.queue)
        self.audio_player.set_volume(self.slider_volume.value())
        self.audio_player.set_enabled(self.btn_audio.isChecked())

        stream_port = int(
            self.config.get("stream_output_port", DEFAULT_CONFIG["stream_output_port"])
        )
        stream_url = self.config.get("stream_url", DEFAULT_CONFIG["stream_url"])
        self.overlay_server = OverlayServerThread(stream_port, stream_url)
        self.overlay_server.start()

        self.cam_thread = CamProxyThread(
            stream_url, self.overlay_server.server.buffer, quality=80
        )
        self.cam_thread.set_transform(
            bool(self.config.get("video_flip_h", False)),
            bool(self.config.get("video_flip_v", False)),
            int(self.config.get("video_rotation", 0)),
        )
        self.cam_thread.error.connect(self.statusBar().showMessage)
        self.cam_thread.start()

        self.statusBar().showMessage(
            f"本地网页已启动: http://<本机IP>:{stream_port}/"
        )

        self._reload_web_view()

    def _stop_connections(self):
        if self.mqtt_thread:
            self.mqtt_thread.stop()
            self.mqtt_thread = None
        if self.audio_player:
            self.audio_player.close()
            self.audio_player = None
        if self.audio_thread:
            self.audio_thread.stop()
            self.audio_thread = None
        if self.cam_thread:
            self.cam_thread.stop()
            self.cam_thread = None
        if self.overlay_server:
            self.overlay_server.stop()
            self.overlay_server = None

    def _on_mqtt_connected(self, connected: bool):
        self._mqtt_connected = connected
        self._update_status()

    def _update_status(self):
        mqtt_text = "已连接" if self._mqtt_connected else "未连接"
        cs = self.latest_telemetry
        gear = cs.get("gear", "low")
        mode = cs.get("mode", "manual")
        limit = cs.get("limit_pct", 70)
        boost = cs.get("boost")
        boost_rem = cs.get("boost_remaining_ms", 0)
        boost_text = f"B:{(boost_rem + 999) // 1000}s" if boost_rem > 0 else "B:0s"
        cooldown = cs.get("cooldown_ms", 0)
        cd_text = f" CD:{cooldown//1000 + 1}s" if cooldown > 0 else ""
        ctrl_rssi = cs.get("ctrlrssi", "--")
        mon_rssi = cs.get("monrssi", "--")
        m2_level = cs.get("m2", 0)
        m3_level = cs.get("m3", 0)
        self.lbl_status.setText(
            f"MQTT: {mqtt_text} | {gear.upper()} | {mode.upper()} | 限{limit}% {boost_text}{cd_text} | "
            f"CR:{ctrl_rssi} MR:{mon_rssi} | M2:{m2_level} M3:{m3_level}"
        )

    def _on_telemetry(self, topic: str, data: Dict[str, Any]):
        cfg_topic = self.config.get("mqtt_topic", DEFAULT_CONFIG["mqtt_topic"])
        monitor_topic = self.config.get(
            "mqtt_monitor_topic", DEFAULT_CONFIG["mqtt_monitor_topic"]
        )

        if topic == monitor_topic:
            self.camera_telemetry = data
            self._check_auto_quality()
        elif topic == cfg_topic or topic == LEGACY_CONTROL_TELEMETRY_TOPIC:
            self.car_telemetry = data
        else:
            # 某些旧拓扑/测试环境下，主题名字不完全一致时做兜底保守处理。
            if "monitor" in topic:
                self.camera_telemetry = data
                self._check_auto_quality()
            else:
                self.car_telemetry = data

        self.latest_telemetry = {
            **self.car_telemetry,
            **self.camera_telemetry,
            **self.controller.control_state,
        }
        if self.overlay_server:
            self.overlay_server.set_telemetry(self.latest_telemetry)

    def _control_tick(self):
        now_ms = int(time.time() * 1000)
        cmd = self.controller.tick(now_ms)
        if self.mqtt_thread and self.mqtt_thread.publish(
            "tank/cmd", json.dumps(cmd)
        ):
            pass
        if self.overlay_server:
            self.latest_telemetry = {
                **self.car_telemetry,
                **self.camera_telemetry,
                **self.controller.control_state,
            }
            self.overlay_server.set_telemetry(self.latest_telemetry)
        self._update_status()

    def eventFilter(self, obj, event):
        if self.settings_overlay.isVisible():
            return False
        if event.type() == QEvent.Type.KeyPress:
            key = event.key()
            # 方向键保持按下状态
            arrow_keys = {
                int(self.config.get("control_key_forward", Qt.Key.Key_Up)),
                int(self.config.get("control_key_backward", Qt.Key.Key_Down)),
                int(self.config.get("control_key_left", Qt.Key.Key_Left)),
                int(self.config.get("control_key_right", Qt.Key.Key_Right)),
            }
            if key in arrow_keys:
                self.controller.set_key(key, True)
                return True
            # 动作键只响应一次按下，防止长按自动重复
            action_keys = {
                int(self.config.get("control_key_boost", Qt.Key.Key_B)),
                int(self.config.get("control_key_shift_up", Qt.Key.Key_Q)),
                int(self.config.get("control_key_shift_down", Qt.Key.Key_A)),
                int(self.config.get("control_key_mode_toggle", Qt.Key.Key_M)),
            }
            # 伺服电机段按键（按下时触发）
            servo_keys_handled = False
            servo_count = int(self.config.get("servo_count", 2))
            for idx in range(1, servo_count + 1):
                seg_count = int(self.config.get(f"servo_{idx}_seg_count", 2))
                for seg in range(1, seg_count + 1):
                    seg_key = int(self.config.get(f"servo_{idx}_seg{seg}_key", 0))
                    if key == seg_key and seg_key != 0:
                        mode = str(self.config.get(f"servo_{idx}_seg{seg}_mode", "toggle"))
                        if mode == "toggle":
                            if key in self._action_keys_handled:
                                return True
                            self._action_keys_handled.add(key)
                        self.controller.handle_servo_segment(idx, seg, int(time.time() * 1000), pressed=True)
                        servo_keys_handled = True
            if servo_keys_handled:
                return True
            if key in action_keys:
                if key not in self._action_keys_handled:
                    self._action_keys_handled.add(key)
                    if key == int(self.config.get("control_key_boost", Qt.Key.Key_B)):
                        key_fwd = int(self.config.get("control_key_forward", Qt.Key.Key_Up))
                        key_bwd = int(self.config.get("control_key_backward", Qt.Key.Key_Down))
                        throttle = 0.0
                        if self.controller.is_pressed(key_fwd):
                            throttle = 1.0
                        elif self.controller.is_pressed(key_bwd):
                            throttle = -self.config.get("control_brake_strength_pct", 50) / 100.0
                        self.controller.toggle_boost(int(time.time() * 1000), throttle)
                    elif key == int(self.config.get("control_key_shift_up", Qt.Key.Key_Q)):
                        self.controller.shift_up()
                    elif key == int(self.config.get("control_key_shift_down", Qt.Key.Key_A)):
                        self.controller.shift_down()
                    elif key == int(self.config.get("control_key_mode_toggle", Qt.Key.Key_M)):
                        self.controller.toggle_mode()
                return True
        elif event.type() == QEvent.Type.KeyRelease:
            key = event.key()
            arrow_keys = {
                int(self.config.get("control_key_forward", Qt.Key.Key_Up)),
                int(self.config.get("control_key_backward", Qt.Key.Key_Down)),
                int(self.config.get("control_key_left", Qt.Key.Key_Left)),
                int(self.config.get("control_key_right", Qt.Key.Key_Right)),
            }
            if key in arrow_keys:
                self.controller.set_key(key, False)
                return True
            # 伺服电机按住触发：释放时回到不触发
            servo_count = int(self.config.get("servo_count", 2))
            for idx in range(1, servo_count + 1):
                seg_count = int(self.config.get(f"servo_{idx}_seg_count", 2))
                for seg in range(1, seg_count + 1):
                    seg_key = int(self.config.get(f"servo_{idx}_seg{seg}_key", 0))
                    if key == seg_key and seg_key != 0:
                        mode = str(self.config.get(f"servo_{idx}_seg{seg}_mode", "toggle"))
                        if mode == "hold":
                            self.controller.handle_servo_segment(idx, seg, int(time.time() * 1000), pressed=False)
            self._action_keys_handled.discard(key)
        return False

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, "settings_overlay") and self.settings_overlay.isVisible():
            self.settings_overlay.resizeEvent(event)

    def closeEvent(self, event):
        self.control_timer.stop()
        self._stop_connections()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
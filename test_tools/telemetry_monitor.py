# Copyright (c) 2026 eric6227
# Released under the MIT License. See LICENSE file in the project root for full text.
"""
玩具坦克遥测监控器（TUI）

左右并排两个原始表格：
  - 左：控制板遥测 tank/telemetry/control
  - 右：摄像头遥测 tank/telemetry/monitor

用法：
    python telemetry_monitor.py
"""

import csv
import json
import os
import queue
import sys
import threading
from datetime import datetime

import paho.mqtt.client as mqtt
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.widgets import (
    Button,
    DataTable,
    Input,
    Static,
)

# 默认配置
DEFAULT_HOST = "192.168.2.45"
DEFAULT_PORT = 1883
DEFAULT_TOPIC = "tank/telemetry/control"
DEFAULT_MONITOR_TOPIC = "tank/telemetry/monitor"

# UI 常量
MAX_TABLE_ROWS = 200       # 每个表格中最多保留多少行实时数据
MAX_HISTORY = 10000000     # 导出历史最多保留多少条

CONTROL_COLUMNS = [
    "时间",
    "电池(V)",
    "左电机",
    "右电机",
    "M1",
    "M2",
    "M3",
    "RSSI",
    "运行时长",
]

CAMERA_COLUMNS = [
    "时间",
    "RSSI",
    "运行时长",
]


def create_mqtt_client() -> mqtt.Client:
    try:
        return mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1)
    except AttributeError:
        return mqtt.Client()


class MqttThread(threading.Thread):
    def __init__(self, host: str, port: int, topics: list, message_queue: queue.Queue):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.topics = topics
        self.queue = message_queue
        self._running = True
        self.client: mqtt.Client | None = None

    def stop(self):
        self._running = False
        if self.client:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            for t in self.topics:
                client.subscribe(t)
            self.queue.put({"_connected": True})
        else:
            self.queue.put({"_status": f"连接失败，返回码: {rc}"})

    def _on_disconnect(self, client, userdata, rc):
        self.queue.put({"_connected": False})

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
            data["_topic"] = msg.topic
            data["_ts"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            self.queue.put(data)
        except Exception as e:
            self.queue.put({"_status": f"解析错误: {e}"})

    def run(self):
        import time

        while self._running:
            try:
                self.client = create_mqtt_client()
                self.client.on_connect = self._on_connect
                self.client.on_disconnect = self._on_disconnect
                self.client.on_message = self._on_message
                self.client.connect(self.host, self.port, keepalive=60)
                # 先跑几次 loop 完成 TCP + MQTT 握手，不要刚 connect 就判断 is_connected
                for _ in range(100):
                    if not self._running:
                        break
                    self.client.loop(timeout=0.05)
                    if self.client.is_connected():
                        break
                # 连接建立后持续 loop，断开时退出重连
                while self._running and self.client.is_connected():
                    self.client.loop(timeout=0.1)
            except Exception as e:
                self.queue.put({"_status": f"MQTT 错误: {e}"})
            time.sleep(3)


class TelemetryMonitorApp(App):
    CSS = """
    #controls {
        height: auto;
        padding: 1 2;
    }
    #controls Input {
        width: 25;
        margin: 0 1;
    }
    #controls Button {
        margin: 0 1;
    }
    #status {
        height: auto;
        padding: 0 2;
        color: $text-muted;
    }
    #main {
        padding: 0 2 1 2;
    }
    .panel {
        width: 1fr;
        height: 1fr;
        border: solid $primary;
    }
    .panel-title {
        height: auto;
        padding: 0 1;
        color: $text;
        text-style: bold;
    }
    """

    BINDINGS = [("q", "quit", "退出")]

    def __init__(self):
        super().__init__()
        self.host = DEFAULT_HOST
        self.port = DEFAULT_PORT
        self.topic = DEFAULT_TOPIC
        self.monitor_topic = DEFAULT_MONITOR_TOPIC
        self.mqtt_thread: MqttThread | None = None
        self.msg_queue: queue.Queue = queue.Queue()
        self._connected = False
        self._reconnect_timer = None
        self._control_history: list = []
        self._camera_history: list = []
        self._control_row_keys: list = []
        self._camera_row_keys: list = []

    def compose(self) -> ComposeResult:
        with Vertical():
            yield Static(
                f"控制板: {self.topic}    摄像头: {self.monitor_topic}",
                id="status",
            )

            with Horizontal(id="controls"):
                yield Input(value=self.host, placeholder="MQTT 服务器地址", id="host")
                yield Input(value=str(self.port), placeholder="端口", id="port")
                yield Input(value=self.topic, placeholder="控制板主题", id="topic")
                yield Button("连接", id="connect", variant="success")
                yield Button("断开", id="disconnect", variant="error")
                yield Button("导出 CSV", id="export_csv")
                yield Button("导出 JSON", id="export_json")
                yield Button("清空", id="clear")

            with Horizontal(id="main"):
                with Vertical(classes="panel"):
                    yield Static("控制板遥测", classes="panel-title")
                    table_ctrl = DataTable(id="table_ctrl")
                    table_ctrl.add_columns(*CONTROL_COLUMNS)
                    yield table_ctrl

                with Vertical(classes="panel"):
                    yield Static("摄像头遥测", classes="panel-title")
                    table_cam = DataTable(id="table_cam")
                    table_cam.add_columns(*CAMERA_COLUMNS)
                    yield table_cam

    def on_mount(self):
        self.set_interval(0.1, self._poll_queue)

    def _fmt(self, value):
        if value is None:
            return "--"
        if isinstance(value, float):
            return f"{value:.2f}"
        return str(value)

    def _control_row(self, data: dict) -> tuple:
        return (
            data.get("_ts", ""),
            self._fmt(data.get("vbat")),
            self._fmt(data.get("L")),
            self._fmt(data.get("R")),
            self._fmt(data.get("M1")),
            self._fmt(data.get("M2")),
            self._fmt(data.get("M3")),
            self._fmt(data.get("ctrlrssi")),
            self._fmt(data.get("uptime")),
        )

    def _camera_row(self, data: dict) -> tuple:
        return (
            data.get("_ts", ""),
            self._fmt(data.get("monrssi")),
            self._fmt(data.get("uptime")),
        )

    def _add_row_limited(self, table: DataTable, row: tuple, keys: list):
        key = table.add_row(*row)
        keys.append(key)
        if len(keys) > MAX_TABLE_ROWS:
            old_key = keys.pop(0)
            table.remove_row(old_key)
        table.scroll_end(animate=False)

    def _poll_queue(self):
        while not self.msg_queue.empty():
            try:
                data = self.msg_queue.get_nowait()
            except queue.Empty:
                break

            if "_connected" in data:
                self._connected = data["_connected"]
                self._set_status("已连接" if self._connected else "已断开")
                continue
            if "_status" in data:
                self._set_status(data["_status"])
                continue

            topic = data.pop("_topic", "")
            ts = data.pop("_ts", "")
            data["_ts"] = ts

            if topic == self.topic:
                table = self.query_one("#table_ctrl", DataTable)
                self._add_row_limited(
                    table, self._control_row(data), self._control_row_keys
                )
                self._control_history.append(data)
            elif topic == self.monitor_topic:
                table = self.query_one("#table_cam", DataTable)
                self._add_row_limited(
                    table, self._camera_row(data), self._camera_row_keys
                )
                self._camera_history.append(data)

        if len(self._control_history) > MAX_HISTORY:
            self._control_history = self._control_history[-MAX_HISTORY:]
        if len(self._camera_history) > MAX_HISTORY:
            self._camera_history = self._camera_history[-MAX_HISTORY:]

    def _set_status(self, text: str):
        self.query_one("#status", Static).update(
            f"控制板: {self.topic}    摄像头: {self.monitor_topic}    状态: {text}"
        )

    def _start_mqtt(self):
        self._stop_mqtt(wait=True)
        # 清空残留消息，避免旧状态干扰新连接
        while not self.msg_queue.empty():
            try:
                self.msg_queue.get_nowait()
            except queue.Empty:
                break

        self.host = self.query_one("#host", Input).value.strip() or DEFAULT_HOST
        try:
            self.port = int(self.query_one("#port", Input).value)
        except ValueError:
            self.port = DEFAULT_PORT
        self.topic = self.query_one("#topic", Input).value.strip() or DEFAULT_TOPIC

        self._connected = False
        self._set_status("正在连接...")
        self.mqtt_thread = MqttThread(
            self.host, self.port, [self.topic, self.monitor_topic], self.msg_queue
        )
        self.mqtt_thread.start()

    def _stop_mqtt(self, wait: bool = False):
        if self.mqtt_thread:
            self.mqtt_thread.stop()
            if wait:
                self.mqtt_thread.join(timeout=2)
            self.mqtt_thread = None
        self._connected = False
        self._set_status("未连接")

    def action_export_csv(self):
        if not self._control_history and not self._camera_history:
            self._set_status("没有数据可导出")
            return
        filename = f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            with open(filename, "w", newline="", encoding="utf-8-sig") as f:
                all_data = []
                for r in self._control_history:
                    row = dict(r)
                    row["source"] = "control"
                    all_data.append(row)
                for r in self._camera_history:
                    row = dict(r)
                    row["source"] = "camera"
                    all_data.append(row)
                all_data.sort(key=lambda x: x.get("_ts", ""))

                if not all_data:
                    self._set_status("没有数据可导出")
                    return
                keys = set()
                for r in all_data:
                    keys.update(r.keys())
                keys.discard("_ts")
                keys = ["ts", "source"] + sorted(keys)
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                for r in all_data:
                    row = dict(r)
                    row["ts"] = row.pop("_ts", "")
                    writer.writerow(row)
            self._set_status(f"已导出 CSV: {os.path.abspath(filename)}")
        except Exception as e:
            self._set_status(f"导出 CSV 失败: {e}")

    def action_export_json(self):
        if not self._control_history and not self._camera_history:
            self._set_status("没有数据可导出")
            return
        filename = f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        try:
            all_data = []
            for r in self._control_history:
                row = dict(r)
                row["source"] = "control"
                all_data.append(row)
            for r in self._camera_history:
                row = dict(r)
                row["source"] = "camera"
                all_data.append(row)
            all_data.sort(key=lambda x: x.get("_ts", ""))
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(all_data, f, ensure_ascii=False, indent=2)
            self._set_status(f"已导出 JSON: {os.path.abspath(filename)}")
        except Exception as e:
            self._set_status(f"导出 JSON 失败: {e}")

    def action_clear(self):
        self._control_history.clear()
        self._camera_history.clear()
        self._control_row_keys.clear()
        self._camera_row_keys.clear()
        self.query_one("#table_ctrl", DataTable).clear()
        self.query_one("#table_cam", DataTable).clear()
        self._set_status("数据已清空")

    def on_button_pressed(self, event: Button.Pressed):
        button_id = event.button.id
        if button_id == "connect":
            self._start_mqtt()
        elif button_id == "disconnect":
            self._stop_mqtt()
        elif button_id == "export_csv":
            self.action_export_csv()
        elif button_id == "export_json":
            self.action_export_json()
        elif button_id == "clear":
            self.action_clear()


def main():
    app = TelemetryMonitorApp()
    app.run()


if __name__ == "__main__":
    main()
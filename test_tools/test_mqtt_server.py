# Copyright (c) 2026 eric6227
# Released under the MIT License. See LICENSE file in the project root for full text.
"""
MQTT 连通性测试脚本（Windows / Linux / macOS 均可使用）
使用方法：
  1. pip install paho-mqtt
  2. 根据需要修改下面的 HOST、PORT、TOPIC、USERNAME、PASSWORD 变量
  3. 运行 python test_mqtt.py
如果脚本最终打印 "✅ MQTT 通信测试通过！"，说明服务器工作正常。
"""

import time
import paho.mqtt.client as mqtt

# ============ 请根据实际情况修改以下参数 ============
HOST = "192.168.2.45"          # MQTT 服务器地址
PORT = 1883                 # 端口
TOPIC = "test/python"       # 测试主题
USERNAME = None             # 如果启用了认证，填入用户名；否则保持 None
PASSWORD = None             # 如果启用了认证，填入密码；否则保持 None
TIMEOUT = 5                 # 等待接收消息的超时时间（秒）
# ==================================================

received_message = False

def on_connect(client, userdata, flags, rc):
    """连接成功后的回调"""
    if rc == 0:
        print(f"✅ 已连接到 {HOST}:{PORT}")
        # 订阅测试主题
        client.subscribe(TOPIC)
        print(f"📥 已订阅主题: {TOPIC}")
        # 立即发布一条测试消息
        client.publish(TOPIC, "hello from python")
        print(f"📤 已发布消息到 {TOPIC}")
    else:
        print(f"❌ 连接失败，返回码: {rc}")

def on_message(client, userdata, msg):
    """收到消息后的回调"""
    global received_message
    payload = msg.payload.decode()
    print(f"📨 收到消息: topic={msg.topic}, payload={payload}")
    received_message = True
    # 收到消息后断开连接
    client.disconnect()

def main():
    client = mqtt.Client()
    
    # 设置回调
    client.on_connect = on_connect
    client.on_message = on_message
    
    # 如果设置了认证信息
    if USERNAME and PASSWORD:
        client.username_pw_set(USERNAME, PASSWORD)
    
    try:
        client.connect(HOST, PORT, keepalive=60)
        # 启动网络循环（非阻塞）
        client.loop_start()
        
        # 等待直到收到消息或超时
        start_time = time.time()
        while not received_message and (time.time() - start_time) < TIMEOUT:
            time.sleep(0.1)
        
        if received_message:
            print("\n✅ MQTT 通信测试通过！")
        else:
            print(f"\n❌ 超时 {TIMEOUT} 秒，未收到任何消息。")
            print("   可能原因：服务器未运行、端口不通、认证失败或网络问题。")
        
        client.loop_stop()
        
    except Exception as e:
        print(f"❌ 连接异常: {e}")

if __name__ == "__main__":
    main()
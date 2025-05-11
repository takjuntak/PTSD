import paho.mqtt.client as mqtt
import json
import requests
import math

def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    percentage = math.floor(payload.get("percentage", 0))

    print(f"Received battery percentage: {percentage}")
    try:
        # FastAPI에 배터리 상태 전송
        requests.post("http://192.168.228.30:8000/api/battery-state", json={"percentage": percentage})
        print("Sent to FastAPI")

    except Exception as e:
        print(f"Failed to send: {e}")

def start_mqtt_loop():
    client = mqtt.Client()
    client.on_message = on_message
    client.connect("192.168.228.76", 1883, 60)  # broker_ip에 맞게 수정
    client.subscribe("mqtt/battery")
    client.loop_forever()

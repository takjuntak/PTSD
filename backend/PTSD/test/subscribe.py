import paho.mqtt.client as mqtt

TOPIC = "iot/notification"

def on_connect(client, userdata, flags, rc):
    print(f"[SUBSCRIBE] 연결 성공 (코드 {rc})")
    client.subscribe(TOPIC)
    print(f"[SUBSCRIBE] '{TOPIC}' 토픽 구독 시작")

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print(f"[SUBSCRIBE] 수신된 메시지: {message}")

mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

mqtt_client.connect("localhost", 1883, 60)
mqtt_client.loop_forever()

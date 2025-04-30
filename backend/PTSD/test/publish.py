import time
import paho.mqtt.client as mqtt

# MQTT 서버 연결
mqtt_client = mqtt.Client()
mqtt_client.connect("localhost", 1883, 60)

counter = 1  # 메시지 번호 초기화

def send_message():
    global counter
    topic = "iot/notification"
    message = f"새로운 알림 메시지 #{counter}입니다!"
    mqtt_client.publish(topic, message)
    print(f"[PUBLISH] 메시지 발행: {message}")
    counter += 1

# 백그라운드 루프 시작
mqtt_client.loop_start()

# 주기적으로 메시지 발행
try:
    while True:
        send_message()
        time.sleep(1)
except KeyboardInterrupt:
    print("발행 중단됨")
    mqtt_client.loop_stop()

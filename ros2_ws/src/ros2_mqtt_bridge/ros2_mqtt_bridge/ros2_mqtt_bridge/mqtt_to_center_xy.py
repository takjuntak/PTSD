import paho.mqtt.client as mqtt 
import json
import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# 환경변수 설정
MQTT_BROKER = os.getenv('MQTT_BROKER')
MQTT_PORT = int(os.getenv('MQTT_PORT', '1883'))
MQTT_TOPIC = os.getenv('MQTT_DETECTION_TOPIC')
MQTT_KEEPALIVE = int(os.getenv('MQTT_KEEPALIVE', '60'))

# 검증
if not MQTT_BROKER:
    print('[ERROR] MQTT_BROKER 환경변수가 설정되지 않았습니다!')
    exit(1)

if not MQTT_TOPIC:
    print('[ERROR] MQTT_DETECTION_TOPIC 환경변수가 설정되지 않았습니다!')
    exit(1)

print(f'[INFO] MQTT 설정 - 브로커: {MQTT_BROKER}:{MQTT_PORT}, 토픽: {MQTT_TOPIC}')

# 콜백 정의
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('[MQTT] 연결 성공')
        client.subscribe(MQTT_TOPIC)
        print(f'[MQTT] 토픽 구독: {MQTT_TOPIC}')
    else:
        print(f'[MQTT] 연결 실패: 코드 {rc}')

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        center_x = payload.get("center_x")
        center_y = payload.get("center_y")
        timestamp = payload.get("timestamp")

        print(f'[MQTT] center_x: {center_x}, center_y: {center_y}, timestamp: {timestamp}')
    except Exception as e:
        print(f'[ERROR] 메시지 처리 중 오류 발생: {e}')

def on_disconnect(client, userdata, rc):
    print(f'[MQTT] 연결 해제: 코드 {rc}')

def main():
    try:
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.on_disconnect = on_disconnect

        print(f'[INFO] MQTT 브로커에 연결 시도: {MQTT_BROKER}:{MQTT_PORT}')
        client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
        client.loop_forever()

    except KeyboardInterrupt:
        print('\n[INFO] 사용자에 의해 종료됨')
        client.disconnect()
    except Exception as e:
        print(f'[ERROR] 실행 중 오류: {e}')
    finally:
        print('[INFO] 프로그램 종료')

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import serial
import paho.mqtt.client as mqtt
import os
import time
from dotenv import load_dotenv

load_dotenv()

MQTT_BROKER = os.getenv('MQTT_BROKER')
MQTT_PORT = int(os.getenv('MQTT_PORT', '1883'))
MQTT_TOPIC = os.getenv('MQTT_ARM_TOPIC')
MQTT_KEEPALIVE = int(os.getenv('MQTT_KEEPALIVE', '60'))

BT_PORT = os.getenv('BT_PORT')
BT_BAUD = int(os.getenv('BT_BAUD', '9600'))

if not MQTT_BROKER:
    print('[ERROR] MQTT_BROKER 환경변수가 설정되지 않았습니다!')
    exit(1)

if not MQTT_TOPIC:
    print('[ERROR] MQTT_ARM_TOPIC 환경변수가 설정되지 않았습니다!')
    exit(1)

if not BT_PORT:
    print('[ERROR] BT_PORT 환경변수가 설정되지 않았습니다!')
    exit(1)

print(f'[INFO] MQTT 설정 - 브로커: {MQTT_BROKER}:{MQTT_PORT}, 토픽: {MQTT_TOPIC}')
print(f'[INFO] 블루투스 설정 - 포트: {BT_PORT}, 보드레이트: {BT_BAUD}')

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('[MQTT] 연결 성공')
        client.subscribe(MQTT_TOPIC)
        print(f'[MQTT] 토픽 구독: {MQTT_TOPIC}')
    else:
        print(f'[MQTT] 연결 실패: 코드 {rc}')

# 블루투스 메시지 입력 확인
def on_message(client, userdata, msg):
    command = msg.payload.decode().strip()
    print(f"[MQTT] 수신: '{command}'")

    try:
        with serial.Serial(BT_PORT, BT_BAUD, timeout=1) as bt:
            bt.write(command.encode())
            time.sleep(1)
            print(f"[Bluetooth] '{command}' 전송 완료")
    except Exception as e:
        print(f"[Bluetooth] 전송 실패: {e}")

def main():
    try:
        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message

        print(f'[MQTT] 브로커 연결 시도: {MQTT_BROKER}:{MQTT_PORT}')
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

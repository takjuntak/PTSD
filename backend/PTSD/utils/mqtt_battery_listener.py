import paho.mqtt.client as mqtt
import json
import requests
import math
from collections import deque
from PTSD.core.database import get_db
from sqlalchemy.orm import Session
from PTSD.models.devices import Device  # Device 모델 임포트
from dotenv import load_dotenv
import os

# .env 파일을 로드합니다.
load_dotenv()

# 환경 변수에서 값을 불러옵니다.
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")  # 기본값 설정
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))      # 문자열 -> 정수 변환 + 기본값
SERVER_URL = os.getenv("SERVER_URL")

websocket_connected = False  # WebSocket 연결 상태를 추적하는 변수
notification_sent = {}  # 알림 전송 여부 추적 (serial_number 별로)
battery_readings = {}  # 각 serial_number에 대해 최근 배터리 값을 저장할 딕셔너리
battery_queue = {}  # 배터리 값들의 큐 (평균 계산을 위해)

def get_user_id_by_serial(serial_number):
    """serial_number에 해당하는 user_id를 devices 테이블에서 조회합니다."""
    db: Session = next(get_db())  # 데이터베이스 세션 가져오기
    try:
        # devices 테이블에서 serial_number에 해당하는 user_id 조회
        device = db.query(Device).filter(Device.serial_number == serial_number).first()

        if device:
            print(f"디바이스 user_id: {device.user_id}")
            return device.user_id
        else:
            print(f"serial_number {serial_number}에 해당하는 디바이스를 찾을 수 없습니다.")
            return None
    except Exception as e:
        print(f"데이터베이스 쿼리 중 오류 발생: {e}")
        return None
    finally:
        db.close()  # 세션 종료

def smooth_battery_percentage(serial_number, new_value, smoothing_factor=0.1, max_change=5):
    """배터리 값 스무딩 처리 및 급격한 변화 필터링"""
    if serial_number not in battery_readings:
        battery_readings[serial_number] = new_value  # 처음 값은 그대로 사용
    else:
        # 이전 값과 새로운 값의 차이가 너무 크면 무시하고 이전 값을 유지
        if abs(new_value - battery_readings[serial_number]) <= max_change:
            battery_readings[serial_number] += (new_value - battery_readings[serial_number]) * smoothing_factor
        else:
            print(f"급격한 배터리 변화 ({battery_readings[serial_number]} -> {new_value})가 감지되어 무시됨.")
    # 배터리 값이 100을 초과하지 않도록 제한
    return min(round(battery_readings[serial_number]), 100)

def calculate_average_percentage(serial_number, new_value, window_size=10):
    """배터리 값의 평균을 계산하여 부드럽게 만듦"""
    if serial_number not in battery_queue:
        battery_queue[serial_number] = deque(maxlen=window_size)  # 큐를 maxlen 크기로 초기화

    battery_queue[serial_number].append(new_value)

     # 10초가 지났을 때 평균을 계산
    if len(battery_queue[serial_number]) == 10:  # 10초 동안의 데이터 수집
        return round(sum(battery_queue[serial_number]) / len(battery_queue[serial_number]))
    else:
        return new_value  # 10초가 되지 않았으면 원래 값 반환


def process_battery_percentage(serial_number, percentage):
    """배터리 퍼센트를 처리하여 안정화된 값을 반환"""
    smoothed_percentage = smooth_battery_percentage(serial_number, percentage)
    averaged_percentage = calculate_average_percentage(serial_number, percentage)
    # 두 가지 방법을 모두 적용하여 평균값과 스무딩값을 조합
    final_percentage = round((smoothed_percentage + averaged_percentage) / 2)
    # 배터리 값이 100을 초과하지 않도록 제한
    return min(final_percentage, 100)


def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    percentage = math.floor(payload.get("percentage", 0))
    serial_number = payload.get("serial", "")

    print(f"받은 배터리 퍼센트: {percentage}, Serial: {serial_number}")
    
    # WebSocket이 연결된 경우에만 알림 전송
    if not websocket_connected:
        print("WebSocket이 연결되지 않았습니다. 알림을 전송하지 않습니다.")
        return

    # serial_number로 user_id 조회
    user_id = get_user_id_by_serial(serial_number)
    
    if user_id:
        try:
            # 두 가지 방식(스무딩 + 평균값)을 모두 적용하여 배터리 퍼센트 계산
            final_percentage = process_battery_percentage(serial_number, percentage)

            # ✅ 알림 재전송 가능 조건 추가
            if final_percentage > 30:
                notification_sent[serial_number] = False
                print(f"[초기화] 배터리 충전됨 ({final_percentage}%), 알림 상태 초기화")

        
            # ⚠️ 알림 조건: 25% 이하 & 아직 알림 안 보냈을 때
            if final_percentage <= 25 and (serial_number not in notification_sent or not notification_sent[serial_number]):
                response = requests.post(
                    #  f"{SERVER_URL}/api/battery-notification",  # 배터리 부족 알림 API
                    "http://localhost:8000/api/battery-notification", 
                    json={"user_id": user_id, "percentage": final_percentage}
                )
                if response.status_code == 200:
                    print(f"[알림 전송 완료] 배터리 부족 알림: {final_percentage}%")
                    notification_sent[serial_number] = True  # 알림 전송 상태 기록
                else:
                    print(f"배터리 부족 알림 전송 실패: {response.status_code}")

            # 배터리 상태와 user_id를 FastAPI로 전송
            response = requests.post(
                # f"{SERVER_URL}/api/battery-state",
                "http://localhost:8000/api/battery-state",
                json={"percentage": final_percentage, "user_id": user_id}
            )
            if response.status_code == 200:
                print(f"[전송 완료] {percentage}% 전송됨")
            else:
                print(f"FastAPI 전송 실패: {response.status_code}")
        except Exception as e:
            print(f"전송 실패: {e}")
    else:
        print("유효한 user_id를 찾을 수 없습니다.")

def on_connect(client, userdata, flags, rc):
    global websocket_connected
    if rc == 0:
        websocket_connected = True
        print("배터리 연결됨.")
    else:
        websocket_connected = False
        print(f"배터리 연결 실패, 코드: {rc}")

def start_battery_mqtt_loop():
    client = mqtt.Client()
    client.on_message = on_message
    client.on_connect = on_connect
    client.connect(MQTT_BROKER, MQTT_PORT, 60)  # broker_ip에 맞게 수정
    # client.connect("192.168.100.165", MQTT_PORT, 60)  # broker_ip에 맞게 수정
    client.subscribe("mqtt/battery")
    client.loop_forever()

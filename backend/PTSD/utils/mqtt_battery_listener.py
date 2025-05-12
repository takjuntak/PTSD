import paho.mqtt.client as mqtt
import json
import requests
import math
from PTSD.core.database import get_db
from sqlalchemy.orm import Session
from PTSD.models.devices import Device  # Device 모델 임포트


# 최소 배터리 값 저장용 변수
min_battery_percentage = {}

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


def on_message(client, userdata, msg):
    payload = json.loads(msg.payload.decode())
    percentage = math.floor(payload.get("percentage", 0))
    serial_number = payload.get("serial", "")

    print(f"받은 배터리 퍼센트: {percentage}, Serial: {serial_number}")
    
    
    # 현재 배터리 퍼센트와 이전 최소값 비교
    if serial_number not in min_battery_percentage:
        min_battery_percentage[serial_number] = percentage  # 처음 수신 시 기본값 설정
        print(f"첫 배터리 값 수신: {percentage}%, 바로 전송합니다.")
    else:
        prev_min = min_battery_percentage[serial_number]
        if percentage >= prev_min:
            print(f"[스킵] 이전 값({prev_min}%)보다 높거나 같음.")
            return

    # serial_number로 user_id 조회
    user_id = get_user_id_by_serial(serial_number)
    
    if user_id:
        try:
            # 배터리 퍼센트가 40% 이하일 때 배터리 부족 알림 전송
            if percentage <= 40:
                response = requests.post(
                    "http://192.168.0.65:8000/api/battery-notification",  # 배터리 부족 알림 API
                    json={"user_id": user_id, "percentage": percentage}
                )
            if response.status_code == 200:
                print(f"[알림 전송 완료] 배터리 부족 알림: {percentage}%")
            else:
                print(f"배터리 부족 알림 전송 실패: {response.status_code}")

            # 배터리 상태와 user_id를 FastAPI로 전송
            response = requests.post(
                "http://192.168.0.65:8000/api/battery-state",
                json={"percentage": percentage, "user_id": user_id}
            )
            if response.status_code == 200:
                print(f"[전송 완료] 새로운 최소값 {percentage}% 전송됨")
                min_battery_percentage[serial_number] = percentage  # 최소값 갱신
            else:
                print(f"FastAPI 전송 실패: {response.status_code}")
        except Exception as e:
            print(f"전송 실패: {e}")
    else:
        print("유효한 user_id를 찾을 수 없습니다.")

def start_mqtt_loop():
    client = mqtt.Client()
    client.on_message = on_message
    MQTT_BROKER = "k12d101.p.ssafy.io"
    client.connect(MQTT_BROKER, 1883, 60)  # broker_ip에 맞게 수정
    client.subscribe("mqtt/battery")
    client.loop_forever()

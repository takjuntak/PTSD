import paho.mqtt.client as mqtt
import json
import requests
import math
from collections import deque
from PTSD.core.database import get_db
from sqlalchemy.orm import Session
from PTSD.models.devices import Device  # Device 모델 임포트
import logging
from dotenv import load_dotenv
import os

# .env 파일을 로드합니다.
load_dotenv()

# 환경 변수에서 값을 불러옵니다.
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")  # 기본값 설정
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))      # 문자열 -> 정수 변환 + 기본값
SERVER_URL = os.getenv("SERVER_URL", "http://localhost:8000")

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# MQTT 연결 상태 추적 변수
mqtt_connected = False


def get_user_id_by_serial(serial_number):
    """serial_number에 해당하는 user_id를 devices 테이블에서 조회합니다."""
    db: Session = next(get_db())  # 데이터베이스 세션 가져오기
    try:
        # devices 테이블에서 serial_number에 해당하는 user_id 조회
        device = db.query(Device).filter(Device.serial_number == serial_number).first()

        if device:
            logger.info(f"디바이스 user_id {device.user_id} for serial {serial_number}")
            return device.user_id
        else:
            logger.warning(f"serial_number {serial_number}에 해당하는 디바이스를 찾을 수 없습니다.")
            return None
    except Exception as e:
        logger.error(f"데이터베이스 쿼리 중 오류 발생: {e}")
        return None
    finally:
        db.close()  # 세션 종료


def on_message(client, userdata, msg):
    if not mqtt_connected:
        logger.warning("MQTT not connected. Ignoring message.")
        return
    
    payload = json.loads(msg.payload.decode())
    serial_number = payload.get("serial", "")
    status = payload.get("status")
    mode = payload.get("mode")

    logger.info(f"Received MQTT msg on {msg.topic}: serial={serial_number}, status={status}, mode={mode}")

    # serial_number로 user_id 조회
    user_id = get_user_id_by_serial(serial_number)
    
    if user_id:
        try:
            # 로봇 상태와 user_id를 FastAPI로 전송
            response = requests.post(
                # f"{SERVER_URL}/api/robot-notification",
                "http://localhost:8000/api/robot-notification",
                json={"user_id": user_id, "status": status, "mode": mode}
            )
            if response.status_code == 200:
                logger.info(f"[전송 완료] 로봇 상태: {status}, 모드: {mode} 전송됨")
            else:
                logger.error(f"FastAPI 전송 실패: {response.status_code}")
        except Exception as e:
            logger.error(f"전송 실패: {e}")
    else:
        logger.error("유효한 user_id를 찾을 수 없습니다.")

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        mqtt_connected = True
        print("로봇 연결됨.")
    else:
        mqtt_connected = False
        print(f"로봇 연결 실패, 코드: {rc}")

def start_robot_mqtt_loop():
    client = mqtt.Client()
    client.on_message = on_message
    client.on_connect = on_connect
    client.connect(MQTT_BROKER, MQTT_PORT, 60)  # broker_ip에 맞게 수정
    # client.connect("192.168.100.165", MQTT_PORT, 60)  # broker_ip에 맞게 수정
    client.subscribe("robot/status")
    client.loop_forever()

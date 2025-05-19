import paho.mqtt.client as mqtt
import json, logging
import math
import asyncio
from collections import deque
from datetime import datetime
from sqlalchemy.orm import Session
from PTSD.core.database import SessionLocal
from PTSD.models.devices import Device 
from PTSD.services.notification_service import (
    create_battery_notification,
    send_battery_status
)
from dotenv import load_dotenv
import os

# .env 파일을 로드합니다.
load_dotenv()
logger = logging.getLogger(__name__)

# 환경 변수에서 값을 불러옵니다.
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))

# 전역에 메인 루프 참조 저장
main_loop = asyncio.get_event_loop()

mqtt_connected = False  # 연결 상태를 추적하는 변수
notification_sent = {}  # 알림 전송 여부 추적 (serial_number 별로)
battery_readings = {}  # 각 serial_number에 대해 최근 배터리 값을 저장할 딕셔너리
battery_queue = {}  # 배터리 값들의 큐 (평균 계산을 위해)
last_avg_sent = {}    

def get_user_id_by_serial(serial_number: str) -> int | None:
    db: Session = SessionLocal()
    try:
        device = db.query(Device).filter(Device.serial_number == serial_number).first()
        if device:
            logger.info(f"Found user_id={device.user_id} for serial={serial_number}")
            return device.user_id
        logger.warning(f"No device for serial={serial_number}")
        return None
    except Exception as e:
        logger.error(f"DB error: {e}")
        return None
    finally:
        db.close()

def smooth_battery_percentage(serial, new_val, factor=0.1, max_change=5):
    prev = battery_readings.get(serial, new_val)
    if abs(new_val - prev) <= max_change:
        val = prev + (new_val - prev) * factor
    else:
        val = prev
    battery_readings[serial] = val
    # 0~100으로 clamp
    return max(0, min(round(val), 100))

def avg_battery_percentage(serial, new_val, window=5):
    q = battery_queue.setdefault(serial, deque(maxlen=window))
    q.append(new_val)
    avg =  round(sum(q)/len(q)) if len(q)==window else new_val
    # 0~100으로 clamp
    return max(0, min(avg, 100))

def process_battery_percentage(serial, pct):
    sm = smooth_battery_percentage(serial, pct)
    av = avg_battery_percentage(serial, pct)
    # 두 값을 다시 평균 내고 clamp
    combined = round((sm + av) / 2)
    return max(0, min(combined, 100))

def on_message(client, userdata, msg):
    if not mqtt_connected:
        logger.warning("MQTT not connected, ignoring message")
        return
    try:
        payload = json.loads(msg.payload.decode())
        raw_pct = math.floor(payload.get("percentage", 0))
        serial = payload.get("serial", "")

        logger.info(f"받은 배터리 퍼센트: {raw_pct}, MQTT msg on {msg.topic}: serial={serial}")

        # serial_number로 user_id 조회
        user_id = get_user_id_by_serial(serial)
        if not user_id:
            return

        # 스무딩·평균 처리
        final_pct = process_battery_percentage(serial, raw_pct)
        logger.info(f"Processed battery % for {serial}: {final_pct}%")

        now = datetime.utcnow()
        # 1) 처음 메시지는 무조건 전송
        if serial not in last_avg_sent:
            main_loop.call_soon_threadsafe(
                lambda: asyncio.create_task(send_battery_status(user_id, final_pct))
            )
            last_avg_sent[serial] = now
            logger.debug(f"[즉시전송] {serial}: {final_pct}%")
        # 2) 그 이후로는 10초 간격으로만 전송
        elif (now - last_avg_sent[serial]).total_seconds() >= 10:
            main_loop.call_soon_threadsafe(
                lambda: asyncio.create_task(send_battery_status(user_id, final_pct))
            )
            last_avg_sent[serial] = now
            logger.debug(f"[10초전송] {serial}: {final_pct}%")
        
       # ⚠️ 임계치 이하일 때만 create_battery_notification 호출
        if final_pct <= 25 and not notification_sent.get(serial, False):
            db = SessionLocal()
            try:
                main_loop.call_soon_threadsafe(
                    lambda: asyncio.create_task(create_battery_notification(user_id, final_pct, db))
                )
                notification_sent[serial] = True
                logger.info(f"[알림 전송 완료] 배터리 부족 알림: {final_pct}% (serial={serial})")
            finally:
                db.close()
        elif final_pct > 30:
            # 재충전 감지 시 알림 보내기 허용 상태로 초기화
            notification_sent[serial] = False
            logger.info(f"[초기화] 배터리 충전됨({final_pct}%), 알림 상태 초기화")

    except Exception as e:
        logger.error(f"Error in on_message: {e}")

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        mqtt_connected = True
        logger.info("Connected to MQTT broker")
    else:
        logger.error(f"MQTT connect failed, code={rc}")

def start_battery_mqtt_loop():
    client = mqtt.Client()
    client.on_message = on_message
    client.on_connect = on_connect
    client.connect(MQTT_BROKER, MQTT_PORT, 60)  # broker_ip에 맞게 수정
    client.subscribe("mqtt/battery")
    client.loop_forever()

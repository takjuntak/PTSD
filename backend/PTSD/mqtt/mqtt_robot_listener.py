import paho.mqtt.client as mqtt
import json
import logging
from dotenv import load_dotenv
import os
import asyncio
from sqlalchemy.orm import Session
from PTSD.core.database import SessionLocal
from PTSD.models.devices import Device
from PTSD.services.notification_service import create_robot_notification

# .env 로드
load_dotenv()

MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT   = int(os.getenv("MQTT_PORT", "1883"))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

mqtt_connected = False

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

def on_connect(client, userdata, flags, rc):
    global mqtt_connected
    if rc == 0:
        mqtt_connected = True
        logger.info("Connected to MQTT broker")
    else:
        logger.error(f"MQTT connect failed, code={rc}")

def on_message(client, userdata, msg):
    if not mqtt_connected:
        logger.warning("MQTT not connected, ignoring message")
        return

    try:
        payload = json.loads(msg.payload.decode())
        serial = payload.get("serial")
        status = payload.get("status")

        logger.info(f"MQTT msg on {msg.topic}: serial={serial}, status={status}")

        user_id = get_user_id_by_serial(serial)
        if not user_id:
            return

        # 비즈니스 로직 직접 호출 (비동기 함수)
        db = SessionLocal()
        try:
            # asyncio.run 은 블로킹이므로, 백그라운드 스레드에선 문제될 수 있지만
            # 간단한 예제라 이렇게 실행합니다.
            asyncio.run(create_robot_notification(user_id, status, db))
            logger.info(f"Notification created for user {user_id}")
        finally:
            db.close()

    except json.JSONDecodeError:
        logger.error("Failed to decode MQTT payload as JSON")
    except Exception as e:
        logger.error(f"Error in on_message: {e}")

def start_robot_mqtt_loop():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe("robot/status")
    client.loop_forever()
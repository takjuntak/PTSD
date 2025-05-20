import paho.mqtt.client as mqtt
import json, logging
from dotenv import load_dotenv
import os, asyncio
import paho.mqtt.publish as publish
from sqlalchemy.orm import Session
from PTSD.core.database import SessionLocal
from PTSD.models.devices import Device

# .env 로드
load_dotenv()

MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")
MQTT_PORT   = int(os.getenv("MQTT_PORT", "1883"))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

mqtt_connected = False

# --- serial -> user_id 캐시 ---
serial_user_cache = {}

def get_user_id_by_serial(serial_number: str) -> int | None:
    if serial_number in serial_user_cache:
        return serial_user_cache[serial_number]

    db: Session = SessionLocal()
    try:
        device = db.query(Device).filter(Device.serial_number == serial_number).first()
        if device:
            serial_user_cache[serial_number] = device.user_id
            logger.info(f"Found user_id={device.user_id} for serial={serial_number} (cached)")
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


def send_mqtt_command(command: str) -> None:
    """
    로봇 팔 제어용 MQTT 메시지 발행
    - topic: robot/arm 으로 payload=command 전송
    """
    topic = "robot/arm"
    payload = command
    try:
        logger.info(f"[MQTT Publish] topic={topic}, payload={payload}")
        publish.single(
            topic=topic,
            payload=payload,
            hostname=MQTT_BROKER,
            port=MQTT_PORT
        )
    except Exception as e:
        logger.error(f"Error publishing MQTT message: {e}")

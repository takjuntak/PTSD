from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import json
import asyncio
import traceback
import logging
import paho.mqtt.publish as publish
from sqlalchemy.orm import Session
from PTSD.core.database import get_db
from PTSD.models.devices import Device

router = APIRouter()

MQTT_BROKER = "k12d101.p.ssafy.io"
MQTT_PORT = 1883

# logging 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_serial_by_device_id(device_id: int) -> str | None:
    """DB에서 device_id로 serial_number 조회"""
    db: Session = next(get_db())
    try:
        device = db.query(Device).filter(Device.device_id == device_id).first()
        if device:
            return device.serial_number
        return None
    except Exception as e:
        logger.error(f"DB 조회 실패: {e}")
        return None
    finally:
        db.close()

def publish_command(topic: str, command: str):
    """MQTT 메시지 발행 함수 (동기)"""
    publish.single(topic, payload=command, hostname=MQTT_BROKER, port=MQTT_PORT)
    print(f"[디버그] 전송할 topic: {topic}, payload: {command}")

@router.websocket("/ws/manual-control")
async def websocket_control(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            device_id = msg.get("device_id")
            command = msg.get("command")

            logger.info(f"[수신] device_id: {device_id}, command: {command}")

            if device_id is None or command is None:
                await websocket.send_text(json.dumps({
                    "status": "error",
                    "message": "device_id 또는 command 누락됨"
                }))
                continue

            serial_number = get_serial_by_device_id(device_id)
            if serial_number:
                topic = "robot/control"
                try:
                    # MQTT 전송을 별도 스레드에서 실행 (비동기)
                    await asyncio.to_thread(publish_command, topic, command)
                    logger.info(f"[전송 완료] {topic} ← {command}")
                    await websocket.send_text(json.dumps({
                        "status": "success",
                        "message": f"{command} 전송 완료"
                    }))
                except Exception as e:
                    logger.error(f"[MQTT 오류] {traceback.format_exc()}")
                    await websocket.send_text(json.dumps({
                        "status": "error",
                        "message": "MQTT 전송 실패"
                    }))
            else:
                logger.warning(f"device_id {device_id}에 해당하는 serial_number를 찾을 수 없음")
                await websocket.send_text(json.dumps({
                    "status": "error",
                    "message": f"device_id {device_id}에 해당하는 serial_number를 찾을 수 없음"
                }))

    except WebSocketDisconnect:
        logger.info("[연결 종료] WebSocket 연결 끊김")

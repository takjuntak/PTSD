# routers/websocket_control.py

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import json
import paho.mqtt.publish as publish
from sqlalchemy.orm import Session
from PTSD.core.database import get_db
from PTSD.models.devices import Device

router = APIRouter()

MQTT_BROKER = "k12d101.p.ssafy.io"
MQTT_PORT = 1883

def get_serial_by_device_id(device_id: int) -> str | None:
    """DB에서 device_id로 serial_number 조회"""
    db: Session = next(get_db())
    try:
        device = db.query(Device).filter(Device.device_id == device_id).first()
        if device:
            return device.serial_number
        return None
    except Exception as e:
        print(f"[오류] DB 조회 실패: {e}")
        return None
    finally:
        db.close()

@router.websocket("/ws/control")
async def websocket_control(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            device_id = msg.get("device_id")
            command = msg.get("command")

            print(f"[수신] device_id: {device_id}, command: {command}")

            if device_id is None or command is None:
                print("device_id 또는 command 누락됨")
                continue

            serial_number = get_serial_by_device_id(device_id)
            if serial_number:
                topic = f"robot/control/{serial_number}"
                publish.single(topic, payload=command, hostname=MQTT_BROKER, port=MQTT_PORT)
                print(f"[전송 완료] {topic} ← {command}")
            else:
                print(f"[경고] device_id {device_id}에 해당하는 serial_number를 찾을 수 없음")

    except WebSocketDisconnect:
        print("[연결 종료] WebSocket 연결 끊김")

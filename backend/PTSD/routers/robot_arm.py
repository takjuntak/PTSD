from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import logging
import json
from PTSD.mqtt.mqtt_arm_listener import send_mqtt_command
from sqlalchemy.orm import Session
from PTSD.core.database import SessionLocal
from PTSD.models.devices import Device
from dotenv import load_dotenv
import os

# .env 파일을 로드합니다.
load_dotenv()

router = APIRouter()

# logging 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def get_serial_by_device_id(device_id: int) -> str | None:
    """DB에서 device_id로 serial_number 조회"""
    db: Session = SessionLocal()
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

@router.websocket("/ws/robot-arm")
async def websocket_robot_arm(websocket: WebSocket):
    await websocket.accept()
    logger.info("[WebSocket 연결] 로봇 팔 조작 연결 시작")
    try:
        while True:
            data = await websocket.receive_text()
            # JSON 파싱 예외 처리
            try:
                msg = json.loads(data)
            except json.JSONDecodeError:
                await websocket.send_text(json.dumps({
                    "status": "error",
                    "message": "유효하지 않은 JSON 형식입니다."
                }))
                continue

            device_id = msg.get("device_id")
            command = msg.get("command")

            if device_id is None or command is None:
                await websocket.send_text(json.dumps({
                    "status": "error",
                    "message": "device_id 또는 command 누락됨"
                }))
                continue
            
            logger.info(f"[WebSocket 수신] 로봇 팔 명령 수신: {device_id}, {command}")


            serial_number = get_serial_by_device_id(device_id)
            if not serial_number:
                await websocket.send_text(json.dumps({
                    "status": "error",
                    "message": f"device_id={device_id}에 해당하는 장치가 존재하지 않습니다."
                }))
                continue

            # MQTT 명령 전송
            if command == "Switch!":
                send_mqtt_command("Switch!")
                response = {
                    "status": "success",
                    "message": "로봇 팔 집기 신호 전송완료"
                }
            elif command == "Updown!":
                send_mqtt_command("Updown!")
                response = {
                    "status": "success", 
                    "message": "로봇 팔 위아래 신호 전송완료"
                }
            else:
                logger.warning(f"[무시됨] 알 수 없는 명령어: {data}")
                response = {
                    "status": "error",
                    "message": "지원되지 않는 명령입니다."
                }
            # JSON 문자열로 응답
            await websocket.send_text(json.dumps(response))
                
    except WebSocketDisconnect:
        logger.info("[WebSocket 종료] 로봇 팔 조작 연결 종료")



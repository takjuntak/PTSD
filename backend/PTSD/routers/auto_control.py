from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import logging
import json
from PTSD.utils.routine_scheduler import send_mqtt_command
from dotenv import load_dotenv
import os

# .env 파일을 로드합니다.
load_dotenv()

router = APIRouter()

# logging 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@router.websocket("/ws/auto-control")
async def websocket_auto_control(websocket: WebSocket):
    await websocket.accept()
    logger.info("[WebSocket 연결] 자동조작 연결 시작")
    try:
        while True:
            data = await websocket.receive_text()
            cmd = data.strip().lower()
            logger.info(f"[WebSocket 수신] 자동조작 명령 수신: {data}")

            if cmd == "start":
                send_mqtt_command("start")
                response = {
                    "status": "success",
                    "message": "자동 조작 시작 신호 전송완료"
                }
            elif cmd == "complete":
                send_mqtt_command("complete")
                response = {
                    "status": "success", 
                    "message": "자동 조작 완료 신호 전송완료"
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
        logger.info("[WebSocket 종료] 자동조작 연결 종료")
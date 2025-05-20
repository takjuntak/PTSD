from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict
import asyncio
import logging
from pydantic import BaseModel

router = APIRouter()

# 로깅 설정
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[int, WebSocket] = {}
        self.lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket, user_id: int):
        await websocket.accept()
        async with self.lock:
            self.active_connections[user_id] = websocket
        logger.info(f"User {user_id} connected with WebSocket.")

    async def disconnect(self, user_id: int):
        async with self.lock:
            if user_id in self.active_connections:
                del self.active_connections[user_id]
        logger.info(f"User {user_id} disconnected.")

    async def send_to_user(self, user_id: int, message: dict):
        user_id = int(user_id)
        async with self.lock:
            logger.info(f"[🔔 DEBUG] 현재 연결된 유저 목록: {list(self.active_connections.keys())}")
            logger.info(f"[🔔 DEBUG] user_id: {user_id} 로 메시지 전송 시도: {message}")

            websocket = self.active_connections.get(user_id)
            if websocket:
                try:
                    await websocket.send_json(message)
                    logger.info(f"Message sent to user {user_id}: {message}")
                except Exception as e:
                    logger.error(f"❌ Failed to send message to user {user_id}: {e}")
                    del self.active_connections[user_id]
            else:
                logger.warning(f"User {user_id} not connected, message not sent.")

manager = ConnectionManager()

@router.websocket("/ws/notifications/{user_id}")
async def websocket_endpoint(websocket: WebSocket, user_id: int):
    user_id = int(user_id)
    logger.info(f"Attempting to connect user {user_id}...")
    await manager.connect(websocket, user_id)

    try:
        while True:
            await asyncio.sleep(60)
            message = await websocket.receive_text()
            if message == "ping":
                await websocket.send_text("pong")
    except asyncio.CancelledError:
        # 서버 종료나 연결 취소 시 호출됨
        logger.info(f"WebSocket task for user {user_id} cancelled. Cleaning up.")
        await manager.disconnect(user_id)
        raise  # 예외 재발생 시켜서 상위에서 종료 처리 가능
    except WebSocketDisconnect:
        manager.disconnect(user_id, websocket)
        logger.info(f"User {user_id} disconnected")
    except Exception as e:
        manager.disconnect(user_id, websocket)
        logger.error(f"WebSocket error for user {user_id}: {e}")

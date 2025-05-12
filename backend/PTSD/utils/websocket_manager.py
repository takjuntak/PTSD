from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict
import asyncio
import logging

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

    async def send_to_user(self, user_id: int, message: str):
        async with self.lock:
            websocket = self.active_connections.get(user_id)
            if websocket:
                try:
                    await websocket.send_text(message)
                    logger.info(f"Message sent to user {user_id}: {message}")
                except Exception as e:
                    logger.error(f"Failed to send message to user {user_id}: {e}")
                    del self.active_connections[user_id]
            else:
                logger.warning(f"User {user_id} not connected, message not sent.")

manager = ConnectionManager()

@router.websocket("/ws/notifications/{user_id}")
async def websocket_endpoint(websocket: WebSocket, user_id: int):
    logger.info(f"Attempting to connect user {user_id}...")
    await manager.connect(websocket, user_id)
    try:
        while True:
            await websocket.receive_text()  # 연결 유지용 (ping 대체)
    except WebSocketDisconnect:
        logger.info(f"User {user_id} disconnected unexpectedly.")
        await manager.disconnect(user_id)

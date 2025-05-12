from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict
import asyncio

router = APIRouter()

# 웹소켓 연결 관리
class ConnectionManager:
    def __init__(self):
        self.active_connections: Dict[str, WebSocket] = {}  # user_id를 키로 사용하는 딕셔너리
        self.lock = asyncio.Lock()

    # 클라이언트와 연결을 수락하고 딕셔너리에 추가 (user_id를 key로 사용)
    async def connect(self, user_id: str, websocket: WebSocket):
        await websocket.accept()
        self.active_connections[user_id] = websocket

    # 클라이언트 연결이 끊기면 딕셔너리에서 제거
    async def disconnect(self, user_id: str):
        async with self.lock:
            if user_id in self.active_connections:
                del self.active_connections[user_id]

    # 특정 사용자에게만 메시지를 전송
    async def send_to_user(self, user_id: str, message: str):
        async with self.lock:
            websocket = self.active_connections.get(user_id)
            if websocket:
                try:
                    await websocket.send_text(message)
                except Exception:
                    del self.active_connections[user_id]
    
    # 모든 사용자에게 메시지를 전송
    async def broadcast(self, message: str):
        async with self.lock:
            disconnected_users = []
            for user_id, connection in self.active_connections.items():
                try:
                    await connection.send_text(message)
                except Exception:
                    disconnected_users.append(user_id)
            for user_id in disconnected_users:
                del self.active_connections[user_id]


manager = ConnectionManager()

# 실제 웹소켓 통신 처리 -> 특정 사용자에게 알림 전송
@router.websocket("/ws/notifications/{user_id}")
async def websocket_endpoint(websocket: WebSocket, user_id: str):
    await manager.connect(user_id, websocket)
    try:
        while True:
            data = await websocket.receive_text()
            # 수신된 메시지를 받은 사용자에게만 알림
            await manager.send_to_user(user_id, f"🔔 {data}")
    except WebSocketDisconnect:
        await manager.disconnect(user_id)
        

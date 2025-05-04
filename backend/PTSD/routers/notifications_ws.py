from fastapi import WebSocket, WebSocketDisconnect, APIRouter, status
from typing import Dict, List
from PTSD.models.user import User  # 유저 모델 (예시)

router = APIRouter()
active_connections: Dict[int, List[WebSocket]] = {}  # user_id -> WebSocket 목록


async def connect_user(user_id: int, websocket: WebSocket):
    await websocket.accept()
    if user_id not in active_connections:
        active_connections[user_id] = []
    active_connections[user_id].append(websocket)


def disconnect_user(user_id: int, websocket: WebSocket):
    if user_id in active_connections:
        active_connections[user_id].remove(websocket)
        if not active_connections[user_id]:
            del active_connections[user_id]


async def send_notification_to_user(user_id: int, data: dict):
    connections = active_connections.get(user_id, [])
    for conn in connections:
        await conn.send_json(data)


@router.websocket("/ws/notifications")
async def websocket_endpoint(websocket: WebSocket):
    token = websocket.query_params.get("token")

    if not token:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

    try:
        # JWT 디코드 → 사용자 정보 추출
        payload = verify_jwt_token(token)
        user_id = payload.get("user_id")

        if not user_id:
            raise ValueError("유저 정보 없음")

        await connect_user(user_id, websocket)

        while True:
            await websocket.receive_text()

    except WebSocketDisconnect:
        disconnect_user(user_id, websocket)
    except Exception as e:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)

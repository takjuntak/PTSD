# WebSocket 연결 + 유저별 관리
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Header, status
from PTSD.utils.jwt_handler import verify_jwt_token

router = APIRouter()
connected_users = {}  # user_id: WebSocket

@router.websocket("/ws/notifications")
async def websocket_endpoint(websocket: WebSocket, authorization: str = Header(None)):
    if not authorization:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

    token = authorization.split(" ")[1] if authorization.startswith("Bearer ") else None
    if not token:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

    try:
        payload = verify_jwt_token(token)
        user_id = payload.get("user_id")

        if not user_id:
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
            return

        await websocket.accept()
        connected_users[user_id] = websocket
        print(f"✅ 사용자 {user_id} WebSocket 연결됨")

        while True:
            await websocket.receive_text()  # 메시지 수신 대기 (사용 안해도 끊김 방지)

    except WebSocketDisconnect:
        connected_users.pop(user_id, None)
        print(f"❌ 사용자 {user_id} WebSocket 연결 종료됨")
    except Exception as e:
        print(f"⚠️ 오류: {e}")
        await websocket.close(code=status.WS_1011_INTERNAL_ERROR)
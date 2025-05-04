# WebSocket을 통해 사용자에게 알림을 전송하는 유틸리티 함수입니다.
from PTSD.routers.notifications_ws import connected_users

async def send_notification_to_user(user_id: int, data: dict):
    websocket = connected_users.get(user_id)
    if websocket:
        try:
            await websocket.send_json(data)
            print(f"📨 사용자 {user_id} 에게 알림 전송됨")
        except Exception as e:
            print(f"❌ 알림 전송 실패: {e}")
            connected_users.pop(user_id, None)
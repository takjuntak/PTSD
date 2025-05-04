# from fastapi import APIRouter, WebSocket, WebSocketDisconnect, status
# from typing import Dict

# router = APIRouter()

# # user_id와 연결된 WebSocket 객체 저장
# connected_users: Dict[int, WebSocket] = {}

# async def connect_user(user_id: int, websocket: WebSocket):
#     connected_users[user_id] = websocket
#     print(f"사용자 {user_id} 연결됨")

# async def disconnect_user(user_id: int):
#     if user_id in connected_users:
#         connected_users.pop(user_id)
#         print(f"사용자 {user_id} 연결 종료됨")

# @router.websocket("/ws/notifications")
# async def websocket_endpoint(websocket: WebSocket):
#     try:
#         await websocket.accept()

#         # 클라이언트가 첫 메시지로 user_id 전송
#         data = await websocket.receive_json()
#         user_id = data.get("user_id")

#         if not user_id:
#             await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
#             return

#         await connect_user(user_id, websocket)

#         # 메시지 수신 대기
#         while True:
#             message = await websocket.receive_text()
#             print(f"[user_id: {user_id}] 수신된 메시지: {message}")

#     except WebSocketDisconnect:
#         await disconnect_user(user_id)
#     except Exception as e:
#         print(f"웹소켓 처리 중 오류 발생: {e}")
#         await websocket.close(code=status.WS_1011_INTERNAL_ERROR)

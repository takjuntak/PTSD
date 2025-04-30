from fastapi import APIRouter

router = APIRouter()

# 수신된 MQTT 메시지 조회 API
received_messages = []  # 수신된 메시지를 저장하는 임시 리스트 (여기서 데이터를 처리하세요)

@router.get("/mqtt/messages")
async def get_received_messages():
    return {"messages": received_messages}

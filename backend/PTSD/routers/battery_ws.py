from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel
from PTSD.utils.websocket_manager import manager  # 웹소켓 매니저
import logging
import math

router = APIRouter()
logger = logging.getLogger(__name__)

# Pydantic 모델
class BatteryData(BaseModel):
    user_id: int
    percentage: float

# HTTP POST endpoint
@router.post("/api/battery-state")
async def receive_battery_state(data: BatteryData):
    percentage_int = math.floor(data.percentage)  # 소수점 아래 내림 처리
    message ={
        "category": "battery_status",
        "percentage": percentage_int
    }
    
    # postman에서 테스트용
    # message =f"배터리: {percentage_int}"

    try:
        # 특정 사용자에게 메시지 전송
        await manager.send_to_user(data.user_id, message)
        logger.info(f"Battery data sent to user {data.user_id}: {message}")
        return {"message": "Battery state sent to user"}
    except Exception as e:
        logger.error(f"Failed to send battery state to user {data.user_id}: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to send battery state to user"
        )

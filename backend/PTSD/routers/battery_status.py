from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel
from PTSD.utils.websocket_manager import manager  # 웹소켓 매니저
from PTSD.schemas.response import ResponseModel
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
        
        return ResponseModel(
            isSuccess=True,
            code=200,
            message="배터리 상태가 성공적으로 전송되었습니다.",
            result=None  # 알림 ID 등 결과 데이터가 있으면 여기에 삽입
        )
    except Exception as e:
        logger.error(f"사용자 {data.user_id}에게 배터리 상태 전송 실패: {e}")
        raise HTTPException(
            status_code=500,
            detail="사용자에게 배터리 상태를 전송하는 데 실패했습니다."
        )

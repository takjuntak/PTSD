from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.orm import Session
from pydantic import BaseModel
from PTSD.utils.websocket_manager import manager  # 웹소켓 매니저
from PTSD.schemas.response import ResponseModel
from PTSD.core.database import get_db
from PTSD.services.notification_service import send_battery_status
import logging
import math

router = APIRouter()
logger = logging.getLogger(__name__)

# Pydantic 모델
class BatteryData(BaseModel):
    user_id: int
    percentage: float

# HTTP POST endpoint
@router.post(
    "/api/battery-state",
    tags=["배터리 알림"],
    summary="배터리 상태 전송",
    description="""  
### 📌 **배터리 상태 전송을 진행합니다.**

이 API는 사용자의 배터리 상태를 받아와 해당 사용자에게 배터리 상태 알림을 전송합니다. 

### ✅ [요청 필드]
- `user_id`: 사용자 고유 ID (integer)
- `percentage`: 배터리 잔량 (%) (float)

### ✅ [응답 필드]
- `isSuccess`: 요청 처리 성공 여부 (boolean)
- `code`: HTTP 응답 코드 (integer)
- `message`: 처리 상태 메시지 (string)
- `result`: 추가 데이터 (null일 경우, 데이터 없음)
"""
)
async def receive_battery_state(data: BatteryData):
    percentage_int = math.floor(data.percentage)  # 소수점 아래 내림 처리
    message ={
        "category": "battery_status",
        "percentage": percentage_int
    }
    
    try:
        # 특정 사용자에게 메시지 전송
        await manager.send_to_user(data.user_id, message)
        logger.info(f"Battery data sent to user {data.user_id}: {message}")
        
        return ResponseModel(
            isSuccess=True,
            code=200,
            message="배터리 상태가 성공적으로 전송되었습니다.",
            result=message
        )
    except Exception as e:
        logger.error(f"사용자 {data.user_id}에게 배터리 상태 전송 실패: {e}")
        raise HTTPException(
            status_code=500,
            detail="사용자에게 배터리 상태를 전송하는 데 실패했습니다."
        )

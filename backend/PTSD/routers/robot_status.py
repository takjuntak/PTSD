from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from PTSD.core.database import get_db
from PTSD.schemas.response import ResponseModel
from pydantic import BaseModel
from PTSD.services.notification_service import create_robot_notification
import asyncio

router = APIRouter()

# 요청 데이터 스키마 정의
class RobotData(BaseModel):
    user_id: int 
    status: str  # "start" or "complete"


@router.post(
    "/api/robot-notification",
    tags=["로봇 상태 알림"],
    summary="로봇 동작 상태 알림 전송",
        description="""
### 📌 **로봇 청소 상태 알림 API**

이 API는 로봇의 **청소 시작(start)** 또는 **청소 완료(complete)** 상태를 받아  
해당 사용자에게 알림을 생성하고,  
알림 기록을 DB에 저장하며,  
WebSocket을 통해 실시간으로 전송합니다.

---

### ✅ [요청 필드]
- `user_id` : 사용자 고유 ID (integer)
- `status` (string): 로봇 상태, `"start"` 또는 `"complete"`

### ✅ [처리 흐름]
1. 입력된 `status` 값을 확인하여  
   - `"start"`: 청소 시작 알림  
   - `"complete"`: 청소 완료 알림  
   메시지를 생성합니다.  
2. 생성된 알림을 DB에 저장합니다.  
3. WebSocket으로 사용자에게 실시간 전송합니다.

### ✅ [응답 필드]
- `isSuccess`: 요청 처리 성공 여부 (boolean)
- `code`: HTTP 상태 코드 (integer)
- `message`: 처리 결과 메시지 (string)
- `result`: 저장된 알림 ID (object)
  - `notification_id` (integer): 새로 생성된 알림의 ID  
"""
)
async def send_robot_state(payload: RobotData, db: Session = Depends(get_db)):
    try:
        notification_id = await create_robot_notification(
            user_id=payload.user_id,
            status=payload.status,
            db=db
        )
        return ResponseModel(
            isSuccess=True,
            code=200,
            message="로봇 상태 알림이 성공적으로 전송되었습니다.",
            result={"notification_id": notification_id}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

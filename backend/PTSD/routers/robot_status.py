from datetime import datetime
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from PTSD.models.notifications import Notification, NotificationType
from PTSD.schemas.response import ResponseModel
from PTSD.utils.websocket_manager import manager  # 웹소켓 매니저 객체
from typing import Dict
from PTSD.core.database import get_db
import logging
from datetime import datetime
from pydantic import BaseModel
import math

router = APIRouter()

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 요청 바디 스키마 정의
class NotificationRequest(BaseModel):
    user_id: int
    title: str
    message: str
    type: NotificationType

# 요청 데이터 스키마 정의
class RobotData(BaseModel):
    user_id: int 
    status: str  # "start" or "complete"
    mode: str  # "auto", "manual" 


@router.post(
    "/api/robot-notification",
    tags=["로봇 상태 알림"],
    summary="로봇 시작과 완료 알림 전송",
    description="""
### 📌 **로봇 상태(시작, 완료)와 모드 전송을 진행합니다.**

이 API는 로봇의 상태와 모드를 받아와 해당 사용자에게 로봇 상태 알림을 전송합니다. 

### ✅ [요청 필드]
- `user_id` : 사용자 고유 ID (integer)
- `status`: 로봇 상태 (string)
- `mode`: 로봇 모드 (string)

### ✅ [처리 흐름]
1. 요청으로 받은 `status`·`mode`를 기반으로 알림 메시지 생성  
2. `Notification` 모델에 저장 후 DB 커밋  
3. WebSocket을 통해 실시간 알림 전송

### ✅ [응답 필드]
- `isSuccess`: 요청 처리 성공 여부 (boolean)
- `code`: HTTP 상태 코드 (integer)
- `message`: 처리 결과 메시지 (string)
- `result`: 저장된 알림 ID (object)
"""
)
async def send_robot_state(
    payload: RobotData,
    db: Session = Depends(get_db)
):
    try:
        logging.info(f"로봇 상태: {payload.status}, 모드: {payload.mode}")
        # 예: "자동 모드에서 작업이 시작되었습니다."
        title = f"{payload.status}"
        message_text = f"{payload.mode} 모드에서 작업이 {payload.status}되었습니다."
        
        # NotificationRequest로 변환
        notification_data = Notification(
            user_id=payload.user_id,
            title=title,
            message=message_text,
            type=payload.status,
            timestamp=datetime.utcnow(),
            is_read=False
        )
        db.add(notification_data)
        db.commit()
        db.refresh(notification_data)

        # message = {
        #     "category": "robot_alert",
        #     "notification": {
        #         "notification_id": notification_data.notification_id,
        #         "title": notification_data.title,
        #         "message": notification_data.message,
        #         "type": notification_data.type,
        #         "timestamp": notification_data.timestamp.isoformat(),
        #         "is_read": False
        #     }
        # }

        # 웹소켓 postman 테스트용
        message = f"로봇 알림! 상태: {payload.status}, 모드: {payload.mode}"

        # 웹소켓을 통해 실시간 알림 전송
        await manager.send_to_user(payload.user_id, message)

        return ResponseModel(
            isSuccess=True,
            code=200,
            message="로봇 상태 알림이 성공적으로 전송되었습니다.",
            result={"notification_id": notification_data.notification_id}
        )
    except Exception as e:
        db.rollback()
        logger.error(f"로봇 상태 알림 전송 실패 (user_id={payload.user_id}): {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"로봇 알림 전송 중 오류 발생: {str(e)}"
        )

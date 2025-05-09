from datetime import datetime
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from PTSD.models.notifications import Notification, NotificationType
from PTSD.schemas.response import ResponseModel
from PTSD.utils.websocket_manager import manager  # 웹소켓 매니저 객체
from typing import Dict
from PTSD.core.database import get_db
from PTSD.schemas.response import ResponseModel  # 공통 응답 포맷
import logging
from datetime import datetime
from pydantic import BaseModel


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
class BatteryStatus(BaseModel):
    percentage: float

@router.post(
    "/api/battery-state",
    tags=["배터리 상태"],
    summary="배터리 상태 알림 전송"
)
async def send_battery_notification(
    payload: dict,  # /battery_state에서 전달받은 raw 데이터
    db: Session = Depends(get_db)
):
    try:
        # percentage 추출 및 메시지 구성
        percentage = round(payload.get("percentage", 0.0), 2)
        voltage = round(payload.get("voltage", 0.0), 2)
        message = f"배터리 잔량: {percentage}%, 전압: {voltage}V"

        # NotificationRequest로 변환
        notification_data = Notification(
            user_id=1,  # 실제 사용자의 ID로 설정
            title="배터리 상태 알림",
            message=message,
            type="battery",
            timestamp=datetime.utcnow(),
            is_read=False
        )
        db.add(notification_data)
        db.commit()
        db.refresh(notification_data)

        # 웹소켓 메시지 전송
        await manager.send_to_user(1, {
            "notification_id": notification_data.notification_id,
            "title": "배터리 상태 알림",
            "message": message,
            "type": "battery",
            "timestamp": notification_data.timestamp.isoformat(),
            "is_read": False
        })

        return ResponseModel(
            isSuccess=True,
            code=200,
            message="배터리 알림이 성공적으로 전송되었습니다.",
            result={"notification_id": notification_data.notification_id}
        )
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"배터리 알림 전송 중 오류 발생: {str(e)}"
        )

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
class BatteryStatus(BaseModel):
    user_id: int 
    percentage: float


@router.post(
    "/api/battery-notification",
    tags=["배터리 알림"],
    summary="배터리 부족 알림 전송"
)
async def send_battery_notification(
    payload: BatteryStatus,
    db: Session = Depends(get_db)
):
    try:
        percentage_int = math.floor(payload.percentage)
        logging.info(f"배터리 부족 알림! 배터리 잔량: {percentage_int}%")
        battery_message = f"배터리가 부족합니다."
        
        # NotificationRequest로 변환
        notification_data = Notification(
            user_id=payload.user_id,
            title="경고",
            message=battery_message,
            type="battery",
            timestamp=datetime.utcnow(),
            is_read=False
        )
        db.add(notification_data)
        db.commit()
        db.refresh(notification_data)

        message = {
            "category": "battery_alert",
            "notification": {
                "notification_id": notification_data.notification_id,
                "title": "경고",
                "message": "배터리가 부족합니다.",
                "type": "battery",
                "timestamp": notification_data.timestamp.isoformat(),
                "is_read": False
            }
        }

        # 웹소켓 postman 테스트용
        # message = f"배터리 부족! 잔량: {percentage_int}%"

        # 웹소켓을 통해 실시간 알림 전송
        await manager.send_to_user(payload.user_id, message)

        return ResponseModel(
            isSuccess=True,
            code=200,
            message="배터리 부족 알림이 성공적으로 전송되었습니다.",
            result={"notification_id": notification_data.notification_id}
        )
    except Exception as e:
        db.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"배터리 알림 전송 중 오류 발생: {str(e)}"
        )

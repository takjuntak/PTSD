from PTSD.models.notifications import Notification
from PTSD.utils.websocket_manager import manager
from sqlalchemy.orm import Session
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

async def create_robot_notification(user_id: int, status: str, db: Session):
    try:
        title = f"{status}"
        message_text = f"작업이 {status}되었습니다."

        notification = Notification(
            user_id=user_id,
            title=title,
            message=message_text,
            type=status,
            timestamp=datetime.utcnow(),
            is_read=False
        )
        db.add(notification)
        db.commit()
        db.refresh(notification)

        ws_message = {
            "category": "robot_alert",
            "notification": {
                "notification_id": notification.notification_id,
                "title": notification.title,
                "message": notification.message,
                "type": notification.type,
                "timestamp": notification.timestamp.isoformat(),
                "is_read": False
            }
        }

        # 웹소켓 테스트용
        # ws_message = f"로봇 알림! 상태: {status}"
        await manager.send_to_user(user_id, ws_message)

        return notification.notification_id
    except Exception as e:
        db.rollback()
        logger.error(f"Notification 생성 실패: {e}")
        raise

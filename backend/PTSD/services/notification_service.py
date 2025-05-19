from PTSD.models.notifications import Notification
from PTSD.utils.websocket_manager import manager
from sqlalchemy.orm import Session
from datetime import datetime
import logging

logger = logging.getLogger(__name__)


async def create_battery_notification(user_id: int, percentage: int, db: Session):
    try:
        title = "경고"
        message_text = f"배터리가 부족합니다. ({percentage}%)"

        notification = Notification(
            user_id=user_id,
            title=title,
            message=message_text,
            type="battery",
            timestamp=datetime.utcnow(),
            is_read=False
        )
        db.add(notification)
        db.commit()
        db.refresh(notification)

        # ws_message = {
        #     "category": "battery_alert",
        #     "notification": {
        #         "notification_id": notification.notification_id,
        #         "title": notification.title,
        #         "message": notification.message,
        #         "type": notification.type,
        #         "timestamp": notification.timestamp.isoformat(),
        #         "is_read": False
        #     }
        # }
        # 웹소켓 테스트용
        ws_message = f"배터리 부족! 상태: {percentage}%"
        await manager.send_to_user(user_id, ws_message)

        return notification.notification_id
    except Exception as e:
        db.rollback()
        logger.error(f"배터리 알림 생성 실패: {e}")
        raise

async def send_battery_status(user_id: int, percentage: int):
    """
    현재 배터리 상태만 WebSocket으로 실시간 전송
    (DB 기록 필요 없을 때 사용)
    """
    ws_message = {
        "category": "battery_status",
        "percentage": percentage
    }

    # 웹소켓 테스트용
    # ws_message = f"배터리 상태: {percentage}%"
    await manager.send_to_user(user_id, ws_message)

async def create_robot_notification(user_id: int, status: str, db: Session):
    try:
        """
        로봇 상태(시작/완료) 알림 저장 + WebSocket 전송
        """
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


from apscheduler.schedulers.background import BackgroundScheduler
from datetime import datetime, timedelta
from sqlalchemy.orm import Session
from PTSD.models.notifications import Notification
from PTSD.core.database import get_db
import logging

# 로깅 설정
logger = logging.getLogger(__name__)

# 백그라운드 스케줄러 인스턴스 생성
scheduler = BackgroundScheduler()

# 알림 삭제 함수
def delete_old_notifications(db_generator: Session):
    try:
        db = next(db_generator)  # 제너레이터에서 세션을 추출

        # 7일 이상된 알림 삭제
        time_threshold = datetime.utcnow() - timedelta(days=7)
        old_notifications = db.query(Notification).filter(Notification.timestamp < time_threshold)
        
        deleted_count = old_notifications.delete(synchronize_session=False)
        db.commit()

        logger.info(f"{deleted_count}개의 오래된 알림을 삭제했습니다.")
    except Exception as e:
        logger.error(f"알림 삭제 중 오류가 발생했습니다: {e}")


# 알림 삭제 주기적으로 1일마다 실행
def start_notification_deletion_scheduler():
    # 매일 자정(00:00:00)에 실행되도록 설정
    scheduler.add_job(delete_old_notifications, 'cron', hour=0, minute=0, second=0, args=[get_db()])
    scheduler.start()

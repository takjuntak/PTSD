#알림
from sqlalchemy import Column, Integer, String, Text, Boolean, Enum, DateTime, ForeignKey
from sqlalchemy.orm import relationship
from datetime import datetime
from PTSD.core.database import Base
import enum

# 알림 타입 ENUM 정의
class NotificationType(str, enum.Enum):
    start = "start"      # 작업 시작 알림
    complete = "complete"  # 작업 완료 알림

# 알림 로그 모델 정의
class NotificationLog(Base):
    __tablename__ = "notificationlogs"  

    log_id = Column(Integer, primary_key=True, autoincrement=True, nullable=False)
    # 알림 로그 고유 ID

    userId = Column(Integer, ForeignKey("users.userId", ondelete="CASCADE"), nullable=False)
    # 알림을 발생시킨 사용자 ID 

    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)
    # 알림 발생 시각 (기본값: 현재 시간)

    type = Column(Enum(NotificationType), nullable=False)
    # 알림 타입 (start 또는 complete)

    title = Column(String(100), nullable=True)
    # 알림 제목 (100자 이내, 선택 입력)

    message = Column(Text, nullable=True)
    # 알림 내용 (긴 텍스트, 선택 입력)

    read = Column(Boolean, default=False, nullable=False)
    # 알림 읽음 여부 (기본값: 읽지 않음)

    # 조인
    user = relationship("User", back_populates="notification_logs")
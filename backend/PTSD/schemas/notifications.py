from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from enum import Enum

# 알림 타입 Enum
class NotificationType(str, Enum):
    start = "start"
    complete = "complete"

# 알림 하나를 나타내는 스키마
class NotificationRead(BaseModel):
    notification_id: int
    timestamp: datetime
    type: NotificationType
    title: Optional[str] = None
    message: Optional[str] = None
    is_read: bool

    class Config:
        orm_mode = True

# 알림 전체 조회
class NotificationLogsRead(BaseModel):
    total_count: int
    page: int
    limit: int
    total_pages: int
    has_next: bool
    has_previous: bool
    logs: List[NotificationRead]

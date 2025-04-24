from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List
from enum import Enum

# Enum for routine types
class ScheduleTypeEnum(str, Enum):
    once = "once"
    daily = "daily"

# 요청 모델
class RoutineCreate(BaseModel):
    start_time: datetime  # ISO 8601 format
    schedule_type: ScheduleTypeEnum = ScheduleTypeEnum.once
    isWork: bool = False
    repeat_days: Optional[List[int]] = None  # 1-7 (월-일)

from pydantic import BaseModel, field_validator
from datetime import datetime
from typing import Optional, List
from enum import Enum

# Enum for routine types
class RoutineTypeEnum(str, Enum):
    once = "once"
    daily = "daily"

# 요청 모델
class RoutineCreate(BaseModel):
    start_time: datetime  # ISO 8601 format
    routine_type: RoutineTypeEnum = RoutineTypeEnum.once
    is_work: bool = False
    repeat_days: Optional[List[int]] = None  # 1-7 (월-일)
    
    @field_validator('repeat_days')
    def validate_repeat_days(cls, v):
        if v is not None:
            if any(day < 1 or day >= 8 for day in v):
                raise ValueError('repeat_days must be between 1 and 7')
        return v

class RoutineGet(BaseModel):
    start_time: datetime
    routine_type: RoutineTypeEnum = RoutineTypeEnum.once
    is_work: bool
    repeat_days: List[int]
    
    
class RoutineUpdate(BaseModel):
    start_time: Optional[datetime] = None
    routine_type: Optional[RoutineTypeEnum] = None
    is_work: Optional[bool] = None
    repeat_days: Optional[List[int]] = None  # 1-7 (월-일)
    
    @field_validator('repeat_days')
    def validate_repeat_days(cls, v):
        if v is not None:
            if any(day < 1 or day >= 8 for day in v):
                raise ValueError('repeat_days must be between 1 and 7')
        return v


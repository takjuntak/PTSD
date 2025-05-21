# schemas/devices.py 로봇 기기 관리 API
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional

# 등록
class DeviceCreate(BaseModel):
    # user_id: int
    serial_number: str = Field(..., max_length=50)
    name: str = Field(..., max_length=50, description="기기 이름")

# 조회
class DeviceRead(BaseModel):
    device_id: int
    created_at: datetime
    user_id: int
    serial_number: str
    name : str

    class Config:
        from_attributes = True

# 수정
class DeviceUpdate(BaseModel):
    serial_number: Optional[str] = Field(None, max_length=30)
    name: Optional[str] = Field(None, max_length=50)
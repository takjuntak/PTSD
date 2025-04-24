# schemas/devices.py
from pydantic import BaseModel
from datetime import datetime

# 생성용
class DeviceCreate(BaseModel):
    user_id: int
    serial_number: str

# 조회용
class DeviceRead(BaseModel):
    device_id: int
    created_at: datetime
    user_id: int
    serial_number: str

    class Config:
        orm_mode = True
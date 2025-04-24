# models/devices/devices.py
from sqlalchemy import Column, Integer, DateTime, String, ForeignKey
from datetime import datetime
from PTSD.core.database import Base  # 너희 core/database.py에서 Base 임포트

class Device(Base):
    __tablename__ = "devices"

    device_id = Column(Integer, primary_key=True, index=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    user_id = Column(Integer, ForeignKey("users.user_id"))  # users 테이블의 user_id를 참조
    serial_number = Column(String(30), nullable=False)
# models/devices/devices.py
from datetime import datetime
from sqlalchemy import Integer, DateTime, String, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column 
from PTSD.core.database import Base



class Device(Base):
    __tablename__ = "devices"

    device_id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow) #생성일 
    user_id: Mapped[int] = mapped_column(ForeignKey("users.userId"))  #user_ID
    serial_number: Mapped[str] = mapped_column(String(50), nullable=False)  #시리얼 번호
    name: Mapped[str] = mapped_column(String(50), nullable=False) # 별칭
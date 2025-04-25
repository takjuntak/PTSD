from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.orm import relationship
from datetime import datetime
from PTSD.core.database import Base


class User(Base):
    __tablename__ = "users"

    userId = Column(Integer, primary_key=True, index=True, autoincrement=True)
    email = Column(String, unique=True, index=True, nullable=False)
    password = Column(String(128), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow) 
    
    # User 모델에 routines 속성을 추가하여 Routine과 관계를 설정
    # routines = relationship("Routine", back_populates="user")
    
    # cascade 옵션 추가
    routines = relationship("Routine", back_populates="user", cascade="all, delete-orphan")
    devices = relationship("Device", back_populates="user", cascade="all, delete-orphan")

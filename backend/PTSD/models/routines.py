from sqlalchemy import Column, Integer, DateTime, Boolean, Enum, ForeignKey, ARRAY
from sqlalchemy.orm import relationship
from datetime import datetime
from PTSD.core.database import Base


class Routine(Base):
    __tablename__ = "routines"

    routineId  = Column(Integer, primary_key=True, index=True, autoincrement=True)
    userId = Column(Integer, ForeignKey('users.userId',ondelete='CASCADE'), nullable=False)  # users 테이블의 user_id를 참조
    startTime = Column(DateTime, nullable=False)
    routineType = Column(
        Enum('once', 'daily', name='routine_type_enum'), 
        nullable=False, 
        default='once'
    )
    isWork = Column(Boolean,nullable=False)
    repeatDays = Column(ARRAY(Integer),nullable=False)
    
    # Many-to-One: 여러 routine이 한 user에 속함
    user = relationship("User", back_populates="routines")


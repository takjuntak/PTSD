from sqlalchemy import Column, Integer, DateTime, Boolean, Enum, ForeignKey, ARRAY
from sqlalchemy.orm import relationship
from datetime import datetime
from PTSD.core.database import Base


class Routine(Base):
    __tablename__ = "routines"

    routine_id  = Column(Integer, primary_key=True, index=True, autoincrement=True)
    user_id = Column(Integer, ForeignKey('users.user_id',ondelete='CASCADE'), nullable=False)  # users 테이블의 user_id를 참조
    start_time = Column(DateTime, nullable=False)
    routine_type = Column(
        Enum('once', 'daily', name='routine_type_enum'), 
        nullable=False, 
        default='once'
    )
    is_work = Column(Boolean,nullable=False)
    repeat_days = Column(ARRAY(Integer),nullable=False)
    
    # Many-to-One: 여러 routine이 한 user에 속함
    user = relationship("User", back_populates="routines")


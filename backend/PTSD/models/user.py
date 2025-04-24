from sqlalchemy import Column, Integer, String, DateTime
from datetime import datetime
from PTSD.core.database import Base


class User(Base):
    __tablename__ = "users"

    userId = Column(Integer, primary_key=True, index=True, autoincrement=True)
    email = Column(String, unique=True, index=True)
    password = Column("password", String)  # Actual DB column name is `password`
    createdAt = Column(DateTime, default=datetime.utcnow) 
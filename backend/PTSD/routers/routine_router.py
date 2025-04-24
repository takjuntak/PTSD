from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List
from enum import Enum
from ..schemas import routines, response
from ..models import routines, user
from ..core.database import get_db



router = APIRouter()

@router.post("/api/routine", tags=["루틴"], summary="로봇 스케줄 생성")
def create_schedule():
    ...

@router.get("/api/routine", tags=["루틴"], summary="로봇 스케줄 조회")
def get_schedule(user_id: int):
    ...

@router.patch("/api/routine/{routine_id}", tags=["루틴"], summary="로봇 스케줄 편집")
def update_schedule(user_id: int, routine_id: int):
    ...

@router.delete("/api/routine/{routine_id}", tags=["루틴"], summary="로봇 스케줄 삭제")
def delete_schedule(user_id: int, routine_id: int):
    ...
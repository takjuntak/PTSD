from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from PTSD.core.database import get_db
from PTSD.schemas.response import ResponseModel
from pydantic import BaseModel
from PTSD.services.notification_service import create_robot_notification
import asyncio

router = APIRouter()

# 요청 데이터 스키마 정의
class RobotData(BaseModel):
    user_id: int 
    status: str  # "start" or "complete"


@router.post("/api/robot-notification")
async def send_robot_state(payload: RobotData, db: Session = Depends(get_db)):
    try:
        notification_id = await create_robot_notification(
            user_id=payload.user_id,
            status=payload.status,
            db=db
        )
        return ResponseModel(
            isSuccess=True,
            code=200,
            message="로봇 상태 알림이 성공적으로 전송되었습니다.",
            result={"notification_id": notification_id}
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

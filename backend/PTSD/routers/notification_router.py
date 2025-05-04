from fastapi import APIRouter, Depends, Query, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict
from PTSD.core.database import get_db
from PTSD.models.notifications import Notification
from PTSD.schemas.response import ResponseModel  # 공통 응답 포맷
from PTSD.utils.dependency import get_current_user  # 사용자 인증
import logging

router = APIRouter(
    tags=["알림"],
    dependencies=[Depends(get_current_user)] 
)

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@router.get(
    "/api/notifications", 
    tags=["알림"], 
    summary="알림 로그 조회"
)
def get_notification_logs(
    current_user: Dict = Depends(get_current_user),
    sort: str = Query("desc", pattern="^(asc|desc)$", description="정렬 방식 (desc: 최신순, asc: 오래된순)"),
    page: int = Query(1, ge=1, description="페이지 번호 (기본값 1)"),
    limit: int = Query(20, ge=1, le=100, description="페이지당 항목 수 (기본값 20, 최대 100)"),
    db: Session = Depends(get_db)
):
    
    try:
        user_id = current_user["user_id"]
        query = db.query(Notification).filter(Notification.user_id == user_id)

        total_count = db.query(Notification).filter(Notification.user_id == user_id).count()

        if sort == "desc":
            query = query.order_by(Notification.timestamp.desc())
        else:
            query = query.order_by(Notification.timestamp.asc())

        offset = (page - 1) * limit
        logs = query.offset(offset).limit(limit).all()

        logs_list = [
            {
                "notification_id": log.notification_id,
                "timestamp": log.timestamp.isoformat(),
                "type": log.type.value if hasattr(log.type, "value") else log.type,  # Enum 처리
                "title": log.title,
                "message": log.message,
                "is_read": log.is_read
            }
            for log in logs
        ]

        total_pages = (total_count + limit - 1) // limit
        has_next = page < total_pages
        has_previous = page > 1

        return ResponseModel(
            isSuccess=True,
            code=200,
            message="요청에 성공하였습니다.",
            result={
                "total_count": total_count,
                "page": page,
                "limit": limit,
                "total_pages": total_pages,
                "has_next": has_next,
                "has_previous": has_previous,
                "logs": logs_list
            }
        )
    
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"알림 로그 조회 중 오류가 발생했습니다: {str(e)}"
        )
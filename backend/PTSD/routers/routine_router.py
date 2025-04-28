from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict
from pydantic import BaseModel
from datetime import datetime
from typing import Dict, Generic, TypeVar, List
from enum import Enum
from ..schemas.routines import RoutineCreate, RoutineTypeEnum
from ..schemas.response import ResponseModel

from ..models.routines import Routine
from ..core.database import get_db
from ..utils.jwt_handler import get_current_user  
from fastapi.security import OAuth2PasswordBearer


router = APIRouter()
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/auth/login")

@router.post("/api/routine", tags=["루틴"], summary="로봇 스케줄 생성")
def create_routine(
    routine_data: RoutineCreate,
    current_user: Dict = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    
    """    
    - **start_time**: 스케줄 시작 시간 (DateTime)
    - **routine_type**: 스케줄 유형 ('once', 'daily')
    - **isWork**: 작업 활성화 여부
    - **repeat_days**: 반복 요일 (1=월요일, 7=일요일)
    """
    
    try:
        # repeat_days 유효성 검사 (None이 아닌 경우)
        if routine_data.repeat_days:
            if any(day < 1 or day >= 8 for day in routine_data.repeat_days):
                raise HTTPException(
                    status_code=status.HTTP_400_BAD_REQUEST,
                    detail="repeat_days must be between 1 and 7"
                )
                # Routine 모델 인스턴스 생성
                
        new_routine = Routine(
            userId=current_user["user_id"],  # JWT에서 추출한 user_id
            start_time=routine_data.start_time,
            routine_type=routine_data.routine_type.value,  # Enum의 value 사용
            is_work=routine_data.is_work,
            repeat_days=routine_data.repeat_days or []  # None인 경우 빈 리스트
        )
        
        # 데이터베이스에 저장
        db.add(new_routine)
        db.commit()
        db.refresh(new_routine)
        
        
      # 응답 데이터 구성
        response_data = {
            "routine_id": new_routine.routine_id,
            "userId": new_routine.userId,
            "start_time": new_routine.start_time,
            "routine_type": new_routine.routine_type,
            "is_work": new_routine.is_work,
            "repeat_days": new_routine.repeat_days
        }
        
        # ResponseModel 형식으로 응답
        return ResponseModel(
            isSuccess=True,
            code=200,
            message="요청에 성공하였습니다.",
            result=response_data
        )    
    except HTTPException:
        raise
    except Exception as e:
        db.rollback()  # 에러 발생 시 롤백
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"An error occurred while creating the routine: {str(e)}"
        )


@router.get("/api/routine", tags=["루틴"], summary="로봇 스케줄 조회")
def get_routine(user_id: int):
    ...

@router.patch("/api/routine/{routine_id}", tags=["루틴"], summary="로봇 스케줄 편집")
def update_schedule(user_id: int, routine_id: int):
    ...

@router.delete("/api/routine/{routine_id}", tags=["루틴"], summary="로봇 스케줄 삭제")
def delete_schedule(user_id: int, routine_id: int):
    ...
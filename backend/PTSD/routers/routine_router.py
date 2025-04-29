from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import Dict
from pydantic import BaseModel
from datetime import datetime
from typing import Dict, Generic, TypeVar, List
from enum import Enum
from ..schemas.routines import RoutineCreate, RoutineTypeEnum, RoutineUpdate
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
    
        return ResponseModel(
            isSuccess=False,
            code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            message=f"루틴 업데이트 중 오류가 발생했습니다: {str(e)}",
            result=None
        )



@router.get("/api/routine", tags=["루틴"], summary="로봇 스케줄 조회")
def get_routine(
    db: Session = Depends(get_db),
    current_user: Dict = Depends(get_current_user)
    ):
    
    try:
        routines = db.query(Routine).filter(Routine.userId == current_user["user_id"]).all()
        routines_list = []
    
        for routine in routines:
            routines_list.append({
                "routine_id": routine.routine_id,
                "start_time": routine.start_time,
                "routine_type": routine.routine_type,
                "is_work": routine.is_work,
                "repeat_days": routine.repeat_days
            })
    
        return ResponseModel(
            isSuccess=True,
            code=200,
            message="요청에 성공하였습니다.",
            result={"routines": routines_list}
        )

    except Exception as e:
        return ResponseModel(
            isSuccess=False,
            code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            message=f"루틴 업데이트 중 오류가 발생했습니다: {str(e)}",
            result=None
        )


@router.patch("/api/routine/{routine_id}", tags=["루틴"], summary="로봇 스케줄 편집")
def update_routine(
    routine_id: int,
    routine_data: RoutineUpdate,
    current_user: Dict = Depends(get_current_user),
    db: Session = Depends(get_db)):
    """
    로봇 스케줄을 수정합니다.
    
    - **routine_id**: 수정할 스케줄의 ID
    - **start_time**: 스케줄 시작 시간 (DateTime, 선택적)
    - **routine_type**: 스케줄 유형 ('once', 'daily', 선택적)
    - **is_work**: 작업 활성화 여부 (선택적)
    - **repeat_days**: 반복 요일 (1=월요일, 7=일요일, 선택적)
    """
    
    try:
        # 기존 루틴 조회
        routine = db.query(Routine).filter(
            Routine.routine_id == routine_id,
            Routine.userId == current_user["user_id"]
        ).first()
        
        # 루틴이 존재하지 않을 경우
        if not routine:
            return ResponseModel(
                isSuccess=False,
                code=status.HTTP_404_NOT_FOUND,
                message="루틴을 찾을 수 없거나 수정 권한이 없습니다",
                result=None
            )
            
    
        # 데이터 형식 JSON으로 변환    
        update_data = routine_data.model_dump(exclude_unset=True)
        
        # routine_type이 있고 Enum인 경우 value 추출
        if "routine_type" in update_data and update_data["routine_type"] is not None:
            update_data["routine_type"] = update_data["routine_type"].value
        
        # 필드 업데이트
        for key, value in update_data.items():
            setattr(routine, key, value)
        
        # 데이터베이스에 변경사항 저장
        db.commit()
        db.refresh(routine)
        
        # 응답 데이터 구성
        response_data = {
            "routine_id": routine.routine_id,
            "userId": routine.userId,
            "start_time": routine.start_time,
            "routine_type": routine.routine_type,
            "is_work": routine.is_work,
            "repeat_days": routine.repeat_days
        }
        
        # ResponseModel 형식으로 응답 - schemas/response.py에서 가져온 클래스 사용
        return ResponseModel(
            isSuccess=True,
            code=200,
            message="루틴이 성공적으로 업데이트되었습니다.",
            result=response_data
        )
        
    except Exception as e:
        db.rollback()  # 에러 발생 시 롤백
        return ResponseModel(
            isSuccess=False,
            code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            message=f"루틴 업데이트 중 오류가 발생했습니다: {str(e)}",
            result=None
        )



@router.delete("/api/routine/{routine_id}", tags=["루틴"], summary="로봇 스케줄 삭제")
def delete_schedule(user_id: int, routine_id: int):
    ...
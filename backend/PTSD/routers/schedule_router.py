from fastapi import APIRouter
router = APIRouter()

@router.post("/api/schedule", tags=["루틴"], summary="로봇 스케줄 생성")
def create_schedule():
    ...

@router.get("/api/schedule/{user_id}", tags=["루틴"], summary="로봇 스케줄 조회")
def get_schedule(user_id: int):
    ...

@router.patch("/api/schedule/{user_id}/{schedule_id}", tags=["루틴"], summary="로봇 스케줄 편집")
def update_schedule(user_id: int, schedule_id: int):
    ...

@router.delete("/api/schedule/{user_id}/{schedule_id}", tags=["루틴"], summary="로봇 스케줄 삭제")
def delete_schedule(user_id: int, schedule_id: int):
    ...
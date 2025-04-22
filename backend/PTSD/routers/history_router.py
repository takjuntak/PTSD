from fastapi import APIRouter
router = APIRouter()

@router.get("/api/history/operatingtime/{user_id}", tags=["히스토리"], summary="가동 시간 조회")
def get_operating_time(user_id: int):
    ...

@router.get("/api/history/alarmlog/{user_id}", tags=["알림"], summary="알림 로그 조회")
def get_alarm_log(user_id: int):
    ...

@router.delete("/api/history/alarmlog/{user_id}", tags=["알림"], summary="알림 로그 삭제")
def delete_alarm_log(user_id: int):
    ...

@router.get("/api/history/tasklog/{user_id}", tags=["작업 로그"], summary="작업 로그 조회")
def get_task_log(user_id: int):
    ...

@router.delete("/api/history/tasklog/{user_id}", tags=["작업 로그"], summary="작업 로그 삭제")
def delete_task_log(user_id: int):
    ...

@router.post("/api/history/alarm/{user_id}", tags=["알림"], summary="알람 수신 후 DB 저장")
def save_alarm(user_id: int):
    ...
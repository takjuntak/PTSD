from fastapi import APIRouter, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel
from PTSD.utils.websocket_manager import manager  # 웹소켓 매니저
import logging
import math

router = APIRouter()
logger = logging.getLogger(__name__)


# Pydantic 모델
class BatteryData(BaseModel):
    percentage: float


# 배터리 상태용 WebSocket 엔드포인트
@router.websocket("/ws/battery")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    logger.info(f"WebSocket 연결됨: {websocket.client}")
    try:
        while True:
            await websocket.receive_text()  # 연결 유지용
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        logger.info(f"WebSocket 연결 종료됨: {websocket.client}")


# HTTP POST endpoint
@router.post("/api/battery-state")
async def receive_battery_state(data: BatteryData):
    percentage_int = math.floor(data.percentage)  # 소수점 아래 내림 처리
    # message = {"battery": percentage_int }
    # 문자열로 변환->postman에서 테스트용
    message =f"배터리: {percentage_int}"
    
    try:
        await manager.broadcast(message)
        logger.info(f"Battery data broadcasted: {message}")
        return {"message": "Battery state broadcasted"}
    except Exception as e:
        logger.error(f"Failed to broadcast battery state: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to broadcast battery state"
        )

# battery_ws.py
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi import WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from PTSD.utils.websocket_manager import manager  # 웹소켓 매니저
import logging

router = APIRouter()
logger = logging.getLogger(__name__)


# WebSocket endpoint
@router.websocket("/ws/alert")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    logger.info(f"WebSocket 연결됨: {websocket.client}")
    try:
        while True:
            await websocket.receive_text()  # 연결 유지 목적
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        logger.info(f"WebSocket 연결 종료됨: {websocket.client}")

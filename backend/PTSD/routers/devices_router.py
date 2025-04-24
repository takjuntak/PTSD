# routers/devices_router.py
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from PTSD.core.database import get_db
from PTSD.models.devices.devices import Device
from PTSD.schemas.devices import DeviceCreate, DeviceRead
from typing import List

router = APIRouter()

# 📌 기기 등록
@router.post("/api/devices/", response_model=DeviceRead, tags=["기기"], summary="기기 등록")
def create_device(device: DeviceCreate, db: Session = Depends(get_db)):
    db_device = Device(**device.dict())
    db.add(db_device)
    db.commit()
    db.refresh(db_device)
    return db_device

# 📌 전체 기기 조회
@router.get("/api/devices/", response_model=List[DeviceRead], tags=["기기"], summary="전체 기기 조회")
def read_devices(db: Session = Depends(get_db)):
    return db.query(Device).all()

# 📌 사용자별 기기 조회
@router.get("/api/devices/user/{user_id}", response_model=List[DeviceRead], tags=["기기"], summary="사용자별 기기 조회")
def get_devices_by_user(user_id: int, db: Session = Depends(get_db)):
    devices = db.query(Device).filter(Device.user_id == user_id).all()
    return devices

# 📌 기기 상세 조회
@router.get("/api/devices/{device_id}", response_model=DeviceRead, tags=["기기"], summary="기기 상세 조회")
def get_device(device_id: int, db: Session = Depends(get_db)):
    device = db.query(Device).filter(Device.device_id == device_id).first()
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")
    return device

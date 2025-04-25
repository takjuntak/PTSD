from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session
from PTSD.core.database import get_db
from PTSD.models.devices import Device
from PTSD.schemas.devices import DeviceCreate, DeviceRead, DeviceUpdate
from typing import List
from PTSD.utils.dependency import get_current_user 

router = APIRouter(
    tags=["기기"],
    dependencies=[Depends(get_current_user)] 
)

# 📌 기기 등록
@router.post("/api/devices/", response_model=DeviceRead, tags=["기기"], summary="기기 등록")
def create_device(device: DeviceCreate, db: Session = Depends(get_db)):
    db_device = Device(**device.dict())
    db.add(db_device)
    db.commit()
    db.refresh(db_device)
    return db_device


# 📌 기기 단일 조회
@router.get("/api/devices/{device_id}", response_model=DeviceRead, tags=["기기"], summary="기기 조회")
def read_device(device_id: int, db: Session = Depends(get_db)):
    db_device = db.query(Device).filter(Device.device_id == device_id).first()
    if not db_device:
        raise HTTPException(status_code=404, detail="Device not found")
    return db_device


# 📌 기기 수정 (부분 수정용 PATCH)
@router.patch("/api/devices/{device_id}", response_model=DeviceRead, tags=["기기"], summary="기기 수정")
def update_device(device_id: int, device: DeviceUpdate, db: Session = Depends(get_db)):
    db_device = db.query(Device).filter(Device.device_id == device_id).first()
    if not db_device:
        raise HTTPException(status_code=404, detail="Device not found")

    update_data = device.dict(exclude_unset=True)
    for key, value in update_data.items():
        setattr(db_device, key, value)

    db.commit()
    db.refresh(db_device)
    return db_device


# 📌 기기 삭제
@router.delete("/api/devices/{device_id}", status_code=204, tags=["기기"], summary="기기 삭제")
def delete_device(device_id: int, db: Session = Depends(get_db)):
    db_device = db.query(Device).filter(Device.device_id == device_id).first()
    if not db_device:
        raise HTTPException(status_code=404, detail="Device not found")

    db.delete(db_device)
    db.commit()
    return  
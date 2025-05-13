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

# 기기 등록
@router.post(
    "/api/devices/",
    response_model=DeviceRead,
    tags=["기기"],
    summary="기기 등록",
    description="""
**새로운 기기를 등록합니다.**

- 기기의 시리얼 넘버와 이름을 입력해 기기 등록을 완료합니다.

### ✅ [요청 필드]
- `serial_number` : 기기의 고유 시리얼 번호
- `name` : 기기 별칭

### ✅ [응답 필드]
- `device_id` : 등록된 기기의 고유 ID
- `serial_number` : 등록된 기기의 시리얼 번호
- `name` : 등록된 기기의 별칭
- `user_id` : 기기를 등록한 사용자 ID
- `created_at` : 기기 등록일
"""
)

def create_device(
    device: DeviceCreate,
    db: Session = Depends(get_db),
    current_user: dict = Depends(get_current_user),
):
    db_device = Device(
        user_id=current_user["user_id"],  # ✅ 현재 로그인한 사용자 ID로 지정
        serial_number=device.serial_number,
        name=device.name
    )
    db.add(db_device)
    db.commit()
    db.refresh(db_device)
    return db_device



# 기기 단일 조회
@router.get(
    "/api/devices/{device_id}",
    response_model=DeviceRead,
    tags=["기기"],
    summary="기기 단일 조회",
    description="""
 **등록된 기기의 정보를 조회합니다.**

- 기기의 고유 ID를 통해 기기 정보를 가져옵니다.

### ✅ [요청 경로 변수]
- `device_id` : 조회할 기기의 고유 ID

### ✅ [응답 필드]
- `device_id` : 기기의 고유 ID
- `serial_number` : 기기의 시리얼 번호
- `name` : 기기의 별칭
- `user_id` : 기기를 등록한 사용자 ID
- `created_at` : 기기 등록일
"""
)
def read_device(device_id: int, db: Session = Depends(get_db)):
    db_device = db.query(Device).filter(Device.device_id == device_id).first()
    if not db_device:
        raise HTTPException(status_code=404, detail="Device not found")
    return db_device


# 기기 전체 조회
@router.get(
    "/api/devices/",
    response_model=List[DeviceRead],
    tags=["기기"],
    summary="기기 전체 조회",
    description="""
**등록된 모든 기기 정보를 조회합니다.**

- 사용자 인증 후, 본인이 등록한 기기 목록을 모두 조회합니다.

### ✅ [응답 필드]
- 각 기기에 대해:
  - `device_id` : 기기의 고유 ID
  - `serial_number` : 기기의 시리얼 번호
  - `name` : 기기의 별칭
  - `user_id` : 등록한 사용자 ID
  - `created_at` : 등록 일시
"""
)
def read_all_devices(db: Session = Depends(get_db), user=Depends(get_current_user)):
    devices = db.query(Device).filter(Device.user_id == user["user_id"]).all()
    return devices


# 기기 수정 (부분 수정용 PATCH)
@router.patch(
    "/api/devices/{device_id}",
    response_model=DeviceRead,
    tags=["기기"],
    summary="기기 수정",
    description="""
**등록된 기기의 정보를 수정합니다.**

- 수정하고 싶은 필드만 선택적으로 입력할 수 있습니다.

### ✅ [요청 경로 변수]
- `device_id` : 수정할 기기의 고유 ID

### ✅ [요청 필드]
- `serial_number` (선택) : 새로운 시리얼 번호
- `name` (선택) : 새로운 기기 별칭

### ✅ [응답 필드]
- `device_id` : 수정된 기기의 고유 ID
- `serial_number` : 수정된 기기의 시리얼 번호
- `name` : 수정된 기기의 별칭
- `user_id` : 기기를 등록한 사용자 ID
- `created_at` : 기기 등록일
"""
)
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


# 기기 삭제
@router.delete(
    "/api/devices/{device_id}",
    status_code=204,
    tags=["기기"],
    summary="기기 삭제",
    description="""
**등록된 기기를 삭제합니다.**

- 기기의 고유 ID를 통해 해당 기기를 삭제합니다.

### ✅ [요청 경로 변수]
- `device_id` : 삭제할 기기의 고유 ID

### ✅ [응답]
- 성공 시 204 No Content 응답
"""
)
def delete_device(device_id: int, db: Session = Depends(get_db)):
    db_device = db.query(Device).filter(Device.device_id == device_id).first()
    if not db_device:
        raise HTTPException(status_code=404, detail="Device not found")

    db.delete(db_device)
    db.commit()
    return
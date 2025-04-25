from typing import Generic, TypeVar
from pydantic import BaseModel

T = TypeVar("T")

class ResponseModel(BaseModel, Generic[T]):
    isSuccess: bool
    code: int
    message: str
    result: T

    class Config:
        orm_mode = True  # ← 이것도 같이 넣어주는 게 안정적
        validate_by_name = True  # ✅ Pydantic v2에서 allow_population_by_field_name 대체
        populate_by_name = True
        alias_generator = None  # ✅ alias 비활성화
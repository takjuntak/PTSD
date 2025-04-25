from typing import Generic, TypeVar
from pydantic import BaseModel

T = TypeVar("T")

class ResponseModel(BaseModel, Generic[T]):
    isSuccess: bool
    code: int
    message: str
    result: T
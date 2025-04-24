from typing import Generic, TypeVar
from pydantic.generics import GenericModel

T = TypeVar("T")

class ResponseModel(GenericModel, Generic[T]):
    isSuccess: bool
    code: int
    message: str
    result: T

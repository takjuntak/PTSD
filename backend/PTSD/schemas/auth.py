from pydantic import BaseModel, EmailStr, field_validator
from datetime import datetime

class SignupRequest(BaseModel):
    email: EmailStr  # 이메일 형식 자동 검증
    password: str

    @field_validator("password")
    @classmethod
    def validate_password(cls, value):
        if len(value) < 8:
            raise ValueError("비밀번호는 최소 8자 이상이어야 합니다.")
        return value

class LoginRequest(BaseModel):
    email: EmailStr
    password: str
    
class LoginResult(BaseModel):
    email: str
    accessToken: str


class SignupResponse(BaseModel):
    email: str
    createdAt: datetime
    accessToken: str 
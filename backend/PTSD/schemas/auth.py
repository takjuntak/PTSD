from pydantic import BaseModel, EmailStr, field_validator
from pydantic import FieldValidationInfo 
from datetime import datetime
from pydantic import BaseModel, Field

class SignupRequest(BaseModel):
    email: EmailStr  # 이메일 형식 자동 검증
    password: str
    password_confirm: str
    name: str

    @field_validator("password")
    @classmethod
    def validate_password(cls, value):
        if len(value) < 8:
            raise ValueError("비밀번호는 최소 8자 이상이어야 합니다.")
        return value
    
    @field_validator("password_confirm")
    @classmethod
    def passwords_match(cls, value, info: FieldValidationInfo):
        password = info.data.get("password")
        if password is not None and value != password:
            raise ValueError("비밀번호가 일치하지 않습니다.")
        return value
        
class LoginRequest(BaseModel):
    email: EmailStr
    password: str
    
class LoginResult(BaseModel):
    user_id: int = Field(..., alias="userId")
    email: str
    access_token: str = Field(..., alias="accessToken")

    class Config:
        from_attributes = True
        populate_by_name = True
        validate_by_name = True


class SignupResponse(BaseModel):
    email: str
    name: str
    created_at: datetime = Field(..., alias="createdAt")
    access_token: str = Field(..., alias="accessToken")
    message: str

    class Config:
        from_attributes = True
        populate_by_name = True
        validate_by_name = True
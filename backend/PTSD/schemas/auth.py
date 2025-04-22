from pydantic import BaseModel
from datetime import datetime

class SignupRequest(BaseModel):
    email: str
    password: str

class LoginRequest(BaseModel):
    email: str
    password: str

class LoginResult(BaseModel):
    access_token: str
    token_type: str

class SignupResponse(BaseModel):
    email: str
    created_at: datetime
    access_token: str 
    message: str
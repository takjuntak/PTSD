from datetime import datetime, timedelta
from jose import JWTError, jwt
from PTSD.core.config import SECRET_KEY, ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES
import logging
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer

# OAuth2 설정
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")


# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def create_access_token(data: dict):
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)

def decode_access_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None
   
# JWT 토큰에서 사용자 정보 추출하는 함수 추가
def get_current_user(token: str = Depends(oauth2_scheme)):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    
    payload = decode_access_token(token)
    if payload is None:
        raise credentials_exception
    
    # JWT 페이로드에서 필요한 정보 추출
    email: str = payload.get("email")
    user_id: int = payload.get("user_id")
    
    if email is None or user_id is None:
        raise credentials_exception
    
    return {"email": email, "user_id": user_id} 

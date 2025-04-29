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
    print(f"Token received in get_current_user: {token}")  
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    
    payload = decode_access_token(token)
    if payload is None:
        raise credentials_exception
    
    logger.info(f"Decoded payload: {payload}")  # 디코딩 결과 확인

    # email만 추출 (sub 필드에 있음)
    email = payload.get("sub")

    if email is None:
        raise credentials_exception

    # 이메일로 사용자 조회하는 DB 함수 추가
    from PTSD.core.database import get_db
    from sqlalchemy.orm import Session
    from PTSD.models.user import User

    
    db = next(get_db())
    user = db.query(User).filter(User.email == email).first()
    
    if not user:
        raise credentials_exception
    print(f'user_id = {user.userId}')
    return {"email": email, "user_id": user.userId}
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from PTSD.utils.jwt_handler import decode_access_token
import logging
from datetime import datetime, timedelta
from jose import JWTError, jwt

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/auth/login", scheme_name="BearerAuth")

def get_current_user(token: str = Depends(oauth2_scheme)):
    logger.info(f"받은 토큰: {token}")  # 토큰 로깅
    
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="로그인이 필요합니다. 올바른 토큰을 제공해주세요.",
        headers={"WWW-Authenticate": "Bearer"},
    )
    
    # JWT 토큰 디코딩
    payload = decode_access_token(token)
    logger.info(f"디코딩된 페이로드: {payload}")  # 디코딩된 페이로드 확인
    
    if payload is None:
        logger.warning("토큰 디코딩 실패! 인증되지 않은 사용자입니다.")
        raise credentials_exception
        
    logger.info(f"✅ 토큰 디코딩 성공: {payload}")  
    
    # email 추출 (sub 필드에 있음)
    email = payload.get("sub")
    
    # 이메일로 사용자 조회하는 DB 함수 추가
    from PTSD.core.database import get_db
    from sqlalchemy.orm import Session
    from PTSD.models.user import User

    
    if email is None:
        logger.warning("토큰에 이메일 정보가 없습니다.")
        raise credentials_exception
        
    # 이메일로 사용자 DB 조회
    db = next(get_db())
    user = db.query(User).filter(User.email == email).first()
    
    if not user:
        logger.warning(f"이메일 {email}에 해당하는 사용자를 찾을 수 없습니다.")
        raise credentials_exception
    
    logger.info(f"사용자 인증 완료: user_id={user.user_id}, email={email}")
    
    # 사용자 정보 반환
    return {"email": email, "user_id": user.user_id}

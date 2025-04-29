from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from PTSD.utils.jwt_handler import decode_access_token
import logging

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/api/auth/login", scheme_name="BearerAuth")

def get_current_user(token: str = Depends(oauth2_scheme)):
    logger.info(f"받은 토큰: {token}")  # 여기서 토큰이 실제 들어오는지 확인!

    payload = decode_access_token(token)
    logger.info(f"디코딩된 페이로드: {payload}")  # 디코딩된 페이로드 확인

    if payload is None:
        logger.warning("토큰 디코딩 실패! 인증되지 않은 사용자입니다.")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="로그인이 필요합니다. 올바른 토큰을 제공해주세요.",
        )

    logger.info(f"✅ 토큰 디코딩 성공: {payload}")  # 디코딩이 제대로 됐는지 확인!
    
    
    return payload
from passlib.context import CryptContext
import logging

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# 로깅 설정
logging.basicConfig(level=logging.DEBUG)  # DEBUG로 설정하여 더 많은 로그를 확인
logger = logging.getLogger(__name__)

def hash_password(password: str) -> str:
    hashed_password = pwd_context.hash(password)
    logger.debug(f"Hashed password: {hashed_password}")
    return hashed_password

def verify_password(plain_password: str, hashed_password: str) -> bool:
    logger.info(f"Plain password: {plain_password}")
    logger.info(f"Hashed password: {hashed_password}")
    logger.info(f"Verifying password: {plain_password} against hash: {hashed_password}")
    try:
        result = pwd_context.verify(plain_password, hashed_password)
        logger.debug(f"Password verification result: {result}")
        if result:
            logger.info("Password is valid.")
        else:
            logger.warning("Password verification failed.")
        return result
    except Exception as e:
        logger.error(f"Error during password verification: {e}")
        return False

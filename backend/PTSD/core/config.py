from pydantic import BaseSettings
from dotenv import load_dotenv
import os


# 환경 감지 및 적절한 .env 로드
ENV = os.getenv("ENV", "local")
if ENV == "production":
    load_dotenv(".env.prod")
else:
    load_dotenv(".env")

class Settings(BaseSettings):
    DATABASE_URL: str
    SECRET_KEY: str
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60
    ALGORITHM: str = "HS256"

    class Config:
        env_file = ".env"  # 실제로는 이미 load_dotenv로 처리했으므로 생략해도 됨

settings = Settings()


# SECRET_KEY = "your_secret_key"  # 실제 비밀 키는 환경 변수나 다른 안전한 곳에서 관리해야 함
# ALGORITHM = "HS256"
# ACCESS_TOKEN_EXPIRE_MINUTES = 60
#wDFS
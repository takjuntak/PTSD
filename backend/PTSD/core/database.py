from sqlalchemy import create_engine
from sqlalchemy.orm import declarative_base, sessionmaker
from dotenv import load_dotenv
import os

# ✅ .env 파일 로드
load_dotenv()

# ✅ .env에서 DATABASE_URL 가져오기
DATABASE_URL = os.getenv("DATABASE_URL")

# ✅ SQLAlchemy 엔진 생성
engine = create_engine(DATABASE_URL)

# ✅ 세션 생성기
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# ✅ 모델의 베이스 클래스
Base = declarative_base()

# ✅ DB 세션 주입용 함수
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
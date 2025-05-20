from sqlalchemy import create_engine
from sqlalchemy.orm import declarative_base, sessionmaker
from dotenv import load_dotenv
import os

# 실행 환경에 따라 분기
ENV = os.getenv("ENV", "local")  # 기본은 'local'

if ENV == "production":
    load_dotenv(".env.prod")  # 배포 환경용 .env
else:
    load_dotenv(".env")       # 기본 로컬용 .env

# .env에서 DB 연결 URL 로드
# DATABASE_URL = os.getenv("DATABASE_URL").replace("\\x3a", ":")
DATABASE_URL = "postgresql://ptsd:ssafy1234!@k12d101.p.ssafy.io:5432/ptsd"
print("DB URL:", DATABASE_URL)

# SQLAlchemy 엔진 생성
engine = create_engine(DATABASE_URL)
0
try:
    with engine.connect() as conn:
        print("✅ DB 연결 성공!")
except Exception as e:
    print("❌ DB 연결 실패:", e)

# 세션 생성기
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 모델의 베이스 클래스
Base = declarative_base()

# DB 세션 주입 함수
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
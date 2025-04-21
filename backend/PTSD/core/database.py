# database.py
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker

# 데이터베이스 URL 설정
# format: "postgresql://사용자이름:비밀번호@호스트/데이터베이스이름"
DATABASE_URL = "postgresql://postgres:ssafy1234!@5432/PTSD"

# 데이터베이스 엔진 생성
engine = create_engine(DATABASE_URL)
# 세션 생성 클래스 설정
# autocommit=False: 수동으로 커밋해야 변경사항이 저장됨
# autoflush=False: 수동으로 flush 해야 데이터가 DB에 반영됨
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 모델 기본 클래스 생성
Base = declarative_base()


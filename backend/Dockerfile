# 1. Python 이미지 기반
FROM python:3.10-slim

# 2. 작업 디렉토리
WORKDIR /app

# 3. OpenCV 필요한 시스템 패키지 설치
RUN apt-get update && apt-get install -y libgl1 libglib2.0-0

# 4. 의존성 파일 복사 및 설치
COPY requirements.txt .

RUN pip install --no-cache-dir -r requirements.txt

# 5. 전체 소스 복사
COPY . .

# 6. 환경 변수 로딩을 위한 uvicorn 실행 (FastAPI 진입점: PTSD.main:app)
CMD ["uvicorn", "PTSD.main:app", "--host", "0.0.0.0", "--port", "8000"]

# backend/Dockerfile

# 1. Python 이미지 기반
FROM python:3.10-slim

# 2. 작업 디렉토리 설정
WORKDIR /app

# 3. 필요한 파일 복사
COPY ./requirements.txt /app/requirements.txt

# 4. 패키지 설치
RUN pip install --no-cache-dir --upgrade -r /app/requirements.txt

# 5. 소스 복사
COPY ./ /app

# 6. FastAPI 서버 실행
CMD ["uvicorn", "PTSD.main:app", "--host", "0.0.0.0", "--port", "8000", "--reload"]

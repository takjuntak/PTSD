# 빌드 단계
FROM node:22.13.0-alpine as builder

# 작업 디렉토리 설정
WORKDIR /app

# 의존성 설치
COPY package*.json ./
RUN npm install

# 앱 소스 복사 및 빌드 
COPY . .
RUN npm run build   

# 프로덕션 단계
FROM nginx:alpine

# 빌드 결과물 복사
# COPY --from=builder /app/build /usr/share/nginx/html
COPY --from=builder /app/dist /usr/share/nginx/html

# 포트 설정
EXPOSE 80

# 실행 명령어
CMD ["nginx", "-g", "daemon off;"]

# 왜 안바뀌니?

# main.py
from fastapi import FastAPI
from fastapi.security import OAuth2PasswordBearer
from PTSD.exceptions import register_exception_handlers
from PTSD.routers import notification_router, user_router, routine_router , devices_router
from PTSD.routers import battery_status, battery_alert, robot_status, manual_control, auto_control, robot_arm
from PTSD.utils import websocket_manager 
from fastapi.openapi.utils import get_openapi
from fastapi.middleware.cors import CORSMiddleware
from PTSD.core.database import Base, engine
from PTSD.models import notifications, user, routines,devices
from fastapi.openapi.utils import get_openapi
from fastapi.openapi.models import OAuthFlows, OAuthFlowPassword
from PTSD.mqtt.mqtt_battery_listener import start_battery_mqtt_loop
from PTSD.mqtt.mqtt_robot_listener import start_robot_mqtt_loop
from PTSD.utils.notification_deletion_scheduler import start_notification_deletion_scheduler
from PTSD.utils.routine_loader import load_routines_from_db
from PTSD.utils.routine_scheduler import scheduler
from PTSD.mqtt.mqtt_object_detector import start_object_detector, stop_object_detector

import threading
import asyncio

app = FastAPI(
    title="PTSD API",
    description="헬스 정리 로봇 API 문서입니다.",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)

origins = [
    "https://k12d101.p.ssafy.io",     # 배포 서버 도메인
    "http://localhost:3000",          # React 개발용 (http)
    "http://127.0.0.1:3000",          # React 개발용 (localhost IP)
    "http://localhost:5173",
]
# CORS 미들웨어 추가
app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,  # 정확히 지정
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ✅ 예외 핸들러 등록
register_exception_handlers(app)

# ✅ 회원 API 라우터 등록
app.include_router(user_router.router, prefix="/api")
app.include_router(routine_router.router)
app.include_router(notification_router.router)
# app.include_router(devices_router.router)
app.include_router(devices_router.router, prefix="/api")

# ✅ 배터리 웹소켓 라우터 등록
app.include_router(battery_status.router)
app.include_router(battery_alert.router)
app.include_router(websocket_manager.router)

# ✅ 터틀봇 수동 조작 웹소켓 라우터 등록
app.include_router(manual_control.router)

# ✅ 로봇 상태 웹소켓 라우터 등록
app.include_router(robot_status.router)

# ✅ 자동 조작 웹소켓 라우터 등록
app.include_router(auto_control.router)

# ✅ 로봇 팔 조작 웹소켓 라우터 등록
app.include_router(robot_arm.router)

# ✅ 서버 시작 시 테이블 생성
@app.on_event("startup")
async def startup_event():
    Base.metadata.create_all(bind=engine)
    print("테이블 생성 완료!")
    thread_battery = threading.Thread(target=start_battery_mqtt_loop)
    thread_battery.daemon = True
    thread_robot = threading.Thread(target=start_robot_mqtt_loop)
    thread_robot.daemon = True
    thread_battery.start()
    thread_robot.start()

    # 앱 시작 시 알림 삭제 스케줄러 시작
    start_notification_deletion_scheduler()
    if not scheduler.running:
        scheduler.start()
        print("루틴 스케줄러 시작")
    # DB에 저장된 활성 루틴을 불러와 APScheduler에 등록
    load_routines_from_db()
    
    # 🚀 객체 감지기도 스레드로 동작하도록 수정
    # thread_objdet = threading.Thread(target=start_object_detector, daemon=True)
    # thread_objdet.start()
    
    # # 객체 감지기 시작 (이 한 줄만 추가)
    start_object_detector()

# 서버 종료 시 리소스 정리를 위한 shutdown 이벤트 핸들러 추가
@app.on_event("shutdown")
async def shutdown_event():
    # 객체 감지기 중지
    stop_object_detector()


# ✅ Swagger에 Bearer Token 인증 정보 추가
from fastapi.openapi.models import APIKey, APIKeyIn, SecuritySchemeType
from fastapi.openapi.models import SecurityScheme
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.openapi.models import OAuthFlows as OAuthFlowsModel

# 커스텀 openapi 스키마 정의
def custom_openapi():
    if app.openapi_schema:
        return app.openapi_schema

    # ✅ 핵심: Swagger 문서에서 alias 사용하지 않도록 설정
    openapi_schema = get_openapi(
        title=app.title,
        version=app.version,
        description=app.description,
        routes=app.routes
    )
    
    # ✅ components가 없으면 초기화
    if "components" not in openapi_schema:
        openapi_schema["components"] = {}

    # ✅ securitySchemes 설정
    openapi_schema["components"]["securitySchemes"] = {
        "BearerAuth": {
            "type": "http",
            "scheme": "bearer",
            "bearerFormat": "JWT"
        }
    }

    # ✅ 모든 경로에 기본 인증 요구사항 추가
    openapi_schema["security"] = [{"BearerAuth": []}]

    app.openapi_schema = openapi_schema
    return openapi_schema

app.openapi = custom_openapi

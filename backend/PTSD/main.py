from fastapi import FastAPI
from fastapi.security import OAuth2PasswordBearer
from PTSD.exceptions import register_exception_handlers
from PTSD.routers import notification_router, user_router, routine_router , devices_router, mqtt_router
from fastapi.openapi.utils import get_openapi
from fastapi.middleware.cors import CORSMiddleware
from PTSD.core.database import Base, engine
from PTSD.models import notifications, user, routines,devices
from fastapi.openapi.utils import get_openapi
from fastapi.openapi.models import OAuthFlows, OAuthFlowPassword
import paho.mqtt.client as mqtt
import logging


app = FastAPI(
    title="PTSD API",
    description="헬스 정리 로봇 API 문서입니다.",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)


# CORS 미들웨어 추가
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # 개발용. 운영에서는 특정 도메인만 지정
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ✅ 예외 핸들러 등록
register_exception_handlers(app)

# ✅ 회원 API 라우터 등록
app.include_router(user_router.router, tags=["회원관리"])
app.include_router(routine_router.router)
app.include_router(notification_router.router)
app.include_router(devices_router.router)

# ✅ MQTT 라우터 등록
app.include_router(mqtt_router.router)

# ✅ 서버 시작 시 테이블 생성
@app.on_event("startup")
async def startup_event():
    Base.metadata.create_all(bind=engine)
    print("테이블 생성 완료!")

    # MQTT 클라이언트 연결 설정
    mqtt_client.connect("localhost", 1883, 60)
    mqtt_client.loop_start()  # 비동기적으로 메시지 처리 시작

# ✅ 서버 종료 시 MQTT 클라이언트 종료
@app.on_event("shutdown")
async def shutdown_event():
    mqtt_client.loop_stop()  # MQTT 메시지 루프 종료
    mqtt_client.disconnect()  # MQTT 연결 종료

# ✅ MQTT 클라이언트 설정
mqtt_client = mqtt.Client()

# 메시지 수신 처리
received_messages = []

def on_connect(client, userdata, flags, rc):
    print(f"MQTT Connected with result code {rc}")
    client.subscribe("iot/notification")  # 알림을 받을 토픽을 구독

def on_message(client, userdata, msg):
    message = msg.payload.decode()
    print(f"Received MQTT message: {message}")
    received_messages.append(message)  # 수신된 메시지를 리스트에 저장

mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

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

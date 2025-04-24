from fastapi import FastAPI
from fastapi.security import OAuth2PasswordBearer
from PTSD.exceptions import register_exception_handlers
from PTSD.routers import user_router, routine_router , history_router, devices_router
from fastapi.openapi.utils import get_openapi
from fastapi.middleware.cors import CORSMiddleware
from PTSD.core.database import Base, engine
from PTSD.models import user, routines


app = FastAPI(
    title="PTSD API",
    description="헬스 정리 로봇 API 문서입니다.",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)



# ✅ 예외 핸들러 등록
register_exception_handlers(app)

# ✅ 회원 API 라우터 등록
app.include_router(user_router.router, tags=["회원관리"])
app.include_router(routine_router.router)
app.include_router(history_router.router)
app.include_router(devices_router.router)

# ✅ 서버 시작 시 테이블 생성
@app.on_event("startup")
async def startup_event():
    Base.metadata.create_all(bind=engine)
    print("테이블 생성 완료!")


# ✅ Swagger에 Bearer Token 인증 정보 추가
from fastapi.openapi.models import APIKey, APIKeyIn, SecuritySchemeType
from fastapi.openapi.models import SecurityScheme
from fastapi.openapi.docs import get_swagger_ui_html
from fastapi.openapi.models import OAuthFlows as OAuthFlowsModel

# 커스텀 openapi 스키마 정의
def custom_openapi():
    if app.openapi_schema:
        return app.openapi_schema

    openapi_schema = get_openapi(
        title=app.title,
        version=app.version,
        description=app.description,
        routes=app.routes,
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
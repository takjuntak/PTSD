from fastapi import FastAPI
from PTSD.exceptions import register_exception_handlers
from PTSD.routers import user_router

app = FastAPI(
    title="PTSD API",
    description="헬스 정리 로봇 API 문서입니다.",
    version="1.0.0",
    docs_url="/docs",          # ✅ Swagger UI 경로 (기본값)
    redoc_url="/redoc",        # ✅ ReDoc 문서 경로
    openapi_url="/openapi.json",
)
register_exception_handlers(app)

app.include_router(user_router.router, tags=["회원관리"])
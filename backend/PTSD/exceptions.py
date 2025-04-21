# ✅ exceptions.py

from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException

# 예외 응답 모델
class FieldErrorDetail(BaseModel):
    field: str
    message: str

class ErrorResponse(BaseModel):
    statusCode: int
    message: str
    errors: list[FieldErrorDetail] | None = None

# ✅ 핸들러 등록 함수들만 정의 (FastAPI 인스턴스는 인자로 받음)
def register_exception_handlers(app):
    @app.exception_handler(ValidationError)
    async def validation_exception_handler(request: Request, exc: ValidationError):
        field_errors = [
            FieldErrorDetail(field=e['loc'][-1], message=e['msg']) for e in exc.errors()
        ]
        return JSONResponse(
            status_code=status.HTTP_400_BAD_REQUEST,
            content=ErrorResponse(
                statusCode=400,
                message="잘못된 요청입니다. 입력 값을 확인해주세요.",
                errors=field_errors
            ).dict()
        )

    @app.exception_handler(HTTPException)
    async def unauthorized_exception_handler(request: Request, exc: HTTPException):
        if exc.status_code == 401:
            return JSONResponse(
                status_code=401,
                content=ErrorResponse(
                    statusCode=401,
                    message="인증이 필요합니다. 로그인 후 다시 시도해주세요.",
                    errors=None
                ).dict()
            )
        elif exc.status_code == 422:
            return JSONResponse(
                status_code=422,
                content=ErrorResponse(
                    statusCode=422,
                    message="요청 데이터를 처리할 수 없습니다.",
                    errors=None
                ).dict()
            )
        raise exc

    @app.exception_handler(StarletteHTTPException)
    async def starlette_exception_handler(request: Request, exc: StarletteHTTPException):
        if exc.status_code == 403:
            return JSONResponse(
                status_code=403,
                content=ErrorResponse(
                    statusCode=403,
                    message="접근이 거부되었습니다.",
                    errors=None
                ).dict()
            )
        elif exc.status_code == 404:
            return JSONResponse(
                status_code=404,
                content=ErrorResponse(
                    statusCode=404,
                    message="요청하신 리소스를 찾을 수 없습니다.",
                    errors=None
                ).dict()
            )
        raise exc

    @app.exception_handler(ValueError)
    async def conflict_handler(request: Request, exc: ValueError):
        return JSONResponse(
            status_code=409,
            content=ErrorResponse(
                statusCode=409,
                message="이미 존재하는 데이터입니다.",
                errors=None
            ).dict()
        )

    @app.exception_handler(Exception)
    async def global_exception_handler(request: Request, exc: Exception):
        return JSONResponse(
            status_code=500,
            content=ErrorResponse(
                statusCode=500,
                message="서버 내부 오류가 발생했습니다.",
                errors=None
            ).dict()
        )

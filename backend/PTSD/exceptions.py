from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from pydantic import ValidationError, BaseModel
from starlette.exceptions import HTTPException as StarletteHTTPException
import logging

logger = logging.getLogger(__name__)

class FieldErrorDetail(BaseModel):
    field: str
    message: str

class ErrorResponse(BaseModel):
    isSuccess: bool = False
    code: int
    message: str
    errors: list[FieldErrorDetail] | None = None


def register_exception_handlers(app):
    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        logger.error(f"HTTPException: {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content=ErrorResponse(
                isSuccess=False,
                code=exc.status_code,
                message=str(exc.detail),
                errors=None
            ).model_dump()
        )

    @app.exception_handler(StarletteHTTPException)
    async def starlette_http_exception_handler(request: Request, exc: StarletteHTTPException):
        logger.error(f"StarletteHTTPException: {exc.detail}")
        return JSONResponse(
            status_code=exc.status_code,
            content=ErrorResponse(
                isSuccess=False,
                code=exc.status_code,
                message=str(exc.detail),
                errors=None
            ).model_dump()
        )

    @app.exception_handler(RequestValidationError)
    async def request_validation_exception_handler(request: Request, exc: RequestValidationError):
        field_errors = [
            FieldErrorDetail(
                field=e['loc'][-1],
                message=e.get('msg')
            ) for e in exc.errors()
        ]
        return JSONResponse(
            status_code=400,
            content=ErrorResponse(
                isSuccess=False,
                code=400,
                message="입력 형식을 확인해주세요.",
                errors=field_errors
            ).model_dump()
        )

    @app.exception_handler(ValidationError)
    async def validation_exception_handler(request: Request, exc: ValidationError):
        field_errors = [
            FieldErrorDetail(
                field=e['loc'][-1],
                message=e.get('msg')
            ) for e in exc.errors()
        ]
        return JSONResponse(
            status_code=400,
            content=ErrorResponse(
                isSuccess=False,
                code=400,
                message="입력 형식을 확인해주세요.",
                errors=field_errors
            ).model_dump()
        )

    @app.exception_handler(Exception)
    async def global_exception_handler(request: Request, exc: Exception):
        logger.exception("Unhandled Exception")
        return JSONResponse(
            status_code=500,
            content=ErrorResponse(
                isSuccess=False,
                code=500,
                message="서버 내부 오류가 발생했습니다.",
                errors=None
            ).model_dump()
        )

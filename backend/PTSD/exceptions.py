from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from pydantic import BaseModel, ValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
from fastapi.exceptions import RequestValidationError

# 예외 응답 모델
class FieldErrorDetail(BaseModel):
    field: str
    message: str

class ErrorResponse(BaseModel):
    isSuccess: bool = False
    code: int
    message: str
    errors: list[FieldErrorDetail] | None = None


# 핸들러 등록 함수들만 정의 (FastAPI 인스턴스는 인자로 받음)
def register_exception_handlers(app):
    @app.exception_handler(RequestValidationError)
    async def request_validation_exception_handler(request: Request, exc: RequestValidationError):
        field_errors = [
            FieldErrorDetail(
                field=e['loc'][-1],
                message=e.get('ctx', {}).get('reason', e['msg'])
            ) for e in exc.errors()
        ]
        return JSONResponse(
            status_code=status.HTTP_400_BAD_REQUEST,
            content=ErrorResponse(
                isSuccess=False,
                code=400,
                message="입력 형식을 확인해주세요.",
                errors=field_errors
            ).model_dump()
        )

    @app.exception_handler(ValidationError)
    async def validation_exception_handler(request: Request, exc: ValidationError):
        field_errors = []
        for e in exc.errors():
            field = e['loc'][-1]
            message = e.get('msg', '')

             # 이메일 형식 오류 메시지 커스터마이즈
            if 'email' in field and "The part after the @-sign is not valid" in message:
                message = "이메일 형식이 잘못되었습니다. @ 뒤에 도메인을 확인하세요."
            
            # 비밀번호 길이 오류 메시지 커스터마이즈
            elif 'password' in field and "ensure this value has at least 8 characters" in message:
                message = "비밀번호는 최소 8자 이상이어야 합니다."

            field_errors.append(FieldErrorDetail(field=field, message=message))

        return JSONResponse(
            status_code=status.HTTP_400_BAD_REQUEST,
            content=ErrorResponse(
                isSuccess=False,
                code=400,
                message="입력 형식을 확인해주세요.",
                errors=field_errors
            ).model_dump()
        )

    @app.exception_handler(HTTPException)
    async def unauthorized_exception_handler(request: Request, exc: HTTPException):
        if exc.status_code == 401:
            return JSONResponse(
                status_code=401,
                content=ErrorResponse(
                    isSuccess=False,
                    code=401,
                    message="인증이 필요합니다. 로그인 후 다시 시도해주세요.",
                    errors=None
                ).model_dump()
            )
        elif exc.status_code == 422:
            return JSONResponse(
                status_code=422,
                content=ErrorResponse(
                    isSuccess=False,
                    code=422,
                    message="요청 데이터를 처리할 수 없습니다.",
                    errors=None
                ).model_dump()
            )
        raise exc

    @app.exception_handler(StarletteHTTPException)
    async def starlette_exception_handler(request: Request, exc: StarletteHTTPException):
        if exc.status_code == 403:
            return JSONResponse(
                status_code=403,
                content=ErrorResponse(
                    isSuccess=False,
                    code=403,
                    message="접근이 거부되었습니다.",
                    errors=None
                ).model_dump()
            )
        elif exc.status_code == 404:
            return JSONResponse(
                status_code=404,
                content=ErrorResponse(
                    isSuccess=False,
                    code=404,
                    message="요청하신 리소스를 찾을 수 없습니다.",
                    errors=None
                ).model_dump()
            )
        raise exc
    
    @app.exception_handler(ValueError)
    async def conflict_handler(request: Request, exc: ValueError):
        # 로그인 시 발생하는 '409 Conflict'를 '401 Unauthorized'로 변경
        if isinstance(exc, ValueError):
            return JSONResponse(
                status_code=401,
                content=ErrorResponse(
                    isSuccess=False,
                    code=401,
                    message="로그인 정보가 잘못되었습니다.",
                    errors=None
                ).model_dump()
            )
        return JSONResponse(
            status_code=409,
            content=ErrorResponse(
                isSuccess=False,
                code=409,
                message="이미 존재하는 데이터입니다.",
                errors=None
            ).model_dump()
        )


    @app.exception_handler(Exception)
    async def global_exception_handler(request: Request, exc: Exception):
        return JSONResponse(
            status_code=500,
            content=ErrorResponse(
                isSuccess=False,
                code=500,
                message="서버 내부 오류가 발생했습니다.",
                errors=None
            ).model_dump()
        )

    @app.exception_handler(StarletteHTTPException)
    async def custom_http_exception_handler(request: Request, exc: StarletteHTTPException):
        return JSONResponse(
            status_code=exc.status_code,
            content={
                "isSuccess": False,
                "code": exc.status_code,
                "message": str(exc.detail),
                "result": None
            },
        )
    @app.exception_handler(HTTPException)
    async def unauthorized_exception_handler(request: Request, exc: HTTPException):
        if exc.status_code == 401:
            return JSONResponse(
                status_code=401,
                content=ErrorResponse(
                    isSuccess=False,
                    code=401,
                    message="인증이 필요합니다. 로그인 후 다시 시도해주세요.",
                    errors=None
                ).model_dump()
            )
        elif exc.status_code == 422:
            return JSONResponse(
                status_code=422,
                content=ErrorResponse(
                    isSuccess=False,
                    code=422,
                    message="요청 데이터를 처리할 수 없습니다.",
                    errors=None
                ).model_dump()
            )
        raise exc

    @app.exception_handler(StarletteHTTPException)
    async def starlette_exception_handler(request: Request, exc: StarletteHTTPException):
        if exc.status_code == 403:
            return JSONResponse(
                status_code=403,
                content=ErrorResponse(
                    isSuccess=False,
                    code=403,
                    message="접근이 거부되었습니다.",
                    errors=None
                ).model_dump()
            )
        elif exc.status_code == 404:
            return JSONResponse(
                status_code=404,
                content=ErrorResponse(
                    isSuccess=False,
                    code=404,
                    message="요청하신 리소스를 찾을 수 없습니다.",
                    errors=None
                ).model_dump()
            )
        raise exc

    @app.exception_handler(ValueError)
    async def conflict_handler(request: Request, exc: ValueError):
        return JSONResponse(
            status_code=409,
            content=ErrorResponse(
                isSuccess=False,
                code=409,
                message="이미 존재하는 데이터입니다.",
                errors=None
            ).model_dump()
        )

    @app.exception_handler(Exception)
    async def global_exception_handler(request: Request, exc: Exception):
        return JSONResponse(
            status_code=500,
            content=ErrorResponse(
                isSuccess=False,
                code=500,
                message="서버 내부 오류가 발생했습니다.",
                errors=None
            ).model_dump()
        )
#제발 되라라
from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.orm import Session
from pydantic import BaseModel
from PTSD.core.database import SessionLocal
from PTSD.models.user import User
from PTSD.schemas.auth import SignupRequest, LoginRequest, SignupResponse, LoginResult
from PTSD.utils.jwt_handler import create_access_token
from PTSD.utils.security import hash_password, verify_password
import logging


# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter()

# DB 연결
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
        
# 회원가입 API
@router.post(
    "/api/auth/signup",
    summary="회원가입",
    description="""  
📌 **회원가입을 진행합니다.**

- 이메일과 비밀번호를 입력해 회원가입을 완료합니다.

### ✅ [요청 필드]
- `email` : 회원 이메일 주소
- `password` : 회원 비밀번호

### ✅ [응답 필드]
- `email` : 가입한 이메일 주소
- `created_at` : 계정 생성일
- `access_token` : 인증 토큰
""",
    response_model=SignupResponse
)
def signup(payload: SignupRequest, db: Session = Depends(get_db)):
    # 이메일 중복 체크
    user = db.query(User).filter(User.email == payload.email).first()
    if user:
        raise HTTPException(status_code=400, detail="이메일이 이미 존재합니다.")

    # 비밀번호 해싱 후 저장
    hashed_password = hash_password(payload.password)
    new_user = User(email=payload.email, password=hashed_password)
    db.add(new_user)
    db.commit()
    db.refresh(new_user)

    # JWT 토큰 생성
    access_token = create_access_token({"sub": new_user.email})

    logger.info(f"회원가입 성공: 이메일 {new_user.email}")
    return SignupResponse(
        email=new_user.email,
        created_at = new_user.created_at,
        access_token=access_token,
        message="회원가입이 성공적으로 완료되었습니다."
    ).model_dump(by_alias=True)


#로그인
# @router.post("/api/auth/login", summary="로그인")
# def login():
#     return {"msg": "user login"}

@router.post(
    "/api/auth/login",
    summary="로그인",
    description="""
📌 **로그인을 진행합니다.**

- 이메일과 비밀번호를 입력해 로그인합니다.

### ✅ [요청 필드]
- `email` : 회원 이메일 주소
- `password` : 회원 비밀번호

### ✅ [응답 필드]
- `email` : 로그인한 사용자 이메일
- `access_token` : JWT 인증 토큰
""",
    response_model=LoginResult
)
def login(payload: LoginRequest, db: Session = Depends(get_db)):
    # 이메일로 사용자 검색
    user = db.query(User).filter(User.email == payload.email).first()
    if not user:
        raise HTTPException(status_code=401, detail="이메일 또는 비밀번호가 올바르지 않습니다.")

    # 비밀번호 확인
    if not verify_password(payload.password, user.password):
        raise HTTPException(status_code=401, detail="이메일 또는 비밀번호가 올바르지 않습니다.")

    # JWT 발급
    access_token = create_access_token({"sub": user.email})

    logger.info(f"로그인 성공: 이메일 {user.email}")
    return LoginResult(email=user.email, access_token=access_token).model_dump(by_alias=True)
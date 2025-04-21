from fastapi import APIRouter

router = APIRouter()

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
""",

)
def signup():
    return {"msg": "user signup"}

@router.post("/api/auth/login", summary="로그인")
def login():
    return {"msg": "user login"}
from fastapi import APIRouter

router = APIRouter()

@router.post("/api/auth/signup", summary="회원가입")
def admin_signup():
    return {"msg": "user signup"}

@router.post("/api/auth/login", summary="로그인")
def admin_login():
    return {"msg": "user login"}
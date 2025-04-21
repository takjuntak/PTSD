from fastapi import FastAPI

app = FastAPI()


@app.get("/") # 요청 url 경로
def root():
    return {"message": "Hello World"} # 반환할 데이터


@app.get("/home")
def home():
    return {"message": "home"}
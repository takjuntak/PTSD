from fastapi import FastAPI
from PTSD.exceptions import register_exception_handlers

app = FastAPI()

register_exception_handlers(app)


@app.get("/")
def root():
    return {"message": "Hello, PTSD Project!"}


@app.get("/home")
def home():
    return {"message": "home"}
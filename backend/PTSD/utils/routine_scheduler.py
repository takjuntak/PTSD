from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.date import DateTrigger
from apscheduler.triggers.cron import CronTrigger
import paho.mqtt.publish as publish
from datetime import datetime
from typing import List
import pytz
from dotenv import load_dotenv
import os

# .env 파일을 로드합니다.
load_dotenv()

# 환경 변수에서 값을 불러옵니다.
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")  # 기본값 설정
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))      # 문자열 -> 정수 변환 + 기본값

# APScheduler 초기화
scheduler = BackgroundScheduler(timezone="Asia/Seoul")
scheduler.start()

# MQTT 명령 전송 함수
def send_mqtt_command(routine_id: int, command: str = "start"):
    topic = f"robot/routine/{routine_id}"
    payload = command
    print(f"[스케줄 실행] MQTT Publish: topic={topic}, payload={payload}")

    publish.single(
        topic=topic,
        payload=payload,
        hostname=MQTT_BROKER,
        port=MQTT_PORT
    )

# 루틴 예약 함수
def schedule_routine(
    routine_id: int,
    routine_type: str,  # 'once' or 'daily'
    start_time: datetime,
    repeat_days: List[str] = []
):
    job_id = f"routine_{routine_id}"

    # 기존 작업 제거 (중복 방지)
    try:
        scheduler.remove_job(job_id)
    except:
        pass

    if routine_type == "once":
        trigger = DateTrigger(run_date=start_time)

    elif routine_type == "daily":
        if not repeat_days:
            print(f"[오류] repeat_days가 비어 있습니다.")
            return
        
        # 사용자 요일(1~7)을 cron 요일(0~6)로 변환
        try:
            cron_days = [str(int(day) % 7) for day in repeat_days]
            days = ",".join(cron_days)
        except ValueError:
            print(f"[오류] repeat_days 값이 잘못되었습니다: {repeat_days}")
            return

        trigger = CronTrigger(day_of_week=days, hour=start_time.hour, minute=start_time.minute)

    else:
        print(f"[오류] 지원되지 않는 루틴 타입: {routine_type}")
        return

    scheduler.add_job(
        send_mqtt_command,
        trigger=trigger,
        args=[routine_id],
        id=job_id
    )
    print(f"[예약 완료] 루틴 ID {routine_id}, 타입: {routine_type}, 시간: {start_time}, 요일: {repeat_days}")

# 예약 취소 함수
def cancel_routine_schedule(routine_id: int):
    job_id = f"routine_{routine_id}"
    try:
        scheduler.remove_job(job_id)
        print(f"[예약 취소] 루틴 ID {routine_id}")
    except Exception as e:
        print(f"[오류] 루틴 예약 취소 실패: {e}")

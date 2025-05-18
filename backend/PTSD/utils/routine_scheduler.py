from apscheduler.schedulers.background import BackgroundScheduler
from apscheduler.triggers.date import DateTrigger
from apscheduler.triggers.cron import CronTrigger
import paho.mqtt.publish as publish
from datetime import datetime
from PTSD.core.database import SessionLocal 
from PTSD.models.routines import Routine
from typing import List
from pytz import timezone
from dotenv import load_dotenv
import os
import logging

# .env 파일을 로드합니다.
load_dotenv()

# 환경 변수에서 값을 불러옵니다.
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")  # 기본값 설정
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))      # 문자열 -> 정수 변환 + 기본값

# APScheduler 초기화
scheduler = BackgroundScheduler(timezone=timezone("Asia/Seoul"))

# 로깅 설정
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

# is_work 상태를 저장하는 딕셔너리
routine_status = {}

# MQTT 명령 전송 함수
def send_mqtt_command(command: str = "start"):
    """
    robot/auto-control 토픽으로 payload=command를 전송.
    기본값은 'start'이며, 필요시 'complete' 등 다른 문자열을 넣어 호출.
    """
    topic = "robot/auto_control"
    payload = command
    logger.info(f"[MQTT Publish] topic={topic}, payload={payload}")

    publish.single(
        topic=topic,
        payload=payload,
        hostname=MQTT_BROKER,
        port=MQTT_PORT
    )

# once 스케줄 실행 시 상태 확인 및 업데이트하는 래퍼
def execute_once(routine_id: int):
    session = SessionLocal()
    try:
        routine = session.query(Routine).filter(Routine.routine_id == routine_id).first()
        if routine and routine.is_work:
            logger.info(f"[once 스케줄 실행] Routine {routine_id} is_work=True, 실행합니다.")
            send_mqtt_command()

            # DB 상태 변경
            routine.is_work = False
            session.commit()

            logger.info(f"[상태 변경] Routine {routine_id}, is_work=False")
        else:
            logger.info(f"[스케줄 스킵] Routine {routine_id} is_work=False, 실행되지 않습니다.")
    except Exception as e:
        session.rollback()
        logger.error(f"[스케줄러 에러] {e}")
    finally:
        session.close()

# daily 스케줄 실행 시 상태 확인 및 업데이트하는 래퍼
def execute_daily_routine(routine_id: int):
    session = SessionLocal()
    logger.info("[스케줄러 실행] daily 루틴 실행")
    logger.info(f"[스케줄러 실행됨] routine_id={routine_id}, time={datetime.now()}")

    try:
        routine = session.query(Routine).filter(Routine.routine_id == routine_id).first()
        if routine and routine.is_work:
            logger.info(f"[daily 스케줄 실행] Routine {routine_id} is_work=True, 실행합니다.")
            send_mqtt_command()
            # daily는 한 번 실행 후에도 계속 반복되므로 is_work 상태 변경 안 함
        else:
            logger.info(f"[스케줄 스킵] Routine {routine_id} is_work=False, 실행되지 않습니다.")
    except Exception as e:
        session.rollback()
        logger.error(f"[스케줄러 에러] {e}")
    finally:
        session.close()

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
        logger.info(f"[기존 작업 제거] Job ID: {job_id}")
    except:
        pass

    # 초기 상태 설정
    routine_status[routine_id] = True

    seoul = timezone("Asia/Seoul")
    now = datetime.now(seoul)

    if routine_type == "once":
        # start_time이 naive datetime이면 Asia/Seoul timezone 지정
        if start_time.tzinfo is None:
            start_time = seoul.localize(start_time)

        # 현재 시간보다 이전이면 예약 안함
        if start_time <= now:
            logger.warning(f"[예약 실패] 루틴 ID {routine_id}의 once 시작 시간 {start_time}은 현재 시간 {now}보다 이전입니다.")
            return

        trigger = DateTrigger(run_date=start_time)
        job_func = execute_once 

    elif routine_type == "daily":
        if not repeat_days:
            logger.error(f"[오류] repeat_days가 비어 있습니다.")
            return
        try:
            cron_days = [(int(day) - 1) % 7 for day in repeat_days]
            cron_days_str = ",".join(str(d) for d in cron_days)
            logger.info(f"[루틴 요일] {repeat_days} → {cron_days_str}")
        except ValueError:
            logger.error(f"[오류] repeat_days 값이 잘못되었습니다: {repeat_days}")
            return
        
        # daily 예약 시도
        logger.info(f"daily 루틴 예약: {start_time}, hour={start_time.hour}, minute={start_time.minute}, 요일: {repeat_days}")
        trigger = CronTrigger(day_of_week=cron_days_str, hour=start_time.hour, minute=start_time.minute)
        job_func = execute_daily_routine

    else:
        logger.error(f"[오류] 지원되지 않는 루틴 타입: {routine_type}")
        return

    # ✅ job_func과 trigger를 기반으로 작업 등록
    scheduler.add_job(
        func=job_func,
        trigger=trigger,
        args=[routine_id],
        id=job_id
    )
    logger.info(f"[예약 완료] 루틴 ID {routine_id}, 타입: {routine_type}, 시간: {start_time}, 요일: {repeat_days}")

# 예약 취소 함수
def cancel_routine_schedule(routine_id: int):
    job_id = f"routine_{routine_id}"
    try:
        job = scheduler.get_job(job_id)
        if job:
            scheduler.remove_job(job_id)
            routine_status.pop(routine_id, None)
            logger.info(f"[예약 취소] 루틴 ID {routine_id}")
        else:
            logger.warning(f"[취소 스킵] 루틴 ID {routine_id}는 예약되어 있지 않음")
    except Exception as e:
        logger.error(f"[오류] 루틴 예약 취소 실패: {e}")

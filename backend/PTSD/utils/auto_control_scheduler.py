import logging
from datetime import datetime

from apscheduler.schedulers.background import BackgroundScheduler
from sqlalchemy.orm import Session

from PTSD.core.database import get_db
from PTSD.models.routines import Routine
from PTSD.models.devices import Device  # 디바이스 모델 추가
import paho.mqtt.publish as publish
from dotenv import load_dotenv
import os

# .env 파일을 로드합니다.
load_dotenv()

# 환경 변수에서 MQTT_BROKER 값을 불러옵니다.
MQTT_BROKER = os.getenv("MQTT_BROKER")
MQTT_PORT = os.getenv("MQTT_PORT")

# logging 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_routines(db: Session):
    """DB에서 활성화된 루틴들을 조회"""
    return db.query(Routine).filter(Routine.is_work == True).all()

def execute_routine(db: Session, routine: Routine):
    """루틴에 해당하는 작업을 수행하는 함수"""
    # 유저에 연결된 디바이스 조회
    device = db.query(Device).filter(Device.user_id == routine.user_id).first()
    if not device:
        logger.warning(f"[루틴 실행 스킵] user_id {routine.user_id}에 연결된 device 없음")
        return
    
    serial_number = device.serial_number
    topic = "robot/auto-control"
    payload = "auto_control_start" 

    try:
        logger.info(f"Sending MQTT message to {topic}: {payload}")
        publish.single(topic, payload=payload, hostname=MQTT_BROKER, port=MQTT_PORT)
        logger.info(f"[루틴 실행 완료] routine_id: {routine.routine_id}, serial: {serial_number}, routine_type: {routine.routine_type}, start_time: {routine.start_time}")
    except Exception as e:
        logger.error(f"[MQTT 전송 실패] {e}")

def schedule_routine():
    """스케줄러가 주기적으로 실행되는 함수"""
    db: Session = next(get_db())
    try:
        routines = get_routines(db)

        if not routines:
            logger.info("[스케줄러] 활성화된 루틴 없음")
            return

        now = datetime.now()
        current_day = now.weekday() + 1  # 월요일: 1, 일요일: 7

        for routine in routines:
            start_time = routine.start_time
            if start_time is None:
                continue

            if routine.routine_type == "once":
                if now >= start_time:
                    execute_routine(db, routine)
                    routine.is_work = False
                    db.commit()

            elif routine.routine_type == "daily":
                if current_day in routine.repeat_days:
                    if now.hour == start_time.hour and now.minute == start_time.minute:
                        execute_routine(db, routine)

    except Exception as e:
        logger.error(f"[스케줄러 오류] {str(e)}")
    finally:
        db.close()

def start_auto_control_scheduler():
    """스케줄러 시작 함수"""
    scheduler = BackgroundScheduler()
    scheduler.add_job(schedule_routine, 'interval', minutes=1)
    scheduler.start()
    logger.info("[스케줄러 시작됨] 루틴을 1분 간격으로 검사합니다.")

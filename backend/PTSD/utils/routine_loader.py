from sqlalchemy.orm import Session
from typing import List
from datetime import datetime
import json
from .routine_scheduler import schedule_routine
from ..core.database import SessionLocal
from ..models.routines import Routine
from pytz import timezone

def load_routines_from_db() -> None:
    """
    데이터베이스에서 활성화된 루틴을 로드하여 APScheduler에 등록합니다.
    - 일회성 루틴(once)은 start_time이 미래인 경우에만 등록
    - 매일 반복 루틴(daily)은 repeat_days에 지정된 요일에 등록
    """
    db: Session = SessionLocal()
    try:
        # 루틴 조회 시도
        try:
            routines: List[Routine] = (
                db.query(Routine)
                .filter(Routine.is_work == True)
                .all()
            )
        except Exception as e:
            print(f"[DB 조회 오류] 루틴 목록 조회 실패: {e}")
            return

        seoul_tz = timezone('Asia/Seoul')
        now = datetime.now(seoul_tz)

        for routine in routines:
            try:
                if not routine.start_time:
                    print(f"[오류] 루틴 ID {routine.routine_id}의 start_time이 None입니다. 건너뜁니다.")
                    continue

                start_time = routine.start_time
                if start_time.tzinfo is None:
                    start_time = seoul_tz.localize(start_time)  # awareness 적용

                print(f"[루틴 시작 시간] {start_time}")

                if routine.routine_type == "once" and start_time <= now:
                    print(f"[건너뜀] 루틴 ID {routine.routine_id}는 일회성 루틴이며 이미 지난 시간입니다.")
                    print(start_time, now)
                    continue

                repeat_days = []

                if routine.routine_type == "daily":
                    if isinstance(routine.repeat_days, str):
                        try:
                            repeat_days = json.loads(routine.repeat_days)
                        except Exception as e:
                            print(f"[JSON 오류] 루틴 ID {routine.routine_id}의 repeat_days 파싱 실패: {e}")
                            continue
                    elif isinstance(routine.repeat_days, list):
                        repeat_days = routine.repeat_days
                    else:
                        print(f"[오류] 루틴 ID {routine.routine_id}의 repeat_days 타입이 잘못되었습니다: {type(routine.repeat_days)}")
                        continue

                    if not repeat_days:
                        print(f"[오류] repeat_days가 비어 있습니다. 루틴 ID {routine.routine_id} 건너뜁니다.")
                        continue

                try:
                    schedule_routine(
                        routine_id=routine.routine_id,
                        routine_type=routine.routine_type,
                        start_time=start_time,
                        repeat_days=repeat_days
                    )
                    print(f"[등록 완료] 루틴 ID {routine.routine_id} 등록 성공.")
                except Exception as e:
                    print(f"[스케줄링 오류] 루틴 ID {routine.routine_id} 등록 실패: {e}")

            except Exception as e:
                print(f"[루틴 처리 오류] 루틴 ID {getattr(routine, 'routine_id', '알 수 없음')} 처리 중 오류 발생: {e}")

        print(f"[초기화 완료] 총 {len(routines)}개의 루틴을 로드하여 스케줄에 등록 시도 완료.")

    except Exception as e:
        print(f"[오류] 루틴 로드 실패: {e}")
    finally:
        db.close()

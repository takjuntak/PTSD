// src/pages/schedule/TimeSelectPage.tsx
import { useState, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import { useRoutines } from '../../hooks/useRoutines';

export default function TimeSelectPage() {
  const navigate = useNavigate();
  const { addRoutine } = useRoutines();

  const [isAM, setIsAM] = useState(true);
  const [hour, setHour] = useState(9);
  const [minute, setMinute] = useState(0);
  const [selectedDays, setSelectedDays] = useState<boolean[]>(Array(7).fill(false));
  const [enabled, setEnabled] = useState(true);
  const [skipHolidays, setSkipHolidays] = useState(false);
  const [defaultDate, setDefaultDate] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const amPmTouchStartY = useRef<number>(0);
  const hourTouchStartY = useRef<number>(0);
  const minuteTouchStartY = useRef<number>(0);

  const dayLabels = ['일', '월', '화', '수', '목', '금', '토'];

  useEffect(() => {
    const tomorrow = new Date();
    tomorrow.setDate(tomorrow.getDate() + 1);
    const month = tomorrow.getMonth() + 1;
    const date = tomorrow.getDate();
    const day = dayLabels[tomorrow.getDay()];
    setDefaultDate(`${month}월 ${date}일 (${day})`);
  }, []);

  const toggleDay = (index: number) => {
    setSelectedDays(prev => {
      const newDays = [...prev];
      newDays[index] = !newDays[index];
      return newDays;
    });
  };

  const handleWheel = (type: 'ampm' | 'hour' | 'minute', deltaY: number) => {
    if (type === 'ampm') {
      setIsAM(deltaY <= 0);
    } else if (type === 'hour') {
      setHour(prev => {
        let next = prev + (deltaY > 0 ? 1 : -1);
        if (next > 12) next = 1;
        if (next < 1) next = 12;
        return next;
      });
    } else if (type === 'minute') {
      setMinute(prev => {
        let next = prev + (deltaY > 0 ? 5 : -5);
        if (next > 55) next = 0;
        if (next < 0) next = 55;
        return next;
      });
    }
  };

  const handleTouchStart = (type: 'ampm' | 'hour' | 'minute', e: React.TouchEvent<HTMLDivElement>) => {
    const y = e.touches[0].clientY;
    if (type === 'ampm') amPmTouchStartY.current = y;
    if (type === 'hour') hourTouchStartY.current = y;
    if (type === 'minute') minuteTouchStartY.current = y;
  };

  const handleTouchMove = (type: 'ampm' | 'hour' | 'minute', e: React.TouchEvent<HTMLDivElement>) => {
    const y = e.touches[0].clientY;
    const diff =
      type === 'ampm'
        ? amPmTouchStartY.current - y
        : type === 'hour'
        ? hourTouchStartY.current - y
        : minuteTouchStartY.current - y;

    if (Math.abs(diff) > 10) {
      if (diff > 0) {
        handleWheel(type, 1);
      } else {
        handleWheel(type, -1);
      }
      if (type === 'ampm') amPmTouchStartY.current = y;
      if (type === 'hour') hourTouchStartY.current = y;
      if (type === 'minute') minuteTouchStartY.current = y;
    }
  };

  const handleSave = async () => {
    try {
      // 버튼 중복 클릭 방지
      if (isSubmitting) return;
      setIsSubmitting(true);
      setError(null);

      // 선택된 요일이 없으면 반복 타입은 once(한 번)
      const hasSelectedDays = selectedDays.some(day => day);
      const routineType = hasSelectedDays ? 'daily' : 'once';
      
      // 24시간 형식으로 시간 변환
      let hour24 = isAM ? (hour === 12 ? 0 : hour) : (hour === 12 ? 12 : hour + 12);
      
      // ISO 문자열로 시간 생성 (daily인 경우는 내일 날짜, once인 경우는 선택된 요일에 따라)
      const startTime = new Date();
      startTime.setHours(hour24, minute, 0, 0);
      
      if (routineType === 'once') {
        // 일회성 예약 - 내일 날짜로 설정
        startTime.setDate(startTime.getDate() + 1);
      }
      
      // 선택된 요일 인덱스 배열 생성 (API에서는 월(1)~일(7)을 사용)
      const repeatDays = selectedDays
        .map((selected, index) => (selected ? (index === 0 ? 7 : index) : null))
        .filter((day): day is number => day !== null);
      
      // 요일을 선택하지 않았고 반복이 아닌 경우, 내일의 요일을 설정
      if (repeatDays.length === 0 && routineType === 'once') {
        const tomorrowDay = (new Date().getDay() + 1) % 7;
        repeatDays.push(tomorrowDay === 0 ? 7 : tomorrowDay);
      }

      const success = await addRoutine({
        start_time: startTime.toISOString(),
        routine_type: routineType,
        is_work: enabled,
        repeat_days: repeatDays
      });

      if (success) {
        navigate('/schedule');
      } else {
        setError('스케줄 저장에 실패했습니다.');
      }
    } catch (err) {
      setError('오류가 발생했습니다. 다시 시도해주세요.');
      console.error('Error saving schedule:', err);
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className="schedule-content">
      <div className="time-box">
        <div className="scroll-container">
          {/* 오전/오후 */}
          <div
            className="scroll-column"
            onWheel={e => handleWheel('ampm', e.deltaY)}
            onTouchStart={e => handleTouchStart('ampm', e)}
            onTouchMove={e => handleTouchMove('ampm', e)}
          >
            <div className="scroll-list">
              {isAM ? null : <div className="faded-text">오전</div>}
              <div className="selected-text">{isAM ? '오전' : '오후'}</div>
              {isAM ? <div className="faded-text">오후</div> : null}
            </div>
          </div>

          <div className="gap" />

          {/* 시간 */}
          <div
            className="scroll-column"
            onWheel={e => handleWheel('hour', e.deltaY)}
            onTouchStart={e => handleTouchStart('hour', e)}
            onTouchMove={e => handleTouchMove('hour', e)}
          >
            <div className="scroll-list">
              <div className="faded-text">{hour === 1 ? 12 : hour - 1}</div>
              <div className="selected-text">{hour}</div>
              <div className="faded-text">{hour === 12 ? 1 : hour + 1}</div>
            </div>
          </div>

          <div className="colon">:</div>

          {/* 분 */}
          <div
            className="scroll-column"
            onWheel={e => handleWheel('minute', e.deltaY)}
            onTouchStart={e => handleTouchStart('minute', e)}
            onTouchMove={e => handleTouchMove('minute', e)}
          >
            <div className="scroll-list">
              <div className="faded-text">{minute === 0 ? '55' : (minute - 5).toString().padStart(2, '0')}</div>
              <div className="selected-text">{minute.toString().padStart(2, '0')}</div>
              <div className="faded-text">{minute === 55 ? '00' : (minute + 5).toString().padStart(2, '0')}</div>
            </div>
          </div>
        </div>
      </div>

      {/* 옵션 선택 */}
      <div className="option-box">
        <div className="date-display">내일 - {defaultDate}</div>

        <div className="day-select">
          {dayLabels.map((day, idx) => (
            <button
              key={idx}
              className={`day-btn ${selectedDays[idx] ? 'active' : ''} ${day === '일' ? 'sun' : day === '토' ? 'sat' : ''}`}
              onClick={() => toggleDay(idx)}
            >
              {day}
            </button>
          ))}
        </div>

        <div className="separator" />

        <div className="toggle-section">
          <div className="toggle-item">
            <div className="toggle-text">
              <div className="toggle-title">예약 켜짐</div>
              <div className="toggle-desc">설정된 청소 시간이 적용됩니다.</div>
            </div>
            <label className="switch">
              <input type="checkbox" checked={enabled} onChange={e => setEnabled(e.target.checked)} />
              <span className="slider" />
            </label>
          </div>

          <div className="toggle-item">
            <div className="toggle-text">
              <div className="toggle-title">공휴일에는 끄기</div>
              <div className="toggle-desc">대체 및 임시 공휴일에는 끄기</div>
            </div>
            <label className="switch">
              <input type="checkbox" checked={skipHolidays} onChange={e => setSkipHolidays(e.target.checked)} />
              <span className="slider" />
            </label>
          </div>
        </div>

        {error && <div className="error-message">{error}</div>}

        <div className="button-row">
          <button className="cancel-btn" onClick={() => navigate('/schedule')}>취소</button>
          <button 
            className="save-btn" 
            onClick={handleSave}
            disabled={isSubmitting}
          >
            {isSubmitting ? '저장 중...' : '저장'}
          </button>
        </div>
      </div>

      <style>{`
        .schedule-content {
          max-width: 600px;
          margin: 0 auto;
          padding-bottom: 100px;
        }
        .time-box, .option-box {
          background: #373738;
          padding: 26px;
          border-radius: 10px;
          margin-bottom: 16px;
        }
        .time-box {
          margin-bottom: 10px;
          height: 25vh; /* 화면 높이의 25% */
          min-height: 180px; /* 최소 180px 보장 */
          max-height: 280px; /* 최대 280px 제한 */
          display: flex;
          flex-direction: column;
          justify-content: center;
        }
        .option-box {
          height: 50vh; /* 화면 높이의 45% */
          padding: 26px 26px 26px 26px; /* 아래 패딩 없앰 */
          min-height: 300px;
          max-height: 500px;
          display: flex;
          flex-direction: column;
        }
        .date-display {
          text-align: left;
          margin-bottom: 20px;
        }
        .scroll-container {
          display: flex;
          justify-content: center;
          align-items: center;
          gap: 20px;
        }
        .scroll-column {
          display: flex;
          flex-direction: column;
          align-items: center;
          height: 90px;
          overflow: hidden;
        }
        .selected-text {
          font-size: 32px;
          color: white;
          font-weight: bold;
          line-height: 30px;
        }
        .faded-text {
          font-size: 20px;
          color: #888;
          line-height: 30px;
        }
        .colon {
          font-size: 32px;
          color: white;
          font-weight: bold;
        }
        .gap {
          width: 20px;
        }
        .day-select {
          display: flex;
          flex-wrap: nowrap;
          justify-content: center;
          gap: 12px;
          margin-bottom: 25px;
        }
        .day-btn {
          width: 40px;
          height: 40px;
          border: none;
          background: transparent;
          color: white;
          font-size: 16px;
          display: flex;
          align-items: center;
          justify-content: center;
          border-radius: 50%;
        }
        .day-btn.active {
          border: 2px solid #0098FF;
          color: #0098FF;
        }
        .day-btn.sun {
          color: #FF0044;
        }
        .day-btn.sat {
          color: #0098FF;
        }
        .separator {
          border-top: 1px dashed #666;
          margin: 0;
        }
        .toggle-section {
          display: flex;
          flex-direction: column;
          gap: 40px;
          margin-bottom: 0; /* 기존 여백 제거 */
          margin-top: 34px;
        }
        .toggle-item {
          display: flex;
          justify-content: space-between;
          align-items: center;
        }
        .toggle-text {
          text-align: left;
        }
        .toggle-title {
          font-weight: bold;
          font-size: 16px;
          color: white;
        }
        .toggle-desc {
          font-size: 12px;
          color: #0098FF;
          margin-top: 4px;
        }
        .switch {
          position: relative;
          width: 50px;
          height: 24px;
        }
        .switch input {
          opacity: 0;
          width: 0;
          height: 0;
        }
        .slider {
          position: absolute;
          top: 0;
          left: 0;
          right: 0;
          bottom: 0;
          background-color: #555;
          border-radius: 24px;
          cursor: pointer;
          transition: 0.4s;
        }
        .slider:before {
          position: absolute;
          content: "";
          height: 18px;
          width: 18px;
          left: 3px;
          bottom: 3px;
          background-color: white;
          border-radius: 50%;
          transition: 0.4s;
        }
        input:checked + .slider {
          background-color: #0098FF;
        }
        input:checked + .slider:before {
          transform: translateX(26px);
        }
        .button-row {
          display: flex;
          gap: 8px;
          margin-top: auto;
          padding-top: 24px; /* 버튼과 위 내용 사이 여백 */
        }
        .cancel-btn, .save-btn {
          flex: 1;
          padding: 12px 0;
          border: none;
          font-size: 16px;
          border-radius: 8px;
          cursor: pointer;
        }
        .cancel-btn {
          background: transparent;
          color: white;
          border: 1px solid #888;
        }
        .save-btn {
          background: #0098FF;
          color: white;
        }
        .save-btn:disabled {
          background: #777;
          cursor: not-allowed;
        }
        .error-message {
          color: #ff6b6b;
          background-color: rgba(255, 107, 107, 0.1);
          border-radius: 6px;
          padding: 10px;
          margin-top: 16px;
          text-align: center;
          font-size: 14px;
        }
      `}</style>
    </div>
  );
}
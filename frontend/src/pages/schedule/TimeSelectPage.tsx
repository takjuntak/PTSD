import { useState, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import { useRoutines } from '../../hooks/useRoutines';

export default function TimeSelectPage() {
  const navigate = useNavigate();
  const { addRoutine } = useRoutines();

  const [isAM, setIsAM] = useState(true);
  const [hour, setHour] = useState(6);
  const [minute, setMinute] = useState(0);
  const [selectedDays, setSelectedDays] = useState<boolean[]>(Array(7).fill(false));
  const [enabled, setEnabled] = useState(true);
  const [skipHolidays, setSkipHolidays] = useState(false);
  const [defaultDate, setDefaultDate] = useState('');

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
        let next = prev + (deltaY > 0 ? 1 : -1);
        if (next > 59) next = 0;
        if (next < 0) next = 59;
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
      handleWheel(type, diff > 0 ? 1 : -1);
      if (type === 'ampm') amPmTouchStartY.current = y;
      if (type === 'hour') hourTouchStartY.current = y;
      if (type === 'minute') minuteTouchStartY.current = y;
    }
  };

  const handleSave = async () => {
    const now = new Date();
    const tomorrow = new Date(now);
    tomorrow.setDate(now.getDate() + 1);

    let actualHour = hour;
    if (isAM && hour === 12) actualHour = 0;
    else if (!isAM && hour !== 12) actualHour += 12;

    let startTime = new Date();
    startTime.setHours(actualHour, minute, 0, 0);

    if (!selectedDays.some(Boolean)) {
      startTime.setDate(startTime.getDate() + 1);
      if (startTime <= now) {
        startTime.setDate(startTime.getDate() + 1);
      }
    }

    const routineType = selectedDays.some(Boolean) ? 'daily' : 'once';
    const repeatDays = selectedDays.map((selected, i) => selected ? i : -1).filter(i => i !== -1);

    try {
      const success = await addRoutine({
        start_time: startTime.toISOString(),
        routine_type: routineType,
        iswork: enabled,
        repeat_days: repeatDays,
      });
      if (success) navigate('/schedule');
      else alert('저장 중 오류가 발생했습니다.');
    } catch (error) {
      alert('저장 중 오류가 발생했습니다.');
    }
  };

  return (
    <div className="w-full h-full mx-auto overflow-hidden">
      <div className="bg-[#373738] p-6 rounded-[10px] mb-2 mt-4 min-h-[160px] max-h-[280px] flex flex-col justify-center">
        <div className="flex justify-center items-center gap-5">
          {/* AM/PM */}
          <div
            className="flex flex-col items-center h-[90px] overflow-hidden mr-6"
            onWheel={e => handleWheel('ampm', e.deltaY)}
            onTouchStart={e => handleTouchStart('ampm', e)}
            onTouchMove={e => handleTouchMove('ampm', e)}
          >
            <div className="text-[20px] text-[#888] leading-[30px]">{isAM ? null : '오전'}</div>
            <div className="text-[32px] text-white font-bold leading-[30px]">{isAM ? '오전' : '오후'}</div>
            <div className="text-[20px] text-[#888] leading-[30px]">{isAM ? '오후' : null}</div>
          </div>

          {/* 시간 */}
           <div
              className="flex flex-col items-center h-[90px] overflow-hidden"
              onWheel={e => handleWheel('hour', e.deltaY)}
              onTouchStart={e => handleTouchStart('hour', e)}
              onTouchMove={e => handleTouchMove('hour', e)}
            >
            <div className="text-[20px] text-[#888] leading-[30px]">{hour === 1 ? 12 : hour - 1}</div>
            <div className="text-[32px] text-white font-bold leading-[30px]">{hour}</div>
            <div className="text-[20px] text-[#888] leading-[30px]">{hour === 12 ? 1 : hour + 1}</div>
          </div>

          {/* ":" 기호를 시간과 분 사이에 위치 */}
          <div className="text-[32px] text-white font-bold mx-2">:</div>

          {/* 분 */}
          <div
            className="flex flex-col items-center h-[90px] overflow-hidden"
            onWheel={e => handleWheel('minute', e.deltaY)}
            onTouchStart={e => handleTouchStart('minute', e)}
            onTouchMove={e => handleTouchMove('minute', e)}
          >
            <div className="text-[20px] text-[#888] leading-[30px]">{minute === 0 ? '59' : (minute - 1).toString().padStart(2, '0')}</div>
            <div className="text-[32px] text-white font-bold leading-[30px]">{minute.toString().padStart(2, '0')}</div>
            <div className="text-[20px] text-[#888] leading-[30px]">{minute === 59 ? '00' : (minute + 1).toString().padStart(2, '0')}</div>
          </div>
        </div>

      </div>

      <div className="bg-[#373738] p-6 rounded-[10px] h-[420px] flex flex-col">
        <div className="text-left mb-5">내일 - {defaultDate}</div>

        <div className="flex justify-between mb-6">
          {dayLabels.map((day, idx) => (
            <button
              key={idx}
              onClick={() => toggleDay(idx)}
              className={`w-10 h-10 text-sm rounded-full flex items-center justify-center border-2 box-border bg-transparent
                ${selectedDays[idx] ? 'border-[#0098FF]' : 'border-transparent'}
                ${day === '일' ? 'text-[#FF0044]' : day === '토' ? 'text-[#0098FF]' : 'text-white'}`}
            >
              {day}
            </button>
          ))}
        </div>


        <div className="border-t border-dashed border-[#666]" />

        <div className="flex flex-col gap-10 mt-8">
          {[{ title: '예약 켜짐', desc: '설정된 청소 시간이 적용됩니다.', val: enabled, set: setEnabled },
            { title: '공휴일에는 끄기', desc: '대체 및 임시 공휴일에는 끄기', val: skipHolidays, set: setSkipHolidays }].map((item, i) => (
            <div key={i} className="flex justify-between items-center">
              <div className="text-left">
                <div className="text-white font-bold text-base">{item.title}</div>
                <div className="text-[#0098FF] text-xs mt-1">{item.desc}</div>
              </div>
              <label className="relative w-[50px] h-[24px]">
                <input type="checkbox" checked={item.val} onChange={e => item.set(e.target.checked)} className="opacity-0 w-0 h-0" />
                <span className="absolute inset-0 bg-[#555] rounded-full cursor-pointer transition-all duration-300 before:content-[''] before:absolute before:h-[18px] before:w-[18px] before:left-[3px] before:bottom-[3px] before:bg-white before:rounded-full before:transition-all before:duration-300 peer-checked:bg-[#0098FF] peer-checked:before:translate-x-[26px]" />
              </label>
            </div>
          ))}
        </div>

        <div className="flex justify-center gap-3 mt-8 pt-4">
          <button
            className="w-[156px] h-[46px] rounded-[10px] text-lg font-bold font-[Montserrat] leading-[22px] flex items-center justify-center shadow-md cursor-pointer border border-[#617BEE] bg-white text-[#617BEE]"
            onClick={() => navigate('/schedule')}
          >취소</button>
          <button
            className="w-[156px] h-[46px] rounded-[10px] text-lg font-bold font-[Montserrat] leading-[22px] flex items-center justify-center shadow-md cursor-pointer bg-[#617BEE] text-white"
            onClick={handleSave}
          >저장</button>
        </div>
      </div>
    </div>
  );
}

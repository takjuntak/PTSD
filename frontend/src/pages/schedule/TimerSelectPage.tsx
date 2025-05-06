import { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
import './TimerSelectPage.css';

export default function TimerSelectPage() {
  const navigate = useNavigate();
  const [currentTime, setCurrentTime] = useState(new Date());
  const [endTime] = useState(new Date(new Date().getTime() + 3 * 60 * 60 * 1000));

  useEffect(() => {
    const interval = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);
    return () => clearInterval(interval);
  }, []);

  const formatTime = (date: Date) => {
    const hour = date.getHours();
    const minute = date.getMinutes();
    const period = hour >= 12 ? '오후' : '오전';
    const formattedHour = hour % 12 === 0 ? 12 : hour % 12;
    return `${formattedHour}:${minute.toString().padStart(2, '0')} ${period}`;
  };

  return (
    <div className="timer-select-wrapper">

      <div className="timer-header">
        <span className="back-button" onClick={() => navigate(-1)}>{'<'} 타이머 설정</span>
      </div>
      <h2 className="timer-title">원하는 청소 종료 시간을 설정해 주세요.</h2>
      <p className="timer-sub">설정 가능 시간 : 1시간 ~ 12시간</p>

      <div className="timer-circle-wrapper">
        <div className="timer-circle">
          <div className="circle-center">
            <p className="end-label">종료 시각</p>
            <p className="end-time">오늘<br />{formatTime(endTime)}</p>
            <p className="current-label">현재 시각<br />{formatTime(currentTime)}</p>
          </div>
          <div className="clock-numbers">
            {Array.from({ length: 12 }, (_, i) => {
              const angle = (i + 1) * 30;
              const radius = 130;
              const x = radius * Math.sin((angle * Math.PI) / 180);
              const y = -radius * Math.cos((angle * Math.PI) / 180);
              return (
                <span
                  key={i}
                  className={`clock-number ${i === 2 ? 'highlight' : ''}`}
                  style={{
                    position: 'absolute',
                    left: `calc(50% + ${x}px - 12px)`,
                    top: `calc(50% + ${y}px - 12px)`,
                  }}
                >
                  {i + 1}
                </span>
              );
            })}
          </div>
        </div>
        <div className="circle-knob" />
      </div>

      <button className="timer-confirm-button" onClick={() => navigate('/schedule')}>확인</button>
    </div>
  );
}

// src/pages/schedule/SchedulePage.tsx
import { useState, useEffect, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
import moreImage from '../../assets/schedule/more.svg';
import plusImage from '../../assets/schedule/plus.svg';
import ScheduleItem from './ScheduleItem';
import { useRoutines } from '../../hooks/useRoutines';

interface ScheduleItemType {
  id: number;
  time: string;
  date: string;
  active: boolean;
}

const SchedulePage = () => {
  const navigate = useNavigate();
  const { routines, isLoading, error, toggleRoutineActive, deleteRoutines } = useRoutines();
  
  const [scheduleItems, setScheduleItems] = useState<ScheduleItemType[]>([]);
  const [plusMenuOpen, setPlusMenuOpen] = useState(false);
  const [editMenuOpen, setEditMenuOpen] = useState(false);
  const [editMode, setEditMode] = useState(false);
  const [selectedIds, setSelectedIds] = useState<number[]>([]);

  const plusMenuRef = useRef<HTMLDivElement>(null);
  const editMenuRef = useRef<HTMLDivElement>(null);

  // 루틴 데이터를 가공하여 화면에 표시할 형태로 변환
  useEffect(() => {
    if (routines.length > 0) {
      const items = routines.map(routine => {
        // 시간 포맷팅
        const startTime = new Date(routine.start_time);
        const hour = startTime.getHours();
        const minute = startTime.getMinutes();
        const ampm = hour >= 12 ? '오후' : '오전';
        const hour12 = hour % 12 || 12;
        const timeString = `${ampm} ${hour12}:${minute.toString().padStart(2, '0')}`;
        
        // 날짜 포맷팅 (한 번 실행) 또는 반복 요일 (매일 반복)
        let dateString = '';
        if (routine.routine_type === 'once') {
          const date = startTime.getDate();
          const month = startTime.getMonth() + 1;
          const day = ['일', '월', '화', '수', '목', '금', '토'][startTime.getDay()];
          dateString = `${month}월 ${date}일 (${day})`;
        } else {
          // 요일 배열로 변환
          const days = ['일', '월', '화', '수', '목', '금', '토'];
          dateString = routine.repeat_days.length === 7 
            ? '매일' 
            : routine.repeat_days.map(day => days[day % 7]).join(' ');
        }

        return {
          id: routine.routine_id,
          time: timeString,
          date: dateString,
          active: routine.is_work
        };
      });
      
      setScheduleItems(items);
    } else {
      setScheduleItems([]);
    }
  }, [routines]);

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        plusMenuRef.current && !plusMenuRef.current.contains(event.target as Node) &&
        editMenuRef.current && !editMenuRef.current.contains(event.target as Node)
      ) {
        setPlusMenuOpen(false);
        setEditMenuOpen(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  const togglePlusMenu = () => {
    setPlusMenuOpen(p => !p);
    setEditMenuOpen(false);
  };

  const toggleEditMenu = () => {
    setEditMenuOpen(p => !p);
    setPlusMenuOpen(false);
  };

  const enterEditMode = () => {
    setEditMode(true);
    setEditMenuOpen(false);
  };

  const cancelEditMode = () => {
    setEditMode(false);
    setSelectedIds([]);
  };

  const handleDelete = async () => {
    if (selectedIds.length > 0) {
      const success = await deleteRoutines(selectedIds);
      if (success) {
        cancelEditMode();
      }
    }
  };

  const handleToggle = async (id: number, currentState: boolean) => {
    await toggleRoutineActive(id, !currentState);
  };

  const handleTimeSelect = () => {
    navigate('/schedule/time-select');
  };

  return (
    <div className="schedule-page">
      <div className="schedule-content">
        <div className="schedule-title">예약</div>
        <div className="schedule-subtitle">원하는 청소 시간을 설정해 주세요.</div>

        {error && (
          <div className="error-message">
            {error}
          </div>
        )}

        <div className="schedule-buttons">
          <div className="relative" ref={plusMenuRef}>
            <button className="icon-button" onClick={togglePlusMenu}>
              <img src={plusImage} alt="plus" className="icon-svg" />
            </button>
            {plusMenuOpen && (
              <div className="dropdown-menu">
                <div className="menu-item" onClick={handleTimeSelect}>시간 예약</div>
                <hr className="menu-divider" />
                <div className="menu-item" onClick={() => navigate('/schedule/timer-select')}>타이머 설정</div>
              </div>
            )}
          </div>
          <div className="relative" ref={editMenuRef}>
            <button className="icon-button" onClick={toggleEditMenu}>
              <img src={moreImage} alt="more" className="icon-svg" />
            </button>
            {editMenuOpen && (
              <div className="dropdown-menu">
                <div className="menu-item" onClick={enterEditMode}>편집</div>
              </div>
            )}
          </div>
        </div>

        <div className="schedule-list">
          {isLoading ? (
            <div className="schedule-loading">데이터를 불러오는 중...</div>
          ) : scheduleItems.length > 0 ? (
            scheduleItems.map(item => (
              <ScheduleItem
                key={item.id}
                id={item.id}
                time={item.time}
                date={item.date}
                active={item.active}
                onToggle={() => handleToggle(item.id, item.active)}
                editMode={editMode}
                selected={selectedIds.includes(item.id)}
                onSelect={() => {
                  setSelectedIds(prev =>
                    prev.includes(item.id)
                      ? prev.filter(id => id !== item.id)
                      : [...prev, item.id]
                  );
                }}
              />
            ))
          ) : (
            <div className="schedule-empty">아직 예약된 시간이 없어요.</div>
          )}
        </div>

        {editMode && (
          <div className="bottom-button">
            <button className="cancel-button" onClick={cancelEditMode}>취소</button>
            <button 
              className="delete-button" 
              onClick={handleDelete}
              disabled={selectedIds.length === 0}
            >
              삭제
            </button>
          </div>
        )}

        <style>{`
          .error-message {
            background-color: rgba(255, 0, 0, 0.1);
            border: 1px solid rgba(255, 0, 0, 0.3);
            color: #ff6b6b;
            padding: 10px;
            border-radius: 8px;
            margin: 20px 0;
            text-align: center;
          }
          .schedule-loading {
            text-align: center;
            color: #fff;
            margin-top: 40px;
            font-size: 16px;
          }
          html, body {
            height: 100%;
            background: linear-gradient(180deg, #2E2E37 0%, #1D1E23 100%);
            overflow-x: hidden;
          }
          .schedule-page {
            width: 100%;
            min-height: 100vh;
            display: flex;
            justify-content: center;
            padding-bottom: 120px;
          }
          .schedule-content {
            width: 100%;
            max-width: 412px;
            font-family: 'Montserrat', sans-serif;
          }
          .schedule-title {
            font-size: 20px;
            font-weight: 800;
            color: #FFFFFF;
            text-align: left;
          }
          .schedule-subtitle {
            margin-top: 40px;
            font-size: 20px;
            font-weight: 700;
            color: #FFFFFF;
          }
          .schedule-buttons {
            display: flex;
            justify-content: flex-end;
            align-items: center;
            gap: 20px;
            margin-top: 40px;
          }
          .icon-button {
            background: none;
            border: none;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            padding: 0;
          }
          .icon-button svg {
            width: 24px;
            height: 24px;
          }
          .relative {
            position: relative;
          }
          .dropdown-menu {
            position: absolute;
            top: 40px;
            right: 0;
            width: 127px;
            background-color: #373738;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0,0,0,0.25);
            padding: 8px 0;
            z-index: 100;
            white-space: nowrap;
          }
          .menu-item {
            font-family: 'Inter', sans-serif;
            font-weight: 800;
            font-size: 15px;
            line-height: 18px;
            color: #FFFFFF;
            padding: 12px 18px;
            cursor: pointer;
            display: flex;
            align-items: center;
          }
          .menu-item:hover {
            background-color: #4a4a4a;
          }
          .menu-divider {
            width: 90px;
            height: 0;
            border-top: 2px dashed #767676;
            margin: 0 auto;
          }
          .schedule-list {
            margin-top: 24px;
            display: flex;
            flex-direction: column;
            gap: 16px;
            padding: 0
          }
          .schedule-empty {
            text-align: center;
            color: #888;
            margin-top: 40px;
          }
          .bottom-button {
            position: fixed;
            bottom: 120px;
            left: 50%;
            transform: translateX(-50%);
            display: flex;
            gap: 10px;
            max-width: 412px;
            width: 100%;
            padding: 0 20px;
          }
          .cancel-button, .delete-button {
            flex: 1;
            height: 48px;
            font-size: 16px;
            font-weight: bold;
            border-radius: 10px;
          }
          .cancel-button {
            background: white;
            color: #EE6163;
            border: 0.5px solid #EE6163;
          }
          .delete-button {
            background: #EE6163;
            color: white;
            border: none;
          }
          .delete-button:disabled {
            background: #999;
            cursor: not-allowed;
          }
        `}</style>
      </div>
    </div>
  );
};

export default SchedulePage;
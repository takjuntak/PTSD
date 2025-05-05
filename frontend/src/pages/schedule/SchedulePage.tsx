import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { Plus, MoreVertical } from 'lucide-react';
import ScheduleItem from '../../components/schedule/ScheduleItem';

interface Schedule {
    id: number;
    time: string;
    date: string;
    active: boolean;
  }
  
  const SchedulePage = () => {
    const navigate = useNavigate();
  
    const [schedules, setSchedules] = useState<Schedule[]>([
      { id: 1, time: '오전 9:30', date: '5월 11일 (일)', active: true },
      { id: 2, time: '오후 12:30', date: '5월 11일 (일)', active: true },
      { id: 3, time: '오후 3:30', date: '5월 11일 (일)', active: true },
      { id: 4, time: '오후 4:00', date: '5월 11일 (일)', active: false },
      { id: 5, time: '오후 6:30', date: '일 월 수 목 금 토', active: true },
    ]);
  
    const [plusMenuOpen, setPlusMenuOpen] = useState(false);
    const [editMenuOpen, setEditMenuOpen] = useState(false);

    // 삭제 버튼 
    const [editMode, setEditMode] = useState(false);
    const [selectedIds, setSelectedIds] = useState<number[]>([]);
  
    const togglePlusMenu = () => {
      setPlusMenuOpen(p => !p);
      setEditMenuOpen(false);
    };
  
    const toggleEditMenu = () => {
      setEditMenuOpen(p => !p);
      setPlusMenuOpen(false);
    };

    // 편집 모드
    const enterEditMode = () => {
      setEditMode(true);
      setEditMenuOpen(false);
    };

    const cancelEdieMode = () => {
      setEditMode(false);
      setSelectedIds([]);
    }

    const handleDelete = () => {
      setSchedules(prev => prev.filter(s => !selectedIds.includes(s.id)));
      cancelEdieMode();
    }
  
    const handleToggle = (id: number) => {
      setSchedules(prev =>
        prev.map(s => (s.id === id ? { ...s, active: !s.active } : s))
      );
    };
  
    const handleTimeSelect = () => { 
      navigate('/schedule/time-select');
    };
  
    return (
      <div className="schedule-page">
        <div className="schedule-content">
          <div className="schedule-title">예약</div>
          <div className="schedule-subtitle">
            원하는 청소 시간을 설정해 주세요.
          </div>
  
          <div className="schedule-buttons">
            <div className="relative">
              <button className="icon-button" onClick={togglePlusMenu}>
                <Plus size={24} />
              </button>
              {plusMenuOpen && (
                <div className="dropdown-menu">
                  <div className="menu-item" onClick={handleTimeSelect}>예약 시간 설정</div>
                  <hr className="menu-divider" />
                  <div className="menu-item">타이머 설정</div>
                </div>
              )}
            </div>
            <div className="relative">
              <button className="icon-button" onClick={toggleEditMenu}>
                <MoreVertical size={24} />
              </button>
              {editMenuOpen && (
                <div className="dropdown-menu">
                  <div className="menu-item" onClick={enterEditMode}>편집</div>
                </div>
              )}
            </div>
          </div>
  
          <div className="schedule-list">
            {schedules.length > 0 ? (
              schedules.map(item => (
                <ScheduleItem
                  key={item.id}
                  id={item.id}
                  time={item.time}
                  date={item.date}
                  active={item.active}
                  onToggle={() => handleToggle(item.id)}
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
            <div className='bottom-button'>
              <button className='cancel-button' onClick={cancelEdieMode}>취소</button>
              <button className='delete-button' onClick={handleDelete}>삭제</button>
            </div>
          )}
        </div>

      <style>{`
        .schedule-page {
          width: 100%;
          height: 100%;
          display: flex;
          justify-content: center;
        }
        .schedule-content {
          width: 100%;
          max-width: 600px;
          padding: 24px 16px;
          box-sizing: border-box;
          overflow: hidden;
          position: relative;
        }
        .schedule-title {
          color: white;
          font-size: 20px;
          font-weight: bold;
          margin-top: 20px 0 40px 18px;
          text-align: left;
        }
        .schedule-subtitle {
          color: white;
          font-size: 18px;
          font-weight: 600;
          text-align: center;
          margin: 40px 0 24px;
        }
        .schedule-buttons {
          display: flex;
          justify-content: flex-end;
          align-items: center;
          gap: 8px;
          margin-bottom: 24px;
        }
        .relative {
          position: relative;
        }
        .icon-button {
          background: none;
          border: none;
          color: white;
          padding: 0;
          cursor: pointer;
        }
        .dropdown-menu {
          position: absolute;
          top: 100%;
          right: 0;
          margin-top: 8px;
          width: 140px;
          background-color: #2B2B2B;
          border-radius: 10px;
          box-shadow: 0 4px 8px rgba(0,0,0,0.4);
          padding: 8px 0;
          z-index: 10;
        }
        .menu-item {
          color: white;
          font-size: 16px;
          padding: 12px 16px;
          cursor: pointer;
          transition: background-color 0.2s;
        }
        .menu-item:hover {
          background-color: #3c3c3c;
        }
        .menu-divider {
          border: none;
          margin: 4px 0;
          border-top: 1px dotted #555;
        }

        /* ScheduleItem CSS */
        .schedule-item {
          display: flex;
          justify-content: space-between;
          align-items: center;
          width: 100%;
          padding: 28px 40px;
          background-color: #2B2B2B;
          border-radius: 10px;
          box-sizing: border-box;
        }
        .schedule-item.inactive {
          opacity: 0.5;
        }
        .schedule-time {
          display: flex;
          align-items: baseline;
          gap: 4px;
        }
        .schedule-ampm {
          color: white;
          font-size: 15px;
          font-weight: 600;
        }
        .schedule-clock {
          color: white;
          font-size: 25px;
          font-weight: 800;
        }
        .schedule-right {
          display: flex;
          align-items: center;
          gap: 16px;
        }
        .schedule-date {
          color: #aaa;
          font-size: 12px;
        }
        .switch {
          position: relative;
          display: inline-block;
          width: 44px;
          height: 24px;
        }
        .switch input {
          opacity: 0;
          width: 0;
          height: 0;
        }
        .slider {
          position: absolute;
          cursor: pointer;
          top: 0;
          left: 0;
          right: 0;
          bottom: 0;
          background-color: #555;
          transition: .4s;
          border-radius: 24px;
        }
        .slider::before {
          content: "";
          position: absolute;
          height: 18px;
          width: 18px;
          left: 3px;
          bottom: 3px;
          background-color: white;
          transition: .4s;
          border-radius: 50%;
        }
        .switch input:checked + .slider {
          background-color: #0088FF;
        }
        .switch input:checked + .slider::before {
          transform: translateX(20px);
        }

        .schedule-list {
          display: flex;
          flex-direction: column;
          gap: 16px;
          padding-bottom: 80px;
        }
        .schedule-empty {
          color: #888;
          text-align: center;
          margin-top: 80px;
        }

        .bottom-button {
          position: fixed;
          bottom: 100px;
          left: 50%;
          transform: translateX(-50%);
          display: flex;
          flex-direction: row;
          justify-content: center;
          align-items: center;
          gap: 16px;
          width: 520px;
          background: transparent;
          padding: 0;
          z-index: 1000;
        }

        .cancel-button, .delete-button {
          flex: 1;
          min-width: 200px;
          width: 100%;
          flex-shrink: 0;
          height: 48px;
          font-size: 18px;
          padding: 10px 24px;
          border-radius: 10px;
          font-weight: bold;
          justify-content: center;
          align-items: center;
          display: flex;
          border-radius: 10px;
          box-shadow: 0px 2px 4px rgba(97, 123, 238, 0.3);
        }

        .cancel-button {
          background: #FFFFFF;
          color: #617BEE;
          border: 2px solid #617BEE;
          padding: 14px 32px;
          box-shadow: 0px 2px 4px rgba(97, 123, 238, 0.3);
        }

        .delete-button {
          background: #617BEE;
          color: #FFFFFF;
          border: none;
          padding: 14px 32px;
          box-shadow: 0px 2px 4px rgba(97, 123, 238, 0.3);
        }


        @media (max-width: 412px) {
          .schedule-content {
            padding: 16px;
          }
          .schedule-title {
            font-size: 18px;
          }
          .schedule-subtitle {
            font-size: 16px;
          }
          .bottom-button {
            bottom: 72px;
            left: 50px;
            transform: translateX(-50%);
            width: 100%;
            max-width: 600px;
            padding: 16px;
            background: transparent;
            width: auto;
            display: flex;
            justify-content: center; 
            align-items: center;
            gap: 16px;
            flex-wrap: nowrap; 
            overflow-x: auto;
            box-shadow: 0 -2px 6px rgba(0, 0, 0, 0.5);
          }

        
      `}</style>
    </div>
  );
};

export default SchedulePage;

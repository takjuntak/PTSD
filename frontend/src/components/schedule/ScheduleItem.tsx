import React from 'react';

interface ScheduleItemProps {
  id: number;
  time: string;
  date: string;
  active: boolean;
  onToggle: () => void;

  editMode?: boolean;
  selected?: boolean;
  onSelect?: () => void;
}

const ScheduleItem: React.FC<ScheduleItemProps> = ({
  time,
  date,
  active,
  onToggle,
  editMode = false,
  selected = false,
  onSelect,
}) => {
  const [ampm, clock] = time.split(' ');

  return (
    <div className={`schedule-item ${!active ? 'inactive' : ''}`}>
      <div className='schedule-left'>
        {editMode && (
          <label className="custom-checkbox">
            <input
              type="checkbox"
              checked={selected}
              onChange={onSelect}
            />
            <span className="checkbox-icon"></span>
          </label>
        )}

        <div className="schedule-time">
          <span className="schedule-ampm">{ampm}</span>&nbsp;
          <span className="schedule-clock">{clock}</span>
        </div>
      </div>
      
      <div className='schedule-right'>
        <div className="schedule-right">
          <div className="schedule-date">{date}</div>
          <label className="switch">
            <input type="checkbox" checked={active} onChange={onToggle} />
            <span className="slider" />
          </label>
        </div>
      </div>

      <style>{`
        .schedule-item {
          width: 376px;
          height: 90px;
          display: flex;
          align-items: center;
          justify-content: flex-start;
          gap: 80px;
          background: #373738;
          border-radius: 10px;
          box-sizing: border-box;
        }

        .schedule-item.inactive {
          opacity: 0.5;
        }

        .schedule-checkbox {
          margin-right: 12px;
          width: 20px;
          height: 20px;
          accent-color: #5C6BC0;
          cursor: pointer;
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
          font-size: 24px;
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
        .custom-checkbox {
          position: relative;
          width: 24px;
          height: 24px;
          margin-right: 12px;
          display: inline-block;
          cursor: pointer;
        }

        .custom-checkbox input {
          opacity: 0;
          width: 0;
          height: 0;
        }

        .checkbox-icon {
          position: absolute;
          top: 0;
          left: 0;
          width: 24px;
          height: 24px;
          background-image: url('/schedule/unchecked.svg');
          background-size: cover;
          background-repeat: no-repeat;
        }

        .custom-checkbox input:checked + .checkbox-icon {
          background-image: url('/schedule/checked.svg');
        }
        .schedule-left {
          display: flex;
          align-items: center;
          gap: 24px; 
        }

      `}</style>
    </div>
  );
};

export default ScheduleItem;

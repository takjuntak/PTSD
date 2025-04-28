interface ScheduleItemProps {
    id: number;
    time: string;
    date: string;
    active: boolean;
    onToggle: () => void;
  }
  
  const ScheduleItem: React.FC<ScheduleItemProps> = ({
    time,
    date,
    active,
    onToggle,
  }) => {
    const [ampm, clock] = time.split(' ');
  
    return (
      <div className={`schedule-item ${!active ? 'inactive' : ''}`}>
        <div className="schedule-time">
          <span className="schedule-ampm">{ampm}</span>&nbsp;
          <span className="schedule-clock">{clock}</span>
        </div>
  
        <div className="schedule-right">
          <div className="schedule-date">{date}</div>
          <label className="switch">
            <input type="checkbox" checked={active} onChange={onToggle} />
            <span className="slider"></span>
          </label>
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
          margin-bottom: 8px;
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
        @media (max-width: 640px) {
          .schedule-content {
            padding: 16px;
          }
          .schedule-title {
            font-size: 18px;
          }
          .schedule-subtitle {
            font-size: 16px;
          }
        }
      `}</style>
    </div>
  );
};

export default ScheduleItem;

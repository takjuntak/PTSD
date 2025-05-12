// src/pages/AlarmPage.tsx
import React from 'react';
import { useNavigate } from 'react-router-dom';
import { ChevronLeft, Check } from 'lucide-react';
import AlarmItem from '../components/alarm/AlarmItem';
import useAlarms from '../hooks/useAlarms';

const AlarmPage: React.FC = () => {
  const navigate = useNavigate();
  const { alarms, loading, error, markAsRead, markAllAsRead } = useAlarms();

  const handleGoBack = () => {
    navigate(-1);
  };

  // 알람을 날짜별로 그룹화
  const groupByDate = () => {
    const groups: { [key: string]: { date: string, items: typeof alarms } } = {};
    
    alarms.forEach(alarm => {
      const date = new Date(alarm.createdAt);
      const now = new Date();
      
      let dateKey: string;
      
      if (
        date.getDate() === now.getDate() && 
        date.getMonth() === now.getMonth() && 
        date.getFullYear() === now.getFullYear()
      ) {
        dateKey = '오늘';
      } else if (
        date.getDate() === now.getDate() - 1 && 
        date.getMonth() === now.getMonth() && 
        date.getFullYear() === now.getFullYear()
      ) {
        dateKey = '어제';
      } else {
        dateKey = `${date.getFullYear()}-${(date.getMonth() + 1).toString().padStart(2, '0')}-${date.getDate().toString().padStart(2, '0')}`;
      }
      
      if (!groups[dateKey]) {
        groups[dateKey] = { date: dateKey, items: [] };
      }
      
      groups[dateKey].items.push(alarm);
    });
    
    return Object.values(groups);
  };

  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white">
      <header className="p-4 px-3 flex items-center justify-between sticky top-0 z-10" style={{ backgroundColor: '#2E2E37' }}>
        <div className="flex items-center gap-3">
          <button 
            onClick={handleGoBack} 
            className="text-white border-none bg-transparent p-0"
            style={{ background: 'transparent' }}
          >
            <ChevronLeft size={24} />
          </button>
          <span className="text-xl font-bold">알림</span>
        </div>
        
        <button
          onClick={markAllAsRead}
          className="flex items-center text-blue-400 bg-transparent border-none"
        >
          <Check size={16} className="mr-1" />
          <span className="text-sm">모두 읽음</span>
        </button>
      </header>

      <div className="flex-1 overflow-y-auto pb-20">
        {loading ? (
          <div className="p-4 text-center text-gray-400">알림을 불러오는 중...</div>
        ) : error ? (
          <div className="p-4 text-center text-red-500">{error}</div>
        ) : alarms.length === 0 ? (
          <div className="p-4 text-center text-gray-400">알림이 없습니다</div>
        ) : (
          <div>
            {groupByDate().map(group => (
              <div key={group.date} className="mb-2">
                <div className="p-2 px-4 bg-gray-800 text-sm font-medium">
                  {group.date}
                </div>
                {group.items.map(alarm => (
                  <AlarmItem
                    key={alarm.id}
                    alarm={alarm}
                    onMarkAsRead={markAsRead}
                  />
                ))}
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default AlarmPage;
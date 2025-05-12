// src/components/alarm/AlarmItem.tsx
import React from 'react';
import { Alarm } from '../../hooks/useAlarms';

interface AlarmItemProps {
  alarm: Alarm;
  onMarkAsRead: (id: number) => void;
}

const AlarmItem: React.FC<AlarmItemProps> = ({ alarm, onMarkAsRead }) => {
  const { id, type, message, isRead, createdAt } = alarm;

  // 알람 시간 형식화
  const getFormattedTime = (date: Date) => {
    const now = new Date();
    const diffMin = Math.round((now.getTime() - date.getTime()) / (1000 * 60));
    
    if (diffMin < 60) {
      return `${diffMin}분 전`;
    }
    
    const diffHours = Math.floor(diffMin / 60);
    if (diffHours < 24) {
      return `${diffHours}시간 전`;
    }
    
    const diffDays = Math.floor(diffHours / 24);
    return `${diffDays}일 전`;
  };

  // 알람 타입에 따른 색상 설정
  const getDotColor = () => {
    switch (type) {
      case 'battery':
        return 'bg-blue-500';
      case 'cleaning':
        return 'bg-green-500';
      case 'warning':
        return 'bg-red-500';
      default:
        return 'bg-gray-500';
    }
  };

  const handleClick = () => {
    if (!isRead) {
      onMarkAsRead(id);
    }
  };

  return (
    <div
      className={`px-4 py-3 border-b border-gray-700 flex items-start ${!isRead ? 'bg-opacity-10 bg-blue-900' : ''}`}
      onClick={handleClick}
    >
      {!isRead && (
        <div className={`w-2 h-2 rounded-full ${getDotColor()} mt-1.5 mr-2 flex-shrink-0`} />
      )}
      {isRead && <div className="w-2 mr-2" />}
      <div className="flex-1">
        <p className="text-sm text-white">{message}</p>
      </div>
      <div className="text-xs text-gray-400 ml-2 flex-shrink-0">
        {getFormattedTime(createdAt)}
      </div>
    </div>
  );
};

export default AlarmItem;
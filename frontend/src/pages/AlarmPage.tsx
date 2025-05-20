// src/pages/AlarmPage.tsx
import React, { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import {
  ChevronLeft,
  Check,
  ChevronsLeft,
  ChevronsRight,
  BatteryCharging,
  Sparkles,
  Bell
} from 'lucide-react';
import useAlarms, { Alarm } from '../hooks/useAlarms';
import useNotificationSocket from '../hooks/useNotificationSocket';
import { useAuth } from '../hooks/useAuth';

const AlarmPage: React.FC = () => {
  const navigate = useNavigate();
  const { user } = useAuth();
  const { alarms: initialAlarms, loading, error, markAllAsRead } = useAlarms();
  const { notification } = useNotificationSocket(user?.userId);

  const [localAlarms, setLocalAlarms] = useState<Alarm[]>(initialAlarms);
  const [currentPage, setCurrentPage] = useState(1);
  const [sortOrder, setSortOrder] = useState<'latest' | 'oldest'>('latest');
  const itemsPerPage = 8;

  useEffect(() => {
    setLocalAlarms(initialAlarms);
  }, [initialAlarms]);

  useEffect(() => {
    if (notification) {
      const newAlarm: Alarm = {
        id: notification.notification.notification_id,
        type: notification.notification.type as Alarm['type'],
        message: notification.notification.message,
        isRead: notification.notification.is_read,
        createdAt: new Date(notification.notification.timestamp),
      };

      setLocalAlarms((prev) => {
        const exists = prev.some((alarm) => alarm.id === newAlarm.id);
        return exists ? prev : [newAlarm, ...prev];
      });

      window.scrollTo({ top: 0, behavior: 'smooth' });
    }
  }, [notification]);

  useEffect(() => {
    if (notification && document.visibilityState !== 'visible') {
      if (Notification.permission === 'granted') {
        new Notification(notification.notification.title, {
          body: notification.notification.message,
          icon: '/logo192.png'
        });
      } else if (Notification.permission !== 'denied') {
        Notification.requestPermission().then((permission) => {
          if (permission === 'granted') {
            new Notification(notification.notification.title, {
              body: notification.notification.message,
              icon: '/logo192.png'
            });
          }
        });
      }
    }
  }, [notification]);

  const handleGoBack = () => {
    navigate(-1);
  };

  const getIconByType = (type: Alarm['type']) => {
    switch (type) {
      case 'battery':
        return <BatteryCharging className="text-red-400 w-5 h-5" />;
      case 'cleaning':
        return <Sparkles className="text-blue-400 w-5 h-5" />;
      case 'warning':
        return <Bell className="text-yellow-400 w-5 h-5" />;
      default:
        return <Bell className="text-gray-400 w-5 h-5" />;
    }
  };

  const sortedAlarms = [...localAlarms].sort((a, b) =>
    sortOrder === 'latest'
      ? b.createdAt.getTime() - a.createdAt.getTime()
      : a.createdAt.getTime() - b.createdAt.getTime()
  );

  const totalPages = Math.ceil(sortedAlarms.length / itemsPerPage);
  const paginatedAlarms = sortedAlarms.slice(
    (currentPage - 1) * itemsPerPage,
    currentPage * itemsPerPage
  );

  const goToPage = (page: number) => {
    if (page >= 1 && page <= totalPages) {
      setCurrentPage(page);
    }
  };

  const renderPagination = () => {
    const pageNumbers = [];
    const maxVisiblePages = 5;
    let start = Math.max(1, currentPage - 2);
    let end = Math.min(totalPages, start + maxVisiblePages - 1);
    if (end - start < maxVisiblePages - 1) {
      start = Math.max(1, end - maxVisiblePages + 1);
    }

    for (let i = start; i <= end; i++) {
      pageNumbers.push(i);
    }

    return (
      <div className="flex justify-center items-center gap-2 py-6 border-t border-gray-700">
        <button
          onClick={() => goToPage(1)}
          disabled={currentPage === 1}
          className="flex items-center gap-1 text-sm text-gray-300 hover:text-white bg-transparent"
        >
          <ChevronsLeft size={18} />
        </button>

        {pageNumbers.map((num) => (
          <button
            key={num}
            onClick={() => goToPage(num)}
            className={`px-3 py-1.5 mx-1 rounded-xl text-sm font-medium transition-all duration-150 ${
              currentPage === num
                ? 'bg-blue-500 text-white shadow-lg scale-[1.05]'
                : 'bg-blue-100 text-blue-600 hover:bg-blue-200'
            }`}
          >
            {num}
          </button>
        ))}

        <button
          onClick={() => goToPage(totalPages)}
          disabled={currentPage === totalPages}
          className="flex items-center gap-1 text-sm text-gray-300 hover:text-white bg-transparent"
        >
          <ChevronsRight size={18} />
        </button>
      </div>
    );
  };

  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white" style={{ backgroundColor: '#2E2E37' }}>
      <header className="p-4 px-3 flex items-center justify-between sticky top-0 z-10" style={{ backgroundColor: '#2E2E37' }}>
        <div className="flex items-center gap-3">
          <button 
            onClick={handleGoBack} 
            className="text-white border-none bg-transparent p-0"
            style={{ background: 'transparent' }}
          >
            <ChevronLeft size={24} />
          </button>
          <span className="text-lg font-bold text-[#767676]">알림</span>
        </div>
      </header>

      <div className="flex-1 overflow-y-auto px-4 py-3" style={{ backgroundColor: '#2E2E37', paddingBottom: '80px' }}>
        <div className="rounded-lg overflow-hidden" style={{ backgroundColor: '#373738' }}>
          <div className="p-3 border-b border-gray-700 flex justify-between items-center">
            <div className="flex gap-2">
              <button
                onClick={() => setSortOrder('latest')}
                className={`px-3 py-1 text-sm rounded-full transition-colors duration-150 ${sortOrder === 'latest' ? 'bg-blue-500 text-white' : 'bg-gray-600 text-gray-200'}`}
              >최신순</button>
              <button
                onClick={() => setSortOrder('oldest')}
                className={`px-3 py-1 text-sm rounded-full transition-colors duration-150 ${sortOrder === 'oldest' ? 'bg-blue-500 text-white' : 'bg-gray-600 text-gray-200'}`}
              >오래된순</button>
            </div>
            <button
              onClick={markAllAsRead}
              className="flex items-center text-blue-400 hover:text-blue-300 border border-blue-400 px-2 py-1 rounded-md"
            >
              <Check size={16} className="mr-1" />
              <span className="text-sm">모두 읽음</span>
            </button>
          </div>

          {loading ? (
            <div className="p-4 text-center text-gray-400">알림을 불러오는 중...</div>
          ) : error ? (
            <div className="p-4 text-center text-red-500">{error}</div>
          ) : paginatedAlarms.length === 0 ? (
            <div className="p-4 text-center text-gray-400">알림이 없습니다</div>
          ) : (
            <div>
              {paginatedAlarms.map((alarm) => (
                <div key={alarm.id} className={`flex items-start gap-3 p-4 border-b border-gray-600 ${alarm.isRead ? 'opacity-60' : ''}`}>
                  {getIconByType(alarm.type)}
                  <div className="flex-1">
                    <div className="flex justify-between items-center">
                      <p className="text-sm font-medium">{alarm.message}</p>
                      {!alarm.isRead && <span className="text-xs text-blue-400 font-semibold">읽지 않음</span>}
                    </div>
                    <p className="text-xs text-gray-400 mt-1 text-left">{alarm.createdAt.toLocaleString()}</p>
                  </div>
                </div>
              ))}
              {renderPagination()}
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default AlarmPage;
// src/hooks/useAlarms.ts
import { useState, useEffect } from 'react';

export interface Alarm {
  id: number;
  type: 'battery' | 'cleaning' | 'warning';
  message: string;
  isRead: boolean;
  createdAt: Date;
}

const LOCAL_STORAGE_KEY = 'alarmReadStatus';

export const useAlarms = () => {
  const [alarms, setAlarms] = useState<Alarm[]>([]);
  const [hasUnread, setHasUnread] = useState(false);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const fetchAlarms = async () => {
      try {
        setLoading(true);

        const mockAlarms: Alarm[] = [
          { id: 1, type: 'battery', message: '[경고] 배터리가 부족합니다.', isRead: false, createdAt: new Date(Date.now() - 5 * 60 * 1000) },
          { id: 2, type: 'cleaning', message: '[종료] 장애물에 부딪쳤습니다.', isRead: true, createdAt: new Date(Date.now() - 38 * 60 * 1000) },
          { id: 3, type: 'warning', message: '[주의] 5분 이상 충전되지 않고 있습니다.', isRead: true, createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) },
          { id: 4, type: 'battery', message: '[경고] 배터리가 부족합니다.', isRead: false, createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) },
          { id: 5, type: 'battery', message: '[경고] 배터리가 부족합니다.', isRead: true, createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) },
          { id: 6, type: 'battery', message: '[경고] 배터리가 부족합니다.', isRead: false, createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) },
          { id: 7, type: 'cleaning', message: '[종료] 장애물에 부딪쳤습니다.', isRead: false, createdAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000) },
          { id: 8, type: 'battery', message: '[경고] 배터리가 부족합니다.', isRead: false, createdAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000) },
          { id: 9, type: 'cleaning', message: '[종료] 장애물에 부딪쳤습니다.', isRead: false, createdAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000) },
          { id: 10, type: 'battery', message: '[경고] 배터리가 부족합니다.', isRead: true, createdAt: new Date(Date.now() - 5 * 24 * 60 * 60 * 1000) }
        ];

        const stored = localStorage.getItem(LOCAL_STORAGE_KEY);
        const readMap: Record<string, boolean> = stored ? JSON.parse(stored) : {};

        const withReadStatus = mockAlarms.map(alarm => ({
          ...alarm,
          isRead: readMap[alarm.id] ?? alarm.isRead
        }));

        setAlarms(withReadStatus);
        setHasUnread(withReadStatus.some(alarm => !alarm.isRead));
        setLoading(false);
      } catch (err) {
        setError('알람을 불러오는데 실패했습니다.');
        setLoading(false);
      }
    };

    fetchAlarms();
  }, []);

  const markAsRead = (alarmId: number) => {
    setAlarms(prev => {
      const updated = prev.map(alarm =>
        alarm.id === alarmId ? { ...alarm, isRead: true } : alarm
      );
      localStorage.setItem(
        LOCAL_STORAGE_KEY,
        JSON.stringify({ ...JSON.parse(localStorage.getItem(LOCAL_STORAGE_KEY) || '{}'), [alarmId]: true })
      );
      setHasUnread(updated.some(alarm => !alarm.isRead));
      return updated;
    });
  };

  const markAllAsRead = () => {
    setAlarms(prev => {
      const updated = prev.map(alarm => ({ ...alarm, isRead: true }));
      const readMap = Object.fromEntries(updated.map(a => [a.id, true]));
      localStorage.setItem(LOCAL_STORAGE_KEY, JSON.stringify(readMap));
      setHasUnread(false);
      return updated;
    });
  };

  return {
    alarms,
    hasUnread,
    loading,
    error,
    markAsRead,
    markAllAsRead
  };
};

export default useAlarms;
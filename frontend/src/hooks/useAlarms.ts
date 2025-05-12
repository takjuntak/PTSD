// src/hooks/useAlarms.ts
import { useState, useEffect } from 'react';

// 알람 타입 정의
export interface Alarm {
  id: number;
  type: 'battery' | 'cleaning' | 'warning'; // 배터리, 청소, 경고 등 알림 타입
  message: string;
  isRead: boolean;
  createdAt: Date;
}

export const useAlarms = () => {
  const [alarms, setAlarms] = useState<Alarm[]>([]);
  const [hasUnread, setHasUnread] = useState(false);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // 초기 알람 데이터 로드 (목업 데이터)
  useEffect(() => {
    const fetchAlarms = async () => {
      try {
        setLoading(true);
        
        // 목업 데이터 사용 (나중에 API 연결로 대체 예정)
        const mockAlarms: Alarm[] = [
          {
            id: 1,
            type: 'battery',
            message: '[경고] 배터리가 부족합니다.',
            isRead: false,
            createdAt: new Date(Date.now() - 5 * 60 * 1000) // 5분 전
          },
          {
            id: 2,
            type: 'cleaning',
            message: '[종료] 장애물에 부딪쳤습니다.',
            isRead: true,
            createdAt: new Date(Date.now() - 38 * 60 * 1000) // 38분 전
          },
          {
            id: 3, 
            type: 'warning',
            message: '[주의] 5분 이상 충전이지 않고 있습니다.',
            isRead: true, 
            createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) // 2일 전
          },
          {
            id: 4,
            type: 'battery',
            message: '[경고] 배터리가 부족합니다.',
            isRead: false,
            createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) // 2일 전
          },
          {
            id: 5,
            type: 'battery',
            message: '[경고] 배터리가 부족합니다.',
            isRead: true,
            createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) // 2일 전
          },
          {
            id: 6,
            type: 'battery',
            message: '[경고] 배터리가 부족합니다.',
            isRead: false,
            createdAt: new Date(Date.now() - 2 * 24 * 60 * 60 * 1000) // 2일 전
          },
          {
            id: 7,
            type: 'cleaning',
            message: '[종료] 장애물에 부딪쳤습니다.',
            isRead: false,
            createdAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000) // 3일 전
          },
          {
            id: 8,
            type: 'battery',
            message: '[경고] 배터리가 부족합니다.',
            isRead: false,
            createdAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000) // 3일 전
          },
          {
            id: 9,
            type: 'cleaning',
            message: '[종료] 장애물에 부딪쳤습니다.',
            isRead: false,
            createdAt: new Date(Date.now() - 3 * 24 * 60 * 60 * 1000) // 3일 전
          },
          {
            id: 10,
            type: 'battery',
            message: '[경고] 배터리가 부족합니다.',
            isRead: true,
            createdAt: new Date(Date.now() - 5 * 24 * 60 * 60 * 1000) // 5일 전
          }
        ];
        
        setAlarms(mockAlarms);
        
        // 읽지 않은 알람이 있는지 확인
        const unreadExists = mockAlarms.some(alarm => !alarm.isRead);
        setHasUnread(unreadExists);
        
        setLoading(false);
      } catch (err) {
        setError('알람을 불러오는데 실패했습니다.');
        setLoading(false);
      }
    };

    fetchAlarms();
  }, []);

  // 알람을 읽음 처리하는 함수
  const markAsRead = (alarmId: number) => {
    setAlarms(prev => 
      prev.map(alarm => 
        alarm.id === alarmId 
          ? { ...alarm, isRead: true }
          : alarm
      )
    );
    
    // 읽지 않은 알람이 남아있는지 확인
    const stillHasUnread = alarms.some(
      alarm => alarm.id !== alarmId && !alarm.isRead
    );
    
    setHasUnread(stillHasUnread);
  };

  // 모든 알람을 읽음 처리하는 함수
  const markAllAsRead = () => {
    setAlarms(prev => 
      prev.map(alarm => ({ ...alarm, isRead: true }))
    );
    setHasUnread(false);
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
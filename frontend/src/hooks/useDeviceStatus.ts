// src/hooks/useDeviceStatus.ts
import { useState, useEffect } from 'react';

interface DeviceStatus {
  batteryPercentage: number;
  remainingTime: string;
  location: string;
  cleaningStatus: string;
  estimatedEndTime: string;
  scheduleTime: string;
  lastNotification: string;
  isConnected: boolean;
}

export const useDeviceStatus = () => {
  const [status, setStatus] = useState<DeviceStatus>({
    batteryPercentage: 82,
    remainingTime: '1시간 40분',
    location: '웨이트 존',
    cleaningStatus: '청소 중',
    estimatedEndTime: '오전 11:00',
    scheduleTime: '매일 오전 9:00',
    lastNotification: '10분 전 : 청소 완료',
    isConnected: true,
  });

  // 실제 구현에서는 여기서 API 호출 또는 소켓 연결 등을 통해 상태를 업데이트할 수 있습니다
  useEffect(() => {
    // 예: 주기적으로 상태 업데이트
    const intervalId = setInterval(() => {
      // 실제 API 호출로 대체할 수 있습니다
      // 테스트를 위해 배터리 퍼센트만 랜덤하게 변경
      if (Math.random() > 0.7) {
        setStatus(prev => ({
          ...prev,
          batteryPercentage: Math.max(1, Math.min(100, prev.batteryPercentage + (Math.random() > 0.5 ? 1 : -1)))
        }));
      }
    }, 30000); // 30초마다 업데이트

    return () => clearInterval(intervalId);
  }, []);

  return status;
};
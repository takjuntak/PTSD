import { useEffect, useState, useRef } from 'react';

export default function useBatteryStatus(userId: number) {
  const [battery, setBattery] = useState<number | null>(null);
  const socketRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    if (!userId) return;

    const ws = new WebSocket(`ws://localhost:8000/ws/notifications/${userId}`);
    socketRef.current = ws;

    ws.onopen = () => {
      console.log(`🔌 WebSocket 연결됨 - 사용자 ${userId}`);
    };

    ws.onmessage = (event) => {
      const message = event.data;
      const match = message.match(/배터리:\s*(\d+)/);
      if (match) {
        const percentage = Number(match[1]);
        setBattery(percentage);
      }
    };

    ws.onerror = (error) => {
      console.error('WebSocket 에러 발생:', error);
    };

    ws.onclose = () => {
      console.log(`🔌 WebSocket 연결 종료 - 사용자 ${userId}`);
    };

    return () => {
      ws.close();
    };
  }, [userId]);

  return { battery };
}

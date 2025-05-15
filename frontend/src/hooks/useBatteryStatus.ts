// useBatteryStatus.ts
import { useEffect, useState, useRef } from 'react';

export default function useBatteryStatus(userId?: number) {
  const [battery, setBattery] = useState<number | null>(null);
  const socketRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    if (!userId) return;

    const ws = new WebSocket(`ws://localhost:8000/ws/notifications/${userId}`);
    socketRef.current = ws;

    ws.onmessage = (event) => {
      const message = event.data;
      const match = message.match(/배터리:\s*(\d+)/);
      if (match) {
        setBattery(Number(match[1]));
      }
    };

    ws.onerror = (err) => console.error('WebSocket 오류:', err);
    ws.onclose = () => console.log(`🔌 WebSocket 종료 - 사용자 ${userId}`);

    return () => {
      ws.close();
    };
  }, [userId]);

  return { battery };
}

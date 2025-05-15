// useBatteryStatus.ts
import { useEffect, useState, useRef } from 'react';

export default function useBatteryStatus(userId?: number) {
  const [battery, setBattery] = useState<number | null>(null);
  const socketRef = useRef<WebSocket | null>(null);

  useEffect(() => {
    // ✅ userId 없으면 연결하지 않음
    if (!userId) {
      console.warn('userId가 없어 WebSocket 연결 생략됨');
      return;
    }

    const ws = new WebSocket(`wss://k12d101.p.ssafy.io/ws/notifications/7`);
    socketRef.current = ws;

    ws.onopen = () => {
      console.log('🔌 WebSocket 연결됨 (userId 7로 고정)');
    };

    ws.onmessage = (event) => {
      const message = event.data;
      const match = message.match(/배터리:\s*(\d+)/);
      if (match) {
        setBattery(Number(match[1]));
      }
    };

    ws.onerror = (err) => console.error('WebSocket 오류:', err);
    ws.onclose = () => console.log('🔌 WebSocket 종료');

    return () => {
      ws.close();
    };
  }, [userId]); // 의존성 유지

  return { battery };
}

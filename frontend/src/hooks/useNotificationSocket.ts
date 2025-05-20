// src/hooks/useNotificationSocket.ts
import { useEffect, useRef, useState, useCallback } from 'react';

interface NotificationPayload {
  category: string;
  notification: {
    notification_id: number;
    title: string;
    message: string;
    type: string;
    timestamp: string;
    is_read: boolean;
  };
}

export default function useNotificationSocket(userId?: number) {
  const [notification, setNotification] = useState<NotificationPayload | null>(null);
  const [status, setStatus] = useState<'connecting' | 'connected' | 'disconnected' | 'error'>('disconnected');

  const socketRef = useRef<WebSocket | null>(null);

  const connect = useCallback(() => {
    if (!userId) return;

    const ws = new WebSocket(`wss://k12d101.p.ssafy.io/ws/notifications/${userId}`);
    socketRef.current = ws;
    setStatus('connecting');

    ws.onopen = () => setStatus('connected');
    ws.onclose = () => setStatus('disconnected');
    ws.onerror = () => setStatus('error');

    ws.onmessage = (event) => {
      try {
        const data: NotificationPayload = JSON.parse(event.data);
        if (data.category === 'battery_alert') {
          console.log('🔔 알림 수신:', data);
          setNotification(data);
        }
      } catch (e) {
        console.error('🔴 알림 메시지 파싱 실패:', e);
      }
    };
  }, [userId]);

  useEffect(() => {
    connect();
    return () => socketRef.current?.close();
  }, [connect]);

  return { notification, status };
}

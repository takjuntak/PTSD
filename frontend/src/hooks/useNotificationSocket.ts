// src/hooks/useNotificationSocket.ts
import { useState, useEffect, useRef, useCallback } from 'react';

// 알림 타입 정의
export interface NotificationData {
  notification_id: number;
  title: string;
  message: string;
  type: 'start' | 'complete' | 'battery' | 'warning';
  timestamp: string;
  is_read: boolean;
}

export interface NotificationMessage {
  category: string;
  notification: NotificationData;
}

export default function useNotificationSocket(userId?: number) {
  const [notification, setNotification] = useState<NotificationMessage | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected' | 'error'>('disconnected');
  const socketRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const reconnectAttemptsRef = useRef(0);
  const MAX_RECONNECT_ATTEMPTS = 5;
  const RECONNECT_INTERVAL = 3000; // 3초

  // 웹소켓 연결 함수
  const connectWebSocket = useCallback(() => {
    if (!userId) {
      console.log('userId가 없어 WebSocket 연결 생략');
      return;
    }

    // 이미 연결이 시도 중이면 중복 연결 방지
    if (socketRef.current && socketRef.current.readyState === WebSocket.CONNECTING) {
      return;
    }

    // 이전 연결이 있으면 정리
    if (socketRef.current) {
      socketRef.current.close();
    }

    // 재연결 타이머가 있으면 정리
    if (reconnectTimeoutRef.current) {
      window.clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    setConnectionStatus('connecting');
    console.log(`알림 WebSocket 연결 시도... (userId: ${userId}, 시도: ${reconnectAttemptsRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);

    try {
      // ws 프로토콜 사용, 프로덕션에서는 wss 필요
      const wsUrl = import.meta.env.VITE_NOTIFICATION_WS_URL 
  ? `${import.meta.env.VITE_NOTIFICATION_WS_URL}/${userId}`
  : `ws://localhost:8000/ws/notifications/${userId}`;
      const ws = new WebSocket(wsUrl);
      socketRef.current = ws;

      ws.onopen = () => {
        console.log(`알림 WebSocket 연결 성공 (userId: ${userId})`);
        setIsConnected(true);
        setConnectionStatus('connected');
        reconnectAttemptsRef.current = 0; // 연결 성공 시 재시도 카운트 초기화
      };

      ws.onmessage = (event) => {
        try {
          console.log('알림 받음:', event.data);
          const data: NotificationMessage = JSON.parse(event.data);
          setNotification(data);
          
          // 브라우저 알림 기능 (옵션)
          if (Notification.permission === 'granted') {
            new Notification(data.notification.title, {
              body: data.notification.message
            });
          }
        } catch (error) {
          console.error('알림 메시지 파싱 오류:', error);
        }
      };

      ws.onerror = (error) => {
        console.error('알림 WebSocket 오류:', error);
        setConnectionStatus('error');
      };

      ws.onclose = (event) => {
        console.log(`알림 WebSocket 연결 종료 (code: ${event.code}, reason: ${event.reason})`);
        setIsConnected(false);
        setConnectionStatus('disconnected');

        // 재연결 시도 (최대 시도 횟수 내에서)
        if (reconnectAttemptsRef.current < MAX_RECONNECT_ATTEMPTS) {
          reconnectTimeoutRef.current = window.setTimeout(() => {
            reconnectAttemptsRef.current += 1;
            connectWebSocket();
          }, RECONNECT_INTERVAL);
        } else {
          console.error(`최대 재연결 시도 횟수(${MAX_RECONNECT_ATTEMPTS})를 초과했습니다.`);
        }
      };
    } catch (error) {
      console.error('알림 WebSocket 초기화 오류:', error);
      setConnectionStatus('error');
    }
  }, [userId]);

  // 웹소켓 연결 설정
  useEffect(() => {
    connectWebSocket();

    // 컴포넌트 언마운트 시 정리
    return () => {
      if (socketRef.current) {
        console.log('컴포넌트 언마운트로 알림 WebSocket 연결 종료');
        socketRef.current.close();
      }
      
      if (reconnectTimeoutRef.current) {
        window.clearTimeout(reconnectTimeoutRef.current);
      }
    };
  }, [connectWebSocket]);

  // 수동으로 재연결할 수 있는 함수
  const reconnect = useCallback(() => {
    reconnectAttemptsRef.current = 0; // 재시도 카운트 초기화
    connectWebSocket();
  }, [connectWebSocket]);

  return {
    notification,
    isConnected,
    connectionStatus,
    reconnect
  };
}
// src/hooks/useBatteryStatus.ts 수정
import { useEffect, useState, useRef, useCallback } from 'react';

// 두 가지 메시지 형식을 모두 처리하기 위한 타입 정의
interface BatteryStatusMessage {
  category: string;
  percentage: number;
}

interface BatteryAlertMessage {
  category: string;
  notification: {
    notification_id: number;
    title: string;
    message: string;
    type: string;
    timestamp: string;
    is_read: boolean;
  }
}

type WebSocketMessage = BatteryStatusMessage | BatteryAlertMessage;

interface BatteryStatus {
  battery: number | null;
  isConnected: boolean;
  lastMessage: WebSocketMessage | null;
  connectionStatus: 'connecting' | 'connected' | 'disconnected' | 'error';
}

export default function useBatteryStatus(userId?: number) {
  const [status, setStatus] = useState<BatteryStatus>({
    battery: null,
    isConnected: false,
    lastMessage: null,
    connectionStatus: 'disconnected'
  });
  
  const socketRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const reconnectAttemptsRef = useRef(0);
  const MAX_RECONNECT_ATTEMPTS = 5;
  const RECONNECT_INTERVAL = 3000; // 3초

  // 웹소켓 연결 함수를 useCallback으로 감싸서 의존성 배열이 변경될 때만 재생성
  const connectWebSocket = useCallback(() => {
    if (!userId) {
      console.warn('🔴 userId가 없어 WebSocket 연결 생략됨');
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

    setStatus(prev => ({ ...prev, connectionStatus: 'connecting' }));
    console.log(`🟡 WebSocket 연결 시도 중... (userId: ${userId}, 시도: ${reconnectAttemptsRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);

    try {
      const ws = new WebSocket(`ws://k12d101.p.ssafy.io:8081/ws/notifications/${userId}`);
      socketRef.current = ws;

      ws.onopen = () => {
        console.log(`🟢 WebSocket 연결 성공 (userId: ${userId})`);
        setStatus(prev => ({
          ...prev,
          isConnected: true,
          connectionStatus: 'connected'
        }));
        reconnectAttemptsRef.current = 0; // 연결 성공 시 재시도 카운트 초기화
      };

      ws.onmessage = (event) => {
        try {
          // 원본 메시지 로그 (디버깅용)
          console.log(`📨 원본 메시지: ${event.data}`);
          
          // JSON 형식으로 파싱
          const data = JSON.parse(event.data);
          console.log(`📩 WebSocket 메시지 수신:`, data);
          
          // 배터리 정보 추출
          let batteryPercentage: number | null = null;
          
          // case 1: 배터리 상태 메시지 처리 (백엔드의 send_battery_status 함수)
          if (data.category === 'battery_status' && typeof data.percentage === 'number') {
            batteryPercentage = data.percentage;
            console.log(`🔋 배터리 상태 업데이트: ${batteryPercentage}%`);
          }
          // case 2: 배터리 알림 메시지 처리 (백엔드의 create_battery_notification 함수)
          else if (data.category === 'battery_alert' && data.notification) {
            // 배터리 메시지에서 퍼센트 추출 (예: "배터리가 부족합니다. (15%)")
            const percentMatch = data.notification.message.match(/\((\d+)%\)/);
            if (percentMatch && percentMatch[1]) {
              batteryPercentage = parseInt(percentMatch[1], 10);
              console.log(`🔋 배터리 알림에서 추출한 상태: ${batteryPercentage}%`);
            }
          }
          
          // 배터리 상태 업데이트
          if (batteryPercentage !== null) {
            setStatus(prev => ({ 
              ...prev, 
              battery: batteryPercentage,
              lastMessage: data 
            }));
          } else {
            // 다른 카테고리의 메시지도 저장 (처리는 하지 않음)
            console.log(`ℹ️ 지원되지 않는 메시지 카테고리: ${data.category}`);
            setStatus(prev => ({ ...prev, lastMessage: data }));
          }
        } catch (error) {
          console.error('📛 메시지 파싱 오류:', error, event.data);
        }
      };

      ws.onerror = (error) => {
        console.error('🔴 WebSocket 오류:', error);
        setStatus(prev => ({ ...prev, connectionStatus: 'error' }));
      };

      ws.onclose = (event) => {
        console.log(`🔌 WebSocket 연결 종료 (code: ${event.code}, reason: ${event.reason})`);
        setStatus(prev => ({
          ...prev,
          isConnected: false,
          connectionStatus: 'disconnected'
        }));

        // 재연결 시도 (최대 시도 횟수 내에서)
        if (reconnectAttemptsRef.current < MAX_RECONNECT_ATTEMPTS) {
          reconnectTimeoutRef.current = window.setTimeout(() => {
            reconnectAttemptsRef.current += 1;
            connectWebSocket();
          }, RECONNECT_INTERVAL);
        } else {
          console.error(`🔴 최대 재연결 시도 횟수(${MAX_RECONNECT_ATTEMPTS})를 초과하여 재연결을 중단합니다.`);
        }
      };
    } catch (error) {
      console.error('🔴 WebSocket 초기화 오류:', error);
      setStatus(prev => ({ ...prev, connectionStatus: 'error' }));
    }
  }, [userId]);

  // 웹소켓 연결 설정
  useEffect(() => {
    connectWebSocket();

    // 컴포넌트 언마운트 시 정리
    return () => {
      if (socketRef.current) {
        console.log('🔌 컴포넌트 언마운트로 WebSocket 연결 종료');
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
    battery: status.battery,
    isConnected: status.isConnected,
    lastMessage: status.lastMessage,
    connectionStatus: status.connectionStatus,
    reconnect // 수동 재연결 함수
  };
}
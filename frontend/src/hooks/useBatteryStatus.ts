// src/hooks/useBatteryStatus.ts
import { useEffect, useState, useRef, useCallback } from 'react';

interface BatteryStatusResponse {
  category: string;
  percentage: number;
}

interface BatteryStatus {
  battery: number | null;
  isConnected: boolean;
  lastMessage: BatteryStatusResponse | null;
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
      const wsUrl = `ws://k12d101.p.ssafy.io/ws/notifications/${userId}`;
      console.log(`🔌 WebSocket URL: ${wsUrl}`);
      
      const ws = new WebSocket(wsUrl);
      socketRef.current = ws;

      ws.onopen = () => {
        console.log(`🟢 WebSocket 연결 성공 (userId: ${userId})`);
        setStatus(prev => ({
          ...prev,
          isConnected: true,
          connectionStatus: 'connected'
        }));
        reconnectAttemptsRef.current = 0; // 연결 성공 시 재시도 카운트 초기화
        
        // 연결 후 핑 메시지 전송 (선택적)
        try {
          ws.send(JSON.stringify({ type: "ping" }));
          console.log("🏓 Ping 메시지 전송됨");
        } catch (error) {
          console.error("🏓 Ping 메시지 전송 실패:", error);
        }
      };

      ws.onmessage = (event) => {
        try {
          // 원시 데이터 로깅
          console.log(`📥 WebSocket 원시 데이터:`, event.data);
          
          // JSON 형식으로 파싱
          const data = JSON.parse(event.data);
          console.log(`📩 WebSocket 파싱된 데이터:`, data);
          
          // 데이터 구조 로깅
          console.log('📋 데이터 구조:', Object.keys(data));
          
          // 메시지 카테고리 확인
          if (data.category === 'battery_status' && typeof data.percentage === 'number') {
            console.log(`🔋 배터리 상태 업데이트: ${data.percentage}%`);
            
            setStatus(prev => ({ 
              ...prev, 
              battery: data.percentage,
              lastMessage: data 
            }));
          } else if (data.percentage !== undefined && typeof data.percentage === 'number') {
            // category가 없지만 percentage는 있는 경우 (백엔드가 형식을 다르게 보낼 수도 있음)
            console.log(`🔋 배터리 상태 업데이트 (대체 형식): ${data.percentage}%`);
            
            setStatus(prev => ({ 
              ...prev, 
              battery: data.percentage,
              lastMessage: { category: 'battery_status', percentage: data.percentage } 
            }));
          } else {
            // 다른 형식의 메시지
            console.log(`📩 알 수 없는 메시지 형식:`, data);
            // 다른 카테고리의 메시지도 저장
            setStatus(prev => ({ ...prev, lastMessage: data as any }));
          }
        } catch (error) {
          console.error('📛 메시지 파싱 오류:', error, event.data);
          // 파싱 불가능한 메시지 원본 저장
          setStatus(prev => ({ 
            ...prev, 
            lastMessage: { category: 'unknown', percentage: 0, raw: event.data } as any 
          }));
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

  // 디버깅용: 백엔드에서 오는 메시지 형식이 확인되면 
  // 이 함수를 통해 상태를 수동으로 업데이트할 수 있음
  const manuallyUpdateBattery = (percentage: number) => {
    console.log(`🔋 수동 배터리 업데이트: ${percentage}%`);
    setStatus(prev => ({ 
      ...prev, 
      battery: percentage,
      lastMessage: { category: 'battery_status', percentage }
    }));
  };

  // 웹소켓 연결 설정
  useEffect(() => {
    connectWebSocket();

    // 개발 중 테스트를 위한 window 객체에 디버깅 함수 노출
    // @ts-ignore
    window.updateBattery = manuallyUpdateBattery;
    console.log('🔧 디버깅: window.updateBattery(백분율) 함수로 배터리 상태를 수동으로 업데이트할 수 있습니다.');

    // 컴포넌트 언마운트 시 정리
    return () => {
      if (socketRef.current) {
        console.log('🔌 컴포넌트 언마운트로 WebSocket 연결 종료');
        socketRef.current.close();
      }
      
      if (reconnectTimeoutRef.current) {
        window.clearTimeout(reconnectTimeoutRef.current);
      }
      
      // @ts-ignore
      delete window.updateBattery;
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
    reconnect, // 수동 재연결 함수
    manuallyUpdateBattery // 배터리 상태 수동 업데이트 함수
  };
}
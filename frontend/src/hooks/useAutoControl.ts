// src/hooks/useAutoControl.ts
import { useState, useEffect, useRef, useCallback } from 'react';

interface AutoControlMessage {
  device_id: number;
  command: string;
}

interface AutoControlResponse {
  status: 'success' | 'error';
  message: string;
}

export const useAutoControl = (deviceId: number | null) => {
  const [isConnected, setIsConnected] = useState(false);
  const [lastResponse, setLastResponse] = useState<AutoControlResponse | null>(null);
  const [status, setStatus] = useState<string>('연결 시도 중...');
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const reconnectCountRef = useRef(0);
  const isIntentionalCloseRef = useRef(false);
  
  const MAX_RECONNECT_ATTEMPTS = 5;
  const RECONNECT_INTERVAL = 3000; // 3초마다 재연결 시도

  // 웹소켓 URL 결정
  const wsUrl = import.meta.env.VITE_WS_AUTO_CONTROL_URL;

  // 웹소켓 연결 설정
  const connectWebSocket = useCallback(() => {
    if (!deviceId) {
      setStatus('디바이스 연결 필요');
      console.log('useAutoControl: 디바이스 ID가 없어 웹소켓 연결을 시도하지 않습니다.');
      return;
    }
    
    // 재연결 타이머가 남아있으면 정리
    if (reconnectTimeoutRef.current !== null) {
      window.clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    
    // 이전 웹소켓 연결이 있으면 정리
    if (wsRef.current) {
      isIntentionalCloseRef.current = true; 
      wsRef.current.close();
      wsRef.current = null;
    }
    
    isIntentionalCloseRef.current = false;
    setStatus('연결 시도 중...');
    
    try {
      if (!wsUrl) {
        throw new Error('WebSocket URL이 설정되지 않음');
      }
      
      console.log(`useAutoControl: 자동 조작 웹소켓 연결 시도 (${reconnectCountRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);
      const ws = new WebSocket(wsUrl);
      
      ws.onopen = () => {
        console.log('useAutoControl: 웹소켓 연결 성공');
        setIsConnected(true);
        setStatus('연결됨');
        reconnectCountRef.current = 0;
      };
      
      ws.onmessage = (event) => {
        try {
          console.log(`useAutoControl: 메시지 수신: ${event.data}`);
          const data = JSON.parse(event.data) as AutoControlResponse;
          setLastResponse(data);
          setStatus(data.status === 'success' ? '준비됨' : '오류 발생');
        } catch (error) {
          console.error('useAutoControl: 메시지 파싱 오류', error);
          setLastResponse({ 
            status: 'error',
            message: '응답 처리 오류'
          });
        }
      };
      
      ws.onclose = (event) => {
        setIsConnected(false);
        console.log(`useAutoControl: 웹소켓 연결 종료 (코드: ${event.code})`);
        
        if (!isIntentionalCloseRef.current) {
          if (reconnectCountRef.current < MAX_RECONNECT_ATTEMPTS) {
            setStatus(`재연결 중... (${reconnectCountRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);
            
            reconnectTimeoutRef.current = window.setTimeout(() => {
              reconnectCountRef.current += 1;
              connectWebSocket();
            }, RECONNECT_INTERVAL);
          } else {
            setStatus('재연결 실패. 페이지를 새로고침해 주세요.');
          }
        } else {
          setStatus('연결 종료됨');
        }
      };
      
      ws.onerror = (error) => {
        console.error('useAutoControl: 웹소켓 오류', error);
        setStatus('연결 오류');
      };
      
      wsRef.current = ws;
    } catch (error) {
      console.error('useAutoControl: 웹소켓 초기화 오류', error);
      setStatus('연결 실패');
    }
  }, [deviceId, wsUrl]);

  useEffect(() => {
    connectWebSocket();
    
    return () => {
      if (reconnectTimeoutRef.current !== null) {
        window.clearTimeout(reconnectTimeoutRef.current);
      }
      
      if (wsRef.current) {
        isIntentionalCloseRef.current = true;
        wsRef.current.close();
      }
    };
  }, [connectWebSocket]);

  // 명령 전송 함수
  const sendCommand = useCallback((command: 'start' | 'complete') => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.log(`useAutoControl: 웹소켓이 연결되지 않음 (상태: ${wsRef.current?.readyState})`);
      setStatus('명령 전송 실패: 연결 안됨');
      return false;
    }

    if (!deviceId) {
      console.log('useAutoControl: 디바이스 ID 없음');
      setStatus('명령 전송 실패: 디바이스 ID 없음');
      return false;
    }

    try {
      const message: AutoControlMessage = {
        device_id: deviceId,
        command: command
      };
      
      const messageString = JSON.stringify(message);
      wsRef.current.send(messageString);
      console.log(`useAutoControl: 명령 전송: ${command}`);
      return true;
    } catch (error) {
      console.error('useAutoControl: 명령 전송 오류', error);
      setStatus('명령 전송 실패');
      return false;
    }
  }, [deviceId]);

  // 자동 조작 시작
  const startAutoControl = useCallback(() => {
    return sendCommand('start');
  }, [sendCommand]);

  // 복귀 명령
  const returnHome = useCallback(() => {
    return sendCommand('complete');
  }, [sendCommand]);

  return {
    isConnected,
    status,
    lastResponse,
    startAutoControl,
    returnHome
  };
};
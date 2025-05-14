// src/hooks/useRobotWebSocket.ts
import { useState, useEffect, useRef } from 'react';

interface Message {
  device_id: number;
  command: string;
}

export const useRobotWebSocket = (deviceId: number | null) => {
  const [isConnected, setIsConnected] = useState(false);
  const [lastResponse, setLastResponse] = useState<any>(null);
  const [status, setStatus] = useState<string>('연결 시도 중...');
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const reconnectCountRef = useRef(0);
  const isIntentionalCloseRef = useRef(false);
  
  const MAX_RECONNECT_ATTEMPTS = 5;
  const RECONNECT_INTERVAL = 3000; // 3초마다 재연결 시도

  // 웹소켓 URL 결정 (환경 변수에서 가져옴)
  const wsUrl = import.meta.env.VITE_WS_URL;

  // 웹소켓 연결 설정
  useEffect(() => {
    if (!deviceId) {
      setStatus('디바이스 연결 필요');
      return;
    }
    
    let isComponentMounted = true;

    // 웹소켓 연결 함수
    const connectWebSocket = () => {
      if (!isComponentMounted) return;
      
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
        console.log('웹소켓 연결 시도');
        const ws = new WebSocket(wsUrl);
        
        ws.onopen = () => {
          if (!isComponentMounted) return;
          console.log('웹소켓 연결 성공');
          setIsConnected(true);
          setStatus('연결됨');
          reconnectCountRef.current = 0;
        };
        
        ws.onmessage = (event) => {
          if (!isComponentMounted) return;
          try {
            const data = JSON.parse(event.data);
            console.log('수신된 메시지:', data);
            setLastResponse(data);
          } catch (error) {
            console.error('메시지 파싱 오류:', error);
          }
        };
        
        ws.onclose = () => {
          if (!isComponentMounted) return;
          console.log('웹소켓 연결 종료');
          setIsConnected(false);
          
          if (!isIntentionalCloseRef.current) {
            if (reconnectCountRef.current < MAX_RECONNECT_ATTEMPTS) {
              setStatus(`재연결 중... (${reconnectCountRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);
              
              reconnectTimeoutRef.current = window.setTimeout(() => {
                if (isComponentMounted) {
                  reconnectCountRef.current += 1;
                  connectWebSocket();
                }
              }, RECONNECT_INTERVAL);
            } else {
              setStatus('재연결 실패. 페이지를 새로고침해 주세요.');
            }
          } else {
            setStatus('연결 종료됨');
          }
        };
        
        ws.onerror = (error) => {
          if (!isComponentMounted) return;
          console.error('웹소켓 오류:', error);
          setStatus('연결 오류');
        };
        
        wsRef.current = ws;
      } catch (error) {
        if (!isComponentMounted) return;
        console.error('웹소켓 초기화 오류:', error);
        setStatus('연결 실패');
      }
    };

    connectWebSocket();
    
    return () => {
      isComponentMounted = false;
      if (reconnectTimeoutRef.current !== null) {
        window.clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
      
      if (wsRef.current) {
        isIntentionalCloseRef.current = true;
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, [wsUrl, deviceId]);

  // 명령 전송 함수
  const sendCommand = (command: string) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.error('웹소켓이 연결되지 않았습니다');
      setStatus('명령 전송 실패: 연결 안됨');
      return false;
    }

    if (!deviceId) {
      console.error('디바이스 ID가 없습니다');
      setStatus('명령 전송 실패: 디바이스 ID 없음');
      return false;
    }

    try {
      const message: Message = {
        device_id: deviceId,
        command: command
      };
      
      wsRef.current.send(JSON.stringify(message));
      console.log(`명령 전송: ${command}`);
      return true;
    } catch (error) {
      console.error('메시지 전송 오류:', error);
      setStatus('명령 전송 실패');
      return false;
    }
  };

  return {
    isConnected,
    status,
    lastResponse,
    sendCommand
  };
};
// src/hooks/useRobotWebSocket.ts
import { useState, useEffect, useRef } from 'react';

interface Message {
  device_id: number;
  command: string;
}

export const useRobotWebSocket = (deviceId: number | null) => {
  const [isConnected, setIsConnected] = useState(false);
  const [lastResponse, setLastResponse] = useState<any>(null);
  const [status, setStatus] = useState<string>('대기 중');
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const reconnectCountRef = useRef(0);
  // 의도적인 종료 플래그 추가
  const isIntentionalCloseRef = useRef(false);
  
  const MAX_RECONNECT_ATTEMPTS = 5;
  const RECONNECT_INTERVAL = 3000; // 3초마다 재연결 시도

  // 웹소켓 URL 결정 (환경 변수에서 가져옴)
  const wsUrl = import.meta.env.VITE_WS_URL;

  // 웹소켓 연결 설정
  useEffect(() => {
    // 웹소켓 연결 함수
    const connectWebSocket = () => {
      // 재연결 타이머가 남아있으면 정리
      if (reconnectTimeoutRef.current !== null) {
        window.clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
      
      // 이전 웹소켓 연결이 있으면 정리
      if (wsRef.current) {
        isIntentionalCloseRef.current = true; // 의도적인 종료 표시
        wsRef.current.close();
        wsRef.current = null;
      }
      
      // 의도적인 종료 플래그 초기화
      isIntentionalCloseRef.current = false;
      
      try {
        // 웹소켓 객체 생성
        const ws = new WebSocket(wsUrl);
        
        // 연결 이벤트 핸들러
        ws.onopen = () => {
          console.log('웹소켓 연결됨');
          setIsConnected(true);
          setStatus('연결됨');
          reconnectCountRef.current = 0; // 재연결 카운터 초기화
        };
        
        // 메시지 수신 핸들러
        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            console.log('웹소켓 메시지 수신:', data);
            setLastResponse(data);
          } catch (error) {
            console.error('메시지 파싱 오류:', error);
          }
        };
        
        // 연결 종료 핸들러
        ws.onclose = () => {
          console.log('웹소켓 연결 종료');
          setIsConnected(false);
          
          // 의도적인 종료가 아닌 경우에만 재연결 시도
          if (!isIntentionalCloseRef.current) {
            setStatus('연결 끊김');
            
            // 자동 재연결 시도
            if (reconnectCountRef.current < MAX_RECONNECT_ATTEMPTS) {
              setStatus(`재연결 중... (${reconnectCountRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);
              
              // 재연결 타이머 설정
              reconnectTimeoutRef.current = window.setTimeout(() => {
                reconnectCountRef.current += 1;
                connectWebSocket();
              }, RECONNECT_INTERVAL);
            } else {
              setStatus('재연결 실패');
            }
          } else {
            // 의도적인 종료인 경우
            setStatus('연결 종료됨');
            console.log('의도적인 웹소켓 종료');
          }
        };
        
        // 오류 핸들러
        ws.onerror = (error) => {
          console.error('웹소켓 오류:', error);
          setStatus('오류 발생');
        };
        
        // 참조 저장
        wsRef.current = ws;
      } catch (error) {
        console.error('웹소켓 생성 오류:', error);
        setStatus('연결 실패');
      }
    };

    // 초기 연결 시도
    connectWebSocket();
    
    // 컴포넌트 언마운트 시 정리
    return () => {
      if (reconnectTimeoutRef.current !== null) {
        window.clearTimeout(reconnectTimeoutRef.current);
        reconnectTimeoutRef.current = null;
      }
      
      if (wsRef.current) {
        isIntentionalCloseRef.current = true; // 의도적인 종료 표시
        wsRef.current.close();
        wsRef.current = null;
      }
    };
  }, [wsUrl, deviceId]); // deviceId가 변경되면 재연결

  // 명령 전송 함수
  const sendCommand = (command: string) => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      console.error('웹소켓이 연결되지 않았습니다');
      return false;
    }

    if (!deviceId) {
      console.error('디바이스 ID가 없습니다');
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
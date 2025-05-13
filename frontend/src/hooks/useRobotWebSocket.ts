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

  // 웹소켓 URL 결정 (개발/배포 환경에 따라)
  const wsUrl = import.meta.env.VITE_WS_URL;

  // 웹소켓 연결 설정
  useEffect(() => {
    // 웹소켓 객체 생성
    const ws = new WebSocket(wsUrl);
    
    // 연결 이벤트 핸들러
    ws.onopen = () => {
      console.log('웹소켓 연결됨');
      setIsConnected(true);
      setStatus('연결됨');
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
      setStatus('연결 끊김');
    };
    
    // 오류 핸들러
    ws.onerror = (error) => {
      console.error('웹소켓 오류:', error);
      setStatus('오류 발생');
    };
    
    // 참조 저장
    wsRef.current = ws;
    
    // 컴포넌트 언마운트 시 정리
    return () => {
      ws.close();
    };
  }, [wsUrl]);

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
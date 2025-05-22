// src/hooks/useRobotWebSocket.ts (디버깅 개선 버전 - 훅만 수정)
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

  // 디버깅 로그 출력 함수
  const logDebug = (type: string, message: string) => {
    const time = new Date().toISOString().slice(11, 19);
    console.log(`[WS ${time}] [${type}] ${message}`);
  };

  // 웹소켓 연결 설정
  useEffect(() => {
    if (!deviceId) {
      setStatus('디바이스 연결 필요');
      logDebug('초기화', '디바이스 ID가 없음');
      return;
    }
    
    let isComponentMounted = true;
    logDebug('초기화', `디바이스 ID: ${deviceId}, URL: ${wsUrl || '설정 안됨'}`);

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
        logDebug('연결', '이전 웹소켓 연결 종료');
      }
      
      isIntentionalCloseRef.current = false;
      setStatus('연결 시도 중...');
      
      try {
        if (!wsUrl) {
          throw new Error('WebSocket URL이 설정되지 않음');
        }
        
        logDebug('연결', `시도 ${reconnectCountRef.current + 1}/${MAX_RECONNECT_ATTEMPTS}`);
        const ws = new WebSocket(wsUrl);
        
        ws.onopen = () => {
          if (!isComponentMounted) return;
          logDebug('성공', '웹소켓 연결 성공');
          setIsConnected(true);
          setStatus('연결됨');
          reconnectCountRef.current = 0;
        };
        
        ws.onmessage = (event) => {
          if (!isComponentMounted) return;
          try {
            const dataSample = typeof event.data === 'string' 
              ? (event.data.length > 100 ? event.data.substring(0, 100) + '...' : event.data)
              : '비텍스트 데이터';
            
            logDebug('수신', dataSample);
            
            // 응답에서 오류 검사
            if (typeof event.data === 'string') {
              // MQTT 오류 확인
              if (event.data.includes('mqtt') && 
                 (event.data.includes('failure') || event.data.includes('failed') || event.data.includes('error'))) {
                logDebug('오류', `MQTT 오류: ${event.data}`);
              } 
              // 다른 유형의 오류 검사
              else if (event.data.includes('error') || event.data.includes('exception')) {
                logDebug('오류', `응답 오류: ${event.data}`);
              }
              // 통신 상태 확인
              else if (event.data.includes('status') || event.data.includes('state')) {
                logDebug('상태', event.data);
              }
            }
            
            const data = JSON.parse(event.data);
            setLastResponse(data);
          } catch (error) {
            logDebug('파싱오류', `메시지 파싱 실패: ${error instanceof Error ? error.message : '알 수 없는 오류'}`);
            setLastResponse({ 
              raw: event.data,
              parseError: true,
              time: new Date().toISOString()
            });
          }
        };
        
        ws.onclose = (event) => {
          if (!isComponentMounted) return;
          
          // 종료 코드 분석
          let codeMessage = '알 수 없는 코드';
          switch(event.code) {
            case 1000: codeMessage = '정상 종료'; break;
            case 1001: codeMessage = '서버 종료/페이지 이동'; break;
            case 1002: codeMessage = '프로토콜 오류'; break;
            case 1003: codeMessage = '데이터 형식 오류'; break;
            case 1005: codeMessage = '종료 코드 없음'; break;
            case 1006: codeMessage = '비정상 종료 (연결 끊김)'; break;
            case 1007: codeMessage = '데이터 형식 위반'; break;
            case 1008: codeMessage = '정책 위반'; break;
            case 1009: codeMessage = '메시지 크기 초과'; break;
            case 1010: codeMessage = '서버 기능 협상 실패'; break;
            case 1011: codeMessage = '서버 내부 오류'; break;
            case 1012: codeMessage = '서버 재시작'; break;
            case 1013: codeMessage = '서버 과부하'; break;
            case 1014: codeMessage = 'TLS 핸드셰이크 실패'; break;
            case 1015: codeMessage = '서버 종료'; break;
          }
          
          logDebug('종료', `코드 ${event.code} (${codeMessage})${event.reason ? ', 이유: ' + event.reason : ''}`);
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
              
              logDebug('재연결', `${RECONNECT_INTERVAL/1000}초 후 재시도`);
            } else {
              setStatus('재연결 실패. 페이지를 새로고침해 주세요.');
              logDebug('재연결실패', `최대 시도 횟수(${MAX_RECONNECT_ATTEMPTS}) 초과`);
            }
          } else {
            setStatus('연결 종료됨');
            logDebug('종료', '의도적 종료');
          }
        };
        
        ws.onerror = (error) => {
          if (!isComponentMounted) return;
          logDebug('오류', '연결 오류 발생');
          console.error('WebSocket Error:', error);
          setStatus('연결 오류');
        };
        
        wsRef.current = ws;
      } catch (error) {
        if (!isComponentMounted) return;
        logDebug('초기화오류', error instanceof Error ? error.message : '알 수 없는 오류');
        setStatus('연결 실패');
      }
    };

    connectWebSocket();
    
    return () => {
      isComponentMounted = false;
      logDebug('정리', '컴포넌트 언마운트');
      
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
      const stateTexts = ['연결 중', '연결됨', '종료 중', '종료됨'];
      const stateText = wsRef.current ? stateTexts[wsRef.current.readyState] : '연결 객체 없음';
      
      logDebug('전송실패', `명령: ${command}, WebSocket 상태: ${stateText}`);
      setStatus('명령 전송 실패: 연결 안됨');
      return false;
    }

    if (!deviceId) {
      logDebug('전송실패', '디바이스 ID 없음');
      setStatus('명령 전송 실패: 디바이스 ID 없음');
      return false;
    }

    try {
      const message: Message = {
        device_id: deviceId,
        command: command
      };
      
      const messageString = JSON.stringify(message);
      wsRef.current.send(messageString);
      logDebug('전송', `명령: ${command}, 전체: ${messageString}`);
      return true;
    } catch (error) {
      logDebug('전송오류', error instanceof Error ? error.message : '알 수 없는 오류');
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
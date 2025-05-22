// src/hooks/useAutoControl.ts (향상된 디버깅 버전)
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
  const [error, setError] = useState<string | null>(null);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);
  const reconnectCountRef = useRef(0);
  const isIntentionalCloseRef = useRef(false);
  
  const MAX_RECONNECT_ATTEMPTS = 5;
  const RECONNECT_INTERVAL = 3000; // 3초마다 재연결 시도

  // 웹소켓 URL 결정
  const wsUrl = import.meta.env.VITE_WS_AUTO_CONTROL_URL;

  // 디버깅을 위한 로그 함수
  const logDebug = (message: string, data?: any) => {
    const timestamp = new Date().toISOString().slice(11, 19);
    console.log(`[AutoControl ${timestamp}] ${message}`, data ? data : '');
  };

  // 웹소켓 연결 설정
  const connectWebSocket = useCallback(() => {
    if (!deviceId) {
      setStatus('디바이스 연결 필요');
      setError('디바이스 ID가 없습니다');
      logDebug('디바이스 ID가 없어 웹소켓 연결을 시도하지 않습니다.');
      return;
    }
    
    if (!wsUrl) {
      setStatus('설정 오류');
      setError('WebSocket URL이 설정되지 않았습니다');
      logDebug('WebSocket URL이 설정되지 않았습니다. 환경 변수를 확인하세요.', { wsUrl });
      return;
    }

    logDebug(`연결 시도: ${wsUrl}`, { deviceId, attempt: reconnectCountRef.current + 1 });
    
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
    setError(null);
    
    try {
      // 수동 제어와 URL 비교 로깅 (디버깅용)
      const manualWsUrl = import.meta.env.VITE_WS_URL;
      logDebug('WebSocket URL 비교', { 
        자동제어URL: wsUrl, 
        수동제어URL: manualWsUrl,
        차이점: wsUrl !== manualWsUrl ? '다름' : '같음'
      });
      
      const ws = new WebSocket(wsUrl);
      
      ws.onopen = () => {
        logDebug('웹소켓 연결 성공');
        setIsConnected(true);
        setStatus('연결됨');
        setError(null);
        reconnectCountRef.current = 0;
      };
      
      ws.onmessage = (event) => {
        try {
          logDebug('메시지 수신', event.data);
          const data = JSON.parse(event.data) as AutoControlResponse;
          setLastResponse(data);
          setStatus(data.status === 'success' ? '준비됨' : '오류 발생');
        } catch (error) {
          logDebug('메시지 파싱 오류', error);
          setLastResponse({ 
            status: 'error',
            message: '응답 처리 오류'
          });
        }
      };
      
      ws.onclose = (event) => {
        setIsConnected(false);
        
        // 종료 코드 의미 해석
        let closeReason: string;
        switch(event.code) {
          case 1000: closeReason = '정상 종료'; break;
          case 1001: closeReason = '엔드포인트가 종료됨'; break;
          case 1002: closeReason = '프로토콜 오류'; break;
          case 1003: closeReason = '잘못된 데이터 형식'; break;
          case 1005: closeReason = '코드 없이 종료'; break;
          case 1006: closeReason = '비정상 종료 (연결 끊김)'; break;
          case 1007: closeReason = '메시지 형식 오류'; break;
          case 1008: closeReason = '정책 위반'; break;
          case 1009: closeReason = '메시지 크기 초과'; break;
          case 1010: closeReason = '클라이언트 기능 요청 거부'; break;
          case 1011: closeReason = '서버 내부 오류'; break;
          case 1012: closeReason = '서버 재시작'; break;
          case 1013: closeReason = '서버 과부하'; break;
          case 1014: closeReason = 'TLS 핸드셰이크 실패'; break;
          case 1015: closeReason = 'TLS 오류'; break;
          default: closeReason = `알 수 없는 코드: ${event.code}`;
        }
        
        const errorMessage = `연결 종료 - ${closeReason}${event.reason ? ` (${event.reason})` : ''}`;
        setError(errorMessage);
        logDebug(errorMessage, { code: event.code, reason: event.reason });
        
        if (!isIntentionalCloseRef.current) {
          if (reconnectCountRef.current < MAX_RECONNECT_ATTEMPTS) {
            setStatus(`재연결 중... (${reconnectCountRef.current + 1}/${MAX_RECONNECT_ATTEMPTS})`);
            
            reconnectTimeoutRef.current = window.setTimeout(() => {
              reconnectCountRef.current += 1;
              connectWebSocket();
            }, RECONNECT_INTERVAL);
            
            logDebug(`${RECONNECT_INTERVAL/1000}초 후 재연결 시도`);
          } else {
            setStatus('재연결 실패');
            setError(`최대 재시도 횟수(${MAX_RECONNECT_ATTEMPTS})를 초과했습니다. 페이지를 새로고침해 주세요.`);
            logDebug('최대 재시도 횟수 초과');
          }
        } else {
          setStatus('연결 종료됨');
        }
      };
      
      ws.onerror = (error) => {
        // WSS와 WS 프로토콜 불일치 가능성 체크
        const isWss = wsUrl.startsWith('wss:');
        const siteProtocol = window.location.protocol;
        const protocolMismatch = siteProtocol === 'https:' && !isWss;
        
        if (protocolMismatch) {
          const mismatchError = 'HTTPS 사이트에서는 WSS 프로토콜을 사용해야 합니다. 서버 설정을 확인하세요.';
          setError(mismatchError);
          logDebug(mismatchError, { siteProtocol, wsProtocol: isWss ? 'wss' : 'ws' });
        } else {
          setError('연결 오류가 발생했습니다. 콘솔을 확인하세요.');
        }
        
        logDebug('웹소켓 오류 발생', error);
        setStatus('연결 오류');
      };
      
      wsRef.current = ws;
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : '알 수 없는 오류';
      setStatus('연결 실패');
      setError(`WebSocket 초기화 오류: ${errorMessage}`);
      logDebug('WebSocket 초기화 오류', error);
    }
  }, [deviceId, wsUrl]);

  useEffect(() => {
    // 환경 변수 체크 및 출력 (디버깅용)
    logDebug('환경 변수 확인', {
      VITE_WS_AUTO_CONTROL_URL: wsUrl,
      VITE_WS_URL: import.meta.env.VITE_WS_URL,
      deviceId
    });
    
    // WS URL이 상대 경로면 현재 호스트 기반으로 절대 경로 구성 제안
    if (wsUrl && !wsUrl.startsWith('ws:') && !wsUrl.startsWith('wss:')) {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const host = window.location.host;
      logDebug('상대 경로 감지 - 절대 경로 제안', {
        현재URL: wsUrl,
        제안URL: `${protocol}//${host}${wsUrl.startsWith('/') ? '' : '/'}${wsUrl}`
      });
    }
    
    connectWebSocket();
    
    return () => {
      logDebug('컴포넌트 언마운트 - 리소스 정리');
      if (reconnectTimeoutRef.current !== null) {
        window.clearTimeout(reconnectTimeoutRef.current);
      }
      
      if (wsRef.current) {
        isIntentionalCloseRef.current = true;
        wsRef.current.close();
      }
    };
  }, [connectWebSocket]);

  // 상태 진단 및 문제 해결 도우미 함수
  const diagnoseConnection = useCallback(() => {
    logDebug('연결 진단 실행');
    
    // 환경 변수 문제 체크
    if (!wsUrl) {
      return {
        status: 'error',
        message: 'WebSocket URL이 설정되지 않았습니다. 환경 변수를 확인하세요.',
        solution: 'VITE_WS_AUTO_CONTROL_URL 환경 변수가 올바르게 설정되었는지 확인하세요.'
      };
    }
    
    // 프로토콜 문제 체크
    const isWss = wsUrl.startsWith('wss:');
    const isWs = wsUrl.startsWith('ws:');
    const siteProtocol = window.location.protocol;
    
    if (siteProtocol === 'https:' && !isWss) {
      return {
        status: 'warning',
        message: 'HTTPS 사이트에서 WS 프로토콜을 사용하고 있습니다.',
        solution: 'HTTPS 사이트에서는 WSS 프로토콜을 사용해야 합니다. 환경 변수를 WSS로 변경하세요.'
      };
    }
    
    if (!isWs && !isWss) {
      return {
        status: 'error',
        message: 'WebSocket URL이 ws:// 또는 wss://로 시작하지 않습니다.',
        solution: '올바른 WebSocket URL 형식을 사용하세요. 현재 URL: ' + wsUrl
      };
    }
    
    // 기기 ID 체크
    if (!deviceId) {
      return {
        status: 'error',
        message: '디바이스 ID가 없습니다.',
        solution: '디바이스 연결 후 다시 시도하세요.'
      };
    }
    
    // 현재 연결 상태 확인
    if (wsRef.current) {
      const readyStateText = ['연결 중', '연결됨', '종료 중', '종료됨'][wsRef.current.readyState];
      return {
        status: wsRef.current.readyState === 1 ? 'success' : 'info',
        message: `현재 WebSocket 상태: ${readyStateText} (${wsRef.current.readyState})`,
        solution: wsRef.current.readyState !== 1 ? '연결이 완전히 성립될 때까지 기다리세요.' : '연결이 정상입니다.'
      };
    }
    
    return {
      status: 'info',
      message: '연결이 초기화되지 않았습니다.',
      solution: 'connectWebSocket()을 호출하여 연결을 시도하세요.'
    };
  }, [wsUrl, deviceId]);

  // 수동으로 연결 재시도 함수
  const retryConnection = useCallback(() => {
    logDebug('수동 재연결 시도');
    reconnectCountRef.current = 0; // 재시도 카운트 초기화
    connectWebSocket();
  }, [connectWebSocket]);

  // URL 재설정 도우미 (수동 제어 URL을 자동 제어에 활용해보기 위한 임시 함수)
  const tryAlternativeUrl = useCallback(() => {
    // 수동 제어 URL을 기반으로 자동 제어 URL 생성 시도
    const manualWsUrl = import.meta.env.VITE_WS_URL;
    if (!manualWsUrl) {
      logDebug('대체 URL 시도 실패 - 수동 제어 URL이 없음');
      return false;
    }
    
    // 수동 제어 URL에서 경로만 바꿔서 자동 제어 URL로 시도
    const urlObj = new URL(manualWsUrl);
    const parts = urlObj.pathname.split('/');
    const lastPart = parts[parts.length - 1];
    
    // 마지막 경로 부분만 'auto-control'로 변경
    if (lastPart === 'manual-control') {
      parts[parts.length - 1] = 'auto-control';
      urlObj.pathname = parts.join('/');
      const alternativeUrl = urlObj.toString();
      
      logDebug('대체 URL 시도', { 기존URL: wsUrl, 대체URL: alternativeUrl });
      
      try {
        // 기존 연결 닫기
        if (wsRef.current) {
          isIntentionalCloseRef.current = true;
          wsRef.current.close();
          wsRef.current = null;
        }
        
        const ws = new WebSocket(alternativeUrl);
        ws.onopen = () => {
          logDebug('대체 URL 연결 성공');
          setIsConnected(true);
          setStatus('연결됨 (대체 URL)');
          setError(null);
          reconnectCountRef.current = 0;
        };
        
        ws.onclose = () => {
          logDebug('대체 URL 연결 종료');
          setIsConnected(false);
          setStatus('대체 URL 연결 실패');
        };
        
        ws.onerror = (e) => {
          logDebug('대체 URL 연결 오류', e);
        };
        
        wsRef.current = ws;
        return true;
      } catch (error) {
        logDebug('대체 URL 시도 실패', error);
        return false;
      }
    }
    
    logDebug('대체 URL 생성 실패 - 경로 패턴 불일치');
    return false;
  }, [wsUrl]);

  // 명령 전송 함수
  const sendCommand = useCallback((command: 'start' | 'complete') => {
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      const stateTexts = ['연결 중', '연결됨', '종료 중', '종료됨'];
      const stateText = wsRef.current ? stateTexts[wsRef.current.readyState] : '연결 객체 없음';
      
      logDebug(`명령 전송 실패: ${command}`, { 연결상태: stateText });
      setStatus(`명령 전송 실패: 연결 안됨 (${stateText})`);
      return false;
    }

    if (!deviceId) {
      logDebug('명령 전송 실패: 디바이스 ID 없음');
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
      logDebug(`명령 전송: ${command}`, message);
      return true;
    } catch (error) {
      const errorMessage = error instanceof Error ? error.message : '알 수 없는 오류';
      logDebug('명령 전송 오류', error);
      setStatus(`명령 전송 실패: ${errorMessage}`);
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
    error,
    lastResponse,
    startAutoControl,
    returnHome,
    diagnoseConnection,
    retryConnection,
    tryAlternativeUrl
  };
};
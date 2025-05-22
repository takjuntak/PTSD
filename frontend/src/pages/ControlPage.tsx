// src/pages/ControlPage.tsx
import { useNavigate } from 'react-router-dom';
import { useState, useEffect } from 'react';
import { useDevices } from '../hooks/useDevices';
import { useAutoControl } from '../hooks/useAutoControl';

import playImage from '../assets/control/play.svg';
import stopImage from '../assets/control/stop.svg';
import homeImage from '../assets/control/comback.svg';
import currentLocationImage from '../assets/control/currentlocation.svg';
import manualOperationImage from '../assets/control/manualoperation.svg';
import ThreeRobot from '../components/status/ThreeRobot'

const ControlPage = () => {
  const navigate = useNavigate();
  const { connectedDevices } = useDevices();
  
  // 현재 선택된 디바이스 ID
  const deviceId = connectedDevices.length > 0 ? connectedDevices[0].device_id : null;
  
  // 자동 조작 웹소켓 훅 사용
  const { isConnected, startAutoControl, returnHome, lastResponse } = useAutoControl(deviceId);

  const [isPlaying, setIsPlaying] = useState(false);
  const [isReturning, setIsReturning] = useState(false);
  const [statusMessage, setStatusMessage] = useState<string | null>(null);

  // 동작 제어 토글 (PLAY/STOP)
  const togglePlay = () => {
    if (!deviceId || !isConnected) {
      setStatusMessage(deviceId ? '서버 연결 중...' : '연결된 디바이스가 없습니다');
      setTimeout(() => setStatusMessage(null), 3000);
      return;
    }

    if (isPlaying) {
      // 이미 실행 중이면 정지 명령 전송
      const success = returnHome(); // complete 명령 전송
      if (success) {
        setIsPlaying(false);
        setStatusMessage('정지 명령 전송됨');
      } else {
        setStatusMessage('명령 전송 실패');
      }
    } else {
      // 시작 명령 전송
      const success = startAutoControl(); // start 명령 전송
      if (success) {
        setIsPlaying(true);
        setStatusMessage('시작 명령 전송됨');
      } else {
        setStatusMessage('명령 전송 실패');
      }
    }
  };

  // 로봇 복귀 - 동작 제어 중지와 동일한 기능 수행
  const triggerReturn = () => {
    if (!deviceId || !isConnected) {
      setStatusMessage(deviceId ? '서버 연결 중...' : '연결된 디바이스가 없습니다');
      setTimeout(() => setStatusMessage(null), 3000);
      return;
    }
    
    if (!isReturning) {
      // 복귀 명령 전송 (complete)
      const success = returnHome();
      if (success) {
        setIsReturning(true);
        setIsPlaying(false); // 플레이 상태도 중지 - 동작 제어 Stop과 같은 효과
        setStatusMessage('복귀 명령 전송됨');
        setTimeout(() => {
          setIsReturning(false);
          setStatusMessage(null);
        }, 3000);
      } else {
        setStatusMessage('복귀 명령 전송 실패');
        setTimeout(() => setStatusMessage(null), 3000);
      }
    }
  };

  // 서버 응답 처리
  useEffect(() => {
    if (lastResponse) {
      console.log('서버 응답:', lastResponse);
      
      // 명령 결과에 따른 피드백 제공
      if (lastResponse.status === 'success') {
        setStatusMessage(`명령 성공: ${lastResponse.message}`);
      } else {
        setStatusMessage(`오류: ${lastResponse.message}`);
      }
      
      // 3초 후 메시지 숨기기
      setTimeout(() => setStatusMessage(null), 3000);
    }
  }, [lastResponse]);

  const handleMapClick = () => {
    navigate('/location');
  }

  const cardStyle = {
    width: 138,
    height: 146,
    backgroundColor: '#373738',
    borderRadius: 10,
    boxShadow: '2px 2px 2px rgba(0,0,0,0.25)',
    padding: 16,
    position: 'relative' as const,
    display: 'flex',
    flexDirection: 'column' as const,
    justifyContent: 'flex-start' as const,
  };

  const textStyle = {
    fontFamily: 'Montserrat',
    fontWeight: 700,
    fontSize: 15,
    lineHeight: '18px',
    color: '#FFFFFF',
    textAlign: 'left' as const,
  };

  const imageStyle = {
    width: 76,
    height: 76,
    position: 'absolute' as const,
    left: '50%',
    transform: 'translateX(-50%)',
    bottom: 25,
  };

  return (
    <div className="w-full h-full flex flex-col">

      {/* 상태 메시지 표시 영역 (조건부 렌더링) */}
      {statusMessage && (
        <div className="mx-4 mb-2 -mt-1 text-center text-sm text-blue-400 bg-blue-900/20 py-1 px-3 rounded-md">
          {statusMessage}
        </div>
      )}

      <main className="flex-1 px-4 pb-32 overflow-y-auto flex flex-col items-center">
        {/* 로봇 3D 모델 표시 */}
        <div className="my-5">
          <ThreeRobot />
        </div>

        <div className="grid grid-cols-2 gap-4" style={{ maxWidth: 412 }}>
          {/* 동작 제어 */}
          <button 
            onClick={togglePlay} 
            style={{
              ...cardStyle,
              opacity: (deviceId && isConnected) ? 1 : 0.7,
              cursor: (deviceId && isConnected) ? 'pointer' : 'not-allowed'
            }}
          >
            <span style={textStyle}>동작 제어</span>
            <img
              src={isPlaying ? stopImage : playImage}
              alt="동작 제어"
              style={{ ...imageStyle, bottom: 32 }}
            />
            <span
              style={{
                ...textStyle,
                textAlign: 'center',
                position: 'absolute',
                bottom: 12,
                left: '50%',
                transform: 'translateX(-50%) translateY(-4px)',
                fontSize: 13,
              }}
            >
              {isPlaying ? 'STOP' : 'PLAY'}
            </span>
          </button>

          {/* 로봇 복귀 - 동작 제어 Stop과 동일한 기능 */}
          <button 
            onClick={triggerReturn} 
            style={{
              ...cardStyle,
              opacity: (deviceId && isConnected) ? 1 : 0.7,
              cursor: (deviceId && isConnected) ? 'pointer' : 'not-allowed'
            }}
          >
            <span style={textStyle}>로봇 복귀</span>
            <img src={homeImage} alt="로봇 복귀" style={{ ...imageStyle, bottom: 30 }} />
            {isReturning && (
              <span
                style={{
                  ...textStyle,
                  textAlign: 'center',
                  position: 'absolute',
                  bottom: 12,
                  left: '50%',
                  transform: 'translateX(-50%) translateY(-4px)',
                  fontSize: 13,
                }}
              >
                ·· 복귀 중 ··
              </span>
            )}
          </button>

          {/* 현재 위치 */}
          <div style={cardStyle} onClick={handleMapClick}>
            <span style={textStyle}>현재 위치</span>
            <img src={currentLocationImage} alt="현재 위치" style={imageStyle} />
          </div>

          {/* 수동 조작 */}
          <button onClick={() => navigate('/robot-control')} style={cardStyle}>
            <span style={textStyle}>수동 조작</span>
            <img src={manualOperationImage} alt="수동 조작" style={imageStyle} />
          </button>
        </div>
      </main>
    </div>
  );
};

export default ControlPage;
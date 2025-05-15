import { ChevronLeft } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import { useState } from 'react';

import playImage from '../assets/control/play.svg';
import stopImage from '../assets/control/stop.svg';
import homeImage from '../assets/control/comback.svg';
import currentLocationImage from '../assets/control/currentlocation.svg';
import manualOperationImage from '../assets/control/manualoperation.svg';
import ThreeRobot from '../components/status/ThreeRobot'

const ControlPage = () => {
  const navigate = useNavigate();

  const [isPlaying, setIsPlaying] = useState(false);
  const [isReturning, setIsReturning] = useState(false);

  const togglePlay = () => setIsPlaying(prev => !prev);

  const triggerReturn = () => {
    if (!isReturning) {
      setIsReturning(true);
      setTimeout(() => setIsReturning(false), 3000);
    }
  };

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
      <header className="p-4 px-3 flex items-center gap-3 sticky top-0 z-10">
        <button onClick={() => navigate(-1)} className="border-none bg-transparent p-0">
          <ChevronLeft size={24} color="#FFFFFF" />
        </button>
        <span className="text-xl font-bold" style={{ color: '#767676', fontFamily: 'inter' }}>
          제어
        </span>
      </header>

      <main className="flex-1 px-4 pb-32 overflow-y-auto flex flex-col items-center">
        {/* ✅ 로봇 3D 모델 표시 */}
        <div className="my-5">
          <ThreeRobot />
        </div>

        <div className="grid grid-cols-2 gap-4" style={{ maxWidth: 412 }}>
          {/* 동작 제어 */}
          <button onClick={togglePlay} style={cardStyle}>
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

          {/* 로봇 복귀 */}
          <button onClick={triggerReturn} style={cardStyle}>
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
          <div style={cardStyle}>
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

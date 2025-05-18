// MainPage.tsx
import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
// import robotImage from '../assets/robot.png'; 
// import ChargeIndicator from '../components/charge';
// import Header from '../components/common/Header';
import LocationMap from '../components/location/LocationMap'; 
import useScroll from '../hooks/useScroll';
import { useAuth } from '../hooks/useAuth';
import { useDevices } from '../hooks/useDevices';
import useBatteryStatus from '../hooks/useBatteryStatus';
import ThreeRobot from '../components/status/ThreeRobot'
import ChargeInfo from '../components/charge/ChargeInfo';
import ChargeStatus from '../components/charge/ChargeStatus';


const MainPage = () => {
  const { containerRef } = useScroll();
  const [isCharging] = useState(false);
  const [currentLocation] = useState("현재 위치: 웨이트 존");

  const { user } = useAuth();
  const { devices, connectedDevices } = useDevices();
  const navigate = useNavigate();

  const currentDevice = connectedDevices.length > 0
    ? connectedDevices[0]
    : devices.length > 0
      ? devices[0]
      : null;

  // 🔒 user 존재 + 디바이스 연결 시에만 WebSocket 연결
  const { battery } = useBatteryStatus(user?.userId && currentDevice ? user.userId : undefined);

  const handleMapClick = () => {
    navigate('/location');
  }

  return (
    <div className="flex flex-col w-full h-full">
      <main className="flex-1 flex flex-col items-center justify-start w-full px-4 pb-24 overflow-y-auto" ref={containerRef}>
        <div className="text-center text-white text-lg font-medium mt-4 mb-4 w-full">
          {user && currentDevice ? 
            `${user.name}님의 ${currentDevice.name}` : 
            user && !currentDevice ? 
              `${user.name}님, 기기를 등록해주세요` : 
              '로그인이 필요합니다'}
        </div>

        <div className="flex justify-center items-center mb-6">
          <ThreeRobot />
        </div>

        <div className="flex justify-center mb-4">
          {battery === null ? (
            <p className="text-sm text-gray-400">배터리 정보를 불러오는 중...</p>
          ) : (
            <div className="flex flex-col items-center">
              <ChargeStatus percentage={battery} remainingTime="1시간 25분 남았습니다" />
              <ChargeInfo percentage={battery} isCharging={isCharging} />
            </div>
          )}
        </div>
        
        <div className='cursor-pointer w-full max-w-md' onClick={handleMapClick}>
          <LocationMap currentLocation={currentLocation} />
        </div>
      </main>
    </div>
  );
};

export default MainPage;

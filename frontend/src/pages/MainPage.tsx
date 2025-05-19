// MainPage.tsx
import { useState, useEffect } from 'react';
import { useNavigate } from 'react-router-dom';
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

  // 디버깅을 위한 로그 추가
  useEffect(() => {
    console.log("🔍 User 정보:", user);
    console.log("🔍 Current Device:", currentDevice);
    console.log("🔍 WebSocket 연결에 사용될 userId:", user?.userId);
  }, [user, currentDevice]);

  // 🔒 user 존재 + 디바이스 연결 시에만 WebSocket 연결
  const { battery, connectionStatus, lastMessage } = useBatteryStatus(
    user?.userId ? user.userId : undefined
  );
  
  // 디버깅을 위한 추가 로그
  useEffect(() => {
    console.log("🔋 Battery Status:", battery);
    console.log("🔌 Connection Status:", connectionStatus);
    console.log("📩 Last Message:", lastMessage);
  }, [battery, connectionStatus, lastMessage]);

  const handleMapClick = () => {
    navigate('/location');
  }

  // 배터리 상태를 표시하는 부분에 더 많은 정보 추가
  const renderBatteryInfo = () => {
    if (battery === null) {
      return (
        <div className="text-sm text-gray-400">
          <p>배터리 정보를 불러오는 중...</p>
          <p className="text-xs text-gray-500 mt-1">연결 상태: {connectionStatus}</p>
        </div>
      );
    }

    return (
      <div className="flex flex-col items-center">
        <ChargeStatus percentage={battery} remainingTime="1시간 25분 남았습니다" />
        <ChargeInfo percentage={battery} isCharging={isCharging} />
      </div>
    );
  };

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
          {renderBatteryInfo()}
        </div>
        
        <div className='cursor-pointer w-full max-w-md' onClick={handleMapClick}>
          <LocationMap currentLocation={currentLocation} />
        </div>
      </main>
    </div>
  );
};

export default MainPage;
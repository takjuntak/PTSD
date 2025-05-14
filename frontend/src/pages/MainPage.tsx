import { useState } from 'react';
import robotImage from '../assets/robot.png'; 
import ChargeIndicator from '../components/charge';
// import Header from '../components/common/Header';
import LocationMap from '../components/location/LocationMap'; 
import useScroll from '../hooks/useScroll';
import { useAuth } from '../hooks/useAuth';
import { useDevices } from '../hooks/useDevices';
import useBatteryStatus from '../hooks/useBatteryStatus';

const MainPage = () => {
  // useScroll 훅 사용
  const { containerRef } = useScroll();
  
  // 배터리 상태 (75%)
  const [isCharging] = useState(false);
  // 현재 위치 정보 상태
  const [currentLocation] = useState("현재 위치: 웨이트 존");
  
  // 사용자 인증 정보
  const { user } = useAuth();
  const battery = user?.userId ? useBatteryStatus(user.userId).battery : null;

  // 기기 정보 가져오기
  const { devices, connectedDevices } = useDevices();
  
  // 현재 선택된/연결된 기기
  const currentDevice = connectedDevices.length > 0 ? connectedDevices[0] : (devices.length > 0 ? devices[0] : null);

  return (
    <div className="flex flex-col w-full h-full">
      {/* 헤더 */}
      {/* <Header title="PTSD" /> */}
      
      <main className="flex-1 flex flex-col items-center justify-start w-full px-4 pb-24 overflow-y-auto" ref={containerRef}>
        {/* 사용자와 기기명 표시 - 가운데 정렬 */}
        <div className="text-center text-white text-lg font-medium mt-4 mb-4 w-full">
          {user && currentDevice ? 
            `${user.name}님의 ${currentDevice.name}` : 
            user && !currentDevice ? 
              `${user.name}님, 기기를 등록해주세요` : 
              '로그인이 필요합니다'}
        </div>
        
        {/* 로봇 이미지 */}
        <div className="flex justify-center items-center mb-6">
          <img src={robotImage} alt="IoT 로봇" className="w-48 h-48" />
        </div>
        
        {/* 배터리 인디케이터 */}
        <div className="flex justify-center mb-4">
          {battery === null ? (
            <p className="text-sm text-gray-400">배터리 정보를 불러오는 중...</p>
          ) : (
            <ChargeIndicator percentage={battery} isCharging={isCharging} />
          )}
        </div>
        
        {/* 위치 지도 추가 */}
        <LocationMap currentLocation={currentLocation} />
      </main>
    </div>
  );
};

export default MainPage;
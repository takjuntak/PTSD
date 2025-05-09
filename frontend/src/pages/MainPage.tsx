// src/pages/MainPage.tsx
import { useState } from 'react';
import robotImage from '../assets/robot.png'; 
import ChargeIndicator from '../components/charge';
import LocationMap from '../components/location/LocationMap'; // 주석 해제
import useScroll from '../hooks/useScroll';

const MainPage = () => {
  // useScroll 훅 사용
  const { containerRef } = useScroll();
  
  // 배터리 상태 (75%)
  const [isCharging] = useState(false);
  // 현재 위치 정보 상태
  const [currentLocation] = useState("현재 위치: 웨이트 존");

  return (
    <div className="flex flex-col w-full h-full">
      
      <main className="flex-1 flex flex-col items-center justify-start w-full px-4 overflow-y-auto" ref={containerRef}>
        {/* PTSD 로봇 텍스트 - 왼쪽 정렬 */}
        <div className="text-left text-white text-lg font-medium mt-4 mb-4 w-full">
          김싸피님의 PTSD
        </div>
        
        {/* 로봇 이미지 */}
        <div className="flex justify-center items-center mb-6">
          <img src={robotImage} alt="IoT 로봇" className="w-48 h-48" />
        </div>
        
        {/* 배터리 인디케이터 - 75%로 설정 */}
        <div className="flex justify-center mb-4">
          <ChargeIndicator 
            percentage={75} 
            isCharging={isCharging} 
          />
        </div>
        
        {/* 위치 지도 추가 */}
        <LocationMap currentLocation={currentLocation} />
      </main>
    </div>
  );
};

export default MainPage;
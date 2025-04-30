// src/pages/MainPage.tsx
import { useState } from 'react';
import robotImage from '../assets/robot.png'; 
import ChargeIndicator from '../components/charge';
import Header from '../components/common/Header';
import LocationMap from '../components/location/LocationMap';
import useScroll from '../hooks/useScroll';

const MainPage = () => {
  // useScroll 훅 사용 - 매개변수 없이 사용하면 내부에서 containerRef 생성
  const { containerRef } = useScroll();
  
  // 배터리 상태 (75%)
  const [isCharging] = useState(false);
  // 현재 위치 정보 상태
  const [currentLocation] = useState("현재 위치: 운동장 A구역");

  return (
    <div className="flex flex-col w-full h-full">
      {/* 헤더 */}
      <Header title="SSAFY 헬스장" />
      
      {/* containerRef를 main 요소에 연결 */}
      <main 
        className="flex-1 px-4 w-full max-w-screen-sm mx-auto overflow-y-auto pb-20"
        ref={containerRef}
      >
        {/* 콘텐츠를 div로 감싸 스크롤 가능하게 함 */}
        <div className="min-h-full">
          {/* PTSD 로봇 텍스트 - 왼쪽 정렬 */}
          <div className="text-left text-white text-lg font-medium mt-4 mb-4">
            PTSD 로봇
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
          
          {/* 위치 맵 컴포넌트 */}
          <LocationMap currentLocation={currentLocation} />
          
          {/* 스크롤 업 버튼 예시 (필요한 경우 사용) */}
          {/* <button 
            className="fixed bottom-20 right-4 bg-app-blue rounded-full p-2 z-10"
            onClick={scrollToTop}
          >
            <ArrowUp size={24} color="white" />
          </button> */}
        </div>
      </main>
    </div>
  );
};

export default MainPage;
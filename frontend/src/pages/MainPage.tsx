// src/pages/MainPage.tsx
import { useState } from 'react';
import robotImage from '../assets/robot.png'; 
import ChargeIndicator from '../components/charge';
import Header from '../components/common/Header';

const MainPage = () => {
  // 배터리 상태 (75%)로 변경
  const [isCharging] = useState(false);

  return (
    <div className="flex flex-col w-full h-full">
      {/* 헤더 */}
      <Header title="SSAFY 헬스장" />
      
      <main className="flex-1 px-4 w-full max-w-screen-sm mx-auto overflow-y-auto">
        {/* PTSD 로봇 텍스트 - 왼쪽 정렬 */}
        <div className="text-left text-white text-lg font-medium mt-4 mb-4">
          PTSD 로봇
        </div>
        
        {/* 로봇 이미지 */}
        <div className="flex justify-center items-center mb-6">
          <img src={robotImage} alt="IoT 로봇" className="w-48 h-48" />
        </div>
        
        {/* 배터리 인디케이터 - 75%로 설정 */}
        <div className="flex justify-center mb-20"> {/* 네비게이션 바에 가려지지 않도록 하단 마진 추가 */}
          <ChargeIndicator 
            percentage={75} 
            isCharging={isCharging} 
          />
        </div>
      </main>
    </div>
  );
};

export default MainPage;
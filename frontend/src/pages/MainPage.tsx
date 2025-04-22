import { useState } from 'react';
import robotImage from '../assets/robot.png';
import ChargeIndicator from '../components/charge';
import Header from '../components/common/Header';

const MainPage = () => {
  // 배터리 상태 하드코딩 (75%)
  const [isCharging] = useState(false);

  return (
    <div className="flex flex-col h-screen bg-app-dark text-white">
      <Header title="SSAFY 헬스장" />
      
      <main className="flex flex-col flex-grow items-center justify-center gap-8">
        {/* 로봇 이미지 */}
        <div className="flex justify-center items-center">
          <img src={robotImage} alt="IoT 로봇" className="w-32 h-32" />
        </div>
        
        {/* 배터리 인디케이터 - 75%로 하드코딩 */}
        <ChargeIndicator 
          percentage={75} 
          isCharging={isCharging} 
        />
      </main>
      
      <footer className="py-3 px-6 text-sm text-gray-400">
        PTSD 로봇
      </footer>
    </div>
  );
};

export default MainPage;
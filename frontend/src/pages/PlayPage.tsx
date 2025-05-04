// src/pages/PlayPage.tsx (Tailwind CSS 적용 버전)
import Header from '../components/common/Header';
import { Battery, MapPin, Clock, AlertCircle, Settings } from 'lucide-react';
import robot2Image from '../assets/robot2.png';

const PlayPage = () => {
  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white">
      <Header title="김싸피님의 PTSD" />

      <div className="flex-1 overflow-y-auto p-4 pb-24">
        {/* 로봇 카드 */}
        <div className="bg-app-card rounded-lg p-4 mb-4 shadow-md">
          <h2 className="text-xl font-bold text-center">PTSD</h2>
          <div className="flex justify-center items-center mt-1">
            <div className="w-2 h-2 rounded-full bg-green-500 mr-2" />
            <span className="text-sm text-green-500">연결됨</span>
          </div>
          <div className="flex justify-center my-2">
            <img src={robot2Image} alt="PTSD 로봇" className="h-28" />
          </div>
        </div>

        {/* 상태 카드 그리드 */}
        <div className="grid grid-cols-2 gap-4">
          {/* 배터리 */}
          <StatusCard icon={<Battery size={24} className="text-app-blue mb-1" />} title="82%" subtitle="예상 남은 시간" content="1시간 40분" />

          {/* 위치 */}
          <StatusCard icon={<MapPin size={24} className="text-app-blue mb-1" />} title="현재 위치" content="웨이트 존" />

          {/* 청소 상태 */}
          <StatusCard icon={<Settings size={24} className="text-app-blue mb-1" />} title="청소 중" subtitle="예상 종료 시간" content="오전 11:00" />

          {/* 예약 상태 */}
          <StatusCard icon={<Clock size={24} className="text-app-blue mb-1" />} title="예약 상태" content="매일 오전 9:00" />
        </div>

        {/* 알림 카드 */}
        <div className="bg-app-card rounded-lg p-4 mt-4 shadow-md">
          <div className="flex flex-col items-center">
            <AlertCircle size={24} className="text-app-warning mb-1" />
            <p className="text-xl font-bold my-1">알림</p>
            <p className="text-sm">10분 전 : 청소 완료</p>
          </div>
        </div>
      </div>
    </div>
  );
};

interface StatusCardProps {
  icon: React.ReactNode;
  title: string;
  subtitle?: string;
  content: string;
}

const StatusCard = ({ icon, title, subtitle, content }: StatusCardProps) => (
  <div className="bg-app-card rounded-lg p-4 shadow-md">
    <div className="flex flex-col items-center">
      {icon}
      <p className="text-xl font-bold text-white my-1">{title}</p>
      {subtitle && <p className="text-xs text-gray-400 my-0.5">{subtitle}</p>}
      <p className="text-sm text-white my-0.5">{content}</p>
    </div>
  </div>
);

export default PlayPage;

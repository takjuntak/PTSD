import { Battery, MapPin, Clock, AlertCircle, Settings } from 'lucide-react';
import ThreeRobot from '../components/status/ThreeRobot'; 

const PlayPage = () => {
  return (
    <div className="w-full h-full flex flex-col text-white">
      <div className="flex-1 overflow-y-auto p-4" style={{ paddingBottom: '150px' }}>
        {/* PTSD 연결됨 카드 */}
        <div
          className="bg-app-card rounded-lg p-4 mb-4 shadow-md flex justify-center items-center"
          style={{ backgroundColor: '#373738', height: 116 }}
        >
          <div className="flex items-center gap-20">
            <div className="flex flex-col justify-center items-center">
              <h2 className="text-xl font-bold mb-1">PTSD</h2>
              <div className="flex items-center">
                <div className="w-2 h-2 rounded-full bg-green-500 mr-2" />
                <span className="text-sm text-green-500">연결됨</span>
              </div>
            </div>

            {/* ✅ 3D 로봇 */}
            <div className="h-[100px] w-[100px] mt-2">
              <ThreeRobot scale={1} className="w-full h-full" />
            </div>

          </div>
        </div>

        {/* 상태 카드 */}
        <div className="grid grid-cols-2 gap-4">
          <StatusCard
            icon={<Battery size={24} className="text-app-blue" />}
            title="82%"
            subtitle="예상 남은 시간"
            content="1시간 40분"
          />
          <StatusCard
            icon={<MapPin size={24} className="text-app-blue" />}
            title="현재 위치"
            content="웨이트 존"
          />
          <StatusCard
            icon={<Settings size={24} className="text-app-blue" />}
            title="청소 중"
            subtitle="예상 종료 시간"
            content="오전 11:00"
          />
          <StatusCard
            icon={<Clock size={24} className="text-app-blue" />}
            title="예약 상태"
            content="매일 오전 9:00"
          />
        </div>

        {/* 알림 카드 */}
        <div
          className="bg-app-card rounded-lg p-4 mt-4 shadow-md flex items-center"
          style={{ backgroundColor: '#373738', height: 116 }}
        >
          <div className="mr-4">
            <AlertCircle size={24} className="text-app-warning" />
          </div>
          <div className="flex flex-col justify-center">
            <p className="text-xl font-bold mb-[4px] text-left">알림</p>
            <p className="text-sm text-white text-left">10분 전 : 청소 완료</p>
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
  <div
    className="bg-app-card rounded-lg p-4 shadow-md flex items-center"
    style={{ backgroundColor: '#373738', height: 116 }}
  >
    <div className="mr-3 self-start mt-5">{icon}</div>
    <div className="flex flex-col justify-center text-left">
      <p className="text-xl font-bold text-white mb-[4px]">{title}</p>
      {subtitle && <p className="text-xs text-gray-400 mb-[4px]">{subtitle}</p>}
      <p className="text-sm text-white">{content}</p>
    </div>
  </div>
);

export default PlayPage;

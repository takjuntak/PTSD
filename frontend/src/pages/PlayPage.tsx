import ThreeRobot from '../components/status/ThreeRobot';
import { useAuth } from '../hooks/useAuth';
import { useDevices } from '../hooks/useDevices';
import batteryImage from '../assets/play/battery.png';
import clockImage from '../assets/play/clock.png';
import noticeImage from '../assets/play/notice.png';

const PlayPage = () => {
  const { user } = useAuth();
  const { devices, connectedDevices } = useDevices();

  const currentDevice = connectedDevices.length > 0
    ? connectedDevices[0]
    : devices.length > 0
      ? devices[0]
      : null;

  return (
    <div className="w-full h-full flex flex-col text-white">
      <div className="flex-1 overflow-y-auto p-4" style={{ paddingBottom: '150px' }}>
        <div className="text-center text-white text-lg font-medium mt-4 mb-6 w-full">
          {user ? (
            currentDevice ? `${user.name}님의 ${currentDevice.name}` : (
              <>
                안녕하세요, <br /> {user.name}님!
              </>
            )
          ) : (
            '로그인이 필요합니다'
          )}
        </div>

        {/* 연결 카드 - 아이콘은 그대로, 텍스트는 조건부 변경 */}
        <StatusCard
          icon={
            <div className="w-16 h-16">
              <ThreeRobot scale={1} className="w-full h-full" />
            </div>
          }
          title="PTSD"
          content={
            currentDevice ? (
              <div className="flex items-center">
                <div className="w-2 h-2 rounded-full bg-green-500 mr-2" />
                <span className="text-sm text-green-500">연결됨</span>
              </div>
            ) : (
              <span className="text-sm text-white">기기를 먼저 등록해주세요</span>
            )
          }
        />

        {/* 상태 카드들 - 기기가 있을 때만 표시 */}
        {currentDevice && (
          <>
            <div className="flex flex-col gap-4 mt-4 mb-4">
              <StatusCard
                icon={
                  <img
                    src={batteryImage}
                    alt="배터리 아이콘"
                    className="w-16 h-16 object-contain"
                  />
                }
                title="82%"
                subtitle="예상 남은 시간"
                content="1시간 25분"
              />

              <StatusCard
                icon={
                  <img
                    src={clockImage}
                    alt="예약 상태 아이콘"
                    className="w-16 h-16 object-contain"
                  />
                }
                title="예약 상태"
                content="매일 오전 9:00"
              />
            </div>

            <StatusCard
              icon={
                <img
                  src={noticeImage}
                  alt="알림 아이콘"
                  className="w-12 h-12 object-contain"
                />
              }
              title="알림"
              content="10분 전 : 청소 완료"
            />
          </>
        )}
      </div>
    </div>
  );
};

interface StatusCardProps {
  icon: React.ReactNode;
  title: string;
  subtitle?: string;
  content: string | React.ReactNode;
}

const StatusCard = ({ icon, title, subtitle, content }: StatusCardProps) => (
  <div
    className="bg-app-card rounded-lg p-4 shadow-md flex items-center gap-4 w-full"
    style={{ backgroundColor: '#373738', height: 116 }}
  >
    <div className="w-14 h-14 flex-shrink-0 flex items-center justify-center">
      {icon}
    </div>
    <div className="flex flex-col justify-center text-left flex-1">
      <p className="text-xl font-bold text-white mb-[2px]">{title}</p>
      {subtitle && <p className="text-xs text-gray-400 mb-[2px]">{subtitle}</p>}
      <p className="text-sm text-white">{content}</p>
    </div>
  </div>
);

export default PlayPage;

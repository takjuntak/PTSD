// src/pages/PlayPage.tsx
import React from 'react';
import { useNavigate } from 'react-router-dom';
import ThreeRobot from '../components/status/ThreeRobot';
import { useAuth } from '../hooks/useAuth';
import { useDevices } from '../hooks/useDevices';
import useBatteryStatus from '../hooks/useBatteryStatus';
import useAlarms from '../hooks/useAlarms';
import { useRoutines } from '../hooks/useRoutines';
import batteryImage from '../assets/play/battery.png';
import clockImage from '../assets/play/clock.png';
import noticeImage from '../assets/play/notice.png';

const PlayPage = () => {
  const navigate = useNavigate();
  const { user } = useAuth();
  const { devices, connectedDevices } = useDevices();
  const { routines } = useRoutines();
  const { alarms } = useAlarms();

  const currentDevice = connectedDevices.length > 0
    ? connectedDevices[0]
    : devices.length > 0
      ? devices[0]
      : null;

  // 배터리 상태 가져오기
  const { battery } = useBatteryStatus(
    user?.userId ? user.userId : undefined
  );

  // 가장 최근 알림 가져오기
  const latestAlarm = alarms.length > 0 
    ? alarms.sort((a, b) => b.createdAt.getTime() - a.createdAt.getTime())[0] 
    : null;

  // 가장 최근 예약 가져오기
  const activeRoutine = routines.find(routine => routine.is_work);

  // 현재 디바이스가 있으면 항상 '연결됨'으로 표시
  const deviceStatus = currentDevice 
    ? { color: 'text-green-500', dotColor: 'bg-green-500', text: '연결됨' }
    : { color: 'text-gray-400', dotColor: 'bg-gray-400', text: '기기를 먼저 등록해주세요' };

  // 예약 정보 포맷팅
  const getFormattedScheduleTime = () => {
    if (!activeRoutine) return "예약된 시간이 없습니다";
    
    const startTime = new Date(activeRoutine.start_time);
    const hour = startTime.getHours();
    const minute = startTime.getMinutes();
    const ampm = hour >= 12 ? '오후' : '오전';
    const hour12 = hour % 12 || 12;
    const timeString = `${ampm} ${hour12}:${minute.toString().padStart(2, '0')}`;
    
    if (activeRoutine.routine_type === 'daily') {
      // repeat_days 배열 검사
      if (activeRoutine.repeat_days && activeRoutine.repeat_days.length > 0) {
        // 모든 요일이 선택된 경우
        if (activeRoutine.repeat_days.length === 7) {
          return `매일 ${timeString}`;
        } 
        // 일부 요일만 선택된 경우
        else {
          // 숫자 요일을 한글 요일로 변환 (API는 1이 월요일, 7이 일요일)
          const days = ['월', '화', '수', '목', '금', '토', '일'];
          const selectedDays = activeRoutine.repeat_days
            .map(day => {
              // day가 1부터 7까지의 값이므로 1을 빼서 배열 인덱스로 사용
              const dayIndex = (day % 7) - 1;
              return dayIndex >= 0 ? days[dayIndex] : days[6]; // 7은 일요일(인덱스 6)
            })
            .sort((a, b) => {
              // 요일 순서대로 정렬
              const order = { '월': 0, '화': 1, '수': 2, '목': 3, '금': 4, '토': 5, '일': 6 };
              return order[a as keyof typeof order] - order[b as keyof typeof order];
            })
            .join(', ');
          
          return `${selectedDays} ${timeString}`;
        }
      } else {
        return `매일 ${timeString}`; // repeat_days 정보가 없으면 매일로 표시
      }
    } else {
      // 단일 예약인 경우 날짜 포함
      const month = startTime.getMonth() + 1;
      const date = startTime.getDate();
      return `${month}/${date} ${timeString}`;
    }
  };

  // 알림 시간 포맷팅
  const getFormattedAlarmTime = () => {
    if (!latestAlarm) return "알림 없음";
    
    const now = new Date();
    const diffMin = Math.round((now.getTime() - latestAlarm.createdAt.getTime()) / (1000 * 60));
    
    if (diffMin < 60) {
      return `${diffMin}분 전 : ${latestAlarm.message}`;
    } else if (diffMin < 24 * 60) {
      const diffHours = Math.floor(diffMin / 60);
      return `${diffHours}시간 전 : ${latestAlarm.message}`;
    } else {
      const diffDays = Math.floor(diffMin / (24 * 60));
      return `${diffDays}일 전 : ${latestAlarm.message}`;
    }
  };
  
  // 네비게이션 핸들러
  const handleGoToAlarm = () => {
    navigate('/alarm');
  };
  
  const handleGoToSchedule = () => {
    navigate('/schedule');
  };

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

        {/* 연결 카드 */}
        <StatusCard
          icon={
            <div className="w-16 h-16">
              <ThreeRobot scale={1} className="w-full h-full" />
            </div>
          }
          title="PTSD"
          content={
            <div className="flex items-center">
              <div className={`w-2 h-2 rounded-full ${deviceStatus.dotColor} mr-2`} />
              <span className={`text-sm ${deviceStatus.color}`}>{deviceStatus.text}</span>
            </div>
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
                title={battery !== null ? `${battery}%` : "연결 중..."}
                content={
                  <div className="flex items-center">
                    <span>배터리 충전 상태</span>
                  </div>
                }
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
                content={getFormattedScheduleTime()}
                onClick={handleGoToSchedule}
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
              content={getFormattedAlarmTime()}
              onClick={handleGoToAlarm}
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
  onClick?: () => void;
}

const StatusCard = ({ icon, title, subtitle, content, onClick }: StatusCardProps) => (
  <div
    className="bg-app-card rounded-lg p-4 shadow-md flex items-center gap-4 w-full"
    style={{ 
      backgroundColor: '#373738', 
      height: 116,
      cursor: onClick ? 'pointer' : 'default'
    }}
    onClick={onClick}
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
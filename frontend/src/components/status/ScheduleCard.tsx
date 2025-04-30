// src/components/status/ScheduleCard.tsx
import { Clock } from 'lucide-react';

interface ScheduleCardProps {
  scheduleTime: string;
}

const ScheduleCard: React.FC<ScheduleCardProps> = ({ scheduleTime }) => {
  return (
    <div className="bg-app-card rounded-lg p-4">
      <div className="flex items-center">
        <Clock size={24} className="text-app-blue" />
        <span className="text-lg font-bold ml-2">예약 상태</span>
      </div>
      
      <div className="text-white text-left mt-2">
        <div>매일 오전 {scheduleTime}</div>
      </div>
    </div>
  );
};

export default ScheduleCard;
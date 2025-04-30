// src/components/status/BatteryCard.tsx
import { Battery } from 'lucide-react';

interface BatteryCardProps {
  percentage: number;
  remainingTime: string;
}

const BatteryCard: React.FC<BatteryCardProps> = ({ percentage, remainingTime }) => {
  // 배터리 색상 결정 (75% 이상: 초록, 20%-75%: 노랑, 20% 미만: 빨강)
  const getBatteryColor = () => {
    if (percentage >= 75) return 'text-app-green';
    if (percentage >= 20) return 'text-yellow-500';
    return 'text-app-warning';
  };

  return (
    <div className="bg-app-card rounded-lg p-4">
      <div className="flex items-center justify-between">
        <div className="flex items-center">
          <Battery 
            size={24} 
            className={getBatteryColor()}
            fill={percentage > 10 ? 'currentColor' : 'none'} 
          />
          <span className="text-2xl font-bold ml-2">{percentage}%</span>
        </div>
        
        <div className="text-gray-400 text-sm">
          예상 남은 시간
        </div>
      </div>
      
      <div className="text-white text-left mt-2">
        {remainingTime}
      </div>
    </div>
  );
};

export default BatteryCard;
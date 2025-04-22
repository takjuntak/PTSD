import { Battery } from 'lucide-react';

interface ChargeInfoProps {
  percentage?: number;
  isCharging?: boolean;
}

/**
 * 배터리 상태 정보를 텍스트로 표시하는 컴포넌트
 */
const ChargeInfo: React.FC<ChargeInfoProps> = ({ percentage = 75, isCharging = false }) => {
  return (
    <div className="flex items-center gap-2 mt-2">
      <Battery 
        size={24} 
        className="text-app-blue"
        fill={percentage > 10 ? 'currentColor' : 'none'} 
      />
      <span className="font-medium text-app-blue">
        1시간 25분 남았습니다
        {isCharging && ' ⚡'}
      </span>
    </div>
  );
};

export default ChargeInfo;
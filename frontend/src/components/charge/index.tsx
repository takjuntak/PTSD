import ChargeStatus from './ChargeStatus';
import ChargeInfo from './ChargeInfo';

interface ChargeIndicatorProps {
  percentage?: number; // 옵셔널로 변경
  isCharging?: boolean;
}

/**
 * 배터리 인디케이터 통합 컴포넌트
 */
const ChargeIndicator: React.FC<ChargeIndicatorProps> = ({ percentage = 75, isCharging = false }) => {
  return (
    <div className="flex flex-col items-center">
      <ChargeStatus percentage={percentage} />
      <ChargeInfo percentage={percentage} isCharging={isCharging} />
    </div>
  );
};

export default ChargeIndicator;
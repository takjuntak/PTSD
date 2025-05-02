import ChargeStatus from './ChargeStatus';

interface ChargeIndicatorProps {
  percentage?: number;
  isCharging?: boolean;
}

/**
 * 배터리 인디케이터 통합 컴포넌트
 * ChargeStatus만 사용하고 남은 시간 정보도 여기서 관리
 */
const ChargeIndicator: React.FC<ChargeIndicatorProps> = ({ percentage = 75, isCharging = false }) => {
  // 배터리 잔량에 따른 남은 시간 정보 (실제로는 더 복잡한 로직이 필요할 수 있음)
  const remainingTime = isCharging 
    ? "충전 중 ⚡" 
    : "1시간 25분 남았습니다";
  
  return (
    <div className="flex flex-col items-center">
      <ChargeStatus 
        percentage={percentage} 
        remainingTime={remainingTime}
      />
    </div>
  );
};

export default ChargeIndicator;
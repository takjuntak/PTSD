import React from 'react';

interface ChargeStatusProps {
  percentage?: number; // 0-100 사이의 배터리 충전 퍼센트
  size?: number; // 도넛 차트 크기
  strokeWidth?: number; // 도넛 두께
  remainingTime?: string; // 남은 시간 정보
}

/**
 * 배터리 충전 상태를 도넛 차트로 표시하는 컴포넌트
 * 12시 방향부터 시계 방향으로 충전 상태 표시
 * 중앙에 퍼센트와 남은 시간 표시
 */
const ChargeStatus: React.FC<ChargeStatusProps> = ({ 
  percentage = 75, // 기본값 75%로 설정
  size = 160, // 180에서 160으로 크기 조정
  strokeWidth = 16, // 18에서 16으로 약간 줄임
  remainingTime = "1시간 25분 남았습니다" // 기본 남은 시간
}) => {
  // SVG 관련 계산값들
  const radius = (size - strokeWidth) / 2;
  const circumference = 2 * Math.PI * radius;
  const strokeDashoffset = (1 - percentage / 100) * circumference;
  
  return (
    <div className="flex items-center justify-center">
      {/* 도넛 차트 - SVG 내에 텍스트도 포함 */}
      <svg width={size} height={size}>
        {/* 90도 회전해서 12시 방향부터 시작하도록 함 */}
        <g transform={`rotate(-90 ${size/2} ${size/2})`}>
          {/* 배경 원 - 회색 */}
          <circle
            cx={size / 2}
            cy={size / 2}
            r={radius}
            fill="transparent"
            stroke="#2A2A2A" // 직접 회색(app-card) 색상 코드 지정
            strokeWidth={strokeWidth}
          />
          
          {/* 배터리 레벨 원 - 파란색 */}
          <circle
            cx={size / 2}
            cy={size / 2}
            r={radius}
            fill="transparent"
            stroke="#0088FF" // 직접 파란색(app-blue) 색상 코드 지정
            strokeWidth={strokeWidth}
            strokeDasharray={circumference}
            strokeDashoffset={strokeDashoffset}
            strokeLinecap="round"
          />
        </g>
        
        {/* SVG 내에 텍스트 요소로 퍼센트 표시 - 회전의 영향을 받지 않음 */}
        <text
          x={size / 2}
          y={(size / 2) - 10} // 위로 약간 올려서 표시
          textAnchor="middle"
          dominantBaseline="middle"
          fill="white"
          fontSize="22px" // 24px에서 22px로 줄임
          fontWeight="bold"
        >
          {percentage}%
        </text>
        
        {/* 남은 시간 정보 표시 */}
        <text
          x={size / 2}
          y={(size / 2) + 15} // 아래로 약간 내려서 표시
          textAnchor="middle"
          dominantBaseline="middle"
          fill="#0088FF" // 파란색으로 표시
          fontSize="11px" // 12px에서 11px로 줄임
          fontWeight="medium"
        >
          {remainingTime}
        </text>
      </svg>
    </div>
  );
};

export default ChargeStatus;
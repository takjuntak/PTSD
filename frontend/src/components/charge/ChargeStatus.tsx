import React from 'react';

interface ChargeStatusProps {
  percentage?: number; // 0-100 사이의 배터리 충전 퍼센트
  size?: number; // 도넛 차트 크기
  strokeWidth?: number; // 도넛 두께
}

/**
 * 배터리 충전 상태를 도넛 차트로 표시하는 컴포넌트
 * 시계 방향으로 충전 상태 표시
 */
const ChargeStatus: React.FC<ChargeStatusProps> = ({ 
  percentage = 75, // 기본값 75%로 설정
  size = 180, 
  strokeWidth = 18 // 굵은 두께
}) => {
  // SVG 관련 계산값들
  const radius = (size - strokeWidth) / 2;
  const circumference = 2 * Math.PI * radius;
  const strokeDashoffset = (1 - percentage / 100) * circumference; // 시계 방향으로 변경
  
  return (
    <div className="flex items-center justify-center">
      {/* 도넛 차트 - SVG 내에 텍스트도 포함 */}
      <svg width={size} height={size}>
        <g className="transform -rotate-90">
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
            // 시계 방향으로 진행되도록 방향 역전
            transform="scale(-1, 1)"
            transform-origin="center"
          />
        </g>
        
        {/* SVG 내에 텍스트 요소로 퍼센트 표시 - 회전의 영향을 받지 않음 */}
        <text
          x={size / 2}
          y={size / 2}
          textAnchor="middle"
          dominantBaseline="middle"
          fill="white"
          fontSize="24px"
          fontWeight="bold"
        >
          {percentage}%
        </text>
      </svg>
    </div>
  );
};

export default ChargeStatus;
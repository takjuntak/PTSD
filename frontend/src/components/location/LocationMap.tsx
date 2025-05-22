// src/components/location/LocationMap.tsx
import React from 'react';
import mapImage from '../../assets/location/map2.svg';

interface LocationMapProps {
  currentLocation?: string;
}

/**
 * 위치 정보와 지도를 표시하는 컴포넌트
 */
const LocationMap: React.FC<LocationMapProps> = ({ 
  currentLocation = "현재 위치: 운동장 A구역" 
}) => {
  return (
    <div className="bg-gray-700 bg-opacity-30 rounded-lg p-4 mt-4 mb-16 w-full relative">
      {/* 현재 위치 정보 - 박스 좌측 상단에 절대 위치 */}
      <div className="absolute top-4 left-4 text-white text-base font-medium z-10">
        {currentLocation}
      </div>
      
      {/* 지도 이미지 */}
      <div className="w-full pt-8">
        <img 
          src={mapImage} 
          alt="위치 지도" 
          className="w-full h-auto rounded-md"
        />
      </div>
    </div>
  );
};

export default LocationMap;
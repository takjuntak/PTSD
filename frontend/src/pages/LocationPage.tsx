import { useAuth } from '../hooks/useAuth';
import { useDevices } from '../hooks/useDevices';
import { useNavigate } from 'react-router-dom';
import { useState } from 'react';

import mapImageDefault from '../../src/assets/location/map2.svg';
import mapImageTracking from '../../src/assets/map.svg';

import rocketIcon from '../assets/location/rocketicon.svg';
import mapIcon from '../assets/location/mapicon.svg';
import clockIcon from '../assets/location/clockicon.svg';
import upArrowIcon from '../assets/location/uparrow.svg';

const LocationPage = () => {
  const { user } = useAuth();
  const { devices, connectedDevices } = useDevices();
  const navigate = useNavigate();

  const [dropdownOpen, setDropdownOpen] = useState(false);
  const [isTracking, setIsTracking] = useState(false); // 위치 추적 상태
  const [selectedDevice, setSelectedDevice] = useState(
    connectedDevices.length > 0
      ? connectedDevices[0]
      : devices.length > 0
      ? devices[0]
      : null
  );

  const handleScheduleClick = () => {
    navigate('/schedule');
  };

  const handleSelectDevice = (device: typeof selectedDevice) => {
    setSelectedDevice(device);
    setDropdownOpen(false);
  };

  const handleTrackingClick = () => {
    setIsTracking(true);
  };

  return (
    <div className="relative w-full h-full bg-gradient-to-b from-[#2E2E37] to-[#1D1E23] text-white font-inter overflow-hidden">

      {/* 사용자 이름 + 디바이스 명 */}
      <div className="absolute top-[30px] left-1/2 transform -translate-x-1/2 text-[18px] font-extrabold text-white text-center z-10 whitespace-nowrap">
        {user && selectedDevice
          ? `${user.name}님의 ${selectedDevice.name}`
          : user && !selectedDevice
          ? `${user.name}님, 기기를 등록해주세요`
          : '로그인이 필요합니다'}
      </div>

      {/* 버튼 3개 */}
      <div className="absolute top-[80px] w-full flex justify-center gap-2">
        {/* 동작 시작 */}
        <div className="w-[110px] h-[46px] bg-[#373738] rounded-[10px] shadow-md flex flex-row items-center justify-center gap-2">
          <img src={rocketIcon} alt="동작" className="w-5 h-5" />
          <span className="text-[12px]">동작 시작</span>
        </div>

        {/* 위치 추적 */}
        <div
          className="w-[110px] h-[46px] bg-[#373738] rounded-[10px] shadow-md flex flex-row items-center justify-center gap-2 cursor-pointer"
          onClick={handleTrackingClick}
        >
          <img src={mapIcon} alt="위치" className="w-5 h-5" />
          <span className="text-[12px]">위치 추적</span>
        </div>

        {/* 예약 작업 */}
        <div
          className="w-[110px] h-[46px] bg-[#373738] rounded-[10px] shadow-md flex flex-row items-center justify-center gap-2 cursor-pointer"
          onClick={handleScheduleClick}
        >
          <img src={clockIcon} alt="예약" className="w-5 h-5" />
          <span className="text-[12px]">예약 작업</span>
        </div>
      </div>

      {/* 지도 이미지 영역 */}
      <div className="absolute top-[130px] left-1/2 transform -translate-x-1/2 w-[300px] h-[300px]">
        <img
          src={isTracking ? mapImageTracking : mapImageDefault}
          alt="3D 지도"
          className="w-full h-full object-contain"
        />
      </div>

      {/* 모든 기기 버튼 */}
      <div
        className="absolute top-[450px] left-[20px] w-[155px] h-[40px] bg-[#373738] rounded-[10px] shadow-md flex items-center justify-center gap-2 cursor-pointer"
        onClick={() => setDropdownOpen((prev) => !prev)}
      >
        <span className="text-[15px] font-bold font-montserrat">모든 기기</span>
        <img
          src={upArrowIcon}
          alt="기기 목록 열기"
          className={`w-[14px] h-[14px] transition-transform ${dropdownOpen ? 'rotate-180' : ''}`}
        />
      </div>

      {/* 드롭다운 기기 목록 */}
      {dropdownOpen && (
        <div className="absolute top-[500px] left-[20px] bg-[#373738] rounded-[10px] shadow-md z-50 min-w-[180px]">
          {devices.map((device) => (
            <div
              key={device.device_id}
              onClick={() => handleSelectDevice(device)}
              className={`px-4 py-2 text-sm cursor-pointer flex justify-between items-center hover:bg-[#4A4A4A] ${
                selectedDevice?.device_id === device.device_id ? 'text-[#0098FF] font-bold' : 'text-white'
              }`}
            >
              {device.name}
              {selectedDevice?.device_id === device.device_id && (
                <span className="text-[#0098FF] text-xs">선택됨</span>
              )}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default LocationPage;

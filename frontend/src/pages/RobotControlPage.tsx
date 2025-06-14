// src/pages/RobotControlPage.tsx
import { ChevronLeft, ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Power } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import mapImage from '../assets/map.svg';
import React, { useEffect } from 'react';
import { useRobotWebSocket } from '../hooks/useRobotWebSocket';
import { useRobotArmControl } from '../hooks/useRobotArmControl'; // 로봇 팔 제어 훅 추가
import { useDevices } from '../hooks/useDevices';

import pickUpImage from '../assets/control/pickup.svg'
import dropImage from '../assets/control/drop.svg'
import upImage from '../assets/control/up.svg'
import downImage from '../assets/control/down.svg'

const RobotControlPage = () => {
  const navigate = useNavigate();
  const { connectedDevices } = useDevices();
  
  // 현재 선택된 디바이스 ID
  const deviceId = connectedDevices.length > 0 ? connectedDevices[0].device_id : null;
  
  // 이동 제어를 위한 웹소켓 훅
  const { isConnected, status, lastResponse, sendCommand } = useRobotWebSocket(deviceId);
  
  // 로봇 팔 제어를 위한 웹소켓 훅
  const { sendSwitchCommand, sendUpDownCommand } = useRobotArmControl(deviceId);

  // 명령 전송 함수
  const handleCommand = (command: string) => {
    // 로봇 팔 관련 명령 처리
    if (command === 'PICK' || command === 'DROP') {
      sendSwitchCommand();
      return;
    }
    
    if (command === 'UP' || command === 'DOWN') {
      sendUpDownCommand();
      return;
    }
    
    // 이동 관련 명령 처리 (기존 코드 유지)
    sendCommand(command);
  };

  // 네트워크 응답 콘솔로 출력 (UI에는 표시하지 않음)
  useEffect(() => {
    if (lastResponse) {
      console.log('서버 응답:', lastResponse);
    }
  }, [lastResponse]);

  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white" style={{ backgroundColor: '#2E2E37' }}>
      <header className="p-4 px-3 bg-app-dark flex items-center gap-3 border-b border-neutral-700 top-0 z-10" style={{ backgroundColor: '#2E2E37' }} onClick={() => navigate(-1)} >
        <button 
          className="text-white border-none bg-transparent p-0"
          style={{ background: 'transparent' }}
        >
          <ChevronLeft size={24} />
        </button>
        <span className="text-xl font-bold text-[#767676]">제어</span>
      </header>

      <main className="flex-1 overflow-y-auto p-4 pb-20 flex flex-col items-center">
        {/* 상태 표시 */}
        <div className="w-full bg-app-card rounded-lg p-3 flex items-center justify-between mb-3">
          <div className="flex items-center">
            <div className={`w-3 h-3 rounded-full mr-2 ${isConnected ? 'bg-green-500' : 'bg-red-500'}`}></div>
            <span className="text-sm">상태: {status}</span>
          </div>
          <div className="text-xs text-gray-400">
            {deviceId ? `로봇 ID: ${deviceId}` : '디바이스 연결 필요'}
          </div>
        </div>
        
        <div className="w-full h-[200px] bg-app-card rounded-xl mb-3 relative">
          <div className="w-full h-full overflow-auto p-2 box-border">
            <img src={mapImage} alt="헬스장 맵" className="w-full max-h-full object-contain block" />
          </div>
          <div className="absolute bottom-3 right-3 bg-black/70 px-3 py-1.5 rounded-full flex items-center text-xs">
            <div className="w-2.5 h-2.5 rounded-full bg-cyan-400 mr-1.5" />
            <span>로봇 위치</span>
          </div>
        </div>

        {/* PICK/DROP/UP/DOWN 버튼 */}
        <div className="w-full mx-auto mt-4 mb-4 flex justify-between">
          {[
            { label: 'PICK-UP', img: pickUpImage, command: 'PICK' },
            { label: 'DROP', img: dropImage, command: 'DROP' },
            { label: 'UP', img: upImage, command: 'UP' },
            { label: 'DOWN', img: downImage, command: 'DOWN' },
          ].map(({ label, img, command }) => (
            <div
              key={command}
              onClick={() => handleCommand(command)}
              className="w-[82px] h-[97px] bg-[#272934] border border-[#31333D] rounded-[10px] flex flex-col items-center justify-center cursor-pointer"
            >
              <img src={img} alt={label} className="w-[60px] h-[56px]" />
              <span className="mt-0 text-white text-[14px] font-semibold leading-[19px] text-center">
                {label}
              </span>
            </div>
          ))}
        </div>

        {/* 컨트롤 버튼 */}
        <div className="grid grid-rows-3 grid-cols-3 gap-3 mt-3 place-items-center">
          <div className="col-start-2">
            <ControlBtn onClick={() => handleCommand('W')} icon={<ArrowUp size={40} />} />
          </div>
          <div className="row-start-2 col-start-1">
            <ControlBtn onClick={() => handleCommand('A')} icon={<ArrowLeft size={40} />} />
          </div>
          <div className="row-start-2 col-start-2">
            <button
              onClick={() => handleCommand('S')}
              className="bg-[#1A1A1A] border-2 border-cyan-400 rounded-full w-[70px] h-[70px] flex items-center justify-center shadow-[0_0_10px_rgba(0,207,253,0.6)]"
            >
              <Power size={26} color="#00CFFD" />
            </button>
          </div>
          <div className="row-start-2 col-start-3">
            <ControlBtn onClick={() => handleCommand('D')} icon={<ArrowRight size={40} />} />
          </div>
          <div className="row-start-3 col-start-2">
            <ControlBtn onClick={() => handleCommand('X')} icon={<ArrowDown size={40} />} />
          </div>
        </div>
      </main>
    </div>
  );
};

const ControlBtn = ({ onClick, icon }: { onClick: () => void; icon: React.ReactNode }) => (
  <button
    onClick={onClick}
    className="bg-[#1A1A1A] border-2 border-cyan-400 rounded-md w-[76px] h-[76px] flex items-center justify-center text-cyan-400"
  >
    {icon}
  </button>
);

export default RobotControlPage;
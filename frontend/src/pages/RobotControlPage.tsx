// src/pages/RobotControlPage.tsx
import { ChevronLeft, ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Power } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import mapImage from '../assets/map.png';
import React from 'react';

const RobotControlPage = () => {
  const navigate = useNavigate();
  const log = (dir: string) => () => console.log(dir);

  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white">
      <header className="p-4 px-6 bg-app-dark flex items-center gap-3 border-b border-neutral-700 sticky top-0 z-10">
        <button onClick={() => navigate(-1)} className="p-1 text-white hover:opacity-80">
          <ChevronLeft size={24} />
        </button>
        <span className="text-xl font-bold">수동 조작</span>
      </header>

      <main className="flex-1 overflow-y-auto p-4 pb-20 flex flex-col items-center">
        <div className="w-full h-[200px] bg-app-card rounded-xl mt-4 mb-3 relative">
          <div className="w-full h-full overflow-auto p-2 box-border">
            <img src={mapImage} alt="헬스장 맵" className="w-full max-h-full object-contain block" />
          </div>
          <div className="absolute bottom-3 right-3 bg-black/70 px-3 py-1.5 rounded-full flex items-center text-xs">
            <div className="w-2.5 h-2.5 rounded-full bg-cyan-400 mr-1.5" />
            <span>로봇 위치</span>
          </div>
        </div>

        <div className="grid grid-rows-3 grid-cols-3 gap-1.5 mt-3 place-items-center">
          <div className="col-start-2">
            <ControlBtn onClick={log('Up')} icon={<ArrowUp size={20} />} />
          </div>
          <div className="row-start-2 col-start-1">
            <ControlBtn onClick={log('Left')} icon={<ArrowLeft size={20} />} />
          </div>
          <div className="row-start-2 col-start-2">
            <button
              onClick={log('Power')}
              className="bg-[#1A1A1A] border-2 border-cyan-400 rounded-full w-[52px] h-[52px] flex items-center justify-center shadow-[0_0_10px_rgba(0,207,253,0.6)]"
            >
              <Power size={26} color="#00CFFD" />
            </button>
          </div>
          <div className="row-start-2 col-start-3">
            <ControlBtn onClick={log('Right')} icon={<ArrowRight size={20} />} />
          </div>
          <div className="row-start-3 col-start-2">
            <ControlBtn onClick={log('Down')} icon={<ArrowDown size={20} />} />
          </div>
        </div>
      </main>
    </div>
  );
};

const ControlBtn = ({ onClick, icon }: { onClick: () => void; icon: React.ReactNode }) => (
  <button
    onClick={onClick}
    className="bg-[#1A1A1A] border-2 border-cyan-400 rounded-md w-[38px] h-[38px] flex items-center justify-center text-cyan-400"
  >
    {icon}
  </button>
);

export default RobotControlPage;
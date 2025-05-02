// src/pages/ControlPage.tsx - 수동조작 버튼 기능 추가
import { Play, Home, Calendar, Gamepad2, ChevronLeft } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import robotImage from '../assets/robot.png';

const ControlPage = () => {
  const navigate = useNavigate();

  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white">
      <header className="p-4 px-6 bg-app-dark flex items-center gap-3 border-b border-neutral-700 sticky top-0 z-10">
        <button onClick={() => navigate(-1)} className="p-1 text-white hover:opacity-80">
          <ChevronLeft size={24} />
        </button>
        <span className="text-xl font-bold">제어</span>
      </header>

      <main className="flex-1 px-4 pb-32 overflow-y-auto flex flex-col items-center">
        <img src={robotImage} alt="IoT 로봇" className="w-44 h-44 my-5" />

        <div className="grid grid-cols-2 gap-4 w-full max-w-sm">
          {[
            { icon: <Play size={24} color="white" />, label: '동작 제어', color: 'bg-app-blue' },
            { icon: <Home size={24} color="white" />, label: '로봇 복귀', color: 'bg-cyan-300' },
            { icon: <Calendar size={24} color="white" />, label: '루틴 예약', color: 'bg-red-400' },
            { icon: <Gamepad2 size={24} color="white" />, label: '수동 조작', color: 'bg-violet-400', route: '/robot-control' },
          ].map(({ icon, label, color, route }, i) => (
            <button
              key={i}
              onClick={() => route && navigate(route)}
              className="bg-app-card rounded-lg p-6 flex flex-col items-center justify-center"
            >
              <div className={`w-12 h-12 rounded-full ${color} flex items-center justify-center mb-3`}>
                {icon}
              </div>
              <span className="text-sm text-white">{label}</span>
            </button>
          ))}
        </div>
      </main>
    </div>
  );
};

export default ControlPage;
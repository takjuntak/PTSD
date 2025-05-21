import { useNavigate } from 'react-router-dom';
import { ChevronLeft } from 'lucide-react';
import timerImage from '../../assets/schedule/timer.svg';
import { useRef, useState, useEffect } from 'react';
import TimerDeleteModal from './TimerDeleteModal';

const TimerSelectPage = () => {
  const navigate = useNavigate();
  const centerRef = useRef<HTMLDivElement>(null);
  const isDragging = useRef(false);

  const [selectedHour, setSelectedHour] = useState(3);
  const hours = Array.from({ length: 12 }, (_, i) => i + 1);
  const radius = 140;

  const [currentTime, setCurrentTime] = useState('');
  const [selectedTime, setSelectedTime] = useState('');
  const [selectedDateText, setSelectedDateText] = useState('오늘');
  const [confirmed, setConfirmed] = useState(false);
  const [showCancelModal, setShowCancelModal] = useState(false);

  useEffect(() => {
    const updateTime = () => {
      const now = new Date();

      setCurrentTime(
        now.toLocaleTimeString('ko-KR', {
          hour: 'numeric',
          minute: '2-digit',
          hour12: true,
        })
      );

      const end = new Date(now.getTime() + selectedHour * 60 * 60 * 1000);
      setSelectedTime(
        end.toLocaleTimeString('ko-KR', {
          hour: 'numeric',
          minute: '2-digit',
          hour12: true,
        })
      );

      const isTomorrow = now.getDate() !== end.getDate();
      setSelectedDateText(isTomorrow ? '내일' : '오늘');
    };

    updateTime();
    const interval = setInterval(updateTime, 10000);
    return () => clearInterval(interval);
  }, [selectedHour]);

  const handlePointerDown = () => {
    isDragging.current = true;
  };

  const handlePointerUp = () => {
    isDragging.current = false;
  };

  const handlePointerMove = (e: React.PointerEvent) => {
    if (confirmed || !isDragging.current || !centerRef.current) return;

    const rect = centerRef.current.getBoundingClientRect();
    const cx = rect.left + rect.width / 2;
    const cy = rect.top + rect.height / 2;
    const x = e.clientX - cx;
    const y = e.clientY - cy;

    const angleDeg = (Math.atan2(y, x) * 180) / Math.PI;
    const angleFrom3 = (angleDeg + 360) % 360;
    const hour = Math.round(angleFrom3 / 30) + 3;
    const finalHour = hour > 12 ? hour - 12 : hour;

    setSelectedHour(finalHour);
  };

  return (
    <div className="w-full h-full flex justify-center bg-gradient-to-b from-[#2E2E37] to-[#1D1E23] relative">
      <div className={`w-full max-w-[412px] h-full px-4 ${confirmed ? 'overflow-y-auto' : 'overflow-hidden'}`}>
        <header className="p-4 px-0 flex items-center gap-3 sticky top-0 z-10" onClick={() => navigate(-1)}>
          <button className="border-none bg-transparent p-0">
            <ChevronLeft size={24} color="#FFFFFF" />
          </button>
          <span className="text-xl font-bold text-[#767676] font-inter">예약</span>
        </header>

        <div className="mt-2 text-[20px] font-bold text-center text-white whitespace-nowrap">
          원하는 청소 종료 시간을 설정해 주세요.
        </div>
        <div className="mt-2 text-[14px] text-white text-center">
          설정 가능 시간 : 1시간 ~ 12시간
        </div>

        <div
          className="mt-14 flex justify-center relative w-[250px] h-[250px] mx-auto touch-none"
          ref={centerRef}
          onPointerDown={handlePointerDown}
          onPointerUp={handlePointerUp}
          onPointerMove={handlePointerMove}
        >
          <img src={timerImage} alt="타이머" className="w-full h-full absolute" />

          {hours.map((hour) => {
            const angle = ((hour - 3) * 30) * (Math.PI / 180);
            const x = radius * Math.cos(angle);
            const y = radius * Math.sin(angle);
            return (
              <div
                key={hour}
                className={`absolute w-[25px] h-[25px] text-[14px] font-bold flex items-center justify-center ${
                  hour === selectedHour ? 'text-[#617BEE]' : 'text-white'
                }`}
                style={{
                  top: `calc(50% + ${y}px - 12.5px)`,
                  left: `calc(50% + ${x}px - 12.5px)`,
                }}
              >
                {hour}
              </div>
            );
          })}

          {(() => {
            const angle = ((selectedHour - 3) * 30) * (Math.PI / 180);
            const x = radius * Math.cos(angle);
            const y = radius * Math.sin(angle);
            return (
              <div
                className="absolute w-[35px] h-[35px] rounded-full bg-[#617BEE]/70 z-10"
                style={{
                  top: `calc(50% + ${y}px - 17.5px)`,
                  left: `calc(50% + ${x}px - 17.5px)`,
                }}
              />
            );
          })()}

          <div className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 text-white text-center leading-tight z-20">
            <div className="text-[14px] font-semibold">종료 시각</div>
            <div className="flex justify-center items-baseline gap-1 mt-1">
              <span className="text-[14px] whitespace-nowrap">{selectedDateText}</span>
              <span className="text-[40px] font-extrabold whitespace-nowrap">{selectedTime.split(' ')[1]}</span>
              <span className="text-[14px] font-bold whitespace-nowrap">{selectedTime.split(' ')[0]}</span>
            </div>
            <div className="text-[14px] mt-2">현재 시각</div>
            <div className="text-[14px] whitespace-nowrap">{currentTime}</div>
          </div>
        </div>

        {!confirmed ? (
          <div className="mt-16 flex justify-center">
            <button
              onClick={() => setConfirmed(true)}
              className="w-[316px] h-[46px] rounded-[10px] bg-[#617BEE] shadow-md text-white text-[18px] font-bold flex justify-center items-center"
            >
              확인
            </button>
          </div>
        ) : (
          <div className="mt-14 text-white text-center leading-tight pb-20">
            <div className="text-[14px] font-semibold mb-3">현재 예약된 청소 종료 시각</div>
            <div className="flex justify-center items-baseline gap-1 mb-4">
              <span className="text-[14px]">{selectedDateText}</span>
              <span className="text-[40px] font-extrabold">{selectedTime.split(' ')[1]}</span>
              <span className="text-[14px] font-bold">{selectedTime.split(' ')[0]}</span>
            </div>

            <div className="flex justify-center">
              <button
                onClick={() => setShowCancelModal(true)}
                className="w-[316px] h-[46px] bg-white text-[#617BEE] text-[18px] font-bold rounded-[10px] border border-[#617BEE] shadow-md"
              >
                예약 취소
              </button>
            </div>
          </div>
        )}
      </div>

      <TimerDeleteModal
        open={showCancelModal}
        onClose={() => setShowCancelModal(false)}
        onConfirm={() => {
          setConfirmed(false);
          setShowCancelModal(false);
        }}
      />
    </div>
  );
};

export default TimerSelectPage;
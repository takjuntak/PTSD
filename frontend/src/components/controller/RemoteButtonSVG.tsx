import React from 'react';

interface RemoteButtonSVGProps {
  onUp: () => void;
  onDown: () => void;
  onLeft: () => void;
  onRight: () => void;
  onPower: () => void;
}

const RemoteButtonSVG: React.FC<RemoteButtonSVGProps> = ({
  onUp,
  onDown,
  onLeft,
  onRight,
  onPower
}) => {
  return (
    <div className="relative w-[140px] h-[140px]">
      {/* 전원 버튼 */}
      <button
        onClick={onPower}
        className="absolute top-1/2 left-1/2 -translate-x-1/2 -translate-y-1/2 w-[52px] h-[52px] rounded-full border-2 border-cyan-400 bg-[#1A1A1A] shadow-[0_0_10px_rgba(0,207,253,0.6)] flex items-center justify-center"
      >
        <svg width="24" height="24" viewBox="0 0 24 24" fill="#00CFFD" xmlns="http://www.w3.org/2000/svg">
          <path d="M12 2V12" stroke="#00CFFD" strokeWidth="2" strokeLinecap="round" />
          <path
            d="M16.24 7.76C17.5978 9.11775 18.2825 10.9951 18.1215 12.9046C17.9605 14.8142 16.9658 16.5823 15.364 17.7829C13.7622 18.9835 11.7057 19.4971 9.71391 19.2156C7.72215 18.9341 5.94957 17.8787 4.75736 16.2426C3.56516 14.6065 3.05359 12.5624 3.33836 10.5706C3.62313 8.57879 4.68488 6.80321 6.23224 5.51573"
            stroke="#00CFFD"
            strokeWidth="2"
            strokeLinecap="round"
          />
        </svg>
      </button>

      {/* Up */}
      <button onClick={onUp} className="absolute top-0 left-1/2 -translate-x-1/2 w-[36px] h-[36px]">
        <svg viewBox="0 0 100 100" fill="#00CFFD">
          <polygon points="50,10 90,90 10,90" />
        </svg>
      </button>

      {/* Down */}
      <button onClick={onDown} className="absolute bottom-0 left-1/2 -translate-x-1/2 w-[36px] h-[36px] rotate-180">
        <svg viewBox="0 0 100 100" fill="#00CFFD">
          <polygon points="50,10 90,90 10,90" />
        </svg>
      </button>

      {/* Left */}
      <button onClick={onLeft} className="absolute left-0 top-1/2 -translate-y-1/2 rotate-[-90deg] w-[36px] h-[36px]">
        <svg viewBox="0 0 100 100" fill="#00CFFD">
          <polygon points="50,10 90,90 10,90" />
        </svg>
      </button>

      {/* Right */}
      <button onClick={onRight} className="absolute right-0 top-1/2 -translate-y-1/2 rotate-[90deg] w-[36px] h-[36px]">
        <svg viewBox="0 0 100 100" fill="#00CFFD">
          <polygon points="50,10 90,90 10,90" />
        </svg>
      </button>
    </div>
  );
};

export default RemoteButtonSVG;

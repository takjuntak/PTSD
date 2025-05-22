// props로 onClose 받기
import React from 'react';
import settingIcon from '../../assets/header/setting.svg';
import checkIcon from '../../assets/header/check.svg';
import { useNavigate } from 'react-router-dom';

interface Props {
  onClose: () => void;
}

const LocationDropdown: React.FC<Props> = ({ onClose }) => {

    const navigate = useNavigate();

  return (
    // 바깥 오버레이
    <div
      onClick={onClose}
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        zIndex: 998,
      }}
    >
      {/* 드롭다운 내부 - 클릭 이벤트 전파 차단 */}
      <div
        onClick={(e) => e.stopPropagation()}
        style={{
          position: 'absolute',
          top: 60,
          left: 18,
          width: 300,
          background: '#373738',
          borderRadius: 10,
          padding: '16px 20px',
          color: 'white',
          boxShadow: '2px 2px 2px rgba(0, 0, 0, 0.25)',
        }}
      >
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
          <span style={{ fontWeight: 800, fontSize: 15, color: '#0098FF' }}>PTSD</span>
          <img src={checkIcon} alt="선택됨" width={24} height={24} />
        </div>

        <hr
          style={{
            border: 'none',
            borderTop: '2px dashed #767676',
            margin: '12px 0',
          }}
        />

        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <img src={settingIcon} alt="설정" width={24} height={24} />
          <span
            style={{ fontWeight: 800, fontSize: 15, cursor: 'pointer' }}
            onClick={() => {
              navigate('/device-connect');
              onClose(); 
            }}
          >
            기기 관리
          </span>

        </div>
      </div>
    </div>
  );
};

export default LocationDropdown;

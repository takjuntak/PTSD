import React, { useState } from 'react';
import plusImage from '../../assets/header/header-plus.svg';
import bellImage from '../../assets/header/header-bell.svg';
import unreadbellImage from '../../assets/header/header-bell-unread.svg';
import menuImage from '../../assets/header/header-more.svg';
import LocationDropdown from './LocationDropdown';

const AppHeader: React.FC = () => {
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const [hasUnread, setHasUnread] = useState(true); // 읽지 않은 알림 여부

  return (
    <div
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        backgroundColor: '#2E2E37',
        height: 64,
        display: 'flex',
        justifyContent: 'center',
        zIndex: 999,
      }}
    >
      <div
        style={{
          width: '100%',
          maxWidth: 416,
          padding: '0 16px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          position: 'relative',
        }}
      >
        {/* ▼ [기기이름] 버튼 */}
        <button
          onClick={() => setDropdownOpen(!dropdownOpen)}
          style={{
            background: 'none',
            border: 'none',
            color: '#FFFFFF',
            fontSize: 20,
            fontWeight: 800,
            fontFamily: 'Inter',
          }}
        >
          Gymbo ▼
        </button>

        {/* 아이콘 */}
        <div style={{ display: 'flex', gap: 20 }}>
          <img src={plusImage} alt="추가" width={24} height={24} />

          {/* 알림 아이콘 (이미지에 파란 점 포함) */}
          <div style={{ position: 'relative' }}>
            <img
              src={hasUnread ? unreadbellImage : bellImage}
              alt="알림"
              width={24}
              height={24}
              onClick={() => setHasUnread(false)}
              style={{ cursor: 'pointer' }}
            />
          </div>

          <img src={menuImage} alt="더보기" width={24} height={24} />
        </div>

        {/* 드롭다운 */}
        {dropdownOpen && (
          <LocationDropdown onClose={() => setDropdownOpen(false)} />
        )}
      </div>
    </div>
  );
};

export default AppHeader;

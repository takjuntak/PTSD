// src/components/common/NavigationBar.tsx
import { useNavigate, useLocation } from 'react-router-dom';
import React, { useEffect, useState } from 'react';

import homeActive from '../../assets/navigation/home-activate.svg';
import homeInactive from '../../assets/navigation/home-deactivate.svg';
import controlActive from '../../assets/navigation/control-activate.svg';
import controlInactive from '../../assets/navigation/control-deactivate.svg';
import scheduleActive from '../../assets/navigation/schedule-activate.svg';
import scheduleInactive from '../../assets/navigation/schedule-deactivate.svg';
import menuActive from '../../assets/navigation/menu-activate.svg';
import menuInactive from '../../assets/navigation/menu-deactivate.svg';
import playButtonIcon from '../../assets/navigation/play.svg';

const NavigationBar: React.FC = () => {
  const navigate = useNavigate();
  const location = useLocation();
  const currentPath = location.pathname;
  const [windowWidth, setWindowWidth] = useState(window.innerWidth);

  // 기준 너비
  const BASE_WIDTH = 416;

  // 창 크기 변경 감지
  useEffect(() => {
    const handleResize = () => {
      setWindowWidth(window.innerWidth);
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  // 실제 컨테이너 너비 계산 (최대 BASE_WIDTH)
  const containerWidth = Math.min(windowWidth, BASE_WIDTH);

  // 각 아이템 간 간격 비율 조정 (창 크기에 따라)
  const widthRatio = containerWidth / BASE_WIDTH;

  const isActive = (path: string) => currentPath === path;

  const navItems = [
    { path: '/', label: '홈', activeIcon: homeActive, inactiveIcon: homeInactive },
    { path: '/control', label: '제어', activeIcon: controlActive, inactiveIcon: controlInactive },
    { path: '/schedule', label: '예약', activeIcon: scheduleActive, inactiveIcon: scheduleInactive },
    { path: '/menu', label: '메뉴', activeIcon: menuActive, inactiveIcon: menuInactive },
  ];

  return (
    <div
      style={{
        position: 'fixed',
        bottom: 0,
        left: 0,
        right: 0,
        backgroundColor: '#32333B',
        height: 74,
        zIndex: 100,
        display: 'flex',
        justifyContent: 'center',
      }}
    >
      <div
        style={{
          width: '100%',
          maxWidth: BASE_WIDTH,
          height: 74,
          position: 'relative',
        }}
      >
        <div
          style={{
            width: '100%',
            height: '100%',
            display: 'flex',
            alignItems: 'center',
          }}
        >
          {/* 홈 */}
          <div style={{ marginLeft: 16 * widthRatio }}>
            <NavItem {...navItems[0]} isActive={isActive(navItems[0].path)} navigate={navigate} />
          </div>

          {/* 제어 */}
          <div style={{ marginLeft: 26 * widthRatio }}>
            <NavItem {...navItems[1]} isActive={isActive(navItems[1].path)} navigate={navigate} />
          </div>

          {/* 중앙 플레이 버튼 */}
          <div
            style={{
              position: 'absolute',
              bottom: 17,
              left: '50%',
              transform: 'translateX(-50%)',
              width: 80 * (windowWidth < 350 ? 0.8 : 1), // 작은 화면에서는 버튼 크기 조정
              height: 80 * (windowWidth < 350 ? 0.8 : 1),
              zIndex: 10,
            }}
          >
            <button
              onClick={() => navigate('/play')}
              style={{
                width: '100%',
                height: '100%',
                background: 'none',
                border: 'none',
                padding: 0,
              }}
            >
              <img src={playButtonIcon} alt="플레이" style={{ width: '100%', height: '100%' }} />
            </button>
          </div>

          {/* 예약 */}
          <div style={{ 
            marginLeft: 'auto', 
            marginRight: 26 * widthRatio,
            paddingLeft: 40 * widthRatio // 플레이 버튼을 위한 공간 확보
          }}>
            <NavItem {...navItems[2]} isActive={isActive(navItems[2].path)} navigate={navigate} />
          </div>

          {/* 메뉴 */}
          <div style={{ marginRight: 16 * widthRatio }}>
            <NavItem {...navItems[3]} isActive={isActive(navItems[3].path)} navigate={navigate} />
          </div>
        </div>
      </div>
    </div>
  );
};

interface NavItemProps {
  path: string;
  label: string;
  activeIcon: string;
  inactiveIcon: string;
  isActive: boolean;
  navigate: ReturnType<typeof useNavigate>;
}

const NavItem: React.FC<NavItemProps> = ({ path, label, activeIcon, inactiveIcon, isActive, navigate }) => {
  // 화면 크기에 따라 아이콘과 텍스트 크기 조정
  const [windowWidth, setWindowWidth] = useState(window.innerWidth);
  
  useEffect(() => {
    const handleResize = () => {
      setWindowWidth(window.innerWidth);
    };
    
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);
  
  // 작은 화면에서 아이콘/텍스트 크기 조정
  const scale = windowWidth < 350 ? 0.85 : 1;
  
  return (
    <button
      onClick={() => navigate(path)}
      style={{
        background: 'none',
        border: 'none',
        width: 50 * scale,
        height: 46 * scale,
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        gap: 4 * scale,
        padding: 0,
      }}
    >
      <img
        src={isActive ? activeIcon : inactiveIcon}
        alt={label}
        style={{ width: 24 * scale, height: 24 * scale }}
      />
      <span
        style={{
          fontSize: 14 * scale,
          fontWeight: 800,
          color: isActive ? '#66A1F7' : '#767676',
          fontFamily: 'Inter',
          lineHeight: '17px',
          whiteSpace: 'nowrap',
        }}
      >
        {label}
      </span>
    </button>
  );
};

export default NavigationBar;
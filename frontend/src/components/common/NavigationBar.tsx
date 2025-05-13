import { useNavigate, useLocation } from 'react-router-dom';
import React from 'react';

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
        height: 60,
        zIndex: 100,
      }}
    >
      <div style={{ position: 'relative', width: '100%', height: '100%' }}>
        {/* 홈 */}
        <div style={{ position: 'absolute', left: '3.85%', top: '50%', transform: 'translateY(-50%)' }}>
          <NavItem {...navItems[0]} isActive={isActive(navItems[0].path)} navigate={navigate} />
        </div>

        {/* 제어 */}
        <div style={{ position: 'absolute', left: '22.12%', top: '50%', transform: 'translateY(-50%)' }}>
          <NavItem {...navItems[1]} isActive={isActive(navItems[1].path)} navigate={navigate} />
        </div>

        {/* 예약 */}
        <div style={{ position: 'absolute', left: '65.87%', top: '50%', transform: 'translateY(-50%)' }}>
          <NavItem {...navItems[2]} isActive={isActive(navItems[2].path)} navigate={navigate} />
        </div>

        {/* 메뉴 */}
        <div style={{ position: 'absolute', left: '84.13%', top: '50%', transform: 'translateY(-50%)' }}>
          <NavItem {...navItems[3]} isActive={isActive(navItems[3].path)} navigate={navigate} />
        </div>

        {/* 중앙 플레이 버튼 */}
        <div
          style={{
            position: 'absolute',
            bottom: 12,
            left: '50%',
            transform: 'translateX(-50%)',
            width: 60,
            height: 60,
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
  return (
    <button
      onClick={() => navigate(path)}
      style={{
        background: 'none',
        border: 'none',
        width: 50,
        height: 46,
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        gap: 4,
        padding: 0,
      }}
    >
      <img
        src={isActive ? activeIcon : inactiveIcon}
        alt={label}
        style={{ width: 24, height: 24 }}
      />
      <span
        style={{
          fontSize: 14,
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

// src/components/common/NavigationBar.tsx
import { Home, Settings, BarChart3, Layers, Play } from 'lucide-react';
import { useNavigate, useLocation } from 'react-router-dom';
import React from 'react';

const NavigationBar: React.FC = () => {
  const navigate = useNavigate();
  const location = useLocation();
  const currentPath = location.pathname;

  // 현재 활성화된 경로 확인
  const isActive = (path: string): boolean => {
    if (path === '/' && currentPath === '/') return true;
    if (path !== '/' && currentPath === path) return true;
    return false;
  };

  return (
    <div 
      style={{
        position: 'fixed',
        bottom: 0,
        left: 0,
        right: 0,
        display: 'flex',
        flexDirection: 'row',
        justifyContent: 'space-between',
        backgroundColor: '#1E1E1E',
        borderTop: '1px solid #444',
        height: '56px',
        width: '100%'
      }}
    >
      <div 
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          flex: 1
        }}
      >
        <button 
          onClick={() => navigate('/')}
          style={{
            background: 'none',
            border: 'none',
            padding: 0,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            color: isActive('/') ? '#0088FF' : '#888'
          }}
        >
          <Home size={20} style={{ marginBottom: '4px' }} />
          <span style={{ fontSize: '10px' }}>홈</span>
        </button>
      </div>
      
      <div 
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          flex: 1
        }}
      >
        <button 
          onClick={() => navigate('/control')}
          style={{
            background: 'none',
            border: 'none',
            padding: 0,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            color: isActive('/control') ? '#0088FF' : '#888'
          }}
        >
          <Settings size={20} style={{ marginBottom: '4px' }} />
          <span style={{ fontSize: '10px' }}>제어</span>
        </button>
      </div>
      
      <div 
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          flex: 1
        }}
      >
        <button 
          onClick={() => navigate('/play')}
          style={{
            background: 'none',
            border: 'none',
            padding: 0,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center'
          }}
        >
          <div style={{
            backgroundColor: '#0088FF',
            color: 'white',
            borderRadius: '50%',
            width: '40px',
            height: '40px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            marginBottom: '4px'
          }}>
            <Play size={20} />
          </div>
          <span style={{ fontSize: '10px', visibility: 'hidden' }}>플레이</span>
        </button>
      </div>
      
      <div 
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          flex: 1
        }}
      >
        <button 
          onClick={() => navigate('/report')}
          style={{
            background: 'none',
            border: 'none',
            padding: 0,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            color: isActive('/report') ? '#0088FF' : '#888'
          }}
        >
          <BarChart3 size={20} style={{ marginBottom: '4px' }} />
          <span style={{ fontSize: '10px' }}>리포트</span>
        </button>
      </div>
      
      <div 
        style={{
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          justifyContent: 'center',
          flex: 1
        }}
      >
        <button 
          onClick={() => navigate('/menu')}
          style={{
            background: 'none',
            border: 'none',
            padding: 0,
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            color: isActive('/menu') ? '#0088FF' : '#888'
          }}
        >
          <Layers size={20} style={{ marginBottom: '4px' }} />
          <span style={{ fontSize: '10px' }}>메뉴</span>
        </button>
      </div>
    </div>
  );
};

export default NavigationBar;
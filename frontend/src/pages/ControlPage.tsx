// src/pages/ControlPage.tsx - 수동조작 버튼 기능 추가
import { Play, Home, Calendar, Gamepad2, ChevronLeft } from 'lucide-react';
import robotImage from '../assets/robot.png';
import { useNavigate } from 'react-router-dom';

const ControlPage = () => {
  const navigate = useNavigate();

  const handleGoBack = () => {
    navigate(-1);
  };

  return (
    <div style={{ 
      width: '100%', 
      height: '100%', 
      display: 'flex', 
      flexDirection: 'column', 
      backgroundColor: '#1E1E1E', 
      color: '#FFFFFF' 
    }}>
      {/* 헤더 */}
      <header style={{
        padding: '16px 24px',
        backgroundColor: '#1E1E1E',
        display: 'flex',
        alignItems: 'center',
        gap: '12px',
        borderBottom: '1px solid #333',
        position: 'sticky',
        top: 0,
        zIndex: 10
      }}>
        <button 
          onClick={handleGoBack}
          style={{
            background: 'none',
            border: 'none',
            padding: '4px',
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            color: 'white'
          }}
        >
          <ChevronLeft size={24} />
        </button>
        <span style={{ fontSize: '20px', fontWeight: 'bold' }}>제어</span>
      </header>
      
      <main style={{ 
        flex: 1, 
        padding: '0 16px', 
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        overflowY: 'auto',
        paddingBottom: '120px' // 하단 패딩 크게 증가
      }}>
        {/* 로봇 이미지 */}
        <div style={{ 
          marginTop: '20px',
          marginBottom: '5px' 
        }}>
          <img 
            src={robotImage} 
            alt="IoT 로봇" 
            style={{ width: '180px', height: '180px' }}
          />
        </div>
        
        {/* 버튼 그리드 */}
        <div style={{ 
          display: 'grid', 
          gridTemplateColumns: 'repeat(2, 1fr)',
          gap: '16px',
          width: '100%',
          maxWidth: '360px',
        }}>
          {/* 동작 제어 버튼 */}
          <button style={{
            backgroundColor: '#2A2A2A',
            borderRadius: '8px',
            padding: '24px 16px',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            border: 'none',
            cursor: 'pointer'
          }}>
            <div style={{
              backgroundColor: '#0088FF',
              borderRadius: '50%',
              width: '48px',
              height: '48px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              marginBottom: '12px'
            }}>
              <Play size={24} color="white" />
            </div>
            <span style={{ color: 'white', fontSize: '14px' }}>동작 제어</span>
          </button>

          {/* 로봇 복귀 버튼 */}
          <button style={{
            backgroundColor: '#2A2A2A',
            borderRadius: '8px',
            padding: '24px 16px',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            border: 'none',
            cursor: 'pointer'
          }}>
            <div style={{
              backgroundColor: '#4DD0E1',
              borderRadius: '50%',
              width: '48px',
              height: '48px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              marginBottom: '12px'
            }}>
              <Home size={24} color="white" />
            </div>
            <span style={{ color: 'white', fontSize: '14px' }}>로봇 복귀</span>
          </button>

          {/* 루틴 예약 버튼 */}
          <button style={{
            backgroundColor: '#2A2A2A',
            borderRadius: '8px',
            padding: '24px 16px',
            display: 'flex',
            flexDirection: 'column',
            alignItems: 'center',
            justifyContent: 'center',
            border: 'none',
            cursor: 'pointer'
          }}>
            <div style={{
              backgroundColor: '#FF5252',
              borderRadius: '50%',
              width: '48px',
              height: '48px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              marginBottom: '12px'
            }}>
              <Calendar size={24} color="white" />
            </div>
            <span style={{ color: 'white', fontSize: '14px' }}>루틴 예약</span>
          </button>

          {/* 수동 조작 버튼 - 클릭 시 RobotControlPage로 이동 */}
          <button 
            onClick={() => navigate('/robot-control')} 
            style={{
              backgroundColor: '#2A2A2A',
              borderRadius: '8px',
              padding: '24px 16px',
              display: 'flex',
              flexDirection: 'column',
              alignItems: 'center',
              justifyContent: 'center',
              border: 'none',
              cursor: 'pointer'
            }}
          >
            <div style={{
              backgroundColor: '#9C7DF8',
              borderRadius: '50%',
              width: '48px',
              height: '48px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              marginBottom: '12px'
            }}>
              <Gamepad2 size={24} color="white" />
            </div>
            <span style={{ color: 'white', fontSize: '14px' }}>수동 조작</span>
          </button>
        </div>
      </main>
    </div>
  );
};

export default ControlPage;
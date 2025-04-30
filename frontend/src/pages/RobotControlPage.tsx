// src/pages/RobotControlPage.tsx
import { ChevronLeft, ArrowUp, ArrowDown, ArrowLeft, ArrowRight, Power } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import React from 'react';
import mapImage from '../assets/map.png';

const RobotControlPage: React.FC = () => {
  const navigate = useNavigate();

  const handleGoBack = () => {
    navigate(-1);
  };

  // 실제로는 이 함수들이 로봇 제어 API와 연결되어야 함
  const handleUp = () => console.log('Up');
  const handleDown = () => console.log('Down');
  const handleLeft = () => console.log('Left');
  const handleRight = () => console.log('Right');
  const handlePower = () => console.log('Power');

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
        <span style={{ fontSize: '20px', fontWeight: 'bold' }}>수동 조작</span>
      </header>
      
      <main style={{ 
        flex: 1, 
        padding: '16px', 
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        overflowY: 'auto',
        paddingBottom: '80px'
      }}>
        {/* 지도 이미지 영역 */}
        <div style={{ 
          width: '100%',
          height: '200px',
          backgroundColor: '#2A2A2A',
          borderRadius: '12px',
          marginTop: '16px',
          marginBottom: '12px', // 간격 반으로 줄임(24px→12px)
          position: 'relative'
        }}>
          <div style={{
            width: '100%',
            height: '100%',
            overflow: 'auto', // 스크롤 가능하게 변경
            padding: '8px',
            boxSizing: 'border-box'
          }}>
            {/* 헬스장 맵 이미지 */}
            <img 
              src={mapImage} 
              alt="헬스장 맵" 
              style={{ 
                width: '100%',
                maxHeight: '100%',
                display: 'block',
                objectFit: 'contain'
              }}
            />
          </div>
          
          {/* 로봇 위치 표시 마커 - 설명 추가 */}
          <div style={{
            position: 'absolute',
            bottom: '12px',
            right: '12px',
            backgroundColor: 'rgba(0, 0, 0, 0.7)',
            padding: '6px 12px',
            borderRadius: '16px',
            display: 'flex',
            alignItems: 'center',
            fontSize: '12px'
          }}>
            <div style={{
              width: '10px',
              height: '10px',
              borderRadius: '50%',
              backgroundColor: '#00CFFD',
              marginRight: '6px'
            }} />
            <span>로봇 위치</span>
          </div>
        </div>
        
        {/* 컨트롤러 영역 - 추가로 크기 20% 축소 */}
        <div style={{ 
          display: 'grid',
          gridTemplateRows: 'repeat(3, auto)',
          gridTemplateColumns: 'repeat(3, auto)',
          gap: '6px', // 간격 줄임(8px→6px)
          marginTop: '12px', // 상단 간격 줄임(16px→12px)
          placeItems: 'center',
          justifyContent: 'center'
        }}>
          {/* 위 버튼 */}
          <div style={{ gridColumn: '2', gridRow: '1' }}>
            <button 
              onClick={handleUp}
              style={{
                backgroundColor: '#1A1A1A',
                border: '2px solid #00CFFD',
                borderRadius: '8px',
                width: '38px', // 추가 20% 축소(48px→38px)
                height: '38px', // 추가 20% 축소(48px→38px)
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: '#00CFFD',
                padding: '0'
              }}
            >
              <ArrowUp size={20} /> {/* 아이콘 크기 축소(24px→20px) */}
            </button>
          </div>
          
          {/* 왼쪽 버튼 */}
          <div style={{ gridColumn: '1', gridRow: '2' }}>
            <button 
              onClick={handleLeft}
              style={{
                backgroundColor: '#1A1A1A',
                border: '2px solid #00CFFD',
                borderRadius: '8px',
                width: '38px', // 추가 20% 축소(48px→38px)
                height: '38px', // 추가 20% 축소(48px→38px)
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: '#00CFFD',
                padding: '0'
              }}
            >
              <ArrowLeft size={20} /> {/* 아이콘 크기 축소(24px→20px) */}
            </button>
          </div>
          
          {/* 가운데 전원 버튼 */}
          <div style={{ gridColumn: '2', gridRow: '2' }}>
            <button 
              onClick={handlePower}
              style={{
                backgroundColor: '#1A1A1A',
                border: '2px solid #00CFFD',
                borderRadius: '50%',
                width: '52px', // 추가 20% 축소(64px→52px)
                height: '52px', // 추가 20% 축소(64px→52px)
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                boxShadow: '0 0 10px rgba(0, 207, 253, 0.6)', // 그림자 효과 줄임
                padding: '0'
              }}
            >
              <Power size={26} color="#00CFFD" /> {/* 아이콘 크기 축소(32px→26px) */}
            </button>
          </div>
          
          {/* 오른쪽 버튼 */}
          <div style={{ gridColumn: '3', gridRow: '2' }}>
            <button 
              onClick={handleRight}
              style={{
                backgroundColor: '#1A1A1A',
                border: '2px solid #00CFFD',
                borderRadius: '8px',
                width: '38px', // 추가 20% 축소(48px→38px)
                height: '38px', // 추가 20% 축소(48px→38px)
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: '#00CFFD',
                padding: '0'
              }}
            >
              <ArrowRight size={20} /> {/* 아이콘 크기 축소(24px→20px) */}
            </button>
          </div>
          
          {/* 아래 버튼 */}
          <div style={{ gridColumn: '2', gridRow: '3' }}>
            <button 
              onClick={handleDown}
              style={{
                backgroundColor: '#1A1A1A',
                border: '2px solid #00CFFD',
                borderRadius: '8px',
                width: '38px', // 추가 20% 축소(48px→38px)
                height: '38px', // 추가 20% 축소(48px→38px)
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                color: '#00CFFD',
                padding: '0'
              }}
            >
              <ArrowDown size={20} /> {/* 아이콘 크기 축소(24px→20px) */}
            </button>
          </div>
        </div>
      </main>
    </div>
  );
};

export default RobotControlPage;
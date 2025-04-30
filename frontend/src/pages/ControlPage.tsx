// src/pages/PlayPage.tsx
import Header from '../components/common/Header';
import { Battery, MapPin, Clock, AlertCircle, Settings } from 'lucide-react';
import robot2Image from '../assets/robot2.png';

const PlayPage = () => {
  return (
    <div style={{ 
      width: '100%', 
      height: '100%', 
      display: 'flex', 
      flexDirection: 'column', 
      backgroundColor: '#1E1E1E', 
      color: '#FFFFFF' 
    }}>
      <Header title="임정인님의 PTSD" />

      <div style={{ 
        flex: 1, 
        padding: '16px', 
        overflowY: 'auto', 
        paddingBottom: '96px' 
      }}>
        {/* 로봇 카드 */}
        <div style={{ 
          backgroundColor: '#2A2A2A', 
          borderRadius: '8px', 
          padding: '16px', 
          marginBottom: '16px',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)' 
        }}>
          <h2 style={{ 
            fontSize: '1.25rem', 
            fontWeight: 'bold', 
            textAlign: 'center',
            color: 'white' 
          }}>
            PTSD
          </h2>
          <div style={{ 
            display: 'flex', 
            justifyContent: 'center', 
            alignItems: 'center', 
            marginTop: '4px' 
          }}>
            <div style={{ 
              width: '8px', 
              height: '8px', 
              borderRadius: '50%', 
              backgroundColor: '#4CAF50', 
              marginRight: '8px' 
            }} />
            <span style={{ 
              fontSize: '0.875rem', 
              color: '#4CAF50' 
            }}>
              연결됨
            </span>
          </div>
          <div style={{ 
            display: 'flex', 
            justifyContent: 'center', 
            margin: '8px 0' 
          }}>
            <img
              src={robot2Image}
              alt="PTSD 로봇"
              style={{ height: '112px' }}
            />
          </div>
        </div>

        {/* 상태 카드 그리드 */}
        <div style={{ 
          display: 'grid', 
          gridTemplateColumns: 'repeat(2, 1fr)', 
          gap: '16px' 
        }}>
          {/* 배터리 카드 */}
          <div style={{ 
            backgroundColor: '#2A2A2A', 
            borderRadius: '8px', 
            padding: '16px', 
            boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)' 
          }}>
            <div style={{ 
              display: 'flex', 
              flexDirection: 'column', 
              alignItems: 'center' 
            }}>
              <Battery size={24} style={{ color: '#0088FF', marginBottom: '4px' }} />
              <p style={{ 
                fontSize: '1.25rem', 
                fontWeight: 'bold', 
                color: 'white', 
                margin: '4px 0' 
              }}>
                82%
              </p>
              <p style={{ 
                fontSize: '0.75rem', 
                color: '#9CA3AF', 
                margin: '2px 0' 
              }}>
                예상 남은 시간
              </p>
              <p style={{ 
                fontSize: '0.875rem', 
                color: 'white', 
                margin: '2px 0' 
              }}>
                1시간 40분
              </p>
            </div>
          </div>

          {/* 위치 카드 */}
          <div style={{ 
            backgroundColor: '#2A2A2A', 
            borderRadius: '8px', 
            padding: '16px', 
            boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)' 
          }}>
            <div style={{ 
              display: 'flex', 
              flexDirection: 'column', 
              alignItems: 'center' 
            }}>
              <MapPin size={24} style={{ color: '#0088FF', marginBottom: '4px' }} />
              <p style={{ 
                fontSize: '1.25rem', 
                fontWeight: 'bold', 
                color: 'white', 
                margin: '4px 0' 
              }}>
                현재 위치
              </p>
              <p style={{ 
                fontSize: '0.875rem', 
                color: 'white', 
                margin: '2px 0' 
              }}>
                웨이트 존
              </p>
            </div>
          </div>

          {/* 청소 상태 카드 */}
          <div style={{ 
            backgroundColor: '#2A2A2A', 
            borderRadius: '8px', 
            padding: '16px', 
            boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)' 
          }}>
            <div style={{ 
              display: 'flex', 
              flexDirection: 'column', 
              alignItems: 'center' 
            }}>
              <Settings size={24} style={{ color: '#0088FF', marginBottom: '4px' }} />
              <p style={{ 
                fontSize: '1.25rem', 
                fontWeight: 'bold', 
                color: 'white', 
                margin: '4px 0' 
              }}>
                청소 중
              </p>
              <p style={{ 
                fontSize: '0.75rem', 
                color: '#9CA3AF', 
                margin: '2px 0' 
              }}>
                예상 종료 시간
              </p>
              <p style={{ 
                fontSize: '0.875rem', 
                color: 'white', 
                margin: '2px 0' 
              }}>
                오전 11:00
              </p>
            </div>
          </div>

          {/* 예약 상태 카드 */}
          <div style={{ 
            backgroundColor: '#2A2A2A', 
            borderRadius: '8px', 
            padding: '16px', 
            boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)' 
          }}>
            <div style={{ 
              display: 'flex', 
              flexDirection: 'column', 
              alignItems: 'center' 
            }}>
              <Clock size={24} style={{ color: '#0088FF', marginBottom: '4px' }} />
              <p style={{ 
                fontSize: '1.25rem', 
                fontWeight: 'bold', 
                color: 'white', 
                margin: '4px 0' 
              }}>
                예약 상태
              </p>
              <p style={{ 
                fontSize: '0.875rem', 
                color: 'white', 
                margin: '2px 0' 
              }}>
                매일 오전 9:00
              </p>
            </div>
          </div>
        </div>

        {/* 알림 카드 */}
        <div style={{ 
          backgroundColor: '#2A2A2A', 
          borderRadius: '8px', 
          padding: '16px', 
          marginTop: '16px',
          boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)' 
        }}>
          <div style={{ 
            display: 'flex', 
            flexDirection: 'column', 
            alignItems: 'center' 
          }}>
            <AlertCircle size={24} style={{ color: '#FF3B30', marginBottom: '4px' }} />
            <p style={{ 
              fontSize: '1.25rem', 
              fontWeight: 'bold', 
              color: 'white', 
              margin: '4px 0' 
            }}>
              알림
            </p>
            <p style={{ 
              fontSize: '0.875rem', 
              color: 'white', 
              margin: '2px 0' 
            }}>
              10분 전 : 청소 완료
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default PlayPage;
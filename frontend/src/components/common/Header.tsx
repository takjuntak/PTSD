import React, { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import plusImage from '../../assets/header/header-plus.svg';
import bellImage from '../../assets/header/header-bell.svg';
import menuImage from '../../assets/header/header-more.svg';
import { ChevronDown, Check, Settings } from 'lucide-react';
import apiClient from '../../api/axios';
import useAlarms from '../../hooks/useAlarms';
import useNotificationSocket from '../../hooks/useNotificationSocket';

interface Device {
  device_id: number;
  name: string;
}

const AppHeader: React.FC = () => {
  const navigate = useNavigate();
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const [devices, setDevices] = useState<Device[]>([]);
  const [selectedDevice, setSelectedDevice] = useState<Device | null>(null);

  const { alarms } = useAlarms(); // alarms 배열 직접 사용
  const { notification } = useNotificationSocket(); // 실시간 알림도 감지

  const hasUnread =
    alarms.some(alarm => !alarm.isRead) ||
    (!!notification && !notification.notification.is_read && !alarms.some(a => a.id === notification.notification.notification_id));
  
    const fetchDevices = async () => {
    try {
      const response = await apiClient.get('/devices');
      const deviceList: Device[] = response.data;
      setDevices(deviceList);
      if (deviceList.length > 0 && !selectedDevice) {
        setSelectedDevice(deviceList[0]);
      }
    } catch (error) {
      console.error('기기 목록 불러오기 실패', error);
    }
  };

  const handleSelectDevice = (device: Device) => {
    setSelectedDevice(device);
    setDropdownOpen(false); // 선택 후 드롭다운 닫기
  };

  const handleAlarmClick = () => {
    navigate('/alarm');
  };

  useEffect(() => {
    fetchDevices();
  }, []);

  return (
    <div
      style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        backgroundColor: '#2E2E37',
        height: 58,
        display: 'flex',
        justifyContent: 'center',
        zIndex: 999,
      }}
    >
      <div
        style={{
          width: '100%',
          padding: '0 16px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'space-between',
          position: 'relative',
        }}
      >
        {/* ▼ 선택된 기기명 버튼 */}
        <button
          onClick={() => setDropdownOpen(!dropdownOpen)}
          style={{
            background: 'none',
            border: 'none',
            color: '#FFFFFF',
            fontSize: 20,
            fontWeight: 800,
            fontFamily: 'Inter',
            display: 'flex',
            alignItems: 'center',
            gap: 4,
          }}
        >
          {selectedDevice?.name || 'PTSD'}
          <ChevronDown size={16} />
        </button>

        {/* 아이콘 영역 */}
        <div style={{ display: 'flex', gap: 20 }}>
          <img src={plusImage} alt="추가" width={20} height={20} />
          <div style={{ position: 'relative', width: 20, height: 20 }}>
            <img
              src={bellImage}
              alt="알림"
              width={24}
              height={24}
              onClick={handleAlarmClick}
              style={{ cursor: 'pointer' }}
            />
            {hasUnread && (
              <div
                style={{
                  position: 'absolute',
                  top: -4,
                  right: -2,
                  width: 6,
                  height: 6,
                  backgroundColor: '#0098FF',
                  borderRadius: '50%',
                }}
              />
            )}
          </div>
          <img src={menuImage} alt="더보기" width={20} height={20} />
        </div>

        {/* 드롭다운 메뉴 */}
        {dropdownOpen && (
          <div
            style={{
              position: 'absolute',
              top: 56,
              left: 8,
              background: '#373738',
              borderRadius: 10,
              boxShadow: '0 0 10px rgba(0,0,0,0.25)',
              padding: '10px 0',
              zIndex: 1000,
              minWidth: 180,
            }}
          >
            {devices.map((device) => (
              <div
                key={device.device_id}
                onClick={() => handleSelectDevice(device)}
                style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'center',
                  padding: '10px 16px',
                  color: device.device_id === selectedDevice?.device_id ? '#0098FF' : '#FFFFFF',
                  fontWeight: device.device_id === selectedDevice?.device_id ? 800 : 500,
                  fontFamily: 'Inter',
                  fontSize: 14,
                  cursor: 'pointer',
                }}
              >
                {device.name}
                {device.device_id === selectedDevice?.device_id && (
                  <Check size={16} color="#0098FF" />
                )}
              </div>
            ))}
            <hr
              style={{
                borderTop: '1px dashed #767676',
                margin: '8px 16px',
              }}
            />
            <div
              onClick={() => {
                navigate('/device-connect');
                setDropdownOpen(false);
              }}
              style={{
                display: 'flex',
                alignItems: 'center',
                gap: 6,
                padding: '10px 16px',
                color: '#FFFFFF',
                fontSize: 14,
                fontWeight: 500,
                cursor: 'pointer',
              }}
            >
              <Settings size={16} />
              기기 관리
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default AppHeader;

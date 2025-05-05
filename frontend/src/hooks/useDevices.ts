// src/hooks/useDevices.ts
import { useState, useEffect } from 'react';

export interface Device {
  id: string;
  name: string;
  isConnected: boolean;
  image?: string;
}

export const useDevices = () => {
  // 로컬 스토리지에서 기기 정보 불러오기
  const [devices, setDevices] = useState<Device[]>(() => {
    const savedDevices = localStorage.getItem('connectedDevices');
    return savedDevices ? JSON.parse(savedDevices) : [];
  });

  // 기기 정보가 변경될 때마다 로컬 스토리지에 저장
  useEffect(() => {
    localStorage.setItem('connectedDevices', JSON.stringify(devices));
  }, [devices]);

  // 기기 추가
  const addDevice = (device: Omit<Device, 'id'>) => {
    const newDevice = {
      ...device,
      id: Date.now().toString()
    };
    setDevices(prev => [...prev, newDevice]);
    return newDevice;
  };

  // 기기 제거
  const removeDevice = (deviceId: string) => {
    setDevices(prev => prev.filter(device => device.id !== deviceId));
  };

  // 기기 연결 상태 변경
  const toggleConnection = (deviceId: string, isConnected: boolean) => {
    setDevices(prev => 
      prev.map(device => 
        device.id === deviceId 
          ? { ...device, isConnected } 
          : device
      )
    );
  };

  // 모든 기기 연결 해제
  const disconnectAll = () => {
    setDevices(prev => 
      prev.map(device => ({ ...device, isConnected: false }))
    );
  };

  return {
    devices,
    addDevice,
    removeDevice,
    toggleConnection,
    disconnectAll,
    hasDevices: devices.length > 0,
    connectedDevices: devices.filter(d => d.isConnected)
  };
};
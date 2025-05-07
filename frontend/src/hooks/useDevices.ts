// src/hooks/useDevices.ts
import { useState, useEffect, useCallback } from 'react';
import { deviceService, DeviceResponse } from '../services/deviceService';

export interface Device extends DeviceResponse {
  isConnected: boolean;
  image?: string;
}

export const useDevices = () => {
  const [devices, setDevices] = useState<Device[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // 기기 목록 가져오기
  const fetchDevices = useCallback(async () => {
    try {
      setIsLoading(true);
      const response = await deviceService.getAllDevices();
      const devicesWithStatus = response.map(device => ({
        ...device,
        isConnected: true, // 연결 상태는 별도 API로 관리해야 할 수 있음
      }));
      setDevices(devicesWithStatus);
      setError(null);
    } catch (err) {
      setError('기기 목록을 불러오는데 실패했습니다.');
      console.error('Error fetching devices:', err);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchDevices();
  }, [fetchDevices]);

  // 기기 추가
  const addDevice = async (data: { serial_number: string; name: string }) => {
    try {
      // user_id 필드 제거
      const newDevice = await deviceService.registerDevice({
        serial_number: data.serial_number,
        name: data.name
      });
      const deviceWithStatus = {
        ...newDevice,
        isConnected: true
      };
      setDevices(prev => [...prev, deviceWithStatus]);
      return deviceWithStatus;
    } catch (err) {
      setError('기기 등록에 실패했습니다.');
      console.error('Error adding device:', err);
      throw err;
    }
  };

  // 기기 제거
  const removeDevice = async (deviceId: number) => {
    try {
      await deviceService.deleteDevice(deviceId);
      setDevices(prev => prev.filter(device => device.device_id !== deviceId));
    } catch (err) {
      setError('기기 삭제에 실패했습니다.');
      console.error('Error removing device:', err);
      throw err;
    }
  };

  // 기기 연결 상태 변경 (로컬 상태만 업데이트)
  const toggleConnection = (deviceId: number, isConnected: boolean) => {
    setDevices(prev => 
      prev.map(device => 
        device.device_id === deviceId 
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
    connectedDevices: devices.filter(d => d.isConnected),
    isLoading,
    error,
    refresh: fetchDevices
  };
};
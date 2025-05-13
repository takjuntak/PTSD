// src/services/deviceService.ts
import apiClient from '../api/axios'; // 중앙 관리형 axios 인스턴스 사용

export interface DeviceResponse {
  device_id: number;
  created_at: string;
  user_id: number;
  serial_number: string;
  name: string;
}

export interface DeviceRequest {
  serial_number: string;
  name: string;
}

export const deviceService = {
  // 기기 등록
  async registerDevice(data: DeviceRequest): Promise<DeviceResponse> {
    const requestData = {
      user_id: 0, // 서버가 필요로 하는 필드, 실제 값은 서버에서 토큰을 통해 결정됨
      serial_number: data.serial_number,
      name: data.name
    };
    const response = await apiClient.post('/devices', requestData);
    return response.data;
  },

  // 기기 조회
  async getDevice(deviceId: number): Promise<DeviceResponse> {
    const response = await apiClient.get(`/devices/${deviceId}`);
    return response.data;
  },

  // 기기 삭제
  async deleteDevice(deviceId: number): Promise<void> {
    await apiClient.delete(`/devices/${deviceId}`);
  },

  // 모든 기기 조회
  async getAllDevices(): Promise<DeviceResponse[]> {
    const response = await apiClient.get('/devices');
    return response.data;
  }
};
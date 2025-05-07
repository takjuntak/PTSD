// src/services/deviceService.ts
import axios from 'axios';

const api = axios.create({
  baseURL: 'http://localhost:8000', // API 엔드포인트
  headers: {
    'Content-Type': 'application/json'
  }
});

// 고정된 액세스 토큰
const ACCESS_TOKEN = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJ1c2VyM0BleGFtcGxlLmNvbSIsImV4cCI6MTc0NjU5NDk3MH0.2ZcpkX42Glene6mvGG_RTt3KiSYLLYxsWvdTb5wb0Ak';

// 요청 인터셉터
api.interceptors.request.use(
  config => {
    // 인증 토큰을 고정값으로 설정
    config.headers['Authorization'] = `Bearer ${ACCESS_TOKEN}`;
    return config;
  },
  error => {
    return Promise.reject(error);
  }
);

// 응답 인터셉터
api.interceptors.response.use(
  response => {
    return response;
  },
  error => {
    // 에러 형식 통일
    if (error.response) {
      // 서버가 응답했지만 에러 상태 코드
      const status = error.response.status;
      const message = error.response.data?.detail?.[0]?.msg || 
                      error.response.data?.message || 
                      '서버 오류가 발생했습니다.';
      
      if (status === 401) {
        // 인증 오류 처리
        console.log('인증이 필요합니다.');
      }
      
      return Promise.reject(new Error(message));
    } else if (error.request) {
      // 요청이 전송되었지만 응답이 없음
      return Promise.reject(new Error('서버에 연결할 수 없습니다.'));
    } else {
      // 요청 설정 중에 발생한 오류
      return Promise.reject(new Error('요청 중 오류가 발생했습니다.'));
    }
  }
);

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
    const response = await api.post('/api/devices/', data);
    return response.data;
  },

  // 기기 조회
  async getDevice(deviceId: number): Promise<DeviceResponse> {
    const response = await api.get(`/api/devices/${deviceId}`);
    return response.data;
  },

  // 기기 삭제
  async deleteDevice(deviceId: number): Promise<void> {
    await api.delete(`/api/devices/${deviceId}`);
  },

  // 모든 기기 조회 (API 경로 수정)
  async getAllDevices(): Promise<DeviceResponse[]> {
    const response = await api.get('/api/devices/');
    return response.data;
  }
};
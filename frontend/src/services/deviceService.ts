// src/services/deviceService.ts
import axios from 'axios';

const api = axios.create({
  baseURL: 'http://localhost:8000', // API 엔드포인트
  headers: {
    'Content-Type': 'application/json'
  }
});

// 요청 인터셉터
api.interceptors.request.use(
  config => {
    // 로컬 스토리지에서 액세스 토큰 가져오기
    const token = localStorage.getItem('accessToken');
    // 토큰이 있을 경우에만 헤더에 추가
    if (token) {
      config.headers['Authorization'] = `Bearer ${token}`;
    }
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
        // 인증 오류 처리 - 토큰 만료 등의 경우
        console.log('인증이 필요합니다.');
        // 로그인 페이지로 리다이렉트 등의 처리 추가 가능
        // window.location.href = '/login';
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
    const requestData = {
      user_id: 0, // 서버가 필요로 하는 필드, 실제 값은 서버에서 토큰을 통해 결정됨
      serial_number: data.serial_number,
      name: data.name
    };
    const response = await api.post('/api/devices/', requestData);
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

  // 모든 기기 조회
  async getAllDevices(): Promise<DeviceResponse[]> {
    const response = await api.get('/api/devices/');
    return response.data;
  }
};
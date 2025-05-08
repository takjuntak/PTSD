// src/services/routineService.ts
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
    // 로컬 스토리지에서 토큰 가져오기
    const token = localStorage.getItem('accessToken');
    if (token) {
      config.headers['Authorization'] = `Bearer ${token}`;
    }
    return config;
  },
  error => {
    return Promise.reject(error);
  }
);

// 스케줄(루틴) 데이터 타입 정의
export interface Routine {
  routine_id: number;
  start_time: string;
  routine_type: 'once' | 'daily';
  is_work: boolean;
  repeat_days: number[];
  created_at?: string;
}

// API에서 반환되는 응답 구조
interface ApiResponse<T> {
  isSuccess: boolean;
  code: number;
  message: string;
  result: T;
}

// API 루틴 응답 구조
interface RoutinesApiResponse {
  routines: Routine[];
}

export interface RoutineCreateRequest {
  start_time: string;
  routine_type: 'once' | 'daily';
  is_work: boolean;
  repeat_days: number[];
}

export interface RoutineUpdateRequest {
  start_time?: string;
  routine_type?: 'once' | 'daily';
  is_work?: boolean;
  repeat_days?: number[];
}

export const routineService = {
  // 모든 스케줄 조회
  async getAllRoutines(): Promise<Routine[]> {
    try {
      const response = await api.get<ApiResponse<RoutinesApiResponse>>('/api/routine');
      console.log('API 응답 데이터:', response.data);
      
      if (response.data && response.data.isSuccess && response.data.result && response.data.result.routines) {
        return response.data.result.routines;
      }
      
      return [];
    } catch (error) {
      console.error('스케줄 조회 API 오류:', error);
      throw error;
    }
  },

  // 스케줄 생성
  async createRoutine(data: RoutineCreateRequest): Promise<boolean> {
    try {
      const response = await api.post<ApiResponse<any>>('/api/routine', data);
      console.log('스케줄 생성 응답:', response.data);
      
      return response.data && response.data.isSuccess;
    } catch (error) {
      console.error('스케줄 생성 API 오류:', error);
      throw error;
    }
  },

  // 스케줄 수정
  async updateRoutine(routineId: number, data: RoutineUpdateRequest): Promise<boolean> {
    try {
      const response = await api.patch<ApiResponse<any>>(`/api/routine/${routineId}`, data);
      
      return response.data && response.data.isSuccess;
    } catch (error) {
      console.error('스케줄 수정 API 오류:', error);
      throw error;
    }
  },

  // 스케줄 삭제
  async deleteRoutine(routineId: number): Promise<boolean> {
    try {
      const response = await api.delete<ApiResponse<any>>(`/api/routine/${routineId}`);
      
      return response.data && response.data.isSuccess;
    } catch (error) {
      console.error('스케줄 삭제 API 오류:', error);
      throw error;
    }
  }
};
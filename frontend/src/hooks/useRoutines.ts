// src/hooks/useRoutines.ts
import { useState, useEffect, useCallback } from 'react';
import { routineService, Routine, RoutineCreateRequest, RoutineUpdateRequest } from '../services/routineService';

export const useRoutines = () => {
  const [routines, setRoutines] = useState<Routine[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // 루틴 목록 가져오기
  const fetchRoutines = useCallback(async () => {
    try {
      setIsLoading(true);
      const data = await routineService.getAllRoutines();
      setRoutines(data);
      setError(null);
    } catch (err) {
      setError('스케줄 목록을 불러오는데 실패했습니다.');
      console.error('Error fetching routines:', err);
    } finally {
      setIsLoading(false);
    }
  }, []);

  useEffect(() => {
    fetchRoutines();
  }, [fetchRoutines]);

  // 루틴 추가
  const addRoutine = async (data: RoutineCreateRequest) => {
    try {
      // 자바스크립트의 0(일요일)~6(토요일)에서 API 형식인 1(월요일)~7(일요일)로 변환
      const convertedRepeatDays = data.repeat_days.map(day => {
        // 일요일(0)은 7로 변환, 나머지는 그대로 사용 (월:1, 화:2, ...)
        return day === 0 ? 7 : day;
      });

      // 변환된 요일 데이터로 요청
      // 주의: 이미 TimeSelectPage에서 시간대 조정이 완료된 start_time을 그대로 사용
      const requestData = {
        ...data,
        repeat_days: convertedRepeatDays
      };

      await routineService.createRoutine(requestData);
      await fetchRoutines(); // 목록 다시 가져오기
      return true;
    } catch (err) {
      setError('스케줄 등록에 실패했습니다.');
      console.error('Error adding routine:', err);
      return false;
    }
  };

  // 루틴 업데이트
  const updateRoutine = async (routineId: number, data: RoutineUpdateRequest) => {
    try {
      // 요일 데이터가 있는 경우 변환
      if (data.repeat_days) {
        const convertedRepeatDays = data.repeat_days.map(day => {
          return day === 0 ? 7 : day;
        });
        data = { ...data, repeat_days: convertedRepeatDays };
      }

      // 시간 데이터(start_time)가 있는 경우 시간대 조정
      if (data.start_time) {
        const startTime = new Date(data.start_time);
        // 로컬 시간대를 유지하기 위한 처리
        const localISOString = new Date(
          startTime.getTime() - (startTime.getTimezoneOffset() * 60000)
        ).toISOString();
        data = { ...data, start_time: localISOString };
      }

      await routineService.updateRoutine(routineId, data);
      await fetchRoutines(); // 목록 다시 가져오기
      return true;
    } catch (err) {
      setError('스케줄 수정에 실패했습니다.');
      console.error('Error updating routine:', err);
      return false;
    }
  };

  // 루틴 삭제
  const deleteRoutines = async (routineIds: number[]) => {
    try {
      const deletePromises = routineIds.map(id => routineService.deleteRoutine(id));
      await Promise.all(deletePromises);
      await fetchRoutines(); // 목록 다시 가져오기
      return true;
    } catch (err) {
      setError('스케줄 삭제에 실패했습니다.');
      console.error('Error deleting routines:', err);
      return false;
    }
  };

  // 루틴 활성화/비활성화 토글
  const toggleRoutineActive = async (routineId: number, isActive: boolean) => {
    try {
      await routineService.updateRoutine(routineId, { is_work: isActive });
      setRoutines(prev => 
        prev.map(routine => 
          routine.routine_id === routineId 
            ? { ...routine, is_work: isActive } 
            : routine
        )
      );
      return true;
    } catch (err) {
      setError('스케줄 상태 변경에 실패했습니다.');
      console.error('Error toggling routine:', err);
      return false;
    }
  };

  return {
    routines,
    isLoading,
    error,
    addRoutine,
    updateRoutine,
    deleteRoutines,
    toggleRoutineActive,
    refreshRoutines: fetchRoutines
  };
};
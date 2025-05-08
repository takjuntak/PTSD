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
      setError(null);
      const data = await routineService.getAllRoutines();
      console.log('useRoutines에서 가져온 데이터:', data);
      setRoutines(data);
    } catch (err) {
      console.error('Error fetching routines:', err);
      setError('스케줄 목록을 불러오는데 실패했습니다.');
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
      const success = await routineService.createRoutine(data);
      if (success) {
        await fetchRoutines(); // 목록 다시 가져오기
        return true;
      }
      return false;
    } catch (err) {
      console.error('Error adding routine:', err);
      setError('스케줄 등록에 실패했습니다.');
      return false;
    }
  };

  // 루틴 업데이트
  const updateRoutine = async (routineId: number, data: RoutineUpdateRequest) => {
    try {
      const success = await routineService.updateRoutine(routineId, data);
      if (success) {
        await fetchRoutines(); // 목록 다시 가져오기
        return true;
      }
      return false;
    } catch (err) {
      console.error('Error updating routine:', err);
      setError('스케줄 수정에 실패했습니다.');
      return false;
    }
  };

  // 루틴 삭제
  const deleteRoutines = async (routineIds: number[]) => {
    try {
      const deletePromises = routineIds.map(id => routineService.deleteRoutine(id));
      const results = await Promise.all(deletePromises);
      
      if (results.every(result => result)) {
        await fetchRoutines(); // 목록 다시 가져오기
        return true;
      }
      return false;
    } catch (err) {
      console.error('Error deleting routines:', err);
      setError('스케줄 삭제에 실패했습니다.');
      return false;
    }
  };

  // 루틴 활성화/비활성화 토글
  const toggleRoutineActive = async (routineId: number, isActive: boolean) => {
    try {
      const success = await routineService.updateRoutine(routineId, { is_work: isActive });
      if (success) {
        // 로컬 상태 업데이트
        setRoutines(prev => 
          prev.map(routine => 
            routine.routine_id === routineId 
              ? { ...routine, is_work: isActive } 
              : routine
          )
        );
        return true;
      }
      return false;
    } catch (err) {
      console.error('Error toggling routine:', err);
      setError('스케줄 상태 변경에 실패했습니다.');
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
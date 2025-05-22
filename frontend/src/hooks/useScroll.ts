// src/hooks/useScroll.ts
import { useEffect, useRef, useState, RefObject } from 'react';

interface UseScrollOptions {
  target?: RefObject<HTMLElement>;
  onScroll?: (event: Event) => void;
  // threshold와 rootMargin은 현재 사용되지 않으므로 옵션으로 만들고 필요할 때 주석 해제
  // threshold?: number;
  // rootMargin?: string;
}

/**
 * 스크롤 이벤트를 관리하는 커스텀 훅
 * @param options 스크롤 옵션 (옵션 객체를 전달하지 않으면 내부에서 참조 생성)
 * @returns 스크롤 관련 상태 및 유틸리티
 */
function useScroll(options?: UseScrollOptions) {
  // 기본값 설정
  const containerRef = useRef<HTMLElement>(null);
  const target = options?.target || containerRef;
  const onScroll = options?.onScroll;
  const [scrollTop, setScrollTop] = useState<number>(0);
  const [isScrolling, setIsScrolling] = useState<boolean>(false);
  
  // 스크롤 타이머 ref
  const scrollTimerRef = useRef<number | null>(null);

  useEffect(() => {
    const container = target.current;
    if (!container) return;

    // iOS에서 부드러운 스크롤을 위한 속성 추가 (타입 안전하게 처리)
    const style = container.style as any;
    if (typeof style.WebkitOverflowScrolling !== 'undefined') {
      style.WebkitOverflowScrolling = 'touch';
    }

    // 스크롤 이벤트 핸들러
    const handleScroll = (event: Event) => {
      // 현재 스크롤 위치 업데이트
      setScrollTop(container.scrollTop);
      
      // 스크롤 중 상태 설정
      setIsScrolling(true);
      
      // 스크롤 타이머 초기화
      if (scrollTimerRef.current !== null) {
        window.clearTimeout(scrollTimerRef.current);
      }
      
      // 스크롤이 멈추면 상태 업데이트
      scrollTimerRef.current = window.setTimeout(() => {
        setIsScrolling(false);
      }, 150);
      
      // 사용자 정의 스크롤 핸들러 호출
      if (onScroll) {
        onScroll(event);
      }
    };

    // 이벤트 리스너 등록
    container.addEventListener('scroll', handleScroll, { passive: true });
    
    // 컴포넌트 언마운트 시 정리
    return () => {
      if (scrollTimerRef.current !== null) {
        window.clearTimeout(scrollTimerRef.current);
      }
      container.removeEventListener('scroll', handleScroll);
    };
  }, [target, onScroll]);

  // 특정 위치로 스크롤하는 유틸리티 함수
  const scrollTo = (top: number, options?: ScrollToOptions) => {
    if (target.current) {
      target.current.scrollTo({
        top,
        behavior: 'smooth',
        ...options
      });
    }
  };

  // 맨 위로 스크롤하는 유틸리티 함수
  const scrollToTop = () => {
    scrollTo(0);
  };

  // 맨 아래로 스크롤하는 유틸리티 함수
  const scrollToBottom = () => {
    if (target.current) {
      scrollTo(target.current.scrollHeight);
    }
  };

  return {
    containerRef,
    scrollTop,
    isScrolling,
    scrollTo,
    scrollToTop,
    scrollToBottom
  };
}

export default useScroll;
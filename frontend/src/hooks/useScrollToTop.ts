// hooks/useScrollToTop.ts
import { useLocation } from 'react-router-dom';
import { useEffect } from 'react';

const useScrollToTop = () => {
  const { pathname } = useLocation();

  useEffect(() => {
    // `.main-scroll-container`가 있으면 그것만 올리고, 없으면 window 스크롤
    const scrollTarget = document.querySelector('.main-scroll-container');
    if (scrollTarget) {
      scrollTarget.scrollTo({ top: 0, behavior: 'auto' });
    } else {
      window.scrollTo({ top: 0, behavior: 'auto' });
    }
  }, [pathname]); // pathname이 바뀔 때마다 실행
};

export default useScrollToTop;

// src/components/common/Portal.tsx
import { ReactNode, useEffect, useState } from 'react';
import { createPortal } from 'react-dom';

interface PortalProps {
  children: ReactNode;
}

/**
 * Portal 컴포넌트 - 자식 요소를 DOM의 다른 부분으로 렌더링
 * body에 직접 렌더링하여 스타일 간섭을 방지
 */
const Portal: React.FC<PortalProps> = ({ children }) => {
  const [container] = useState(() => document.createElement('div'));

  useEffect(() => {
    // 포털 컨테이너에 z-index 설정
    container.style.position = 'fixed';
    container.style.zIndex = '9999';
    container.style.top = '0';
    container.style.left = '0';
    container.style.width = '100%';
    container.style.height = '0';
    container.style.overflow = 'visible';
    
    // body에 추가
    document.body.appendChild(container);
    
    return () => {
      document.body.removeChild(container);
    };
  }, [container]);

  return createPortal(children, container);
};

export default Portal;
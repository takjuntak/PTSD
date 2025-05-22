// src/serviceWorker.ts
export const registerServiceWorker = async () => {
    if ('serviceWorker' in navigator) {
      try {
        const registration = await navigator.serviceWorker.register('/sw.js', {
          scope: '/'
        });
        
        console.log('서비스 워커가 성공적으로 등록되었습니다:', registration.scope);
        
        // 업데이트 확인
        registration.addEventListener('updatefound', () => {
          const newWorker = registration.installing;
          if (newWorker) {
            newWorker.addEventListener('statechange', () => {
              console.log('서비스 워커 상태:', newWorker.state);
            });
          }
        });
      } catch (error) {
        console.error('서비스 워커 등록 중 오류 발생:', error);
      }
    }
  };
  
  // PWA 설치 이벤트 처리
  export const setupPWAInstallHandler = () => {
    let deferredPrompt: BeforeInstallPromptEvent | null = null;
    
    // beforeinstallprompt 이벤트 캡처
    window.addEventListener('beforeinstallprompt', (e) => {
      // 브라우저 기본 설치 프롬프트 방지
      e.preventDefault();
      // 이벤트 저장
      deferredPrompt = e as BeforeInstallPromptEvent;
      console.log('설치 프롬프트 준비 완료');
      
      // 여기서 사용자에게 앱 설치를 유도하는 UI 표시 가능
    });
    
    // 앱이 설치되었을 때 이벤트
    window.addEventListener('appinstalled', () => {
      console.log('앱이 성공적으로 설치되었습니다!');
      deferredPrompt = null;
    });
    
    // 인스톨 프롬프트 함수 (사용자 트리거 필요)
    return {
      triggerInstall: async () => {
        if (!deferredPrompt) {
          console.log('설치 프롬프트가 준비되지 않았습니다.');
          return false;
        }
        
        // 프롬프트 표시
        deferredPrompt.prompt();
        
        // 사용자의 결정 기다리기
        const { outcome } = await deferredPrompt.userChoice;
        console.log(`사용자 선택: ${outcome}`);
        
        // 프롬프트는 한 번만 사용 가능
        deferredPrompt = null;
        
        return outcome === 'accepted';
      }
    };
  };
  
  // BeforeInstallPromptEvent 타입 정의
  interface BeforeInstallPromptEvent extends Event {
    readonly platforms: Array<string>;
    readonly userChoice: Promise<{
      outcome: 'accepted' | 'dismissed';
      platform: string;
    }>;
    prompt(): Promise<void>;
  }
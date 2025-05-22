// components/common/LogoutModal.tsx
import React from 'react';
import { useNavigate } from 'react-router-dom';
import apiClient from '../../api/axios';
import styles from './LogoutModal.module.css';
import logoutImage from '../../assets/menu/logout.svg';

interface LogoutModalProps {
  open: boolean;
  onClose: () => void;
}

const LogoutModal: React.FC<LogoutModalProps> = ({ open, onClose }) => {
  const navigate = useNavigate();

  if (!open) return null;

  const handleLogout = async () => {
    try {
      await apiClient.post('/auth/logout'); // ✅ API 호출
    } catch (error) {
      console.error('로그아웃 실패:', error); // 실패해도 계속 진행
    } finally {
      localStorage.removeItem('accessToken'); // ✅ 토큰 제거
      navigate('/login', { replace: true }); // ✅ 로그인 페이지로 이동
    }
  };

  return (
    <div className={styles.overlay}>
      <div className={styles.modalBox}>
        <img src={logoutImage} alt="로그아웃 아이콘" className={styles.icon} />
        <h2 className={styles.title}>로그아웃 하시겠어요?</h2>
        <p className={styles.subtitle}>
          로그아웃하면 제품 알림이나 이벤트 소식 등 <br />유용한 정보를 받을 수 없습니다.
        </p>
        <div className={styles.buttonWrapper}>
          <button className={styles.cancelButton} onClick={onClose}>취소</button>
          <button className={styles.logoutButton} onClick={handleLogout}>로그아웃</button>
        </div>
      </div>
    </div>
  );
};

export default LogoutModal;

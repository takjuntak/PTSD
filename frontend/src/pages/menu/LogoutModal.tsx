// components/common/LogoutModal.tsx
import React from 'react';
import styles from './LogoutModal.module.css';
import logoutImage from '../../assets/menu/logout.svg';

interface LogoutModalProps {
  open: boolean;
  onClose: () => void;
  onConfirm: () => void;
}

const LogoutModal: React.FC<LogoutModalProps> = ({ open, onClose, onConfirm }) => {
  if (!open) return null;

  return (
    <div className={styles.overlay}>
      <div className={styles.modalBox}>
        <img src={logoutImage} alt="로그아웃 아이콘" className={styles.icon} />
        <h2 className={styles.title}>로그아웃 하시겠어요?</h2>
        <p className={styles.subtitle}>
          로그아웃하면 제품 알림이나 이벤트 소식 등 유용한 정보를 받을 수 없습니다.
        </p>
        <div className={styles.buttonWrapper}>
          <button className={styles.cancelButton} onClick={onClose}>취소</button>
          <button className={styles.logoutButton} onClick={onConfirm}>로그아웃</button>
        </div>
      </div>
    </div>
  );
};

export default LogoutModal;

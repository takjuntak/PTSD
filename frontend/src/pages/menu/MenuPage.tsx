import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { ChevronLeft } from 'lucide-react';
import styles from './MenuPage.module.css';
import LogoutModal from './LogoutModal';
import { useAuth } from '../../hooks/useAuth';

import connectImage from '../../assets/menu/connect.svg';
import explainImage from '../../assets/menu/explain.svg';
import bellImage from '../../assets/menu/bell.svg';
import questionImage from '../../assets/menu/question.svg';
import personalInfoImage from '../../assets/menu/personal-info.svg';
import logoutImage from '../../assets/menu/logout.svg';
import deleteImage from '../../assets/menu/delete.svg';

import DeleteAccountModal from './DeleteAccountModal';


const MenuPage: React.FC = () => {
  const navigate = useNavigate();
  const [isLogoutOpen, setLogoutOpen] = useState(false);
  const [isDeleteOpen, setDeleteOpen] = useState(false);
  const { user } = useAuth();

  const handleGoBack = () => navigate(-1);
  const handleDeviceConnect = () => navigate('/device-connect');
  const handleLogout = () => setLogoutOpen(true);

  const handleGuideClick = () => navigate('/menu/product-guide');
  const handleFAQClick = () => navigate('/menu/faq')
  const handlePrivacyClick = () => navigate('/menu/privacy')
  const handleAlarmClick = () => navigate('/alarm')

  const menuItems = [
    { label: '기기 연결', image: connectImage, onClick: handleDeviceConnect },
    { label: '제품 사용 설명서', image: explainImage, onClick: handleGuideClick },
    { label: '알림', image: bellImage, onClick: handleAlarmClick },
  ];

  const supportItems = [
    { label: '로그아웃', image: logoutImage, onClick: handleLogout },
    { label: '계정 삭제', image: deleteImage, onClick: () => setDeleteOpen(true) },
    { label: '자주 묻는 질문', image: questionImage, onClick: handleFAQClick },
    { label: '약관 및 개인정보 처리방침', image: personalInfoImage, onClick: handlePrivacyClick },
  ];

  const renderItem = ({ label, image, onClick }: any, idx: number) => (
    <button
      key={idx}
      onClick={onClick}
      className={styles.menuItem}
    >
      <img src={image} alt={label} className={styles.emojiIcon} />
      <span className={styles.label}>{label}</span>
    </button>
  );

  return (
    <div className={styles.wrapper}>
      <header className={styles.header}>
        <button
          onClick={handleGoBack}
          className="text-white border-none bg-transparent p-0"
          style={{ background: 'transparent' }}
        >
          <ChevronLeft size={24} />
        </button>
        <span className="text-xl font-bold" style={{ color: '#767676' }}>홈</span>
      </header>

      <div className={styles.content} style={{ paddingBottom: '120px' }}>
        <section className={styles.profileSection}>
          <div className={styles.avatar} />
          <span className={styles.username}>
            {user?.name || '로그인이 필요합니다'}
          </span>
        </section>

        <section className={styles.sectionCard}>
          <div className={styles.sectionHeader}>제품 사용 | 관리</div>
          {menuItems.map(renderItem)}
        </section>

        <section className={styles.sectionCard}>
          <div className={styles.sectionHeader}>고객 지원</div>
          {supportItems.map(renderItem)}
        </section>
      </div>

      <LogoutModal open={isLogoutOpen} onClose={() => setLogoutOpen(false)} />
      <DeleteAccountModal
        open={isDeleteOpen}
        onClose={() => setDeleteOpen(false)}
        onDelete={() => {
          // 실제 계정 삭제 로직 추가
          setDeleteOpen(false);
          navigate('/login');
        }}
      />
    </div>
  );
};

export default MenuPage;

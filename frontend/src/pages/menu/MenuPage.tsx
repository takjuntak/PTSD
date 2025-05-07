// MenuPage.tsx (with CSS Module + LogoutModal integration)
import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import {
  ChevronLeft, Settings, FileText, CreditCard, Bell,
  Globe, Palette, HelpCircle, FileTerminal, Lock, Trash2
} from 'lucide-react';
import styles from './MenuPage.module.css';
import LogoutModal from './LogoutModal';

const MenuPage: React.FC = () => {
  const navigate = useNavigate();
  const [isLogoutOpen, setLogoutOpen] = useState(false);

  const handleGoBack = () => navigate(-1);
  const handleDeviceConnect = () => navigate('/device-connect');

  const handleLogout = () => setLogoutOpen(true);
  const confirmLogout = () => {
    localStorage.removeItem('accessToken');
    navigate('/login');
  };

  const menuItems = [
    { label: '기기 연결', icon: <Settings size={18} color="white" />, color: styles.violet, onClick: handleDeviceConnect },
    { label: '제품 사용 설명서', icon: <FileText size={18} color="white" />, color: styles.blue },
    { label: '구독 / 이용 플랜', icon: <CreditCard size={18} color="white" />, color: styles.emerald },
    { label: '알림 설정', icon: <Bell size={18} color="white" />, color: styles.amber },
    { label: '언어 설정', icon: <Globe size={18} color="white" />, color: styles.blue },
    { label: '테마 설정 (다크모드 / 라이트모드 전환)', icon: <Palette size={18} color="white" />, color: styles.violet }
  ];

  const supportItems = [
    { label: 'FAQ 문의하기', icon: <HelpCircle size={18} color="white" />, color: styles.red },
    { label: '약관 및 개인정보 처리방침', icon: <FileTerminal size={18} color="white" />, color: styles.amber },
    { label: '로그아웃', icon: <Lock size={18} color="white" />, color: styles.amber, onClick: handleLogout },
    { label: '계정 삭제', icon: <Trash2 size={18} color="white" />, color: styles.red }
  ];

  const renderItem = ({ label, icon, color, onClick }: any, idx: number) => (
    <button
      key={idx}
      onClick={onClick}
      className={styles.menuItem}
    >
      <div className={`${styles.iconWrapper} ${color}`}>{icon}</div>
      <span className={styles.label}>{label}</span>
    </button>
  );

  return (
    <div className={styles.wrapper}>
      <header className={styles.header}>
        <button onClick={handleGoBack} className={styles.backButton}>
          <ChevronLeft size={24} />
        </button>
        <span className={styles.title}>메뉴</span>
      </header>

      <div className={styles.content}>
        <section className={styles.profileSection}>
          <div className={styles.avatar} />
          <span className={styles.username}>SSAFY</span>
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

      <LogoutModal open={isLogoutOpen} onClose={() => setLogoutOpen(false)} onConfirm={confirmLogout} />
    </div>
  );
};

export default MenuPage;

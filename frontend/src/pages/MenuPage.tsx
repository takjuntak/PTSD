// src/pages/MenuPage.tsx (Tailwind CSS 적용 버전)
import { ChevronLeft, Settings, FileText, CreditCard, Bell, Globe, Palette, HelpCircle, FileTerminal, Lock, Trash2 } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import React from 'react';

const MenuPage: React.FC = () => {
  const navigate = useNavigate();

  const handleGoBack = () => navigate(-1);
  const handleDeviceConnect = () => navigate('/device-connect');

  const menuItems = [
    { label: '기기 연결', icon: <Settings size={18} color="white" />, color: 'bg-violet-500', onClick: handleDeviceConnect },
    { label: '제품 사용 설명서', icon: <FileText size={18} color="white" />, color: 'bg-blue-400' },
    { label: '구독 / 이용 플랜', icon: <CreditCard size={18} color="white" />, color: 'bg-emerald-500' },
    { label: '알림 설정', icon: <Bell size={18} color="white" />, color: 'bg-amber-500' },
    { label: '언어 설정', icon: <Globe size={18} color="white" />, color: 'bg-blue-500' },
    { label: '테마 설정 (다크모드 / 라이트모드 전환)', icon: <Palette size={18} color="white" />, color: 'bg-violet-500' }
  ];

  const supportItems = [
    { label: 'FAQ 문의하기', icon: <HelpCircle size={18} color="white" />, color: 'bg-red-500' },
    { label: '약관 및 개인정보 처리방침', icon: <FileTerminal size={18} color="white" />, color: 'bg-amber-500' },
    { label: '로그아웃', icon: <Lock size={18} color="white" />, color: 'bg-amber-500' },
    { label: '계정 삭제', icon: <Trash2 size={18} color="white" />, color: 'bg-red-500' },
  ];

  const renderItem = ({ label, icon, color, onClick }: any, idx: number) => (
    <button
      key={idx}
      onClick={onClick}
      className="w-full flex items-center p-4 border-b border-neutral-700 text-left hover:bg-neutral-800"
    >
      <div className={`w-8 h-8 rounded ${color} flex items-center justify-center mr-4`}>
        {icon}
      </div>
      <span className="text-white">{label}</span>
    </button>
  );

  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white relative">
      <header className="p-4 px-6 bg-app-dark flex items-center gap-3 border-b border-neutral-700 sticky top-0 z-10">
        <button onClick={handleGoBack} className="p-1 text-white hover:opacity-80">
          <ChevronLeft size={24} />
        </button>
        <span className="text-xl font-bold">메뉴</span>
      </header>

      <div className="flex-1 overflow-y-auto pb-20">
        <section className="p-6 flex items-center gap-4">
          <div className="w-16 h-16 rounded-full bg-gray-300 flex items-center justify-center" />
          <span className="text-lg font-semibold">SSAFY</span>
        </section>

        <section className="mx-4 mb-6 bg-app-card rounded-lg overflow-hidden">
          <div className="p-4 border-b border-neutral-700">
            <h3 className="text-base font-bold m-0">제품 사용 | 관리</h3>
          </div>
          {menuItems.map(renderItem)}
        </section>

        <section className="mx-4 mb-6 bg-app-card rounded-lg overflow-hidden">
          <div className="p-4 border-b border-neutral-700">
            <h3 className="text-base font-bold m-0">고객 지원</h3>
          </div>
          {supportItems.map(renderItem)}
        </section>
      </div>
    </div>
  );
};

export default MenuPage;
import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { ChevronLeft } from 'lucide-react';
import PrivacyModal from './PrivacyModal'; // 모달 컴포넌트 분리
const policyList = [
  { key: 'agree1', title: '서비스 이용 약관 동의', updated: '최종 업데이트 : 2024.11.25' },
  { key: 'agree2', title: '개인정보 수집 이용 동의', updated: '최종 업데이트 : 2024.11.25' },
  { key: 'agree3', title: '위치 정보 수집 및 이용 동의', updated: '최종 업데이트 : 2024.11.25' },
  { key: 'agree4', title: '마케팅 정보 수신 동의', updated: '최종 업데이트 : 2024.11.25' },
];

const getModalData = (key: string) => {
  switch (key) {
    case 'agree1':
      return {
        title: '(필수) 서비스 이용약관 동의',
        subtitle: '서비스 이용약관 동의',
        items: [
          '본 약관은 사용자가 PTSD 앱(이하 "서비스")을 이용함에 있어 회사와 사용자 간의 권리, 의무, 책임사항 및 기타 필요한 사항을 규정합니다.',
          '',
          '사용자는 서비스에 회원가입함으로써 본 약관의 내용을 모두 이해하고 동의한 것으로 간주됩니다.',
          '회사는 서비스의 내용, 제공 조건, 중단 또는 변경에 관한 권한을 가질 수 있습니다.',
          '사용자는 타인의 정보를 도용하거나 서비스 운영을 방해해서는 안 됩니다.',
          '※ 전체 약관은 [전체 이용약관 보기] 버튼을 통해 확인하실 수 있습니다.',
        ],
      };
    case 'agree2':
      return {
        title: '(필수) 개인정보 수집 및 이용 동의',
        subtitle: '개인정보 수집 및 이용 동의',
        items: [
          '[ 수집 항목 ]',
          '이름, 이메일 주소, 비밀번호, 휴대전화번호, 기기정보(OS, 디바이스 ID 등)',
          '',
          '[ 수집 목적 ]',
          '회원 식별 및 인증, 고객 문의 응대',
          '서비스 제공 및 맞춤형 서비스 운영',
          '부정 이용 방지 및 서비스 안정성 확보',
          '',
          '[ 보유 및 이용 기간 ]',
          '회원 탈퇴 시 또는 수집 목적 달성 시까지',
        ],
      };
    case 'agree3':
      return {
        title: '(필수) 위치 정보 수집 및 이용 동의',
        subtitle: '위치 정보 수집 및 이용 동의',
        items: [
          '[ 수집 항목 ]',
          '단말기의 GPS 또는 네트워크를 통해 수집된 위치 정보',
          '',
          '[ 수집 목적 ]',
          '로봇 청소 예약 및 위치 기반 자동 제어 서비스 제공',
          '청소 시작/종료 위치 설정 및 기록',
          '',
          '[ 보유 및 이용 기간 ]',
          '실시간 제공 목적 달성 시 즉시 파기',
        ],
      };
    case 'agree4':
      return {
        title: '(선택) 마케팅 정보 수신 동의',
        subtitle: '마케팅 정보 수신 동의',
        items: [
          '[ 수신 정보 ]',
          '앱 내 알림(Push), 이메일, 문자(SMS)',
          '',
          '[ 수신 목적 ]',
          '신규 기능 안내',
          '이벤트, 할인, 캠페인 등의 프로모션 정보 제공',
        ],
      };
    default:
      return { title: '', subtitle: '', items: [] };
  }
};

const PrivacyPage = () => {
  const navigate = useNavigate();
  const [modalOpen, setModalOpen] = useState(false);
  const [modalType, setModalType] = useState('');

  const { title, subtitle, items } = getModalData(modalType);

  return (
    <div className="min-h-screen bg-gradient-to-b from-[#2E2E37] to-[#1D1E23] text-white">
      {/* 헤더 */}
      <header className="p-3 px-4 flex items-center gap-2 top-0 z-10" onClick={() => navigate(-1)}>
        <button className="text-white bg-transparent border-none p-0">
          <ChevronLeft size={20} />
        </button>
        <span className="text-lg font-bold text-[#767676]">메뉴</span>
      </header>

      {/* 약관 항목 카드 */}
      {policyList.map((policy) => (
        <div
          key={policy.key}
          onClick={() => {
            setModalType(policy.key);
            setModalOpen(true);
          }}
          className="cursor-pointer bg-[#373738] rounded-lg shadow-md px-8 py-5 mb-4 mt-2 space-y-2 max-w-[330px] mx-auto"
        >
          <div className="flex justify-between items-start">
            <h2 className="text-base font-bold whitespace-nowrap">{policy.title}</h2>
          </div>
          <div className="flex justify-between items-center text-[10px] mt-2">
            <span>{policy.updated}</span>
            <span className="underline text-white">전체 보기</span>
          </div>
        </div>
      ))}

      {/* 모달 */}
      <PrivacyModal
        open={modalOpen}
        onClose={() => setModalOpen(false)}
        onAgree={() => setModalOpen(false)}
        title={title}
        subtitle={subtitle}
        items={items}
      />
    </div>
  );
};

export default PrivacyPage;

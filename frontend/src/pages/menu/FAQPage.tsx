import { useState } from 'react';
import { ChevronLeft } from 'lucide-react';
import { useNavigate } from 'react-router-dom';
import questionImage from '../../assets/menu/faq-question.svg';
import answerImage from '../../assets/menu/faq-answer.svg';

const tabLabels = ['기기', '청소', '예약', '충전', '기타'] as const;

const faqData = {
  기기: [
    {
      id: 1,
      question: '기기 등록은 어떻게 하나요?',
      date: '2024-11-25',
      views: '5,081회 조회',
      answer: '메뉴 > 기기 연결 > 시리얼 넘버 인식 또는 직접 입력으로 등록할 수 있습니다.',
    },
    {
        id: 2,
        question: '등록한 기기를 삭제하려면 어떻게 하나요?',
        date: '2024-11-20',
        views: '3,201회 조회',
        answer: '메뉴 > 기기 관리에서 삭제할 기기 우측 ‘···’ 버튼을 눌러 삭제할 수 있어요.',
      },
      {
        id: 3,
        question: '여러 기기를 동시에 등록할 수 있나요?',
        date: '2024-11-18',
        views: '1,203회 조회',
        answer: '네, 최대 3개의 기기를 등록하고 전환하여 사용할 수 있어요.',
      },
  ],
  청소: [
    {
      id: 4,
      question: '청소 예약은 어디서 하나요?',
      date: '2024-11-28',
      views: '4,108회 조회',
      answer: "홈 화면 하단 '예약' 탭에서 원하는 종료 시각을 선택해 예약할 수 있어요.",
    },
  ],
  예약: [
    {
      id: 5,
      question: '예약 시간은 최대 몇 시간까지 설정 가능한가요?',
      date: '2024-10-23',
      views: '3,077회 조회',
      answer: '최소 1시간부터 최대 12시간까지 설정할 수 있습니다.',
    },
    {
        id: 6,
        question: ' 예약된 청소를 취소하려면 어떻게 하나요?',
        date: '2024-09-01',
        views: '1,099회 조회',
        answer: '예약 탭에서 ‘예약 취소’ 버튼을 누르면 즉시 취소됩니다.',
      },
  ],
  충전: [
    {
      id: 7,
      question: '충전 중인지 어떻게 알 수 있나요?',
      date: '2024-11-25',
      views: '983회 조회',
      answer: '배터리 링에 번개 아이콘이 표시되면 충전 중입니다.',
    },
    {
        id: 8,
        question: '배터리 상태는 어디에서 확인하나요?',
        date: '2024-03-24',
        views: '1,029회 조회',
        answer: '메인 화면에서 기기 연결 시 배터리 잔량과 예상 사용 시간을 확인할 수 있습니다.',
      },
  ],
  기타: [
    {
      id: 9,
      question: '알림은 어떻게 설정하나요?',
      date: '2024-11-25',
      views: '253회 조회',
      answer: '메뉴 > 알림 설정에서 알림 종류와 시간을 선택할 수 있어요.',
    },
    {
        id: 10,
        question: '앱에서 로그아웃은 어떻게 하나요?',
        date: '2024-01-23',
        views: '534회 조회',
        answer: '메뉴 > 고객 지원 > 로그아웃을 선택하면 계정에서 로그아웃할 수 있습니다.',
      },
  ],
};

export default function FAQPage() {
  const [openId, setOpenId] = useState<number | null>(null);
  const [selectedTab, setSelectedTab] = useState<keyof typeof faqData>('예약');
  const navigate = useNavigate();

  const handleToggle = (id: number) => {
    setOpenId(openId === id ? null : id);
  };

  return (
    <div className="relative w-full h-full bg-transparent">
      {/* 상단 헤더 */}
      <header className="p-3 px-4 flex items-center gap-2 sticky top-0 z-10 bg-[#2E2E37]">
        <button
          onClick={() => navigate(-1)}
          className="text-white bg-transparent border-none p-0"
        >
          <ChevronLeft size={20} />
        </button>
        <span className="text-lg font-bold text-[#767676]">메뉴</span>
      </header>

      <div className="px-4 py-4 space-y-4">
        {/* 탭 선택 */}
        <div className="flex justify-center gap-3 mb-6 whitespace-nowrap">
          {tabLabels.map((label) => (
            <button
              key={label}
              onClick={() => {
                setSelectedTab(label);
                setOpenId(null);
              }}
              className={`text-white text-sm font-semibold px-4 py-1 rounded-full border ${
                selectedTab === label ? 'bg-[#576BC8] border-none' : 'border-[#636363]'
              }`}
            >
              {label}
            </button>
          ))}
        </div>

        {/* 질문 리스트 */}
        {faqData[selectedTab].map((faq) => (
          <div
            key={faq.id}
            className="bg-[#373738] text-white rounded-xl shadow-md px-4 py-3 transition-all duration-300"
          >
            {/* 접힌 상태 */}
            <div
              className="flex justify-between items-start cursor-pointer"
              onClick={() => handleToggle(faq.id)}
            >
              <div className="flex flex-col">
                <div className="flex items-center gap-2">
                  <img src={questionImage} alt="Q" className="w-5 h-5" />
                  <p className="font-bold text-[15px] break-keep text-left">{faq.question}</p>
                </div>
                <div className="flex gap-2 text-xs text-white opacity-80 mt-1">
                  <span>{faq.date}</span>
                  <span className="flex items-center gap-1">
                    <span className="w-2 h-2 bg-[#617BEE] rounded-full" />
                    {faq.views}
                  </span>
                </div>
              </div>
            </div>

            {/* 펼친 상태 (애니메이션 적용) */}
            <div
              className={`mt-2 overflow-hidden transition-all duration-300 ease-in-out ${
                openId === faq.id ? 'max-h-[500px] opacity-100' : 'max-h-0 opacity-0'
              }`}
            >
              <div className="mt-2 flex items-start gap-2 bg-[#212229] p-4 rounded-md">
                <img src={answerImage} alt="A" className="w-5 h-5 mt-1" />
                <div className="text-sm leading-relaxed break-keep text-left">{faq.answer}</div>
              </div>
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

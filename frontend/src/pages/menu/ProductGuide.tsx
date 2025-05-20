import { useNavigate } from 'react-router-dom';
import { ChevronLeft } from 'lucide-react';

import guideImage1 from '../../assets/menu/guide1.svg';
import guideImage2 from '../../assets/menu/guide2.svg';
import guideImage3 from '../../assets/menu/guide3.svg';

const ProductGuide = () => {
  const navigate = useNavigate();

  const handleFAQClick = () => navigate('/menu/faq')

  return (
    <div className="w-full min-h-screen bg-gradient-to-b from-[#2E2E37] to-[#1D1E23] relative overflow-x-hidden">
    {/* 상단 헤더 */}
      <header className="p-3 px-4 flex items-center gap-2 sticky top-0 z-10" onClick={() => navigate(-1)}>
        <button
          className="text-white bg-transparent border-none p-0"
        >
          <ChevronLeft size={20} />
        </button>
        <span className="text-lg font-bold text-[#767676]">메뉴</span>
      </header>

      {/* 설명 카드 */}
      <div className="absolute left-1/2 -translate-x-1/2 top-[60px] w-[330px] h-[630px] bg-[#373738] rounded-[10px] shadow-md px-5 py-6">
        <h2 className="text-white text-[15px] font-bold mb-4 text-left">제품을 처음 사용하시나요?</h2>

        {/* 1단계 */}
        <div className="absolute top-[60px] left-1/2 -translate-x-1/2 w-full max-w-[330px] flex justify-between items-start px-4">
          <div className="w-[140px] h-[140px] flex items-center justify-center animate-breath">
            <img src={guideImage1} alt="기기 연결" className="w-[120px] h-[120px]" />
          </div>
          <div className="ml-4 w-[164px] text-left mt-4">
            <h3 className="text-white font-bold text-[15px] mb-1">1단계 : 기기 연결</h3>
            <p className="text-white text-[12px] leading-snug whitespace-nowrap">
              앱에서 ‘기기 연결’ 버튼을 <br />눌러 주변 기기를 등록하세요.
            </p>
          </div>
        </div>

        {/* 2단계 */}
        <div className="absolute top-[225px] left-1/2 -translate-x-1/2 w-full max-w-[330px] flex justify-between items-start px-4">
          <div className="w-[140px] h-[140px] flex items-center justify-center animate-breath order-2">
            <img src={guideImage2} alt="청소 예약" className="w-[120px] h-[120px]" />
          </div>
          <div className="mr-4 w-[164px] text-right order-1 mt-4">
            <h3 className="text-white font-bold text-[15px] mb-1">2단계 : 청소 예약</h3>
            <p className="text-white text-[12px] leading-snug whitespace-nowrap">
              원하는 시간에 청소가 <br />완료되도록 타이머를 <br />설정해보세요.
            </p>
          </div>
        </div>

        {/* 3단계 */}
        <div className="absolute top-[390px] left-1/2 -translate-x-1/2 w-full max-w-[330px] flex justify-between items-start px-4">
          <div className="w-[140px] h-[140px] flex items-center justify-center animate-breath">
            <img src={guideImage3} alt="배터리 확인" className="w-[119px] h-[119px]" />
          </div>
          <div className="ml-4 w-[164px] text-left mt-4">
            <h3 className="text-white font-bold text-[15px] mb-1">3단계 : 배터리 확인</h3>
            <p className="text-white text-[12px] leading-snug whitespace-nowrap">
              메인 화면에서 배터리 잔량과 <br />충전 상태를 <br />확인할 수 있어요.
            </p>
          </div>
        </div>

        {/* FAQ */}
        <div className="absolute bottom-[30px] left-[24px]">
          <h4 className="text-white font-bold text-[15px] mb-1">자주 묻는 질문</h4>
          <span className="text-[#66A1F7] text-[12px] underline block text-left" onClick={handleFAQClick}>FAQ 보러가기</span>
        </div>
      </div>
    </div>
  );
};

export default ProductGuide;

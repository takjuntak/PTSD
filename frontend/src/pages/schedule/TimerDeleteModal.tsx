import React from 'react';

interface TimerDeleteModalProps {
  open: boolean;
  onClose: () => void;
  onConfirm: () => void;
}

const TimerDeleteModal: React.FC<TimerDeleteModalProps> = ({ open, onClose, onConfirm }) => {
  if (!open) return null;

  return (
    <div className="fixed inset-0 z-[9999] bg-black/80 flex justify-center items-center">
      <div className="bg-[#2E2E37] w-[312px] h-[280px] rounded-[20px] shadow-lg flex flex-col items-center justify-center text-center px-5">
        {/* 아이콘 */}
        <div className="text-[48px] mb-3">⚠️</div>

        {/* 타이틀 */}
        <h2 className="text-white text-[18px] font-bold mb-1">예약을 취소하시겠습니까?</h2>

        {/* 설명 */}
        <p className="text-[#CCCCCC] text-[14px] font-light leading-snug mb-6">
          설정된 청소 시간이 삭제됩니다.
          <br />
          취소 후에는 다시 예약을 설정해야 합니다.
        </p>

        {/* 버튼 그룹 */}
        <div className="flex gap-3 w-full px-4">
          <button
            className="flex-1 h-[40px] rounded-[10px] bg-white text-[#EE6163] border border-[#EE6163] text-[14px] font-semibold shadow-md"
            onClick={onClose}
          >
            닫기
          </button>
          <button
            className="flex-1 h-[40px] rounded-[10px] bg-[#EE6163] text-white text-[14px] font-semibold shadow-md"
            onClick={onConfirm}
          >
            예약 취소
          </button>
        </div>
      </div>
    </div>
  );
};

export default TimerDeleteModal;

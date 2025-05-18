import React from 'react';
import deleteAccountImage from '../../assets/menu/delete-account.svg'

interface DeleteAccountModalProps {
  open: boolean;
  onClose: () => void;
  onDelete: () => void;
  icon?: string;
}

const DeleteAccountModal: React.FC<DeleteAccountModalProps> = ({ open, onClose, onDelete }) => {
  if (!open) return null;

  return (
    <div className="fixed inset-0 bg-[#2E2E37dd] z-50 flex justify-center items-center">
      <div className="w-[300px] h-[280px] bg-[#373738] rounded-[10px] shadow-[0_0_4px_2px_rgba(0,0,0,0.25)] flex flex-col items-center justify-center text-center px-6">
        <img src={deleteAccountImage} alt="계정삭제 아이콘" className="w-[60px] h-[60px] mb-4" />
        <h2 className="text-[18px] font-semibold text-white mb-1">계정을 삭제하시겠어요?</h2>
        <p className="text-[14px] font-light text-white leading-[1.4] mb-6">
          계정을 삭제하면 모든 정보가 삭제되며<br />복구할 수 없습니다.
        </p>
        <div className="flex justify-between gap-3 w-full px-[18px]">
          <button
            onClick={onClose}
            className="flex-1 h-[40px] bg-white text-[#EE6163] border border-[#EE6163] rounded-[10px] text-[14px] font-semibold shadow"
          >
            취소
          </button>
          <button
            onClick={onDelete}
            className="flex-1 h-[40px] bg-[#EE6163] text-white rounded-[10px] text-[14px] font-semibold shadow"
          >
            삭제
          </button>
        </div>
      </div>
    </div>
  );
};

export default DeleteAccountModal;
// src/components/common/DeleteConfirmModal.tsx
import React from 'react';
import Portal from './Portal';

interface DeleteConfirmModalProps {
  isOpen: boolean;
  onClose: () => void;
  onConfirm: () => void;
  title: string;
  message: string;
  cancelText?: string;
  confirmText?: string;
}

const DeleteConfirmModal: React.FC<DeleteConfirmModalProps> = ({
  isOpen,
  onClose,
  onConfirm,
  title,
  message,
  cancelText = "취소",
  confirmText = "기기 제거"
}) => {
  if (!isOpen) return null;

  return (
    <Portal>
      <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 px-4">
        <div className="bg-app-dark rounded-lg w-full max-w-xs overflow-hidden">
          <div className="p-6 flex flex-col items-center">
            <div className="w-16 h-16 bg-red-500 rounded-full flex items-center justify-center mb-4">
              <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="white" strokeWidth="2">
                <path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"></path>
                <line x1="12" y1="9" x2="12" y2="13"></line>
                <line x1="12" y1="17" x2="12.01" y2="17"></line>
              </svg>
            </div>
            <h3 className="text-xl font-bold text-white mb-2">{title}</h3>
            <p className="text-sm text-center text-gray-400 mb-6">{message}</p>
            <div className="flex w-full gap-3">
              <button 
                onClick={onClose} 
                className="flex-1 py-3 px-4 bg-white text-gray-800 rounded-lg font-medium"
              >
                {cancelText}
              </button>
              <button 
                onClick={onConfirm} 
                className="flex-1 py-3 px-4 bg-red-500 text-white rounded-lg font-medium"
              >
                {confirmText}
              </button>
            </div>
          </div>
        </div>
      </div>
    </Portal>
  );
};

export default DeleteConfirmModal;
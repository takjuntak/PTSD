// src/components/common/ErrorMessage.tsx
import { AlertCircle, XCircle } from 'lucide-react';

interface ErrorMessageProps {
  message: string;
  onClose?: () => void;
  variant?: 'error' | 'warning';
}

/**
 * 에러나 경고 메시지를 표시하는 재사용 가능한 컴포넌트
 */
const ErrorMessage: React.FC<ErrorMessageProps> = ({ 
  message, 
  onClose,
  variant = 'error'
}) => {
  const bgColor = variant === 'error' ? 'bg-red-900' : 'bg-yellow-900';
  const borderColor = variant === 'error' ? 'border-red-700' : 'border-yellow-700';
  const textColor = variant === 'error' ? 'text-red-300' : 'text-yellow-300';
  const Icon = variant === 'error' ? XCircle : AlertCircle;

  return (
    <div className={`flex items-center justify-between p-3 rounded-lg mb-4 ${bgColor} border ${borderColor}`}>
      <div className="flex items-center gap-2">
        <Icon size={18} className={textColor} />
        <span className={`text-sm ${textColor}`}>{message}</span>
      </div>
      {onClose && (
        <button 
          onClick={onClose}
          className={`${textColor} hover:opacity-80`}
        >
          <XCircle size={16} />
        </button>
      )}
    </div>
  );
};

export default ErrorMessage;
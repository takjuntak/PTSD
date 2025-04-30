// src/components/status/AlertCard.tsx
import { AlertCircle } from 'lucide-react';

interface AlertCardProps {
  message: string;
}

const AlertCard: React.FC<AlertCardProps> = ({ message }) => {
  return (
    <div className="bg-app-card rounded-lg p-4">
      <div className="flex items-center">
        <AlertCircle size={24} className="text-app-warning" />
        <span className="text-lg font-bold ml-2">알림</span>
      </div>
      
      <div className="text-white text-left mt-2">
        {message}
      </div>
    </div>
  );
};

export default AlertCard;
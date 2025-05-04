// src/components/status/OperationCard.tsx
import { Brush } from 'lucide-react';

interface OperationCardProps {
  status: string;
  estimatedTime: string;
}

const OperationCard: React.FC<OperationCardProps> = ({ 
  status,
  estimatedTime 
}) => {
  return (
    <div className="bg-app-card rounded-lg p-4">
      <div className="flex items-center">
        <Brush size={24} className="text-app-blue" />
        <span className="text-lg font-bold ml-2">청소 중</span>
      </div>
      
      <div className="text-white text-left mt-2">
        <div>{status}</div>
        <div className="text-gray-400 text-sm mt-1">{estimatedTime}</div>
      </div>
    </div>
  );
};

export default OperationCard;
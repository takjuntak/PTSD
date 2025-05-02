// src/components/status/LocationCard.tsx
import { MapPin } from 'lucide-react';

interface LocationCardProps {
  location: string;
}

const LocationCard: React.FC<LocationCardProps> = ({ location }) => {
  return (
    <div className="bg-app-card rounded-lg p-4">
      <div className="flex items-center">
        <MapPin size={24} className="text-app-blue" />
        <span className="text-lg font-bold ml-2">현재 위치</span>
      </div>
      
      <div className="text-white text-left mt-2">
        {location}
      </div>
    </div>
  );
};

export default LocationCard;
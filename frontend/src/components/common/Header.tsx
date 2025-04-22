// src/components/common/Header.tsx
import { useState } from 'react';
import { ChevronDown, Settings, Check } from 'lucide-react';

interface HeaderProps {
  title: string;
}

const Header: React.FC<HeaderProps> = ({ title }) => {
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);

  const toggleDropdown = () => {
    setIsDropdownOpen(prev => !prev);
  };

  return (
    <header className="py-4 px-6 bg-app-dark text-white">
      <div className="relative inline-block">
        <button
          className="flex items-center gap-2 text-white bg-transparent border-none focus:outline-none"
          onClick={toggleDropdown}
        >
          <span className="text-xl font-bold">{title}</span>
          <ChevronDown size={20} />
        </button>

        {isDropdownOpen && (
          <div className="absolute right-0 mt-2 w-52 bg-app-card rounded-md shadow-lg z-10 py-2">
            <ul>
              <li className="px-4 py-2 flex items-center justify-between text-app-blue font-semibold">
                <span>{title}</span>
                <Check size={16} />
              </li>
              <div className="border-t border-dashed border-gray-500 my-1" />
              <li>
                <button
                  className="w-full text-left px-4 py-2 text-white hover:bg-app-blue transition-colors flex items-center gap-2 focus:outline-none"
                  onClick={() => {
                    console.log('장소 관리 클릭');
                    setIsDropdownOpen(false);
                  }}
                >
                  <Settings size={16} />
                  <span>장소 관리</span>
                </button>
              </li>
            </ul>
          </div>
        )}
      </div>
    </header>
  );
};

export default Header;

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
      <div
        style={{
          position: 'relative',
          textAlign: 'left',
          width: '100%',
          zIndex: 1000
        }}
      >
        <button
          className="flex items-center gap-2 text-white bg-transparent border-none focus:outline-none"
          style={{ margin: '0', padding: '0' }}
          onClick={toggleDropdown}
        >
          <span className="text-xl font-bold">{title}</span>
          <ChevronDown size={20} />
        </button>

        <div
          style={{
            position: 'absolute',
            top: '100%',
            left: '0',
            width: '208px',
            backgroundColor: '#2A2A2A',
            borderRadius: '6px',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            padding: '8px 0',
            marginTop: '8px',
            zIndex: 1000,
            display: isDropdownOpen ? 'block' : 'none',
          }}
        >
          <div
            style={{
              padding: '8px 16px',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'space-between',
              color: '#0088FF',
              fontWeight: 600
            }}
          >
            <span>{title}</span>
            <Check size={16} />
          </div>
          
          <div 
            style={{
              borderTop: '1px dashed #444',
              margin: '4px 0'
            }}
          />
          
          <button
            style={{
              width: '100%',
              textAlign: 'left',
              padding: '8px 16px',
              color: 'white',
              backgroundColor: 'transparent',
              border: 'none',
              display: 'flex',
              alignItems: 'center',
              gap: '8px',
              cursor: 'pointer',
              transition: 'background-color 0.2s'
            }}
            onMouseOver={(e) => {
              e.currentTarget.style.backgroundColor = '#0088FF';
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.backgroundColor = 'transparent';
            }}
            onClick={() => {
              console.log('장소 관리 클릭');
              setIsDropdownOpen(false);
            }}
          >
            <Settings size={16} />
            <span>장소 관리</span>
          </button>
        </div>
      </div>
    </header>
  );
};

export default Header;
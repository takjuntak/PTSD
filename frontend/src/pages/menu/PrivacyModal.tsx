import React from 'react';

interface PrivacyModalProps {
  open: boolean;
  onClose: () => void;
  onAgree: () => void;
  title: string;
  subtitle: string;
  items: string[];
}

const PrivacyModal: React.FC<PrivacyModalProps> = ({ open, onClose, onAgree, title, subtitle, items }) => {
  if (!open) return null;

  const handleOverlayClick = () => onClose();
  const stopPropagation = (e: React.MouseEvent) => e.stopPropagation();

  const renderWithLineBreaks = (text: string) =>
    text.split('\n').map((line, i) => (
      <React.Fragment key={i}>
        {line}
        <br />
      </React.Fragment>
    ));

  return (
    <div
      className="fixed inset-0 bg-black bg-opacity-60 z-[9999] flex justify-center items-center"
      onClick={handleOverlayClick}
    >
      <div
        className="w-[320px] bg-[#373738] rounded-[10px] p-8 px-6 text-white shadow-[0_0_4px_2px_rgba(0,0,0,0.25)] text-center"
        onClick={stopPropagation}
      >
        {/* Title */}
        <h3 className="text-[15px] font-bold mb-8 text-left">{title}</h3>

        {/* Subtitle */}
        <h2 className="text-[20px] font-bold mb-8">{subtitle}</h2>

        {/* Text Body */}
        <div className="text-left text-[13px] max-h-[225px] overflow-y-auto break-words">
          {items.map((item, index) => {
            if (item.trim() === '') {
              return <div key={index} className="h-[10px]" />;
            } else if (item.startsWith('[')) {
              return (
                <p key={index} className="font-bold my-[6px] whitespace-pre-line">
                  {renderWithLineBreaks(item)}
                </p>
              );
            } else if (item.startsWith('-')) {
              return (
                <ul key={index} className="list-disc ml-5">
                  <li className="mb-[6px] leading-[1.5] whitespace-pre-line">
                    {renderWithLineBreaks(item.substring(1).trim())}
                  </li>
                </ul>
              );
            } else {
              return (
                <p key={index} className="mb-[6px] leading-[1.5] whitespace-pre-line">
                  {renderWithLineBreaks(item)}
                </p>
              );
            }
          })}
        </div>

        {/* Button */}
        <button
          className="w-full h-[40px] bg-[#617BEE] rounded-[10px] text-white font-bold text-[15px] mt-5"
          onClick={onAgree}
        >
          확인
        </button>
      </div>
    </div>
  );
};

export default PrivacyModal;

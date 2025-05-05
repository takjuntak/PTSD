import React from 'react';

interface SignupModalProps {
  open: boolean;
  onClose: () => void;
  onAgree: () => void;
  title: string;
  subtitle: string;
  items: string[];
}

export default function SignupModal({
  open,
  onClose,
  onAgree,
  title,
  subtitle,
  items,
}: SignupModalProps) {
  if (!open) return null;

  const handleOverlayClick = () => {
    onClose();
  };

  const stopPropagation = (e: React.MouseEvent) => {
    e.stopPropagation();
  };

  return (
    <div className="modal-overlay" onClick={handleOverlayClick}>
      <div className="modal-content" onClick={stopPropagation}>
        <h3>{title}</h3>
        <h2>{subtitle}</h2>
        <div className="modal-text">
          {items.map((item, index) => {
            if (item.trim() === '') {
              return <div key={index} style={{ height: '10px' }} />;
            } else if (item.startsWith('[')) {
              return <p key={index} className="modal-section-title">{item}</p>;
            } else if (item.startsWith('-')) {
              return (
                <ul key={index} className="modal-list">
                  <li>{item.substring(1).trim()}</li>
                </ul>
              );
            } else {
              return <p key={index} className="modal-paragraph">{item}</p>;
            }
          })}
        </div>
        <button className="modal-button" onClick={onAgree}>동의</button>
      </div>

      <style>{`
        .modal-overlay {
          position: fixed;
          top: 0;
          left: 0;
          width: 100%;
          height: 100%;
          background: rgba(0, 0, 0, 0.6);
          display: flex;
          justify-content: center;
          align-items: center;
          z-index: 9999;
        }

        .modal-content {
          width: 364px;
          background: #373738;
          border-radius: 10px;
          padding: 32px 24px;
          box-shadow: 0 0 4px 2px rgba(0, 0, 0, 0.25);
          color: white;
          text-align: center;
        }

        .modal-content h3 {
          font-size: 15px;
          font-weight: 700;
          margin-bottom: 32px;
          text-align: left;
        }

        .modal-content h2 {
          font-size: 20px;
          font-weight: 700;
          margin-bottom: 32px;
        }

        .modal-text {
          text-align: left;
          font-size: 13px;
        }

        .modal-section-title {
          font-weight: bold;
          margin-top: 16px;
          margin-bottom: 6px;
        }

        .modal-paragraph {
          margin-bottom: 6px;
          line-height: 1.5;
        }

        .modal-list {
          margin-left: 20px;
          padding-left: 0;
          list-style-type: disc;
        }

        .modal-list li {
          margin-bottom: 6px;
          line-height: 1.5;
        }

        .modal-button {
          width: 100%;
          height: 40px;
          background: #617BEE;
          border: none;
          border-radius: 10px;
          color: white;
          font-weight: 700;
          font-size: 15px;
          cursor: pointer;
          margin-top: 20px;
        }
      `}</style>
    </div>
  );
}

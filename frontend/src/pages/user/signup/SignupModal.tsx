import React from 'react';
import styles from './SignupModal.module.css';

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

  // ✅ 줄바꿈 처리 함수
  const renderWithLineBreaks = (text: string) => {
    return text.split('\n').map((line, i) => (
      <React.Fragment key={i}>
        {line}
        <br />
      </React.Fragment>
    ));
  };

  return (
    <div className={styles.modalOverlay} onClick={handleOverlayClick}>
      <div className={styles.modalContent} onClick={stopPropagation}>
        <h3 className={styles.title}>{title}</h3>
        <h2 className={styles.subtitle}>{subtitle}</h2>
        <div className={styles.modalText}>
          {items.map((item, index) => {
            if (item.trim() === '') {
              return <div key={index} style={{ height: '10px' }} />;
            } else if (item.startsWith('[')) {
              return (
                <p key={index} className={styles.modalSectionTitle}>
                  {renderWithLineBreaks(item)}
                </p>
              );
            } else if (item.startsWith('-')) {
              return (
                <ul key={index} className={styles.modalList}>
                  <li>{renderWithLineBreaks(item.substring(1).trim())}</li>
                </ul>
              );
            } else {
              return (
                <p key={index} className={styles.modalParagraph}>
                  {renderWithLineBreaks(item)}
                </p>
              );
            }
          })}
        </div>
        <button className={styles.modalButton} onClick={onAgree}>동의</button>
      </div>
    </div>
  );
}

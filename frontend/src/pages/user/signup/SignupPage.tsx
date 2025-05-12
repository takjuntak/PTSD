// SignupPage.tsx
import { useState, useEffect } from 'react';
import styles from './SignupPage.module.css';
import logoImage from '../../../assets/PTSD-logo-neon.png';
import eyeImage from '../../../assets/user/eye-outline.svg';
import hideImage from '../../../assets/user/hide-outline.svg';
import checkOn from '../../../assets/user/check-on.svg';
import checkOff from '../../../assets/user/check-off.svg';
import arrow from '../../../assets/user/arrow.svg';
import SignupModal from './SignupModal';
import axios from 'axios';
import { useNavigate } from 'react-router-dom';

export default function SignupPage() {
  const navigate = useNavigate();
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [password_confirm, setPasswordConfirm] = useState('');
  const [name, setName] = useState('');
  const [errorMessage, setErrorMessage] = useState('');

  const [allAgree, setAllAgree] = useState(false);
  const [agree1, setAgree1] = useState(false);
  const [agree2, setAgree2] = useState(false);
  const [agree3, setAgree3] = useState(false);
  const [agree4, setAgree4] = useState(false);

  const [modalOpen, setModalOpen] = useState(false);
  const [modalType, setModalType] = useState('');

  const togglePassword = () => setShowPassword(prev => !prev);
  const toggleConfirmPassword = () => setShowConfirmPassword(prev => !prev);

  useEffect(() => {
    const allChecked = agree1 && agree2 && agree3 && agree4;
    if (allAgree !== allChecked) setAllAgree(allChecked);
  }, [agree1, agree2, agree3, agree4]);

  const handleAllAgree = () => {
    const newValue = !allAgree;
    setAllAgree(newValue);
    setAgree1(newValue);
    setAgree2(newValue);
    setAgree3(newValue);
    setAgree4(newValue);
  };

  const isFormValid = () => email.includes('@') && password && password_confirm && name && agree1 && agree2 && agree3;

  const handleSignup = async (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    try {
      const res = await axios.post('https://k12d101.p.ssafy.io/api/auth/signup', {
        email, password, password_confirm, name
      });
      const token = res.data?.accessToken;
      if (token) localStorage.setItem('accessToken', token);
      navigate('/login');
    } catch (err: any) {
      const msg = err?.response?.data?.message || '회원가입 중 오류가 발생했습니다.';
      setErrorMessage(msg.includes('이미 존재') ? '이미 존재하는 이메일입니다.' : msg);
    }
  };

  const openModal = (type: string) => {
    setModalType(type);
    setModalOpen(true);
  };

  const handleAgree = () => {
    if (modalType === 'agree1') setAgree1(true);
    if (modalType === 'agree2') setAgree2(true);
    if (modalType === 'agree3') setAgree3(true);
    if (modalType === 'agree4') setAgree4(true);
    setModalOpen(false);
  };

  const getModalContent = () => {
    const common = (title: string, subtitle: string, items: string[]) => ({ title, subtitle, items });
    switch (modalType) {
      case 'agree1': return common('(필수) 서비스 이용약관 동의', '서비스 이용약관 동의', [
        '본 약관은 사용자가 PTSD 앱(이하 "서비스")을 이용함에 있어 회사와 사용자 간의 권리, 의무, 책임사항 및 기타 필요한 사항을 규정합니다.', '',
        '사용자는 서비스에 회원가임함으로써 본 약관의 내용을 모두 이해하고 동의한 것으로 간주됩니다.',
        '회사는 서비스의 내용, 제공 조건, 중단 또는 변경에 관한 권한을 가질 수 있습니다.',
        '사용자는 타인의 정보를 도용하거나 서비스 운영을 방해해서는 안 됩니다.',
        '※ 전체 약관은 [전체 이용약관 보기] 버튼을 통해 확인하실 수 있습니다.',
      ]);
      case 'agree2': return common('(필수) 개인정보 수집 및 이용 동의', '개인정보 수집 및 이용 동의', [
        '[ 수집 항목 ]', '이름, 이메일 주소, 비밀번호', '',
        '[ 수집 목적 ]', '회원 식별 및 인증, 고객 문의 응대',
        '서비스 제공 및 맞춤형 서비스 운영', '부정 이용 방지 및 서비스 안정성 확보', '',
        '[ 보유 및 이용 기간 ]', '회원 탈퇴 시 또는 수집 목적 달성 시까지',
      ]);
      case 'agree3': return common('(필수) 위치 정보 수집 및 이용 동의', '위치 정보 수집 및 이용 동의', [
        '[ 수집 항목 ]', '단말기의 GPS 또는 네트워크를 통해 수집된 위치 정보', '',
        '[ 수집 목적 ]', '로봇 청소 예약 및 위치 기반 자동 제어 서비스 제공',
        '청소 시작/종료 위치 설정 및 기록', '',
        '[ 보유 및 이용 기간 ]', '실시간 제공 목적 달성 시 즉시 파기'
      ]);
      case 'agree4': return common('(선택) 마케팅 정보 수신 동의', '마케팅 정보 수신 동의', [
        '[ 수신 정보 ]', '앱 내 알림(Push), 이메일, 문자(SMS)',
        '[ 수신 목적 ]', '신규 기능 안내', '이벤트, 할인, 캠페인 등의 프로모션 정보 제공'
      ]);
      default: return { title: '', subtitle: '', items: [] };
    }
  };

  return (
    <div className={styles.signupWrapper}>
      <div className={styles.signupContainer}>
        <div className={styles.logoContainer}>
          <img src={logoImage} alt="Logo" className={styles.logo} />
        </div>

        <form className={styles.formSection}>
          <label className={styles.label}>이메일 *</label>
          <div className={styles.inputWrapper}>
            <input
              className={styles.input}
              type="email"
              value={email}
              onChange={(e) => { setEmail(e.target.value); setErrorMessage(''); }}
              placeholder="ssafy@gmail.com"
            />
            {errorMessage && (
              <p className={styles.error}>{errorMessage}</p>
            )}
          </div>

          <label className={styles.label}>비밀번호 *</label>
          <div className={styles.passwordWrapper}>
            <input
              className={styles.input}
              type={showPassword ? 'password' : 'text'}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="비밀번호 입력"
            />
            <img src={showPassword ? hideImage : eyeImage} onClick={togglePassword} className={styles.eyeIcon} alt="toggle" />
          </div>
          {password && (
            <p
              className="password-check-message"
              style={{
                color: password.length < 8 ? '#EE6163' : '#617BEE',
                fontSize: '12px',
                marginTop: '4px',
                marginBottom: '8px',
                textAlign: 'left',
              }}
            >
              {password.length < 8
                ? '비밀번호는 최소 8자 이상이어야 합니다.'
                : '비밀번호 사용이 가능합니다.'}
            </p>
          )}

          <label className={styles.label}>비밀번호 확인 *</label>
          <div className={styles.passwordWrapper}>
            <input
              className={styles.input}
              type={showConfirmPassword ? 'password' : 'text'}
              value={password_confirm}
              onChange={(e) => setPasswordConfirm(e.target.value)}
              placeholder="비밀번호 확인"
            />
            <img src={showConfirmPassword ? hideImage : eyeImage} onClick={toggleConfirmPassword} className={styles.eyeIcon} alt="toggle" />
          </div>
          {password_confirm && (
            <p 
              className={styles.passwordCheck} 
              style={{ 
                color: password === password_confirm ? '#617BEE' : '#EE6163',
                fontSize: '12px',
                marginTop: '4px',
                marginBottom: '8px',
                textAlign: 'left',
                }}>
              {password === password_confirm ? '비밀번호가 일치합니다.' : '비밀번호가 일치하지 않습니다.'}
            </p>
          )}

          <label className={styles.label}>이름 *</label>
          <input
            className={styles.input}
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="이름을 입력하세요"
          />

          <hr className={styles.divider} />

          <div className={styles.agreementSection}>
            <p>약관 동의</p>
            <div className={styles.allAgreeBox} onClick={handleAllAgree}>
              <img src={allAgree ? checkOn : checkOff} alt="all-agree" />
              <span>모두 동의</span>
            </div>

            {[{ key: 'agree1', text: '서비스 이용약관 동의', value: agree1, setValue: setAgree1 },
              { key: 'agree2', text: '개인정보 수집 및 이용 동의', value: agree2, setValue: setAgree2 },
              { key: 'agree3', text: '위치 정보 수집 및 이용 동의', value: agree3, setValue: setAgree3 },
              { key: 'agree4', text: '마케팅 정보 수신 동의', value: agree4, setValue: setAgree4 }]
              .map(({ key, text, value, setValue }) => (
                <label className={styles.checkboxLabel} key={key}>
                  <img src={value ? checkOn : checkOff} onClick={() => setValue(!value)} alt="check" />
                  <span onClick={() => openModal(key)}>{`(${key === 'agree4' ? '선택' : '필수'}) ${text}`}</span>
                  <img src={arrow} className={styles.arrowIcon} alt=">" onClick={() => openModal(key)} />
                </label>
              ))}
          </div>

          <button
            className={`${styles.submitButton} ${isFormValid() ? styles.submitButtonActive : ''}`}
            disabled={!isFormValid()}
            onClick={handleSignup}
          >
            회원가입
          </button>
        </form>
      </div>

      <SignupModal
        open={modalOpen}
        onClose={() => setModalOpen(false)}
        onAgree={handleAgree}
        {...getModalContent()}
      />
    </div>
  );
}

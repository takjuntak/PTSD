import { useState, useEffect } from 'react';
import logoImage from '../../assets/PTSD-logo-neon.png';
import eyeImage from '../../assets/user/eye-outline.svg';
import hideImage from '../../assets/user/hide-outline.svg';
import checkOn from '../../assets/user/check-on.svg';
import checkOff from '../../assets/user/check-off.svg';
import arrow from '../../assets/user/arrow.svg';
import SignupModal from './SignupModal'; // 모달 컴포넌트

export default function SignupPage() {
  const [showPassword, setShowPassword] = useState(false);
  const [showConfirmPassword, setShowConfirmPassword] = useState(false);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirm, setConfirm] = useState('');
  const [name, setName] = useState('');

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
    if (allAgree !== allChecked) {
      setAllAgree(allChecked);
    }
  }, [agree1, agree2, agree3, agree4]);

  const handleAllAgree = () => {
    const newValue = !allAgree;
    setAllAgree(newValue);
    setAgree1(newValue);
    setAgree2(newValue);
    setAgree3(newValue);
    setAgree4(newValue);
  };

  const isFormValid = () =>
    email.includes('@') && password && confirm && name &&
    agree1 && agree2 && agree3;

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
    switch (modalType) {
      case 'agree1':
        return {
          title: '(필수) 서비스 이용약관 동의',
          subtitle: '서비스 이용약관 동의',
          items: [
            '본 약관은 사용자가 PTSD 앱(이하 "서비스")을 이용함에 있어 회사와 사용자 간의 권리, 의무, 책임사항 및 기타 필요한 사항을 규정합니다.',
            '',
            '사용자는 서비스에 회원가임함으로써 본 약관의 내용을 모두 이해하고 동의한 것으로 간주됩니다.',
            '회사는 서비스의 내용, 제공 조건, 중단 또는 변경에 관한 권한을 가질 수 있습니다.',
            '사용자는 타인의 정보를 도용하거나 서비스 운영을 방해해서는 안 됩니다.',
            '회사는 서비스 운영과 관련하여 필요한 경우 공지사항 등을 통해 사용자에게 안내할 수 있습니다.',
            '',
            '※ 전체 약관은 [전체 이용약관 보기] 버튼을 통해 확인하실 수 있습니다.'
          ],
        };
      case 'agree2':
        return {
          title: '(필수) 개인정보 수집 및 이용 동의',
          subtitle: '개인정보 수집 및 이용 동의',
          items: [
            '[ 수집 항목 ]',
            '이름, 이메일 주소, 비밀번호',
            '',
            '[ 수집 목적 ]',
            '회원 식별 및 인증, 고객 문의 응대',
            '서비스 제공 및 맞춤형 서비스 운영',
            '부정 이용 방지 및 서비스 안정성 확보',
            '',
            '[ 보유 및 이용 기간 ]',
            '회원 탈퇴 시 또는 수집 목적 달성 시까지',
            '단, 관계 법령에 의해 일정 기간 보존이 필요한 경우, 해당 기간동안 보관'
          ],
        };
      case 'agree3':
        return {
          title: '(필수) 위치 정보 수집 및 이용 동의',
          subtitle: '위치 정보 수집 및 이용 동의',
          items: [
            '[ 수집 항목 ]',
            '단말기의 GPS 또는 네트워크를 통해 수집된 위치 정보',
            '',
            '[ 수집 목적 ]',
            '로봇 청소 예약 및 위치 기반 자동 제어 서비스 제공',
            '청소 시작/종료 위치 설정 및 기록',
            '',
            '[ 보유 및 이용 기간 ]',
            '실시간 제공 목적 달성 시 즉시 파기',
            '단, 서비스 내 사용 기록이 필요한 경우 일부 데이터는 익명화하여 통계 활용',
            '',
            '※ 사용자는 단말기 설정에서 언제든지 위치 정보 수집을 비활성화할 수 있습니다.'
          ],
        };
      case 'agree4':
        return {
          title: '(선택) 마케팅 정보 수신 동의',
          subtitle: '마케팅 정보 수신 동의',
          items: [
            '[ 수신 정보 ]',
            '앱 내 알림(Push), 이메일, 문자(SMS) 등을 통한 마케팅 메시지',
            '',
            '[ 수신 목적 ]',
            '신규 기능 안내',
            '이벤트, 할인, 캠페인 등의 프로모션 정보 제공',
            '',
            '[ 거부 및 철회 안내 ]',
            '사용자는 언제든지 수신 거부를 설정할 수 있으며, 앱 설정 또는 고객센터를 통해 철회 가능합니다.'
          ],
        };
      default:
        return { title: '', subtitle: '', items: [] };
    }
  };

  return (
    <div className="signup-wrapper">
      <div className="signup-container">
        <div className="logo-container">
          <img src={logoImage} alt="Logo" className="logo" />
        </div>

        <form className="form-section">
          <label className="label">이메일 *</label>
          <input
            className="input"
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="ssafy123@gmail.com"
          />

          <label className="label">비밀번호 *</label>
          <div className="password-wrapper">
            <input
              className="input"
              type={showPassword ? 'password' : 'text'}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="비밀번호 입력"
            />
            <img
              src={showPassword ? hideImage : eyeImage}
              onClick={togglePassword}
              className="eye-icon"
              alt="toggle"
            />
          </div>

          <label className="label">비밀번호 확인 *</label>
          <div className="password-wrapper">
            <input
              className="input"
              type={showConfirmPassword ? 'password' : 'text'}
              value={confirm}
              onChange={(e) => setConfirm(e.target.value)}
              placeholder="비밀번호 확인"
            />
            <img
              src={showConfirmPassword ? hideImage : eyeImage}
              onClick={toggleConfirmPassword}
              className="eye-icon"
              alt="toggle"
            />
          </div>

          {confirm && (
            <p
              className="password-check-message"
              style={{
                color: password === confirm ? '#617BEE' : '#EE6163',
                fontSize: '12px',
                marginTop: '4px',
                marginBottom: '8px',
                textAlign: 'left'
              }}
            >
              {password === confirm
                ? '비밀번호가 일치합니다.'
                : '비밀번호가 일치하지 않습니다.'}
            </p>
          )}

          <label className="label">이름 *</label>
          <input
            className="input"
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="이름을 입력하세요"
          />

          <hr className="divider" />

          <div className="agreement-section">
            <p>약관 동의</p>

            <div className="all-agree-box" onClick={handleAllAgree}>
              <img src={allAgree ? checkOn : checkOff} alt="all-agree" />
              <span>모두 동의</span>
            </div>

            <label className="checkbox-label">
              <img src={agree1 ? checkOn : checkOff} onClick={() => setAgree1(!agree1)} alt="check" />
              <span onClick={() => openModal('agree1')} style={{ flex: 1 }}>
                (필수) 서비스 이용약관 동의
              </span>
              <img src={arrow} className="arrow-icon" alt=">" onClick={() => openModal('agree1')} />
            </label>

            <label className="checkbox-label">
              <img src={agree2 ? checkOn : checkOff} onClick={() => setAgree2(!agree2)} alt="check" />
              <span onClick={() => openModal('agree2')} style={{ flex: 1 }}>
                (필수) 개인정보 수집 및 이용 동의
              </span>
              <img src={arrow} className="arrow-icon" alt=">" onClick={() => openModal('agree2')} />
            </label>

            <label className="checkbox-label">
              <img src={agree3 ? checkOn : checkOff} onClick={() => setAgree3(!agree3)} alt="check" />
              <span onClick={() => openModal('agree3')} style={{ flex: 1 }}>
                (필수) 위치 정보 수집 및 이용 동의
              </span>
              <img src={arrow} className="arrow-icon" alt=">" onClick={() => openModal('agree3')} />
            </label>

            <label className="checkbox-label">
              <img src={agree4 ? checkOn : checkOff} onClick={() => setAgree4(!agree4)} alt="check" />
              <span onClick={() => openModal('agree4')} style={{ flex: 1 }}>
                (선택) 마케팅 정보 수신 동의
              </span>
              <img src={arrow} className="arrow-icon" alt=">" onClick={() => openModal('agree4')} />
            </label>
          </div>

          <button
            className={`submit-button ${isFormValid() ? 'active' : ''}`}
            disabled={!isFormValid()}
          >
            회원가입
          </button>
        </form>
      </div>

      <SignupModal
        open={modalOpen}
        onClose={() => setModalOpen(false)}
        onAgree={handleAgree}
        title={getModalContent().title}
        subtitle={getModalContent().subtitle}
        items={getModalContent().items}
      />
 
      <style>{`
        html, body {
          margin: 0;
          padding: 0;
          height: 100%;
          background: linear-gradient(180deg, #2E2E37 0%, #1D1E23 100%);
        }

        .signup-wrapper {
          display: flex;
          justify-content: center;
          height: 100vh;
          overflow-y: auto;        
          padding-bottom: 120px;  
        }

        .signup-container {
          width: 100%;
          max-width: 355px;
          height: auto;
          font-family: 'Montserrat', sans-serif;
          background: transparent;
          box-sizing: border-box;
          min-height: 100vh;  
          padding-bottom: 100px;   
        }

        .logo-container {
          display: flex;
          justify-content: center;
          margin-top: 48px;
          filter: drop-shadow(0px 0px 50px #1346B8);
        }

        .logo {
          width: 320px;
          height: auto;
          padding: 0 !important;
        }

        .form-section {
          width: 100%;
          padding-bottom: 120px; 

        }

        .label {
          display: block;
          margin: 16px 0 8px;
          font-weight: 700;
          font-size: 15px;
          text-align: left;
        }

        .input {
          width: 100%;
          height: 50px;
          padding: 0 12px;
          background: #212228;
          border: 1px solid #767676;
          border-radius: 10px;
          color: white;
          font-size: 12px;
        }

        .password-wrapper {
          position: relative;
        }

        .eye-icon {
          position: absolute;
          right: 12px;
          top: 50%;
          transform: translateY(-50%);
          width: 24px;
          height: 24px;
          cursor: pointer;
        }

        .divider {
          border: none;
          border-top: 1px solid #767676;
          margin: 40px 0;
        }

        .all-agree-box {
          display: flex;
          align-items: center;
          width: 100%;
          height: 50px;
          gap: 16px;
          background-color: #212228;
          border-radius: 10px;
          padding: 0 16px;
          margin-bottom: 10px;
          border: 1px solid #767676;
          cursor: pointer;
          font-weight: 700;
          color: white;
          font-size: 15px;
        }

        .agreement-section {
          margin-top: 40px;
          font-size: 15px;
        }

        .agreement-section p {
          margin-bottom: 8px;
          font-weight: 700;
          text-align: left;
        }

        .agreement-section label {
          display: flex;
          align-items: center;
          gap: 8px;
          margin: 4px 0;
        }

        .checkbox-label {
          display: flex;
          align-items: center;
          width: 100%;
          height: 50px;
          padding: 0 16px;
          gap: 16px;
          background-color: transparent;
          color: white;
          font-size: 13px;
          border-radius: 10px;
          margin-bottom: 10px;
          position: relative;
        }

        .checkbox-label img:first-child {
          width: 20px;
          height: 20px;
          cursor: pointer;
        }

        .checkbox-label .arrow-icon {
          position: absolute;
          right: 16px;
          width: 11px;
          height: 22px;
        }

        .checkbox-label span {
          flex: 1;
          text-align: left;    
          display: flex;
          align-items: center;
          font-size: 13px;
          padding-left: 4px;   
        }


        .checkbox-row {
          display: flex;
          align-itmes: center;
          gap: 16px;
          margin-bottom: 10px;
          padding-left: 4px;
          font-size: 12px;
          color: white;
          position: relative;
        }
        
        .checkbox-row span {
          flex: 1;
        }
        
        .arrow-icon {
          width: 11px;
          height: 22px;
          margin-left: 41px;
        }

        .checkbox-label img {
          width: 20px;
          height: 20px;
          cursor: pointer;
        }

        .submit-button {
          width: 100%;
          height: 40px;
          margin-top: 24px;
          background: #DADADA;
          color: #9C9B9B;
          font-weight: 700;
          font-size: 15px;
          border: none;
          border-radius: 10px;
          cursor: not-allowed;
          box-shadow: 2px 2px 2px rgba(0, 0, 0, 0.25);
          transition: background 0.2s, color 0.2s;
        }

        .submit-button.active {
          background: #617BEE;
          color: white;
          cursor: pointer;
        }
      `}</style>
    </div>
  );
}

import { useState } from 'react'
import { useNavigate } from 'react-router-dom'
import logoImage from '../../assets/PTSD-logo-neon.png'
import eyeImage from '../../assets/user/eye-outline.svg'
import hideImage from '../../assets/user/hide-outline.svg'

export default function LoginPage() {
  const [showPassword, setShowPassword] = useState(false)
  const [email, setEmail] = useState('')
  const [password, setPassword] = useState('')
  const navigate = useNavigate()

  const togglePassword = () => {
    setShowPassword(prev => !prev)
  }

  const isButtonActive = email.includes('@') && password.length > 0

  const handleSignupClick = () => {
    navigate('/signup') // 경로 이동
  }

  return (
    <div className="outer-wrapper">
      <div className="login-container">
        <div className="logo-container">
          <img src={logoImage} alt="Logo" className="logo-image" />
        </div>

        <div className="form-section">
          <label className="label">이메일</label>
          <div className="input-wrapper">
            <input
              type="email"
              className="input-box"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
            />
          </div>

          <label className="label">비밀번호</label>
          <div className="input-wrapper password-wrapper">
            <input
              type={showPassword ? 'text' : 'password'}
              className="input-box"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
            />
            <img
              src={showPassword ? eyeImage : hideImage}
              alt={showPassword ? 'Hide password' : 'Show password'}
              className="icon"
              onClick={togglePassword}
            />
          </div>

          <button className={`login-button ${isButtonActive ? 'active' : ''}`} disabled={!isButtonActive}>
            로그인
          </button>

          <p className="not-member">아직 회원이 아니신가요?</p>
          <p className="signup" onClick={handleSignupClick}>회원가입</p>
        </div>
      </div>

      <style>{`
        html, body {
          margin: 0;
          padding: 0;
          height: 100%;
          background: linear-gradient(180deg, #2E2E37 0%, #1D1E23 100%) !important;
        }

        .outer-wrapper {
          display: flex;
          justify-content: center;
          height: 100vh;
          padding-bottom: 120px;  
        }

        .login-container {
          width: 100%;
          max-width: 355px;
          height: auto;
          font-family: 'Montserrat', sans-serif;
          overflow-y: auto;
          background: transparent;
          box-sizing: border-box;
        }

        .logo-container {
          display: flex;
          justify-content: center;
          margin-top: 48px;
          filter: drop-shadow(0px 0px 50px #1346B8);
        }

        .logo-image {
          width: 320px;
          height: auto;
        }

        .form-section {
          margin-top: 32px;
        }

        .label {
          color: white;
          font-size: 15px;
          font-weight: 700;
          margin-top: 20px;
          margin-bottom: 8px;
          display: block;
          text-align: left;
        }

        .input-wrapper {
          margin-bottom: 16px;
        }

        .input-box {
          width: 100%;
          height: 50px;
          padding: 0 12px;
          background: #212228;
          border: 1px solid #767676;
          border-radius: 10px;
          color: white;
          font-size: 12px;
          box-sizing: border-box;
        }

        .password-wrapper {
          position: relative;
        }

        .icon {
          position: absolute;
          right: 12px;
          top: 50%;
          transform: translateY(-50%);
          width: 24px;
          height: 24px;
          cursor: pointer;
        }

        .login-button {
          width: 95%;
          height: 40px;
          margin-top: 20px;
          background: #DADADA;
          color: #9C9B9B;
          font-size: 15px;
          font-weight: 700;
          border-radius: 10px;
          box-shadow: 2px 2px 2px rgba(0, 0, 0, 0.25);
          border: none;
          cursor: not-allowed;
          transition: background 0.2s, color 0.2s;
        }

        .login-button.active {
          background: #617BEE;
          color: #FFFFFF;
          cursor: pointer;
        }

        .not-member {
          text-align: center;
          margin-top: 30px;
          font-size: 12px;
          color: #767676;
        }

        .signup {
          text-align: center;
          font-size: 12px;
          color: #617BEE;
          text-decoration: underline;
          margin-top: 5px;
          cursor: pointer;
        }
      `}</style>
    </div>
  )
}

import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import styles from './LoginPage.module.css';
import logoImage from '../../../assets/PTSD-logo-neon.png';
import eyeImage from '../../../assets/user/eye-outline.svg';
import hideImage from '../../../assets/user/hide-outline.svg';
import axios from 'axios';

export default function LoginPage() {
  const [showPassword, setShowPassword] = useState(false);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [errorMessage, setErrorMessage] = useState('');
  const navigate = useNavigate();

  const handleLogin = async () => {
    try {
      const response = await axios.post('http://127.0.0.1:8000/api/auth/login', {
        email,
        password,
      });
      const token = response.data.accessToken;
      if (token) {
        localStorage.setItem('accessToken', token);
        navigate('/');
      }
    } catch (error: any) {
      if (error.response?.status === 401) {
        setErrorMessage('이메일 또는 비밀번호가 올바르지 않습니다.');
      } else {
        setErrorMessage('로그인 중 오류가 발생했습니다.');
      }
    }
  };

  const togglePassword = () => {
    setShowPassword(prev => !prev);
  };

  const isButtonActive = email.includes('@') && password.length > 0;

  const handleSignupClick = () => {
    navigate('/signup');
  };

  return (
    <div className={styles.outerWrapper}>
      <div className={styles.loginContainer}>
        <div className={styles.logoContainer}>
          <img src={logoImage} alt="Logo" className={styles.logoImage} />
        </div>

        <div className={styles.formSection}>
          <label className={styles.label}>이메일</label>
          <div className={styles.inputWrapper}>
            <input
              type="email"
              className={styles.inputBox}
              value={email}
              onChange={(e) => setEmail(e.target.value)}
            />
          </div>

          <label className={styles.label}>비밀번호</label>
          <div className={`${styles.inputWrapper} ${styles.passwordWrapper}`}>
            <input
              type={showPassword ? 'text' : 'password'}
              className={styles.inputBox}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
            />
            <img
              src={showPassword ? eyeImage : hideImage}
              alt={showPassword ? 'Hide password' : 'Show password'}
              className={styles.icon}
              onClick={togglePassword}
            />
          </div>

          {errorMessage && (
            <p className={styles.error}>{errorMessage}</p>
          )}

          <button
            className={`${styles.loginButton} ${isButtonActive ? styles.active : ''}`}
            disabled={!isButtonActive}
            onClick={handleLogin}
          >
            로그인
          </button>

          <p className={styles.notMember}>아직 회원이 아니신가요?</p>
          <p className={styles.signup} onClick={handleSignupClick}>회원가입</p>
        </div>
      </div>
    </div>
  );
}
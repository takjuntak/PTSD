// src/App.tsx
import { BrowserRouter as Router, Routes, Route, useLocation } from 'react-router-dom';
import { AnimatePresence, motion } from 'framer-motion';

import ProtectedLayout from './ProtectedLayout';

import './App.css';
import MainPage from './pages/MainPage';
import SchedulePage from './pages/schedule/SchedulePage';
import NavigationBar from './components/common/NavigationBar';
import Header from './components/common/Header';
import TimeSelectPage from './pages/schedule/TimeSelectPage';
import ControlPage from './pages/ControlPage';
import PlayPage from './pages/PlayPage';
import RobotControlPage from './pages/RobotControlPage';
import MenuPage from './pages/menu/MenuPage';
import DeviceConnectPage from './pages/DeviceConnectPage';
import LoginPage from './pages/user/login/LoginPage';
import SignupPage from './pages/user/signup/SignupPage';
import AlarmPage from './pages/AlarmPage';
import TimerSelectPage from './pages/schedule/TimerSelectPage';
import LocationPage from './pages/LocationPage';
import ProductGuide from './pages/menu/ProductGuide';
import FAQPage from './pages/menu/FAQPage';
import PrivacyPage from './pages/menu/PrivacyPage';

import useScrollToTop from './hooks/useScrollToTop';

function AppContent() {
  useScrollToTop();

  const location = useLocation();
  const hideNavPaths = ['/login', '/signup'];
  const hideHeaderPaths = ['/login', '/signup', '/schedule'];
  const shouldHideNav = hideNavPaths.includes(location.pathname);
  const shouldHideHeader = hideHeaderPaths.includes(location.pathname);

  return (
    <div className="flex flex-col text-white w-full h-screen overflow-hidden">
      {!shouldHideHeader && (
        <div className="shrink-0 h-[36px]">
          <Header />
        </div>
      )}

      <div className="flex-1 w-full overflow-y-auto main-scroll-container" style={{ paddingTop: !shouldHideHeader ? 10 : 0 }}>
        <AnimatePresence mode="wait">
          <motion.div
            key={location.pathname}
            initial={{ opacity: 0, y: 10 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -10 }}
            transition={{ duration: 0.25 }}
          >
            <Routes location={location}>
              {/* 공개 라우트 */}
              <Route path="/login" element={<LoginPage />} />
              <Route path="/signup" element={<SignupPage />} />

              {/* 보호된 라우트 그룹 */}
              <Route element={<ProtectedLayout />}>
                <Route path="/" element={<MainPage />} />
                <Route path="/schedule" element={<SchedulePage />} />
                <Route path="/schedule/time-select" element={<TimeSelectPage />} />
                <Route path="/schedule/timer-select" element={<TimerSelectPage />} />
                <Route path="/control" element={<ControlPage />} />
                <Route path="/play" element={<PlayPage />} />
                <Route path="/robot-control" element={<RobotControlPage />} />
                <Route path="/menu" element={<MenuPage />} />
                <Route path="/device-connect" element={<DeviceConnectPage />} />
                <Route path="/alarm" element={<AlarmPage />} />
                <Route path="/location" element={<LocationPage />} />
                <Route path="/menu/product-guide" element={<ProductGuide />} />
                <Route path="/menu/faq" element={<FAQPage />} />
                <Route path="/menu/privacy" element={<PrivacyPage />} />
              </Route>
            </Routes>
          </motion.div>
        </AnimatePresence>
      </div>

      {!shouldHideNav && (
        <div className="shrink-0 h-[60px]">
          <NavigationBar />
        </div>
      )}
    </div>
  );
}

function App() {
  return (
    <Router>
      <AppContent />
    </Router>
  );
}

export default App;

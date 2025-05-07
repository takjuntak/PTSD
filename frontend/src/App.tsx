// src/App.tsx
import { BrowserRouter as Router, Routes, Route, useLocation } from 'react-router-dom'
import ProtectedLayout from './ProtectedLayout'

import './App.css'
import MainPage from './pages/MainPage'
import SchedulePage from './pages/schedule/SchedulePage'
import NavigationBar from './components/common/NavigationBar'
import TimeSelectPage from './pages/schedule/TimeSelectPage'
import ControlPage from './pages/ControlPage'
import PlayPage from './pages/PlayPage'
import RobotControlPage from './pages/RobotControlPage'
import MenuPage from './pages/menu/MenuPage' // MenuPage 추가 import
import DeviceConnectPage from './pages/DeviceConnectPage'
import LoginPage from './pages/user/login/LoginPage'
import SignupPage from './pages/user/signup/SignupPage'

function AppContent() {
  const location = useLocation()
  const hideNavPaths = ['/login', '/signup']
  const shouldHideNav = hideNavPaths.includes(location.pathname)

  return (
    <div className="flex flex-col text-white w-full h-screen overflow-hidden">
      <div className="flex-1 w-full overflow-y-auto">
        <Routes>
          {/* 공개 라우트 */}
          <Route path="/login" element={<LoginPage />} />
          <Route path="/signup" element={<SignupPage />} />

          {/* 보호된 라우트 그룹 */}
          <Route element={<ProtectedLayout />}>
            <Route path="/" element={<MainPage />} />
            <Route path="/schedule" element={<SchedulePage />} />
            <Route path="/schedule/time-select" element={<TimeSelectPage />} />
            <Route path="/control" element={<ControlPage />} />
            <Route path="/play" element={<PlayPage />} />
            <Route path="/robot-control" element={<RobotControlPage />} />
            <Route path='/menu' element={<MenuPage />} /> {/* MenuPage 라우트 추가 */}
            <Route path='/device-connect' element={<DeviceConnectPage />} />
          </Route>
        </Routes>
      </div>

      {!shouldHideNav && <NavigationBar />}
    </div>
  )
}

function App() {
  return (
    <Router>
      <AppContent />
    </Router>
  )
}

export default App

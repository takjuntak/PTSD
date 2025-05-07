// src/App.tsx
import { BrowserRouter as Router, Routes, Route, useLocation } from 'react-router-dom'

import './App.css'
import MainPage from './pages/MainPage'
import SchedulePage from './pages/schedule/SchedulePage'
import NavigationBar from './components/common/NavigationBar'
import TimeSelectPage from './pages/schedule/TimeSelectPage'
import ControlPage from './pages/ControlPage'
import PlayPage from './pages/PlayPage'
import RobotControlPage from './pages/RobotControlPage'
import MenuPage from './pages/MenuPage' // MenuPage 추가 import
import DeviceConnectPage from './pages/DeviceConnectPage'
import LoginPage from './pages/user/LoginPage'
import SignupPage from './pages/user/SignupPage'
import TimerSelectPage from './pages/schedule/TimerSelectPage'

function AppContent() {
  const location = useLocation()
  const hideNavPaths = ['/login', '/signup']
  const shouldHideNav = hideNavPaths.includes(location.pathname)

  return (
    <div className="flex flex-col text-white w-full h-screen overflow-hidden">
      <div className="flex-1 w-full overflow-y-auto">
        <Routes>
          <Route path="/" element={<MainPage />} />
          <Route path="/schedule" element={<SchedulePage />} />
          <Route path="/schedule/time-select" element={<TimeSelectPage />} />
          <Route path='/schedule/timer-select' element={<TimerSelectPage/>} />
          <Route path="/control" element={<ControlPage />} />
          <Route path="/play" element={<PlayPage />} />
          <Route path="/robot-control" element={<RobotControlPage />} />
          <Route path='/menu' element={<MenuPage />} /> {/* MenuPage 라우트 추가 */}
          <Route path='/device-connect' element={<DeviceConnectPage />} />
          <Route path="/login" element={<LoginPage />} />
          <Route path="/signup" element={<SignupPage />} />
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

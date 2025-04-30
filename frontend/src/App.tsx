// src/App.tsx - RobotControlPage 라우트 추가
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom'
import './App.css'
import MainPage from './pages/MainPage'
import SchedulePage from './pages/schedule/SchedulePage'
import NavigationBar from './components/common/NavigationBar'
import TimeSelectPage from './pages/schedule/TimeSelectPage'
import ControlPage from './pages/ControlPage'
import PlayPage from './pages/PlayPage'
import RobotControlPage from './pages/RobotControlPage' // 추가

function App() {
  return (
    <Router>
      <div className="flex flex-col bg-app-dark text-white w-full h-screen overflow-hidden">
        <div className="flex-1 w-full overflow-y-auto">
          <Routes>
            <Route path="/" element={<MainPage />} />
            <Route path="/schedule" element={<SchedulePage />} />
            <Route path='/schedule/time-select' element={<TimeSelectPage />} />
            <Route path='/control' element={<ControlPage />} />
            <Route path='/play' element={<PlayPage />} />
            <Route path='/robot-control' element={<RobotControlPage />} /> {/* 새 라우트 추가 */}
            {/* 추가 경로는 여기에 작성 */}
          </Routes>
        </div>
        
        <NavigationBar />
      </div>
    </Router>
  )
}

export default App
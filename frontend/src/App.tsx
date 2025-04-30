// src/App.tsx
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom'
import './App.css'
import MainPage from './pages/MainPage'
import SchedulePage from './pages/schedule/SchedulePage'
import NavigationBar from './components/common/NavigationBar'
import TimeSelectPage from './pages/schedule/TimeSelectPage'

function App() {
  return (
    <Router>
      <div className="flex flex-col bg-app-dark text-white w-full h-screen overflow-hidden">
        <div className="flex-1 w-full overflow-y-auto"> {/* 스크롤이 가능하도록 설정 */}
          <Routes>
            <Route path="/" element={<MainPage />} />
            <Route path="/schedule" element={<SchedulePage />} />
            <Route path='/schedule/time-select' element={<TimeSelectPage />} />
            {/* 추가 경로는 여기에 작성 */}
          </Routes>
        </div>
        
        {/* 네비게이션 바는 App.tsx에서 관리하여 모든 페이지에서 공통으로 사용 */}
        <NavigationBar />
      </div>
    </Router>
  )
}

export default App
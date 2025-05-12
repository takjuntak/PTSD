// src/pages/AlarmPage.tsx 카드형태로 수정
import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { ChevronLeft, Check, ChevronRight, ChevronLeft as ChevronLeftCircle } from 'lucide-react';
import AlarmItem from '../components/alarm/AlarmItem';
import useAlarms from '../hooks/useAlarms';

const AlarmPage: React.FC = () => {
  const navigate = useNavigate();
  const { alarms, loading, error, markAsRead, markAllAsRead } = useAlarms();
  
  // 페이지네이션 상태 관리
  const [currentPage, setCurrentPage] = useState(1);
  const itemsPerPage = 8;  // 페이지당 항목 수
  
  const handleGoBack = () => {
    navigate(-1);
  };
  
  // 전체 페이지 수 계산
  const totalPages = Math.ceil(alarms.length / itemsPerPage);
  
  // 현재 페이지에 표시할 알람 필터링
  const paginatedAlarms = alarms.slice(
    (currentPage - 1) * itemsPerPage,
    currentPage * itemsPerPage
  );
  
  // 페이지 이동 핸들러
  const goToPage = (page: number) => {
    if (page >= 1 && page <= totalPages) {
      setCurrentPage(page);
    }
  };

  return (
    <div className="w-full h-full flex flex-col bg-app-dark text-white" style={{ backgroundColor: '#2E2E37' }}>
      <header className="p-4 px-3 flex items-center justify-between sticky top-0 z-10" style={{ backgroundColor: '#2E2E37' }}>
        <div className="flex items-center gap-3">
          <button 
            onClick={handleGoBack} 
            className="text-white border-none bg-transparent p-0"
            style={{ background: 'transparent' }}
          >
            <ChevronLeft size={24} />
          </button>
          <span className="text-xl font-bold">알림</span>
        </div>
        
        <button
          onClick={markAllAsRead}
          className="flex items-center text-blue-400 bg-transparent border-none"
        >
          <Check size={16} className="mr-1" />
          <span className="text-sm">모두 읽음</span>
        </button>
      </header>

      <div 
        className="flex-1 overflow-y-auto px-4 py-3" 
        style={{ 
          backgroundColor: '#2E2E37',
          paddingBottom: '80px' // 하단 여백
        }}
      >
        {/* 알림 카드 컨테이너 */}
        <div className="rounded-lg overflow-hidden" style={{ backgroundColor: '#373738' }}>
          <div className="p-3 border-b border-gray-700 flex justify-between items-center">
            <h3 className="text-lg font-bold">알림</h3>
            <div className="flex">
              <button className="px-2 py-1 text-sm rounded bg-blue-500 text-white mr-1">최신순</button>
              <button className="px-2 py-1 text-sm rounded border border-gray-600 text-gray-300">오래된순</button>
            </div>
          </div>

          {loading ? (
            <div className="p-4 text-center text-gray-400">알림을 불러오는 중...</div>
          ) : error ? (
            <div className="p-4 text-center text-red-500">{error}</div>
          ) : paginatedAlarms.length === 0 ? (
            <div className="p-4 text-center text-gray-400">알림이 없습니다</div>
          ) : (
            <div>
              {paginatedAlarms.map(alarm => (
                <AlarmItem
                  key={alarm.id}
                  alarm={alarm}
                  onMarkAsRead={markAsRead}
                />
              ))}
              
              {/* 페이지네이션 */}
              {totalPages > 1 && (
                <div className="flex justify-center items-center gap-2 py-4 border-t border-gray-700">
                  <button 
                    onClick={() => goToPage(currentPage - 1)} 
                    disabled={currentPage === 1}
                    className="w-8 h-8 flex justify-center items-center rounded-full bg-gray-700"
                  >
                    <ChevronLeftCircle size={16} />
                  </button>
                  
                  {Array.from({ length: Math.min(5, totalPages) }, (_, i) => {
                    // 현재 페이지 주변의 페이지 번호 표시
                    let pageNum;
                    if (totalPages <= 5) {
                      pageNum = i + 1;
                    } else if (currentPage <= 3) {
                      pageNum = i + 1;
                    } else if (currentPage >= totalPages - 2) {
                      pageNum = totalPages - 4 + i;
                    } else {
                      pageNum = currentPage - 2 + i;
                    }
                    
                    return (
                      <button 
                        key={pageNum}
                        onClick={() => goToPage(pageNum)}
                        className={`w-8 h-8 flex justify-center items-center rounded-full ${
                          currentPage === pageNum ? 'bg-blue-500 text-white' : 'bg-gray-700'
                        }`}
                      >
                        {pageNum}
                      </button>
                    );
                  })}
                  
                  <button 
                    onClick={() => goToPage(currentPage + 1)} 
                    disabled={currentPage === totalPages}
                    className="w-8 h-8 flex justify-center items-center rounded-full bg-gray-700"
                  >
                    <ChevronRight size={16} />
                  </button>
                </div>
              )}
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default AlarmPage;
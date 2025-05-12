// src/pages/DeviceConnectPage.tsx
import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { ChevronLeft, Camera, Check, MoreHorizontal, Loader2 } from 'lucide-react';
import { useDevices } from '../hooks/useDevices';
import Portal from '../components/common/Portal';
import robotImage from '../assets/robot2.png';

const DeviceConnectPage = () => {
  const navigate = useNavigate();
  const { devices, addDevice, removeDevice, hasDevices, isLoading, error } = useDevices();

  const [currentView, setCurrentView] = useState<'list' | 'register' | 'serial'>('list');
  const [serialNumber, setSerialNumber] = useState('');
  const [deviceName, setDeviceName] = useState('');
  const [isDeleteModalOpen, setIsDeleteModalOpen] = useState(false);
  const [deviceToDelete, setDeviceToDelete] = useState<number | null>(null);
  const [activeMenu, setActiveMenu] = useState<number | null>(null);
  const [isRegistering, setIsRegistering] = useState(false);

  const handleRegisterDevice = async () => {
    if (serialNumber && deviceName) {
      try {
        setIsRegistering(true);
        await addDevice({ 
          serial_number: serialNumber, 
          name: deviceName 
        });
        setSerialNumber('');
        setDeviceName('');
        setCurrentView('list');
      } catch (err) {
        // 에러는 useDevices 훅에서 처리됨
      } finally {
        setIsRegistering(false);
      }
    }
  };

  const handleDeleteRequest = (deviceId: number) => {
    setDeviceToDelete(deviceId);
    setActiveMenu(null);
    setIsDeleteModalOpen(true);
  };

  const handleConfirmDelete = async () => {
    if (deviceToDelete !== null) {
      try {
        await removeDevice(deviceToDelete);
        setDeviceToDelete(null);
        setIsDeleteModalOpen(false);
      } catch (err) {
        // 에러는 useDevices 훅에서 처리됨
      }
    }
  };

  const toggleMenu = (deviceId: number) => {
    setActiveMenu(activeMenu === deviceId ? null : deviceId);
  };

  return (
    <div className="w-full h-full flex flex-col text-white">
      <header className="p-3 px-4 flex items-center gap-2 border-b border-neutral-700 sticky top-0 z-10">
        <button 
          onClick={() => currentView === 'list' ? navigate(-1) : setCurrentView('list')}
          className="p-1 text-white hover:opacity-80 bg-transparent border-none outline-none focus:outline-none"
        >
          <ChevronLeft size={20} />
        </button>

        <span className="text-lg font-bold">
          {currentView === 'list' ? '기기 연결' : currentView === 'register' ? '기기 등록하기' : '시리얼 번호 입력'}
        </span>
      </header>

      <div className="flex-1 overflow-y-auto p-3 pb-20">
        {currentView === 'list' && (
          <>
            {error && (
              <div className="bg-red-900 border border-red-700 text-red-300 px-4 py-2 rounded-lg mb-4">
                {error}
              </div>
            )}
            
            {isLoading ? (
              <div className="bg-app-card rounded-lg p-4 flex flex-col items-center justify-center h-36">
                <Loader2 size={24} className="animate-spin text-gray-400 mb-2" />
                <p className="text-gray-400 text-sm">기기 목록을 불러오는 중...</p>
              </div>
            ) : !hasDevices ? (
              <div className="bg-app-card rounded-lg p-4 flex flex-col items-center justify-center h-36">
                <p className="text-gray-400 text-sm">현재 등록된 기기가 없습니다.</p>
              </div>
            ) : (
              <div className="bg-app-card rounded-lg overflow-hidden">
                <div className="p-3 border-b border-neutral-700">
                  <h3 className="text-sm font-bold">기기 관리</h3>
                </div>
                {devices.map(device => (
                  <div key={device.device_id} className="relative flex items-center p-3 border-b border-neutral-700">
                    <div className="flex-1 flex items-center justify-center">
                      <div className="flex flex-col mr-3">
                        <div className="flex items-center">
                          <span className="text-sm font-medium">{device.name}</span>
                        </div>
                        <div className="flex items-center text-[10px] text-gray-400 mt-1">
                          {device.isConnected && <Check size={12} className="text-green-500 mr-1" />}
                          <span>{device.isConnected ? '연결됨' : '연결 해제됨'}</span>
                        </div>
                        <div className="text-[10px] text-gray-500 mt-0.5">
                          S/N: {device.serial_number}
                        </div>
                      </div>
                      <div className="w-20 h-20 flex items-center justify-center">
                        <img src={robotImage} alt={device.name} className="w-16 h-16 object-contain" />
                      </div>
                    </div>
                    <button onClick={() => toggleMenu(device.device_id)} className="ml-2 p-1 text-gray-400">
                      <MoreHorizontal size={16} />
                    </button>
                    {activeMenu === device.device_id && (
                      <div className="absolute right-2 top-10 bg-black bg-opacity-80 rounded-md overflow-hidden z-10">
                        <button 
                          onClick={() => handleDeleteRequest(device.device_id)} 
                          className="text-red-500 text-xs py-2 px-4 w-full text-center whitespace-nowrap"
                        >
                          기기 제거
                        </button>
                      </div>
                    )}
                  </div>
                ))}
              </div>
            )}
            <button 
              onClick={() => setCurrentView('register')} 
              className="mt-3 bg-blue-500 text-white rounded-lg py-2 w-full text-sm"
            >
              기기 추가 등록
            </button>
          </>
        )}

        {currentView === 'register' && (
          <div className="flex flex-col items-center">
            <p className="text-center my-3 text-sm">
              기기의 시리얼 번호를 촬영하거나 직접 입력하고,<br />기기에 이름을 설정해주세요.
            </p>
            <div
              className="border border-dashed border-gray-500 rounded-lg p-5 w-full mb-3 flex flex-col items-center cursor-pointer"
              onClick={() => console.log('카메라 촬영 기능')}
            >
              <Camera size={24} className="mb-2 text-gray-400" />
              <span className="text-gray-400 text-sm">시리얼 번호 인식하기</span>
            </div>
            <button onClick={() => setCurrentView('serial')} className="text-gray-400 text-xs mb-3">
              직접 입력할래요? 직접 입력하기
            </button>
          </div>
        )}

        {currentView === 'serial' && (
          <div className="flex flex-col">
            <div className="mb-3">
              <label className="block text-xs text-gray-400 mb-1">시리얼 번호 직접 입력</label>
              <input
                type="text"
                placeholder="EX : SN20240424ABC123"
                value={serialNumber}
                onChange={(e) => setSerialNumber(e.target.value)}
                className="w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white text-sm"
              />
            </div>
            <div className="mb-4">
              <label className="block text-xs text-gray-400 mb-1">로봇 이름</label>
              <input
                type="text"
                placeholder="PTSD 1호기 / Gymbo"
                value={deviceName}
                onChange={(e) => setDeviceName(e.target.value)}
                className="w-full bg-gray-800 border border-gray-700 rounded-md p-2 text-white text-sm"
              />
            </div>
            <button
              onClick={handleRegisterDevice}
              disabled={!serialNumber || !deviceName || isRegistering}
              className={`w-full py-2 rounded-lg text-sm flex items-center justify-center gap-2 ${
                serialNumber && deviceName ? 'bg-blue-500 text-white' : 'bg-gray-700 text-gray-400'
              }`}
            >
              {isRegistering ? (
                <>
                  <Loader2 size={16} className="animate-spin" />
                  등록 중...
                </>
              ) : (
                '기기 등록하기'
              )}
            </button>
          </div>
        )}
      </div>

      {isDeleteModalOpen && (
        <Portal>
          <div className="fixed inset-0 bg-black bg-opacity-60 flex items-center justify-center z-50 px-4">
            <div className="bg-app-card rounded-lg w-full max-w-sm overflow-hidden">
              <div className="p-4 border-b border-neutral-700">
                <h3 className="text-lg font-bold text-white">기기를 정말 삭제하시겠어요?</h3>
              </div>
              <div className="p-4">
                <p className="text-sm text-gray-300 mb-4">
                  기기를 제거하면 해당 기기와의 연결이 완전 해지되며, 관련된 모든 정보도 함께 삭제됩니다.
                </p>
                <div className="flex gap-2">
                  <button
                    onClick={() => setIsDeleteModalOpen(false)}
                    className="flex-1 py-2 bg-gray-700 text-white rounded-md text-sm"
                  >
                    취소
                  </button>
                  <button
                    onClick={handleConfirmDelete}
                    className="flex-1 py-2 bg-red-500 text-white rounded-md text-sm"
                  >
                    삭제
                  </button>
                </div>
              </div>
            </div>
          </div>
        </Portal>
      )}
    </div>
  );
};

export default DeviceConnectPage;
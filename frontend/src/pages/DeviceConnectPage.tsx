// src/pages/DeviceConnectPage.tsx
import { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { ChevronLeft, Camera, Check, MoreHorizontal, Loader2 } from 'lucide-react';
import { useDevices } from '../hooks/useDevices';
import robotImage from '../assets/robot2.png';

const DeviceConnectPage = () => {
  const navigate = useNavigate();
  const { devices, addDevice, hasDevices, isLoading, error } = useDevices();

  const [currentView, setCurrentView] = useState<'list' | 'register'>('list');
  const [serialNumber, setSerialNumber] = useState('');
  const [deviceName, setDeviceName] = useState('');
  const [activeMenu, setActiveMenu] = useState<number | null>(null);
  const [isRegistering, setIsRegistering] = useState(false);

  const handleRegisterDevice = async () => {
    if (serialNumber && deviceName) {
      try {
        setIsRegistering(true);
        await addDevice({
          serial_number: serialNumber,
          name: deviceName,
        });
        setSerialNumber('');
        setDeviceName('');
        setCurrentView('list');
      } catch (err) {
        // error handled in hook
      } finally {
        setIsRegistering(false);
      }
    }
  };

  const toggleMenu = (deviceId: number) => {
    setActiveMenu(activeMenu === deviceId ? null : deviceId);
  };

  return (
    <div className="w-full h-full flex flex-col text-white">
      <header className="p-3 px-4 flex items-center gap-2 sticky top-0 z-10 bg-transparent">
        <button
          onClick={() => currentView === 'list' ? navigate(-1) : setCurrentView('list')}
          className="p-1 text-white hover:opacity-80"
        >
          <ChevronLeft size={20} />
        </button>
        <span className="text-lg font-bold text-[#767676]">
          {currentView === 'list' ? '메뉴' : '기기 관리'}
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
              <div className="bg-[#373738] rounded-lg p-4 flex flex-col items-center justify-center h-36 shadow-md">
                <Loader2 size={24} className="animate-spin text-gray-400 mb-2" />
                <p className="text-gray-400 text-sm">기기 목록을 불러오는 중...</p>
              </div>
            ) : !hasDevices ? (
              <div className="w-[330px] h-[298px] bg-[#373738] rounded-[10px] shadow-md flex flex-col items-center justify-center mx-auto relative">
                <p className="absolute top-5 left-5 text-[15px] font-bold font-montserrat text-white">
                  기기 관리
                </p>
                <p className="text-[12px] text-white">현재 등록된 기기가 없습니다.</p>
              </div>
            ) : (
              <div className="bg-[#373738] rounded-lg overflow-hidden">
                <div className="p-3 border-b border-neutral-700">
                  <h3 className="text-sm font-bold">기기 관리</h3>
                </div>
                {devices.map((device) => (
                  <div key={device.device_id} className="relative flex items-center p-3 border-b border-neutral-700">
                    <div className="flex-1 flex items-center justify-center">
                      <div className="flex flex-col mr-3">
                        <span className="text-sm font-medium">{device.name}</span>
                        <div className="flex items-center text-[10px] text-gray-400 mt-1">
                          {device.isConnected && <Check size={12} className="text-green-500 mr-1" />}
                          <span>{device.isConnected ? '연결됨' : '연결 해제됨'}</span>
                        </div>
                        <p className="text-[10px] text-gray-500 mt-0.5">S/N: {device.serial_number}</p>
                      </div>
                      <div className="w-20 h-20 flex items-center justify-center">
                        <img src={robotImage} alt={device.name} className="w-16 h-16 object-contain" />
                      </div>
                    </div>
                    <button onClick={() => toggleMenu(device.device_id)} className="ml-2 p-1 text-gray-400">
                      <MoreHorizontal size={16} />
                    </button>
                  </div>
                ))}
              </div>
            )}

            <button
              onClick={() => setCurrentView('register')}
              className="w-[330px] h-[40px] bg-blue-500 text-white rounded-[10px] font-montserrat font-bold text-[15px] mt-8 mx-auto"
            >
              기기 추가 등록
            </button>
          </>
        )}

        {currentView === 'register' && (
          <div className="w-[330px] h-[530px] bg-[#373738] rounded-[10px] shadow-md p-5 mx-auto flex flex-col items-center mt-3">
            <p className="text-[15px] font-bold text-white mb-2 self-start">기기 등록하기</p>
            <p className="text-[12px] text-white text-left mt-4">
              기기의 시리얼 넘버를 촬영하거나 직접 입력하고,<br />
              기기에 이름을 설정해주세요.
            </p>

            <div
              onClick={() => console.log('촬영하기')}
              className="w-[290px] h-[84px] border border-dashed border-[#767676] rounded-[10px] mt-6 mb-6 flex items-center justify-center gap-2 cursor-pointer"
            >
              <Camera size={24} color="#FFFFFF" />
              <p className="text-[15px] font-semibold mt-[8px]">시리얼 넘버 인식하기</p>
            </div>

            <label className="text-[12px] font-semibold text-white self-start mt-5">시리얼 넘버 직접 입력</label>
            <input
              type="text"
              placeholder="EX : SN20240423ABC123"
              value={serialNumber}
              onChange={(e) => setSerialNumber(e.target.value)}
              className="w-[290px] h-[40px] bg-[#212228] border border-[#767676] rounded-[10px] mt-2 mb-5 px-3 text-white text-[12px]"
            />

            <label className="text-[12px] font-semibold text-white self-start mt-2">로봇 이름</label>
            <input
              type="text"
              placeholder="싸피짐 봇1 / 청소마루"
              value={deviceName}
              onChange={(e) => setDeviceName(e.target.value)}
              className="w-[290px] h-[40px] bg-[#212228] border border-[#767676] rounded-[10px] mt-2 mb-6 px-3 text-white text-[12px]"
            />

            <button
              onClick={handleRegisterDevice}
              disabled={!serialNumber || !deviceName || isRegistering}
              className={`w-[290px] h-[40px] rounded-[10px] text-white font-montserrat font-bold text-[15px] shadow-md ${
                serialNumber && deviceName ? 'bg-[#617BEE] cursor-pointer' : 'bg-[#555] cursor-not-allowed'
              } flex items-center justify-center`}
            >
              {isRegistering ? (
                <>
                  <Loader2 size={16} className="animate-spin mr-2" />
                  등록 중...
                </>
              ) : (
                '기기 등록하기'
              )}
            </button>
          </div>
        )}
      </div>
    </div>
  );
};

export default DeviceConnectPage;

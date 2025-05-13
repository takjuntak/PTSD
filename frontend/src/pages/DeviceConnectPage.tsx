import { useEffect, useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { ChevronLeft, Camera, MoreHorizontal, Loader2 } from 'lucide-react';
import apiClient from '../api/axios';
import robotImage from '../assets/robot2.png';
import connectSuccessImage from '../assets/menu/connect-success.svg';
import warningImage from '../assets/menu/warning.svg';

interface Device {
  device_id: number;
  name: string;
  serial_number: string;
}

const DeviceConnectPage = () => {
  const navigate = useNavigate();
  const [devices, setDevices] = useState<Device[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState('');
  const [currentView, setCurrentView] = useState<'list' | 'register'>('list');
  const [serialNumber, setSerialNumber] = useState('');
  const [deviceName, setDeviceName] = useState('');
  const [isRegistering, setIsRegistering] = useState(false);
  const [activeMenu, setActiveMenu] = useState<number | null>(null);
  const [showDeleteModal, setShowDeleteModal] = useState(false);
  const [selectedDeviceId, setSelectedDeviceId] = useState<number | null>(null);

  // 기기 전체 목록
  const fetchDevices = async () => {
    try {
      setIsLoading(true);
      const response = await apiClient.get('/devices');
      setDevices(response.data);
    } catch (err) {
      setError('기기 목록을 불러오지 못했습니다.');
    } finally {
      setIsLoading(false);
    }
  };

  // 기기 등록
  const handleRegisterDevice = async () => {
    if (serialNumber.trim() && deviceName.trim()) {
      const payload = {
        serial_number: serialNumber.trim(),
        name: deviceName.trim(),
      };

      console.log('📦 최종 전송 payload:', payload);

      try {
        const response = await apiClient.post('/devices/', payload, {
          headers: {
            Authorization: `Bearer ${localStorage.getItem('accessToken') || ''}`,
          },
        });

        console.log('✅ 등록 성공:', response.data);
        setSerialNumber('');
        setDeviceName('');
        setCurrentView('list');
      } catch (err: any) {
        console.error('❌ 등록 실패:', err.response?.data || err.message);
        alert(`등록 실패: ${err.response?.data?.message || '입력 형식을 확인해주세요.'}`);
      } finally {
        setIsRegistering(false);
      }
    } else {
      alert('기기 이름과 시리얼 넘버를 모두 입력해주세요.');
    }
  };


    // 기기 삭제
    const handleDeleteDevice = async (deviceId: number) => {
      try {
        await apiClient.delete(`/devices/${deviceId}`);
        fetchDevices(); // 삭제 후 목록 갱신
      } catch (err) {
        setError('기기 삭제에 실패했습니다.');
      } finally {
        setActiveMenu(null);
      }
    };

  useEffect(() => {
    fetchDevices();
  }, []);

  return (
    <div className="w-full h-full flex flex-col text-white relative">
      {showDeleteModal && (
        <div className="fixed inset-0 bg-[rgba(46,46,55,0.86)] z-[1000] flex items-center justify-center">
          <div className="bg-[#373738] w-[364px] rounded-[10px] shadow-lg text-center flex flex-col items-center justify-center py-2 px-2">
            <img src={warningImage} alt="경고" className="w-[60px] h-[60px] mb-4" />
            <h2 className="text-white text-[20px] font-extrabold mb-2">기기를 정말 삭제하시겠어요?</h2>
            <p className="text-white text-[14px] leading-relaxed mb-6">
              기기를 제거하면 해당 기기와의 연결이 완전 해지되며,<br />
              관련된 모든 정보도 함께 삭제됩니다.
            </p>
            <div className="flex gap-3 w-full justify-center">
              <button
                onClick={() => setShowDeleteModal(false)}
                className="w-[158px] h-[46px] bg-white text-[#EE6163] font-bold text-[18px] rounded-[10px] border border-[#EE6163] shadow-md flex items-center justify-center"
              >
                취소
              </button>
              <button
                onClick={async () => {
                  if (selectedDeviceId !== null) {
                    await handleDeleteDevice(selectedDeviceId);
                    setShowDeleteModal(false);
                    setSelectedDeviceId(null);
                  }
                }}
                className="w-[158px] h-[46px] bg-[#EE6163] text-white font-bold text-[18px] rounded-[10px] shadow-md flex items-center justify-center"
              >
                기기 제거
              </button>
            </div>
          </div>
        </div>
      )}

      <header className="p-3 px-4 flex items-center gap-2 sticky top-0 z-10 bg-transparent">
        <button
          onClick={() => (currentView === 'list' ? navigate(-1) : setCurrentView('list'))}
          className="bg-transparent border-none p-0 m-0 text-white hover:opacity-80 focus:outline-none"
        >
          <ChevronLeft size={20} />
        </button>
        <span className="text-lg font-bold text-[#767676] text-left">메뉴</span>
      </header>

      <div className="flex-1 overflow-y-auto p-3 pb-20">
        {currentView === 'list' ? (
          <>
            {error && (
              <div className="bg-red-900 border border-red-700 text-red-300 px-4 py-2 rounded-lg mb-4">
                {error}
              </div>
            )}

            {isLoading ? (
              <div className="w-[330px] h-[298px] bg-[#373738] rounded-[10px] shadow-md flex flex-col items-center justify-center mx-auto">
                <Loader2 size={24} className="animate-spin text-gray-400 mb-2" />
                <p className="text-gray-400 text-sm">기기 목록을 불러오는 중...</p>
              </div>
            ) : devices.length === 0 ? (
              <div className="w-[330px] h-[298px] bg-[#373738] rounded-[10px] shadow-md flex flex-col items-center justify-center mx-auto relative">
                <p className="absolute top-5 left-5 text-[15px] font-bold font-montserrat text-white">기기 관리</p>
                <p className="text-[12px] text-white">현재 등록된 기기가 없습니다.</p>
              </div>
            ) : (
              <div className="w-[330px] bg-[#373738] rounded-[10px] shadow-md mx-auto mt-10 p-5">
                <p className="text-left text-white font-bold text-[15px] font-montserrat mb-4 ml-1">기기 관리</p>
                <div className="flex flex-col gap-4 items-center">
                  {devices.map((device) => (
                    <div
                      key={device.device_id}
                      className="relative w-[288px] h-[111px] border border-[#7A7A7A] rounded-[10px] flex justify-between items-center px-4"
                      onClick={() => setActiveMenu(null)}
                    >
                      <div className="flex flex-col justify-center">
                        <span className="text-left text-white font-montserrat font-bold text-[15px]">{device.name}</span>
                        <div className="flex items-center text-[12px] font-medium text-[#7AA973] mt-1">
                          <img src={connectSuccessImage} alt="check" className="w-[16px] h-[16px] mr-1" />
                          연결됨
                        </div>
                        <p className="text-[10px] text-gray-400 mt-1">S/N: {device.serial_number}</p>
                      </div>
                      <img src={robotImage} alt={device.name} className="absolute top-4 right-8 w-[98px] h-[98px] object-contain" />
                      <button
                        onClick={(e) => {
                          e.stopPropagation();
                          setActiveMenu(activeMenu === device.device_id ? null : device.device_id);
                        }}
                        className="absolute top-4 right-4 bg-transparent p-0"
                      >
                        <MoreHorizontal size={20} className="text-white" />
                      </button>

                      {activeMenu === device.device_id && (
                        <div className="absolute right-0 top-10 z-50">
                          <button
                            onClick={() => {
                              setSelectedDeviceId(device.device_id);
                              setShowDeleteModal(true);
                              setActiveMenu(null);
                            }}
                            className="text-[#EE6163] text-sm font-semibold whitespace-nowrap bg-[#25262D] px-4 py-2 rounded-lg shadow"
                          >
                            기기 제거
                          </button>
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              </div>
            )}

            <button
              onClick={() => setCurrentView('register')}
              className="w-[330px] h-[40px] bg-blue-500 text-white rounded-[10px] font-montserrat font-bold text-[15px] mt-8 mx-auto"
            >
              기기 추가 등록
            </button>
          </>
        ) : (
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

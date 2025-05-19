
from ultralytics import YOLO
import cv2
import os
import numpy as np
import paho.mqtt.client as mqtt
import json
import time
import logging
import base64
import threading
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# 환경 변수에서 MQTT 설정 가져오기
MQTT_BROKER = os.getenv("MQTT_BROKER", "localhost")  # 기본값은 localhost
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))  # 문자열을 정수로 변환, 기본값은 1883

# 로깅 설정
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MQTTObjectDetector:
    def __init__(self, model_path, mqtt_broker=MQTT_BROKER, mqtt_port=MQTT_PORT,
                 image_topic="mqtt/image", detection_topic="plate_detection",
                 conf_threshold=0.55, iou_threshold=0.45):
        # 설정 저장
        self.model_path = model_path
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.image_topic = image_topic
        self.detection_topic = detection_topic
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        
        # 모델과 클라이언트는 시작 시 초기화
        self.model = None
        self.client = None
        self.running = False
        self.thread = None
    
    def load_model(self):
        """YOLOv8 모델 로드"""
        self.model = YOLO(self.model_path)
        logger.info(f"모델을 성공적으로 로드했습니다: {self.model_path}")
    
    def process_frame(self, frame):
        """이미지 프레임 처리 및 객체 감지"""
        # 모델로 예측
        results = self.model(frame, conf=self.conf_threshold, iou=self.iou_threshold)
        
        # 결과 처리
        detections = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                
                # 중심점과 크기 계산
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                width, height = x2 - x1, y2 - y1
                
                detections.append({
                    "center_x": round(cx, 1),
                    "center_y": round(cy, 1),
                    "timestamp": time.time()
                })
                
            # 결과 시각화 - 객체 박스 표시
        if len(results) > 0:
        # YOLOv8의 내장 시각화 기능 사용
            annotated_frame = results[0].plot()
            cv2.imshow("Object Detection", annotated_frame)
            cv2.waitKey(1)
        else:
        # 객체가 감지되지 않은 경우에도 원본 이미지 표시
            cv2.imshow("Object Detection", frame)
            cv2.waitKey(1)
        
        # MQTT로 데이터 전송
        if detections and self.client:
            payload = json.dumps({"detections": detections})
            self.client.publish(self.detection_topic, payload)
            logger.info(f"{len(detections)}개 객체 감지, MQTT 전송 완료")
        else:
            logger.info("감지된 객체 없음 또는 MQTT 클라이언트 연결 안됨")
        
        return detections
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT 연결 콜백"""
        logger.info(f"[MQTT] 연결됨 (코드: {rc})")
        client.subscribe(self.image_topic)
        logger.info(f"[MQTT] {self.image_topic} 토픽 구독 시작")
    
    def on_message(self, client, userdata, msg):
        """MQTT 메시지 수신 콜백"""
        logger.info("[MQTT] 이미지 메시지 수신")
        try:
            # MQTT 메시지 → JSON 디코드
            payload = json.loads(msg.payload.decode())
            image_b64 = payload.get("image_data")
            
            if not image_b64:
                logger.warning("[경고] 'image_data' 필드 없음")
                return
                
            # Base64 디코딩 → OpenCV 이미지로 변환
            img_bytes = base64.b64decode(image_b64)
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                
            if img is not None:
                cv2.imshow("MQTT Image", img)
                cv2.waitKey(1)

                # 이미지 추론 처리
                self.process_frame(img)
            else:
                logger.error("[에러] 이미지 디코딩 실패")
                
        except Exception as e:
            logger.error(f"[에러] 메시지 처리 중 예외 발생: {e}")

    def mqtt_loop(self):
        """MQTT 루프 실행"""
        try:
            # 모델 로드
            self.load_model()
            
            # MQTT 클라이언트 설정
            self.client = mqtt.Client()
            self.client.on_connect = self.on_connect
            self.client.on_message = self.on_message
            
            # MQTT 연결
            self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
            logger.info(f"MQTT 브로커 {self.mqtt_broker}:{self.mqtt_port}에 연결됨")
            
            # 무한 루프 시작 (블로킹)
            self.running = True
            self.client.loop_forever()
            
        except Exception as e:
            logger.error(f"MQTT 루프 오류: {e}")
            self.running = False
    
    def start(self):
        """감지기 시작 - 스레드로 실행"""
        if self.thread is not None and self.thread.is_alive():
            logger.warning("객체 감지기가 이미 실행 중입니다")
            return False
        
        # 스레드 생성 및 시작
        self.thread = threading.Thread(target=self.mqtt_loop)
        self.thread.daemon = True  # 메인 스레드 종료시 같이 종료
        self.thread.start()
        logger.info("객체 감지 스레드 시작됨")
        return True
    
    def stop(self):
        """감지기 중지"""
        self.running = False
        if self.client:
            self.client.disconnect()
            logger.info("MQTT 클라이언트 연결 종료됨")
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=5.0)
            logger.info("객체 감지 스레드 종료됨")
            
        self.thread = None


# 전역 객체 감지기 인스턴스
detector = None

def start_object_detector():
    """객체 감지기 초기화 및 시작"""
    global detector
    
    # 이미 실행 중인지 확인
    if detector is not None:
        return False
    
    # 객체 감지기 인스턴스 생성 및 시작
    detector = MQTTObjectDetector(
        model_path="PTSD/mqtt/best_model/best_36.pt",
        mqtt_broker=MQTT_BROKER,
        mqtt_port=MQTT_PORT,
        image_topic="mqtt/image",
        detection_topic="plate_detection"
    )
    
    return detector.start()

def stop_object_detector():
    """객체 감지기 중지"""
    global detector
    if detector:
        detector.stop()
        detector = None
        return True
    return False
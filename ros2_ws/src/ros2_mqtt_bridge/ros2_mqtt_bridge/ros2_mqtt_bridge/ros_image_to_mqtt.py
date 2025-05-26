import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import paho.mqtt.client as mqtt
import base64
import json
import time
import os
from dotenv import load_dotenv

class RosImageToMqtt(Node):
    def __init__(self):
        super().__init__('ros_image_to_mqtt')

        load_dotenv()

        self.mqtt_broker = os.getenv('MQTT_BROKER')
        self.mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        self.mqtt_keepalive = int(os.getenv('MQTT_KEEPALIVE', '60'))
        self.mqtt_topic = os.getenv('MQTT_IMAGE_TOPIC')

        if not self.mqtt_broker or not self.mqtt_topic:
            self.get_logger().error('[환경변수] MQTT_BROKER 또는 MQTT_IMAGE_TOPIC이 설정되지 않았습니다!')
            return

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, self.mqtt_keepalive)
            self.get_logger().info(f'[MQTT] 연결 시도: {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'[MQTT] 연결 실패: {e}')
            return

        self.mqtt_client.loop_start()

        self.serial = self.get_serial_id()
        self.last_published_time = 0.0

        # 이미지 압축 파일 전달달
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('[MQTT] 연결 성공')
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f'[MQTT] 토픽 구독: {self.mqtt_topic}')
        else:
            self.get_logger().error(f'[MQTT] 연결 실패: 코드 {rc}')

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_published_time < 0.2:        # 1초에 5장 전달 진행중중
            return
        self.last_published_time = now

        try:
            encoded_image = base64.b64encode(msg.data).decode('utf-8')
            mqtt_msg = {
                'format': msg.format,
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec
                },
                'frame_id': msg.header.frame_id,
                'serial': self.serial,
                'image_data': encoded_image
            }
            self.mqtt_client.publish(self.mqtt_topic, json.dumps(mqtt_msg))
            self.get_logger().info('[ROS → MQTT] 이미지 전송 완료')
        except Exception as e:
            self.get_logger().error(f'[에러] 이미지 처리 실패: {e}')

    def get_serial_id(self):
        try:
            with open("/proc/cpuinfo", "r") as f:
                for line in f:
                    if line.startswith("Serial"):
                        return line.strip().split(":")[1].strip()
        except Exception as e:
            self.get_logger().error(f'[에러] 시리얼 번호 읽기 실패: {e}')
        return "UNKNOWN"

def main(args=None):
    rclpy.init(args=args)
    node = RosImageToMqtt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

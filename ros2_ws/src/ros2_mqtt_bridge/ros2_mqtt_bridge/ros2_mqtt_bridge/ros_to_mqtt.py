import rclpy
from rclpy.node import Node

import paho.mqtt.client as mqtt
import json
import time
import os
from dotenv import load_dotenv

from sensor_msgs.msg import BatteryState

class RosToMqttBridge(Node):
    def __init__(self):
        super().__init__('ros_to_mqtt_bridge')

        load_dotenv()

        self.mqtt_broker = os.getenv('MQTT_BROKER')
        self.mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        self.mqtt_keepalive = int(os.getenv('MQTT_KEEPALIVE', '60'))
        self.mqtt_battery_topic = os.getenv('MQTT_BATTERY_TOPIC')

        if not self.mqtt_broker or not self.mqtt_battery_topic:
            self.get_logger().error('[환경변수] MQTT_BROKER 또는 MQTT_BATTERY_TOPIC이 설정되지 않았습니다!')
            return

        self.serial = self.get_serial_id()
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, self.mqtt_keepalive)
            self.get_logger().info(f'[MQTT] 연결 시도: {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'[MQTT] 연결 실패: {e}')
            return

        self.mqtt_client.loop_start()

        self.topic_configs = {
            '/battery_state': (BatteryState, self.mqtt_battery_topic)
        }

        self.last_published_time = {}

        for topic_name, (msg_type, _) in self.topic_configs.items():
            self.create_subscription(msg_type, topic_name, self.create_callback(topic_name), 10)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('[MQTT] 연결 성공')
        else:
            self.get_logger().error(f'[MQTT] 연결 실패: 코드 {rc}')

    def create_callback(self, topic_name):
        def callback(msg):
            now = time.time()
            if now - self.last_published_time.get(topic_name, 0.0) < 1.0:
                return

            self.last_published_time[topic_name] = now

            mqtt_topic = self.topic_configs[topic_name][1]
            json_msg = self.convert_msg_to_json(msg)

            if json_msg:
                self.mqtt_client.publish(mqtt_topic, json_msg)
                self.get_logger().info(f'[{topic_name} → MQTT] 전송 완료: {mqtt_topic}')
        return callback

    def convert_msg_to_json(self, msg):
        try:
            if isinstance(msg, BatteryState):
                return json.dumps({
                    'header': {
                        'stamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'frame_id': msg.header.frame_id
                    },
                    'voltage': msg.voltage,
                    'current': msg.current,
                    'percentage': msg.percentage,
                    'temperature': msg.temperature,
                    'design_capacity': msg.design_capacity,
                    'serial': self.serial
                })
            return json.dumps({'error': '지원되지 않는 메시지 타입'})
        except Exception as e:
            self.get_logger().error(f'[에러] 메시지 변환 실패: {e}')
            return None

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
    node = RosToMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

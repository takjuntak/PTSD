import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import os
from dotenv import load_dotenv

class MQTTToCmdVel(Node):
    def __init__(self):
        super().__init__('mqtt_to_cmdvel')

        load_dotenv()

        self.mqtt_broker = os.getenv('MQTT_BROKER')
        self.mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        self.mqtt_keepalive = int(os.getenv('MQTT_KEEPALIVE', '60'))
        self.mqtt_topic = os.getenv('MQTT_CONTROL_TOPIC')

        if not self.mqtt_broker or not self.mqtt_topic:
            self.get_logger().error('[환경변수] MQTT 설정이 누락되었습니다!')
            return

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('[ROS2] MQTT to cmd_vel 노드 시작됨')

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        try:
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, self.mqtt_keepalive)
            self.get_logger().info(f'[MQTT] 연결 시도: {self.mqtt_broker}:{self.mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'[MQTT] 연결 실패: {e}')
            return

        self.mqtt_client.subscribe(self.mqtt_topic)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('[MQTT] 연결 성공')
        else:
            self.get_logger().error(f'[MQTT] 연결 실패: 코드 {rc}')

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode().strip().lower()
        twist = Twist()

        if command == 'w':
            twist.linear.x = 0.1
        elif command == 'x':
            twist.linear.x = -0.1
        elif command == 'a':
            twist.angular.z = 0.4
        elif command == 'd':
            twist.angular.z = -0.4
        elif command == 's':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().warn(f"[MQTT] 알 수 없는 명령어: '{command}'")
            return

        self.publisher_.publish(twist)
        self.get_logger().info(f"[ROS2] cmd_vel 명령어 발행: '{command}'")

def main(args=None):
    rclpy.init(args=args)
    node = MQTTToCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

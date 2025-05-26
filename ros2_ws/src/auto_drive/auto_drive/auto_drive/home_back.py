import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator
import math
import json
import paho.mqtt.client as mqtt
import os
from dotenv import load_dotenv

class HomeBack(Node):
    def __init__(self):
        super().__init__('home_back')
        
        # .env 파일 로드
        load_dotenv()
        
        self.navigator = BasicNavigator()
        self.is_returning = False
        self.return_complete = False

        # 환경변수에서 목표 좌표 가져오기
        target_x_str = os.getenv('TARGET_X')
        target_y_str = os.getenv('TARGET_Y')
        target_yaw_str = os.getenv('TARGET_YAW')

        if not all([target_x_str, target_y_str, target_yaw_str]):
            self.get_logger().error('TARGET_X, TARGET_Y, TARGET_YAW 환경변수가 모두 설정되어야 합니다!')
            return

        self.target_x = float(target_x_str)
        self.target_y = float(target_y_str)
        self.target_yaw = float(target_yaw_str)

        self.get_logger().info(f"맵 기준 복귀 위치 대기 중: x={self.target_x}, y={self.target_y}, yaw={self.target_yaw}")

        # ROS2 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/robot/home_back',
            self.listener_callback,
            10
        )

        # 환경변수에서 시리얼 번호 가져오기
        self.serial = os.getenv('ROBOT_SERIAL')
        if not self.serial:
            self.get_logger().error('ROBOT_SERIAL 환경변수가 설정되지 않았습니다!')
            return

        # 환경변수에서 MQTT 설정 가져오기
        mqtt_host = os.getenv('MQTT_HOST')
        mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        mqtt_keepalive = int(os.getenv('MQTT_KEEPALIVE', '60'))

        if not mqtt_host:
            self.get_logger().error('MQTT_HOST 환경변수가 설정되지 않았습니다!')
            return

        # MQTT 설정 및 콜백 등록
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(mqtt_host, mqtt_port, mqtt_keepalive)
            self.get_logger().info(f'MQTT 연결 시도: {mqtt_host}:{mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'MQTT 연결 실패: {e}')
            
        self.mqtt_client.subscribe("robot/home_back")
        self.mqtt_client.loop_start()

        self.check_timer = None

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("[MQTT] 연결 성공")
        else:
            self.get_logger().error(f"[MQTT] 연결 실패: 코드 {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        payload = msg.payload.decode().strip().lower()
        if payload == "complete" and not self.is_returning:
            self.get_logger().info("[MQTT] 'complete' 수신: 절대좌표로 이동 시작")
            self.is_returning = True
            self.init_timer = self.create_timer(1.0, self.initialize_return)

    def listener_callback(self, msg):
        if msg.data.strip().lower() == "complete" and not self.is_returning:
            self.get_logger().info("[ROS2] 'complete' 수신: 절대좌표로 이동 시작")
            self.is_returning = True
            self.init_timer = self.create_timer(1.0, self.initialize_return)

    def initialize_return(self):
        self.init_timer.cancel()
        self.navigator.waitUntilNav2Active()
        self.go_to_target_position()

    def go_to_target_position(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # 환경변수에서 읽은 좌표 설정
        goal_pose.pose.position.x = self.target_x
        goal_pose.pose.position.y = self.target_y
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation = self.yaw_to_quaternion(self.target_yaw)

        self.get_logger().info(f"절대좌표로 이동 시작: x={self.target_x}, y={self.target_y}, yaw={self.target_yaw}")
        self.navigator.goToPose(goal_pose)
        self.check_timer = self.create_timer(1.0, self.check_return_status)

    def check_return_status(self):
        if self.navigator.isTaskComplete():
            self.check_timer.cancel()
            result = self.navigator.getResult()
            if result:
                self.get_logger().info("절대좌표 도착 완료!")
            else:
                self.get_logger().warn("절대좌표 이동 실패!")

            self.publish_complete_status()
            self.return_complete = True
            self.is_returning = False  # 다음 요청을 위해 플래그 리셋
        else:
            self.get_logger().info("절대좌표로 이동 중...")

    def publish_complete_status(self):
        msg = {
            "status": "complete",
            "serial": self.serial
        }
        self.mqtt_client.publish("robot/status", json.dumps(msg))
        self.get_logger().info(f"[MQTT] 로봇 상태 전송: {msg}")

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = HomeBack()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'mqtt_client'):
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
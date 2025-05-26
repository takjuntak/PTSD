import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import paho.mqtt.client as mqtt
import math
import json
import os
from dotenv import load_dotenv

class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__('multi_goal_navigator')

        # .env 파일 로드
        load_dotenv()

        self.goals = [
            {'x': 1.2313429629453879, 'y': 0.01047011137270536,},
            {'x': 1.177036725570271, 'y': -0.5385732123517728, 'yaw': 180},
            {'x': 0.432090248625888, 'y': -0.5617533081488001, 'yaw': 90},
            {'x': 0.06917152155600209, 'y': 0.008366613060324008, 'yaw': 0},
        ]

        # 환경변수에서 로봇 시리얼 번호 가져오기
        self.robot_serial = os.getenv('ROBOT_SERIAL')

        self.navigator = BasicNavigator()
        self.get_logger().info('Nav2 활성화 대기...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 활성화 완료!')

        self.navigation_started = False
        self.navigation_in_progress = False
        self.current_goal_index = 0

        # 환경변수에서 MQTT 설정 가져오기
        mqtt_host = os.getenv('MQTT_HOST')
        mqtt_port = int(os.getenv('MQTT_PORT', '1883'))
        mqtt_keepalive = int(os.getenv('MQTT_KEEPALIVE', '60'))

        if not mqtt_host:
            self.get_logger().error('MQTT_HOST 환경변수가 설정되지 않았습니다!')
            return

        if not self.robot_serial:
            self.get_logger().error('ROBOT_SERIAL 환경변수가 설정되지 않았습니다!')
            return

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect(mqtt_host, mqtt_port, mqtt_keepalive)
            self.get_logger().info(f'MQTT 연결 시도: {mqtt_host}:{mqtt_port}')
        except Exception as e:
            self.get_logger().error(f'MQTT 연결 실패: {e}')
            
        self.mqtt_client.loop_start()

        self.complete_pub = self.create_publisher(String, 'robot/home_back', 10)

        self.get_logger().info('MQTT "robot/auto_control" 토픽에서 "start" 메시지를 기다립니다.')

        self.timer = self.create_timer(1.0, self.navigation_loop)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('[MQTT] 연결 성공')
            client.subscribe("robot/auto_control")
        else:
            self.get_logger().error(f'[MQTT] 연결 실패: {rc}')

    def on_mqtt_message(self, client, userdata, msg):
        message = msg.payload.decode()
        self.get_logger().info(f'[MQTT] 수신: {message}')
        if message.strip().lower() == 'start' and not self.navigation_started:
            self.get_logger().info('"start" 명령 수신! 주행 시작')
            self.navigation_started = True
            self.navigation_in_progress = False
            self.send_status_mqtt("start")

    def send_status_mqtt(self, status):
        status_msg = {
            "status": status,
            "serial": self.robot_serial
        }
        try:
            self.mqtt_client.publish("robot/status", json.dumps(status_msg))
            self.get_logger().info(f'MQTT robot/status 전송: {status_msg}')
        except Exception as e:
            self.get_logger().error(f'MQTT status 전송 실패: {e}')

    def create_pose(self, x, y, yaw_deg=None):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        if yaw_deg is None:
            yaw_deg = 0.0
        yaw_rad = math.radians(float(yaw_deg))
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

    def navigation_loop(self):
        if self.navigation_started and not self.navigation_in_progress:
            if self.current_goal_index < len(self.goals):
                goal = self.goals[self.current_goal_index]
                goal_pose = self.create_pose(goal['x'], goal['y'], goal.get('yaw'))
                self.get_logger().info(
                    f'목표 {self.current_goal_index+1} 이동: x={goal["x"]}, y={goal["y"]}, yaw={goal.get("yaw", 0.0)}deg'
                )
                self.navigator.goToPose(goal_pose)
                self.navigation_in_progress = True
            else:
                self.get_logger().info('모든 목표 완료!')
                complete_msg = String()
                complete_msg.data = 'complete'
                self.complete_pub.publish(complete_msg)
                self.get_logger().info('robot/home_back 토픽으로 "complete" 메시지 전송')
                self.navigation_started = False
                self.navigation_in_progress = False
                self.current_goal_index = 0

                # 노드 종료
                self.get_logger().info('모든 목표 도달. 노드를 종료합니다.')
                rclpy.shutdown()

        if self.navigation_in_progress:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'목표 {self.current_goal_index+1} 도달 성공')
                    self.current_goal_index += 1
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn(f'목표 {self.current_goal_index+1} 도달 취소됨')
                    self.navigation_started = False
                elif result == TaskResult.FAILED:
                    self.get_logger().error(f'목표 {self.current_goal_index+1} 도달 실패')
                    self.navigation_started = False
                self.navigation_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
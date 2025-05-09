import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import requests
import math

class BatteryForwarder(Node):
    def __init__(self):
        super().__init__('battery_forwarder')
        self.subscription = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # percentage 값 소수점 아래 내림
        percentage = math.floor(msg.percentage)

        # 가공된 데이터로 JSON 전송
        data = {
            "percentage": percentage,
        }
         # 메시지 출력 (터미널에서 확인 가능)
        self.get_logger().info(f"Received battery data: {data}")

        try:
            # 가공된 percentage만 FastAPI로 전송
            requests.post("http://192.168.100.165:8000/api/battery-state", json=data)

            self.get_logger().info("Battery data sent to FastAPI")
        except Exception as e:
            self.get_logger().error(f"Failed to send: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryForwarder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

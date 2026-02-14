import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import math
import time

class VirtualWaterSensor(Node):
    def __init__(self):
        super().__init__('virtual_water_sensor')

        # 1. 구독: 로봇이 얼마나 기울어졌는지 감시 (J6 관절)
        self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.joint_callback,
            10
        )

        # 2. 발행: 가짜 무게 데이터를 쏨 (/loadcell_weight)
        self.weight_pub = self.create_publisher(Float32, '/loadcell_weight', 10)

        # 변수 설정
        self.current_tilt = 0.0  # 현재 기울기
        self.current_weight = 0.0 # 현재 무게 (컵에 담긴 양)
        self.is_pouring = False   # 붓는 중인가?

        # 0.1초마다 무게 계산해서 보내기
        self.timer = self.create_timer(0.1, self.update_weight)
        
        self.get_logger().info('💧 가상 물 붓기 센서가 켜졌습니다!')
        self.get_logger().info('👉 로봇 6번 축(J6)을 90도 근처로 기울이면 무게가 올라갑니다.')

    def joint_callback(self, msg):
        # 관절 데이터가 6개 이상일 때만
        if len(msg.position) >= 6:
            # J6(마지막 관절)의 각도를 가져옴 (라디안 -> 도 변환)
            # 로봇 마다 붓는 축이 다를 수 있지만 보통 J6(손목)이나 J5를 씁니다.
            j6_angle = math.degrees(msg.position[5])
            self.current_tilt = j6_angle

    def update_weight(self):
        # === [가상 물리 법칙] ===
        # 로봇 손목(J6)이 80도 이상 기울어지면 물이 쏟아진다고 가정
        # (각도는 Rviz에서 보면서 조절 필요, 보통 90도가 수직)
        
        POURING_THRESHOLD = 45.0  # 이 각도보다 더 꺾이면 붓기 시작
        POURING_SPEED = 2.5       # 0.1초당 증가하는 무게 (즉, 초당 25g)

        # 절대값 사용 (왼쪽으로 꺾든 오른쪽으로 꺾든)
        if abs(self.current_tilt) > POURING_THRESHOLD:
            self.current_weight += POURING_SPEED
            if not self.is_pouring:
                self.get_logger().info(f'🚰 콸콸콸! 붓는 중... (각도: {self.current_tilt:.1f}°)')
                self.is_pouring = True
        else:
            if self.is_pouring:
                self.get_logger().info('🛑 멈춤 (각도가 돌아옴)')
                self.is_pouring = False

        # 가짜 무게 발행
        msg = Float32()
        msg.data = self.current_weight
        self.weight_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualWaterSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('센서 종료.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
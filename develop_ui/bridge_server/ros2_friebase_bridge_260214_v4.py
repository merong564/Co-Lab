import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
import math

class VirtualWaterSensor(Node):
    def __init__(self):
        super().__init__('virtual_water_sensor')

        # 구독: 로봇 각도 & UI 명령
        self.create_subscription(JointState, '/dsr01/joint_states', self.joint_callback, 10)
        self.create_subscription(String, '/ui_command', self.command_callback, 10)

        # 발행: 가짜 무게
        self.weight_pub = self.create_publisher(Float32, '/loadcell_weight', 10)

        # 상태 변수
        self.current_tilt = 0.0
        self.current_weight = 0.0 
        self.is_active = False # ★ 핵심: 시작 신호 오기 전엔 작동 안 함

        # 0.1초마다 무게 계산
        self.timer = self.create_timer(0.1, self.update_weight)
        
        self.get_logger().info('💧 [Sensor] 대기 중... UI에서 [실험 시작]을 눌러주세요.')

    def command_callback(self, msg):
        cmd = msg.data
        if cmd == "START":
            self.is_active = True
            self.current_weight = 0.0 # 시작 시 무게 리셋
            self.get_logger().info('▶ [Sensor] 시뮬레이션 활성화! (무게 0g 초기화)')
        elif cmd == "STOP":
            self.is_active = False
            self.get_logger().warn('⏹ [Sensor] 시뮬레이션 중지.')

    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            # J6 각도 (도)
            self.current_tilt = math.degrees(msg.position[5])

    def update_weight(self):
        # 비활성 상태면 무게 발행만 하고 계산은 안 함
        if not self.is_active:
            msg = Float32()
            msg.data = self.current_weight
            self.weight_pub.publish(msg)
            return

        # === 가상 물리 법칙 ===
        POURING_THRESHOLD = 45.0  # 45도 이상 기울면
        POURING_SPEED = 1.5       # 속도 조절

        if abs(self.current_tilt) > POURING_THRESHOLD:
            self.current_weight += POURING_SPEED
            
        # 무게 발행
        msg = Float32()
        msg.data = self.current_weight
        self.weight_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualWaterSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
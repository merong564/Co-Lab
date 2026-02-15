import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String
import math

# [중요] Bridge V6와 통신하기 위해 커스텀 메시지 사용
try:
    from colab_interfaces.msg import UiInput
except ImportError:
    print("⚠️ [Error] colab_interfaces 패키지를 찾을 수 없습니다. (source install/setup.bash 확인)")
    class UiInput: pass

class VirtualWaterSensor(Node):
    def __init__(self):
        super().__init__('virtual_water_sensor')

        # 1. 구독: 로봇 관절 (기울기 감지)
        self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.joint_callback,
            10
        )

        # 2. [변경] 구독: 실험 시작 신호 (Bridge V6의 /ui/input)
        self.create_subscription(
            UiInput,
            '/ui/input',
            self.ui_input_callback,
            10
        )

        # 3. [추가] 구독: 긴급 정지 신호
        self.create_subscription(
            String,
            '/ui/stop',
            self.stop_callback,
            10
        )

        # 4. 발행: 가짜 무게 데이터 (/loadcell_weight)
        self.weight_pub = self.create_publisher(Float32, '/loadcell_weight', 10)

        # 상태 변수
        self.current_tilt = 0.0
        self.current_weight = 0.0 
        self.is_active = False # 실험 시작 전에는 물 안 나옴

        # 0.1초마다 무게 계산
        self.timer = self.create_timer(0.1, self.update_weight)
        
        self.get_logger().info('💧 가상 센서 V3 준비 완료 (Topic: /ui/input 대기 중)')

    def ui_input_callback(self, msg):
        """UI에서 [실험 시작]을 누르면 호출됨"""
        if msg.is_confirmed:
            self.current_weight = 0.0  # 무게 리셋 (0g)
            self.is_active = True      # 물 붓기 기능 활성화
            self.get_logger().info(f'▶ [Sensor] 실험 시작! 무게 0g 초기화 (목표: {msg.target_weight}g)')

    def stop_callback(self, msg):
        """UI에서 [긴급 정지]를 누르면 호출됨"""
        self.is_active = False
        self.get_logger().warn('🚨 [Sensor] 긴급 정지! 물 붓기 중단.')

    def joint_callback(self, msg):
        if len(msg.position) >= 6:
            # J6 각도 (도 단위 변환)
            self.current_tilt = math.degrees(msg.position[5])

    def update_weight(self):
        # 1. 활성화 상태가 아니면 무게만 유지하고 계산 안 함
        if not self.is_active:
            self.publish_weight()
            return

        # 2. [가상 물리 법칙]
        POURING_THRESHOLD = 45.0  # 기울기 임계값
        POURING_SPEED = 2.5       # 유속 (0.1초당 2.5g = 초당 25g)

        # 기울기가 임계값을 넘으면 무게 증가
        if abs(self.current_tilt) > POURING_THRESHOLD:
            self.current_weight += POURING_SPEED
            # 로그는 너무 자주 뜨지 않게 1초에 한 번 정도만 뜨게 조절하면 좋음 (여기선 생략)
        
        # 3. 데이터 발행
        self.publish_weight()

    def publish_weight(self):
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
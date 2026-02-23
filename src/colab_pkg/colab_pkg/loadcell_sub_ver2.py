import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class LoadCellSubscriber(Node):
    def __init__(self):
        # 1. 노드 이름도 클래스 이름에 맞춰 'load_cell_subscriber'로 설정
        super().__init__('load_cell_subscriber')

        # 2. 구독 설정 (토픽: 'load_cell/weight')
        self.subscription = self.create_subscription(
            Float32,
            'load_cell/weight',
            self.listener_callback,
            10
        )
        
        # 'unused variable' 경고 방지용 (필수는 아님)
        self.subscription  

        self.get_logger().info('✅ Load Cell Subscriber가 시작되었습니다. 데이터를 기다리는 중...')

    def listener_callback(self, msg):
        """
        데이터가 들어올 때마다 단순히 로그만 출력하는 콜백 함수
        """
        # 수신된 무게 값 (msg.data)
        weight = msg.data
        
        # 로그 출력 (소수점 2자리까지 표시 예시)
        self.get_logger().info(f'수신된 무게: {weight:.2f} g')

def main(args=None):
    rclpy.init(args=args)

    try:
        # 클래스 이름 변경 반영
        subscriber_node = LoadCellSubscriber()
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'subscriber_node' in locals():
            subscriber_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
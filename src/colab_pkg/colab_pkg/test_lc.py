import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import random

class TestLoadCell(Node):
    def __init__(self):
        super().__init__('test_load_cell_node')
        
        # 실제 로드셀 토픽 이름과 맞춰야 함
        self.publisher_ = self.create_publisher(Float32, '/load_cell/weight', 10)
        
        # 0.1초(10Hz)마다 실행
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.simulated_weight = 0.0
        self.get_logger().info("✅ Load Cell Simulator Started. Publishing to /load_cell/weight")

    def timer_callback(self):
        # 시나리오: 0g에서 시작해서 서서히 무게가 증가한다고 가정
        # (실제로는 로봇이 기울일 때만 늘어나겠지만, 테스트용으로 계속 증가시킴)
        
        # 약간의 노이즈 추가 (센서 현실감)
        noise = random.uniform(-0.1, 0.1)
        current_val = self.simulated_weight + noise
        
        msg = Float32()
        msg.data = current_val
        self.publisher_.publish(msg)
        
        # self.get_logger().info(f'Pub: {current_val:.2f} g')

    # 외부에서 무게를 강제로 늘리는 함수 (필요시 사용)
    def increase_weight(self, amount):
        self.simulated_weight += amount

def main(args=None):
    rclpy.init(args=args)
    node = TestLoadCell()
    
    try:
        # 간단한 시나리오: 10초 뒤부터 무게가 조금씩 늘어남
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # 예시: 500g이 될 때까지 천천히 증가
            if node.simulated_weight < 500:
                node.simulated_weight += 0.5  # 0.1초당 0.5g 증가 (초당 5g)
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
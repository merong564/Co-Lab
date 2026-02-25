#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from collections import deque
import math

class ContinuousNoiseAnalyzer(Node):
    def __init__(self):
        super().__init__('noise_analyzer')
        self.subscription = self.create_subscription(
            Float32,
            '/dsr01/load_cell/weight',
            self.listener_callback,
            10)
        
        # 최근 2초(약 200개 데이터)를 기억하는 '구르는 창문(Rolling Window)'
        self.window_size = 200 
        self.samples = deque(maxlen=self.window_size)
        self.print_counter = 0
        
        self.get_logger().info("🔍 [실시간 노이즈 분석기] 실행됨.")
        self.get_logger().info("💡 알갱이를 붓고 난 뒤, 약 2초 후 터미널에 뜨는 '안정화 노이즈'를 확인하세요!")

    def listener_callback(self, msg):
        self.samples.append(msg.data)
        self.print_counter += 1
        
        # 큐가 다 차고(2초 경과), 1초(약 100개 샘플)마다 한 번씩 화면에 결과 출력
        if len(self.samples) == self.window_size and self.print_counter >= 100:
            self.analyze_noise()
            self.print_counter = 0  # 카운터 초기화

    def analyze_noise(self):
        mean_val = sum(self.samples) / len(self.samples)
        max_val = max(self.samples)
        min_val = min(self.samples)
        noise_range = max_val - min_val  # 최대 진폭 (Peak-to-Peak)
        
        # 표준 편차 계산
        variance = sum([((x - mean_val) ** 2) for x in self.samples]) / len(self.samples)
        std_dev = math.sqrt(variance)

        self.get_logger().info(f"▶ [현재 유지 무게]: {mean_val:.3f} g | 🚨 [진동/오차폭]: {noise_range:.3f} g | [편차]: {std_dev:.3f} g")

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousNoiseAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
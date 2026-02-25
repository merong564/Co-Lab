#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class NoiseAnalyzer(Node):
    def __init__(self):
        super().__init__('noise_analyzer')
        self.subscription = self.create_subscription(
            Float32,
            '/dsr01/load_cell/weight',
            self.listener_callback,
            10)
        self.samples = []
        self.max_samples = 200  # 약 2초 간의 데이터 수집
        self.get_logger().info("🔍 [노이즈 분석기] 데이터 수집을 시작합니다. (약 2초 대기...)")

    def listener_callback(self, msg):
        self.samples.append(msg.data)
        
        if len(self.samples) == self.max_samples:
            self.analyze_noise()

    def analyze_noise(self):
        mean_val = sum(self.samples) / len(self.samples)
        max_val = max(self.samples)
        min_val = min(self.samples)
        noise_range = max_val - min_val  # 최대 진폭 (Peak-to-Peak)
        
        # 표준 편차 계산
        variance = sum([((x - mean_val) ** 2) for x in self.samples]) / len(self.samples)
        std_dev = math.sqrt(variance)

        self.get_logger().info("\n=========================================")
        self.get_logger().info("📊 [로드셀 노이즈 정밀 분석 결과]")
        self.get_logger().info(f"▶ 측정된 평균 무게 : {mean_val:.3f} g")
        self.get_logger().info(f"▶ 최고 튀는 값(Max): {max_val:.3f} g")
        self.get_logger().info(f"▶ 최저 튀는 값(Min): {min_val:.3f} g")
        self.get_logger().info(f"🚨 최대 진폭(오차폭): {noise_range:.3f} g (이 값이 중요합니다!)")
        self.get_logger().info(f"▶ 표준 편차        : {std_dev:.3f} g")
        self.get_logger().info("=========================================\n")
        
        if noise_range > 0.0:
            rec_window = noise_range * 1.1
            self.get_logger().info(f"💡 [튜닝 권장값] 화면의 숫자를 완전히 고정하려면 'noise_window'를 {rec_window:.3f} 이상으로 설정해야 합니다.")
        
        # 분석이 끝나면 노드 종료
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = NoiseAnalyzer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
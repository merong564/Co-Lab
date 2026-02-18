#!/usr/bin/env python3
# UI 노드와 서비스 연결 테스트용 컨트롤러 서버
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from colab_interfaces.srv import RobotCommand

class MockServer(Node):
    def __init__(self):
        super().__init__('mock_server')
        
        # 서비스 서버 생성 (/start_process)
        self.srv = self.create_service(RobotCommand, '/start_process', self.handle_start_process)
        
        # 긴급 정지 구독 (/stop)
        self.sub = self.create_subscription(String, '/stop', self.stop_callback, 10)
        
        self.get_logger().info("✅ 테스트용 서버가 시작되었습니다.")
        self.get_logger().info("대기 중: 서비스(/start_process) 및 토픽(/stop)")

    def handle_start_process(self, request, response):
        self.get_logger().info(f"📨 [서비스 요청 수신] Target Weight: {request.target_weight}g, Mixing Duration: {request.mixing_duration}s")
        
        # 응답 전송
        response.success = True
        response.message = "UI 연결 테스트 성공"
        return response

    def stop_callback(self, msg):
        self.get_logger().warn(f"🛑 [긴급 정지 수신] 메시지: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = MockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
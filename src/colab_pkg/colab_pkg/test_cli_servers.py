import rclpy
from rclpy.node import Node
from colab_interfaces.srv import RobotCommand

class MockServerNode(Node):
    def __init__(self):
        super().__init__('MockServerNode')
        
        # 1. ScaleDriver 역할 (/set_tare)
        self.srv_scale = self.create_service(RobotCommand, '/set_tare', self.handle_service)
        
        # 2. TaskTransfer 역할 (/execute_transfer)
        self.srv_transfer = self.create_service(RobotCommand, '/execute_transfer', self.handle_service)
        
        # 3. TaskPouring 역할 (/execute_pouring)
        self.srv_pouring = self.create_service(RobotCommand, '/execute_pouring', self.handle_service)
        
        # 4. TaskMixing 역할 (/execute_mixing)
        self.srv_mixing = self.create_service(RobotCommand, '/execute_mixing', self.handle_service)

        self.get_logger().info("👻 Mock Servers Started! Ready to accept commands.")

    def handle_service(self, request, response):
        """모든 요청에 대해 무조건 성공 응답을 보냄"""
        # 현재 호출된 서비스가 무엇인지 로그로 확인
        self.get_logger().info(f"📩 [Mock] Received Request | Mode: {request.mode}, Weight: {request.target_weight}, Time: {request.mixing_duration}")
        
        # 1초 정도 걸리는 척 (옵션)
        import time; time.sleep(1.0) 

        response.success = True
        response.message = "Simulation Success"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
import sys

# 서비스 타입 임포트
from colab_interfaces.srv import RobotCommand

class SystemController(Node):
    def __init__(self):
        super().__init__('SystemController')
        
        # 클라이언트 생성: /execute_pouring 서비스 연결
        self.cli_pouring = self.create_client(RobotCommand, '/execute_pouring')
        
        # 서비스 서버(TaskPouring)가 켜질 때까지 대기
        self.get_logger().info('Waiting for TaskPouring server...')
        while not self.cli_pouring.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
            
        self.get_logger().info('✅ Service Server Connected!')

    def send_pouring_request(self, target_weight):
        """TaskPouring 노드에 붓기 명령 전송"""
        req = RobotCommand.Request()
        req.target_val = float(target_weight) # 실수형으로 변환하여 전송
        
        self.get_logger().info(f"[Request] Sending command: Pour {target_weight}g ...")
        
        # 비동기 요청 전송
        future = self.cli_pouring.call_async(req)
        
        # 응답 대기 (Blocking)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    
    controller = SystemController()

    try:
        while rclpy.ok():
            print("\n" + "="*40)
            print(" [System Controller] Command Interface")
            print("="*40)
            user_input = input("Enter Target Weight (g) or 'q' to quit: ")
            
            if user_input.lower() == 'q':
                break
            
            try:
                target_val = float(user_input)
                
                # 서비스 요청 및 결과 수신
                response = controller.send_pouring_request(target_val)
                
                # 결과 출력
                if response.success:
                    print(f"✅ Success: {response.message}")
                else:
                    print(f"❌ Failed: {response.message}")
                    
            except ValueError:
                print("[ERROR] Please enter a valid number.")
            except Exception as e:
                print(f"[ERROR] Communication Error: {e}")

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
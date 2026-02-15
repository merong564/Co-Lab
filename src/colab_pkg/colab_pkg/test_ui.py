import rclpy
from rclpy.node import Node
from colab_interfaces.msg import UiInput
import sys

class TestUiPublisher(Node):
    def __init__(self):
        super().__init__('test_ui_node')
        
        # UI 토픽 퍼블리셔 생성
        self.publisher_ = self.create_publisher(UiInput, '/ui/command', 10)
        self.get_logger().info("UI Simulator Started.")

    def publish_command(self, weight):
        msg = UiInput()
        msg.target_weight = float(weight)
        msg.is_confirmed = True  # 사용자가 입력했으므로 확인된 것으로 간주
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command -> Target: {weight} g, Confirmed: True")

def main(args=None):
    rclpy.init(args=args)
    node = TestUiPublisher()

    try:
        print("Enter target weight (g) to publish. Press Ctrl+C to exit.")
        
        # 계속 입력을 받기 위한 루프
        while rclpy.ok():
            try:
                # 1. 사용자 입력 대기
                user_input = input("Target Weight (g): ")
                
                # 2. 입력값 검증 및 변환
                target_weight = float(user_input)
                
                # 3. 메시지 발행
                node.publish_command(target_weight)
                
            except ValueError:
                print("Invalid input. Please enter a number.")
            except Exception as e:
                print(f"Error: {e}")

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
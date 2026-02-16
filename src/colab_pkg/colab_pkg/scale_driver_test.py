import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from colab_interfaces.srv import RobotCommand
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import time
import random  # ✅ [추가] 시뮬레이션용

ROBOT_ID = "dsr01"

class ScaleDriver(Node):
    def __init__(self):
        super().__init__('scale_driver', namespace=ROBOT_ID)
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 1. 설정 변수
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.current_weight = 0.0
        self.is_active = False
        self.ser = None
        
        # ✅ [추가] 시뮬레이션 상태 변수
        self.simulated_weight = 0.0
        
        # 2. 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        
        # 3. 서비스 서버 생성
        self.srv_pouring = self.create_service(
            RobotCommand,
            'set_tare',
            self.execute_pouring_callback,
            callback_group=self.callback_group
        )

        # 4. 타이머 설정
        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.callback_group)

    def execute_pouring_callback(self, request, response):
        self.get_logger().info(f"[Service] Request Received. Connecting to Arduino for Tare...")
        
        # 이미 연결되어 있다면 닫고 새로 연결
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            time.sleep(0.5)

        # ✅ 서비스 호출이 들어오면 "퍼블리시 시작" 상태로 전환 (실기기 없어도 테스트 가능)
        self.is_active = True

        # ✅ [추가] tare 의미로 시뮬 무게도 0으로 리셋
        self.simulated_weight = 0.0

        # 시리얼 연결 시도 (없으면 실패하더라도 시뮬 publish로 동작)
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'✅ 아두이노 연결 및 영점 조절 시작: {self.port}')
            
            time.sleep(2) 
            self.ser.reset_input_buffer()
            
            response.success = True
            response.message = "Tare Completed and Publishing Started (REAL Serial)"
            
        except serial.SerialException as e:
            self.get_logger().warn(f'⚠️ 아두이노 연결 실패. 시뮬레이션 퍼블리시로 대체합니다: {e}')
            
            # ✅ 시리얼 없이도 서비스는 성공으로 응답 (테스트 목적)
            response.success = True
            response.message = f"Tare Completed and Publishing Started (SIM). Serial Failed: {str(e)}"
            
            # ✅ ser는 None으로 두거나(유지), 명확히 None 처리
            self.ser = None
            
        return response

    def timer_callback(self):
        # ✅ 기존 형식 유지: is_active일 때만 publish
        if self.is_active:
            # 1) 실제 시리얼 데이터가 있으면 -> 실제 값 publish (원래 로직 유지)
            if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode('utf-8').strip()
                    
                    if line:
                        try:
                            weight_value = float(line)
                            self.current_weight = weight_value
                            
                            msg = Float32()
                            msg.data = weight_value
                            self.publisher_.publish(msg)
                            
                        except ValueError:
                            self.get_logger().warn(f'잘못된 데이터 무시함: {line}')
                            
                except Exception as e:
                    self.get_logger().error(f'데이터 읽기 중 에러: {e}')

            # 2) 시리얼이 없거나 열려있지 않으면 -> 시뮬 값 publish
            else:
                # 시뮬 시나리오: 조금씩 증가 + 노이즈
                self.simulated_weight += 0.05  # 0.01초마다 0.05g 증가 => 초당 5g
                noise = random.uniform(-0.1, 0.1)
                weight_value = self.simulated_weight + noise

                self.current_weight = weight_value

                msg = Float32()
                msg.data = float(weight_value)
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ScaleDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        if 'node' in locals() and node.ser is not None:
            try:
                if node.ser.is_open:
                    node.ser.close()
            except Exception:
                pass
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

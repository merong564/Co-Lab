import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from colab_interfaces.srv import RobotCommand
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import time

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
        self.ser = None # [추가] 시리얼 객체 초기화
        
        # 2. 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        
        # 3. 서비스 서버 생성
        self.srv_pouring = self.create_service(
            RobotCommand,
            'set_tare',
            self.set_tare_callback,
            callback_group=self.callback_group
        )

        # 4. 타이머 설정
        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.callback_group)

    def set_tare_callback(self, request, response):
        self.get_logger().info(f"[Service] Request Received. Connecting to Arduino for Tare...")
        
        # [추가] 이미 연결되어 있다면 닫고 새로 연결 (재부팅 유도하여 영점 조절 수행)
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            time.sleep(1.0)

        # [수정] 서비스 호출 시 시리얼 연결 수행
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'✅ 아두이노 연결 및 영점 조절 시작: {self.port}')
            
            # 아두이노 부팅 및 영점 조절 완료 대기 시간
            time.sleep(2) 
            self.ser.reset_input_buffer()
            
            self.is_active = True
            response.success = True
            response.message = "Tare Completed and Publishing Started"
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')
            response.success = False
            response.message = f"Serial Connection Failed: {str(e)}"
            
        return response

    def timer_callback(self):
        # [수정] self.ser가 None이 아니고 연결된 상태인지 확인 추가
        if self.is_active and self.ser and self.ser.is_open and self.ser.in_waiting > 0:
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
            node.ser.close()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
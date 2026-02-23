import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class LoadCellPublisher(Node):
    def __init__(self):
        super().__init__('load_cell_node')
        
        # 1. 설정 변수
        self.port = '/dev/ttyACM0'  # 아두이노 포트 (확인 필요: ls /dev/tty*)
        self.baudrate = 115200      # 아두이노 코드와 동일해야 함
        
        # 2. 퍼블리셔 생성 (토픽 이름: 'load_cell/weight', 메시지 타입: Float32)
        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        
        # 3. 시리얼 연결 시도
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'✅ 아두이노 연결 성공: {self.port}')
            # 아두이노가 재부팅될 때까지 잠시 대기 (안정화)
            time.sleep(2)
            self.ser.reset_input_buffer() 
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')
            # 연결 실패 시 노드 종료 방지를 위해 ser를 None으로 설정하거나 여기서 종료 처리
            raise e

        # 4. 타이머 설정 (0.01초마다 실행 - 아두이노가 0.05초마다 보내므로 충분히 빠름)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        #self.get_logger().info(f'{self.ser}, {self.ser.in_waiting}')
        if self.ser and self.ser.in_waiting > 0:
            try:
                # 1. 시리얼 데이터 한 줄 읽기 (바이트 -> 문자열 디코딩 -> 공백제거)
                line = self.ser.readline().decode('utf-8').strip()
                #self.get_logger().info(line)
                
                # 2. 데이터가 비어있지 않으면 처리
                if line:
                    try:
                        # 문자열을 실수(float)로 변환
                        weight_value = float(line)
                        
                        # 3. ROS 메시지로 포장
                        msg = Float32()
                        msg.data = weight_value
                        
                        # 4. 토픽 발행
                        self.publisher_.publish(msg)
                        
                        # (디버깅용) 터미널에 로그 출력 - 잘 되면 주석 처리하세요
                        # self.get_logger().info(f'Published: {weight_value} g')
                        
                    except ValueError:
                        # 가끔 통신 노이즈로 "150.4a" 같은 이상한 값이 오면 무시
                        self.get_logger().warn(f'잘못된 데이터 무시함: {line}')
                        
            except Exception as e:
                self.get_logger().error(f'데이터 읽기 중 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LoadCellPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        # 종료 시 리소스 정리
        if 'node' in locals():
            node.ser.close()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
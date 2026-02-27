#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from colab_interfaces.srv import RobotCommand
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import time

ROBOT_ID = "dsr01"

class ScaleDriverALPF(Node):
    def __init__(self):
        super().__init__('scale_driver', namespace=ROBOT_ID)
        self.callback_group = ReentrantCallbackGroup()
        
        # 1. 시리얼 및 하드웨어 설정
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.is_active = False
        self.ser = None
        self.cal_ratio = 190.0 / 187.8  
        
        # 💡 [적응형 LPF 설정]
        self.filtered_weight = None     
        self.last_printed_weight = None
        self.min_alpha = 0.05   # 최소 가중치 (안정적일 때)
        self.max_alpha = 0.8    # 최대 가중치 (급격히 변할 때)
        self.current_alpha = 0.05
        
        # 2. ROS2 퍼블리셔 및 서비스
        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        self.srv_pouring = self.create_service(
            RobotCommand, 'set_tare', self.execute_pouring_callback,
            callback_group=self.callback_group
        )
        
        # 0.01초(10ms)마다 데이터 수집
        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.callback_group)

    def execute_pouring_callback(self, request, response):
        if self.ser and self.ser.is_open: self.ser.close()
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.filtered_weight = None 
            self.is_active = True
            self.get_logger().info(f'✅ [적응형 LPF 모드] 아두이노 연결 완료')
            response.success = True
            response.message = "Adaptive LPF Tare Completed"
        except Exception as e:
            self.get_logger().error(f'❌ 연결 실패: {e}')
            response.success = False
            response.message = str(e)
        return response

    def timer_callback(self):
        if self.is_active and self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    raw_weight = float(line) * self.cal_ratio
                    
                    if self.filtered_weight is None:
                        self.filtered_weight = raw_weight
                    else:
                        # 🧠 적응형 가중치 알고리즘: 오차(Error)가 클수록 alpha도 커짐
                        error = abs(raw_weight - self.filtered_weight)
                        self.current_alpha = min(max(error * 0.1, self.min_alpha), self.max_alpha)
                        
                        # 필터 연산
                        self.filtered_weight = (self.current_alpha * raw_weight) + ((1.0 - self.current_alpha) * self.filtered_weight)
                    
                    val = round(self.filtered_weight, 3)

                    # 💡 청중 시각화용 로그 출력 (📈 이모지와 현재 가중치 표시)
                    if val != self.last_printed_weight:
                        self.get_logger().info(
                            f"📈 [적응형 LPF]: {val:.3f} g (가중치 alpha: {self.current_alpha:.2f})"
                        )
                        self.last_printed_weight = val

                    msg = Float32()
                    msg.data = val
                    self.publisher_.publish(msg)
            except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ScaleDriverALPF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
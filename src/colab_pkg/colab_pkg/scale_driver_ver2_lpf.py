#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from colab_interfaces.srv import RobotCommand
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import time

ROBOT_ID = "dsr01"

class ScaleDriverLPF(Node):
    def __init__(self):
        super().__init__('scale_driver', namespace=ROBOT_ID)
        self.callback_group = ReentrantCallbackGroup()
        
        # 설정값
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.is_active = False
        self.ser = None
        self.cal_ratio = 190.0 / 187.8  
        
        # 💡 고정형 LPF 파라미터 (지연 발생 확인용)
        self.lpf_alpha = 0.1 
        self.filtered_weight = None     
        self.last_printed_weight = None

        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        self.srv_pouring = self.create_service(
            RobotCommand, 'set_tare', self.execute_pouring_callback,
            callback_group=self.callback_group
        )
        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.callback_group)

    def execute_pouring_callback(self, request, response):
        if self.ser and self.ser.is_open: self.ser.close()
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            self.ser.reset_input_buffer()
            self.filtered_weight = None 
            self.is_active = True
            self.get_logger().info(f'✅ [LPF 모드] 아두이노 연결 완료')
            response.success = True
            response.message = "Fixed LPF Tare Completed"
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
                        # 고정형 LPF 공식
                        self.filtered_weight = (self.lpf_alpha * raw_weight) + ((1.0 - self.lpf_alpha) * self.filtered_weight)
                    
                    val = round(self.filtered_weight, 3)

                    # 💡 [핵심] 터미널 출력 추가 (청중 시각화용)
                    if val != self.last_printed_weight:
                        self.get_logger().info(f"📉 [고정형 LPF]: {val:.3f} g (응답 지연 관찰 중)")
                        self.last_printed_weight = val

                    msg = Float32()
                    msg.data = val
                    self.publisher_.publish(msg)
            except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ScaleDriverLPF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser: node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()
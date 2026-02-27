#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from colab_interfaces.srv import RobotCommand
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import time

ROBOT_ID = "dsr01"

class ScaleDriverKalman(Node):
    def __init__(self):
        super().__init__('scale_driver', namespace=ROBOT_ID)
        self.callback_group = ReentrantCallbackGroup()
        
        # 1. 시리얼 및 하드웨어 설정
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.is_active = False
        self.ser = None
        self.cal_ratio = 190.0 / 187.8  
        
        # 💡 [칼만 필터 파라미터 설정]
        # 이전에 논의했던 폭주 현상을 재현하기 위한 표준 세팅입니다.
        self.x = 0.0  # 추정 상태 (무게)
        self.p = 1.0  # 추정 오차 공분산
        self.q = 0.01 # 프로세스 노이즈 (시스템 변동성)
        self.r = 0.1  # 측정 노이즈 (센서 오차)
        
        self.last_printed_weight = None

        # 2. ROS2 퍼블리셔 및 서비스
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
            
            # 필터 초기화
            self.x, self.p = 0.0, 1.0
            self.is_active = True
            
            self.get_logger().info(f'✅ [칼만 필터 모드] 아두이노 연결 및 초기화 완료')
            response.success = True
            response.message = "Kalman Filter Tare Completed"
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
                    z = float(line) * self.cal_ratio # 측정값(Measurement)
                    
                    # 🧠 칼만 필터 알고리즘 (Prediction & Update)
                    # 1. Predict
                    p_prior = self.p + self.q
                    
                    # 2. Kalman Gain
                    k_gain = p_prior / (p_prior + self.r)
                    
                    # 3. Update
                    self.x = self.x + k_gain * (z - self.x)
                    self.p = (1 - k_gain) * p_prior
                    
                    val = round(self.x, 3)

                    # 💡 청중 시각화용 로그 출력 (🌀 이모지와 상태 경고 추가)
                    if val != self.last_printed_weight:
                        # 우리가 경험했던 0.786g 근처에서 경고가 뜨도록 설정
                        status = "⚠️ 수치 폭주 위험" if abs(val) > 0.5 else "정상 연산 중"
                        self.get_logger().info(
                            f"🌀 [칼만 필터 연산]: {val:.3f} g ({status})"
                        )
                        self.last_printed_weight = val

                    msg = Float32()
                    msg.data = val
                    self.publisher_.publish(msg)
            except: pass

def main(args=None):
    rclpy.init(args=args)
    node = ScaleDriverKalman()
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
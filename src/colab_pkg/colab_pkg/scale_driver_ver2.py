#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from colab_interfaces.srv import RobotCommand
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import time
import statistics
from collections import deque

ROBOT_ID = "dsr01"

class ScaleDriver(Node):
    def __init__(self):
        super().__init__('scale_driver', namespace=ROBOT_ID)
        
        self.callback_group = ReentrantCallbackGroup()
        
        # 1. 시리얼 통신 설정
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.is_active = False
        self.ser = None 
        
        # 💡 [캘리브레이션] 하드웨어 오차 보정
        self.cal_ratio = 190.0 / 187.8  
        
        # 💡 [미디언 + LPF 하이브리드 설정]
        # 최근 5개의 샘플 중 중간값만 취하여 스파이크 노이즈를 원천 차단합니다.
        self.window_size = 5
        self.raw_buffer = deque(maxlen=self.window_size)
        
        # 미디언이 노이즈를 걸러주므로, 반응성을 위해 알파를 0.4로 높게 잡습니다.
        self.lpf_alpha = 0.4
        self.filtered_weight = None     
        self.published_weight = 0.0     
        
        # 분석기 데이터를 기반으로 한 환경 제어 변수
        self.noise_window = 0.05      # PD 제어용 연속 신호 확보
        self.zero_deadband = 0.55     # 0.55g 이하 초기 진동 무시
        self.jump_threshold = 10.0    # 고체 투입 시 즉시 반영
        
        self.last_printed_weight = None
        
        # 2. 퍼블리셔 및 서비스 생성
        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        self.srv_pouring = self.create_service(
            RobotCommand,
            'set_tare',
            self.execute_pouring_callback,
            callback_group=self.callback_group
        )

        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.callback_group)

    def execute_pouring_callback(self, request, response):
        self.get_logger().info(f"[Service] Tare Request Received...")
        
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            time.sleep(0.5)

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'✅ 아두이노 연결 완료: {self.port}')
            
            time.sleep(2) 
            self.ser.reset_input_buffer()
            
            # 버퍼 및 필터 초기화
            self.raw_buffer.clear()
            for _ in range(self.window_size):
                self.raw_buffer.append(0.0)
                
            self.filtered_weight = None 
            self.published_weight = 0.0
            
            self.is_active = True
            response.success = True
            response.message = "Tare Completed"
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 연결 실패: {e}')
            response.success = False
            response.message = str(e)
            
        return response

    def timer_callback(self):
        if self.is_active and self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                if line:
                    try:
                        raw_weight = float(line) * self.cal_ratio
                        
                        # 점프 감지 시 버퍼 강제 동기화
                        if self.filtered_weight is not None and abs(raw_weight - self.filtered_weight) > self.jump_threshold:
                            self.raw_buffer.clear()
                            for _ in range(self.window_size):
                                self.raw_buffer.append(raw_weight)

                        self.raw_buffer.append(raw_weight)

                        # 🧠 1단계: 미디언 필터 (가장 튀는 값 암살)
                        median_weight = statistics.median(self.raw_buffer)

                        # 🧠 2단계: LPF (부드러운 곡선 생성)
                        if self.filtered_weight is None:
                            self.filtered_weight = median_weight
                        else:
                            self.filtered_weight = (self.lpf_alpha * median_weight) + ((1.0 - self.lpf_alpha) * self.filtered_weight)
                        
                        precise_weight = round(self.filtered_weight, 3)
                        
                        # 데드밴드 및 영점 처리
                        diff = abs(precise_weight - self.published_weight)
                        if diff > self.noise_window:
                            self.published_weight = precise_weight
                            
                        if abs(self.published_weight) <= self.zero_deadband:
                            self.published_weight = 0.0

                        if self.published_weight < 0.0:
                            self.published_weight = 0.0

                        if self.published_weight != self.last_printed_weight:
                            self.get_logger().info(f"⚖️ [미디언 필터링]: {self.published_weight:.3f} g")
                            self.last_printed_weight = self.published_weight

                        msg = Float32()
                        msg.data = self.published_weight 
                        self.publisher_.publish(msg)
                        
                    except ValueError:
                        pass 
                        
            except Exception as e:
                self.get_logger().error(f'데이터 읽기 에러: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = ScaleDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals() and node.ser is not None:
            node.ser.close()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
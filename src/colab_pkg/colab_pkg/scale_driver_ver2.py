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
        
        # 1. 시리얼 통신 설정 (아두이노와 연결)
        self.port = '/dev/ttyACM0'  # 아두이노가 연결된 USB 포트
        self.baudrate = 115200      # 통신 속도 (아두이노 코드와 동일해야 함)
        self.is_active = False      # 통신 활성화 상태 플래그
        self.ser = None             # 시리얼 객체를 담을 변수
        
        # 💡 [캘리브레이션] 하드웨어 오차 보정
        self.cal_ratio = 190.0 / 187.8  
        
        # 💡 [미디언 + LPF 하이브리드 설정]
        # 최근 5개의 샘플 중 중간값만 취하여 스파이크 노이즈를 원천 차단합니다.
        # 큐(deque)를 사용하여 최근 7개의 샘플을 보관합니다. 
        # maxlen=7 이므로 8번째 데이터가 들어오면 가장 오래된 데이터가 자동으로 밀려납니다.
        self.window_size = 7
        self.raw_buffer = deque(maxlen=self.window_size)
        
        # 미디언 필터로 튀는 값(스파이크 노이즈)을 먼저 잡았기 때문에, 
        # LPF(로우패스필터)의 가중치(alpha)를 0.25로 다소 높게 잡아 부드러우면서도 '반응 속도'를 살렸습니다.
        self.lpf_alpha = 0.25
        self.filtered_weight = None     
        self.published_weight = 0.0     
        
        # 분석기 데이터를 기반으로 한 환경 제어 변수 (노이즈 컷오프)
        self.noise_window = 0.3       # 값이 0.3g 이상 변했을 때만 갱신 (미세한 떨림 무시) PD 제어용 연속 신호 확보
        self.zero_deadband = 0.55     # 0.55g 이하 초기 진동 무시
        self.jump_threshold = 10.0    # 고체 투입 시 즉시 반영
        
        self.last_printed_weight = None # 터미널 창 도배를 막기 위해 이전 출력값을 기억하는 변수
        
        # 2. 퍼블리셔 및 서비스 생성
        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        self.srv_pouring = self.create_service(
            RobotCommand,
            'set_tare',
            self.execute_pouring_callback,
            callback_group=self.callback_group
        )

        # 0.01초(10ms)마다 timer_callback 함수를 실행하여 아두이노에서 데이터를 가져옵니다.
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
            
            # 필터용 데이터 버퍼를 초기화하여 과거의 엉뚱한 값이 현재 계산에 영향을 주지 않게 합니다.
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
        """ 0.01초마다 반복 실행되며 무게를 측정하고 필터링하는 핵심 함수입니다. """
        if self.is_active and self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                if line:
                    try:
                        raw_weight = float(line) * self.cal_ratio
                        
                        # [핵심 로직] 물체가 갑자기 훅! 들어왔을 때의 처리 (Delay 방지)
                        # 현재 필터링된 값과 방금 들어온 원시 값의 차이가 jump_threshold(10g)보다 크다면
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
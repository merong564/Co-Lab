#!/usr/bin/env python3
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
        
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.is_active = False
        self.ser = None 
        
        self.cal_ratio = 190.0 / 187.8  
        
        # 💡 [세계 최고 수준: 2D 운동학 칼만 필터 설정]
        # 상태 변수: [무게(w), 유량(f)]
        self.dt = 0.01  # 100Hz 타이머 주기
        self.kf_w = 0.0 # 추정된 무게 (g)
        self.kf_f = 0.0 # 추정된 유량 (g/s)
        self.P = [[1.0, 0.0], [0.0, 1.0]] # 오차 공분산 행렬 (초기 불확실성)
        
        # 🧠 칼만 필터 튜닝 파라미터 (Q, R)
        self.Q_w = 0.001  # 모델 노이즈 (무게 변화 신뢰도)
        self.Q_f = 0.01   # 모델 노이즈 (유량 변화 신뢰도)
        
        # 🚨 [핵심] 측정 노이즈 분산 (R): 노이즈 분석기로 직접 구한 표준편차(0.08g)의 제곱!
        # 센서가 흔들리는 물리적 한계를 수학적으로 완벽히 알려줍니다.
        self.R = 0.0064   
        
        self.published_weight = 0.0
        self.noise_window = 0.05
        self.zero_deadband = 0.55
        self.jump_threshold = 10.0
        self.last_printed_weight = None
        
        self.publisher_ = self.create_publisher(Float32, 'load_cell/weight', 10)
        
        self.srv_pouring = self.create_service(
            RobotCommand,
            'set_tare',
            self.execute_pouring_callback,
            callback_group=self.callback_group
        )

        self.timer = self.create_timer(self.dt, self.timer_callback, callback_group=self.callback_group)

    def execute_pouring_callback(self, request, response):
        self.get_logger().info(f"[Service] Request Received. Connecting to Arduino for Tare...")
        
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            time.sleep(0.5)

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'✅ 아두이노 연결 및 영점 조절 시작: {self.port}')
            
            time.sleep(2) 
            self.ser.reset_input_buffer()
            
            # 영점 조절 시 칼만 필터 초기화
            self.kf_w = 0.0
            self.kf_f = 0.0
            self.P = [[1.0, 0.0], [0.0, 1.0]]
            self.published_weight = 0.0
            
            self.is_active = True
            response.success = True
            response.message = "Tare Completed and Publishing Started"
            
        except serial.SerialException as e:
            self.get_logger().error(f'❌ 아두이노 연결 실패: {e}')
            response.success = False
            response.message = f"Serial Connection Failed: {str(e)}"
            
        return response

    def timer_callback(self):
        if self.is_active and self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                
                if line:
                    try:
                        raw_weight = float(line) * self.cal_ratio
                        
                        # 🚀 고체 충격(Jump) 감지 시 칼만 필터 강제 리셋 (오작동 방지)
                        if abs(raw_weight - self.kf_w) > self.jump_threshold:
                            self.kf_w = raw_weight
                            self.kf_f = 0.0
                            self.P = [[1.0, 0.0], [0.0, 1.0]]

                        # ==========================================
                        # 🧠 2D 칼만 필터링 핵심 로직
                        # ==========================================
                        # 1. 예측 (Predict): 물리 운동학 기반 (무게 = 이전 무게 + 유량 * 시간)
                        w_pred = self.kf_w + self.kf_f * self.dt
                        f_pred = self.kf_f
                        
                        P00_pred = self.P[0][0] + self.dt * (self.P[1][0] + self.P[0][1]) + (self.dt ** 2) * self.P[1][1] + self.Q_w
                        P01_pred = self.P[0][1] + self.dt * self.P[1][1]
                        P10_pred = self.P[1][0] + self.dt * self.P[1][1]
                        P11_pred = self.P[1][1] + self.Q_f

                        # 2. 업데이트 (Update): 센서 측정값과 비교하여 오차 교정
                        y = raw_weight - w_pred  # 실제 측정값과 예측값의 차이
                        S = P00_pred + self.R    # 잔차 공분산
                        K0 = P00_pred / S        # 무게에 대한 칼만 게인
                        K1 = P10_pred / S        # 유량에 대한 칼만 게인

                        # 최종 추정값 확정
                        self.kf_w = w_pred + K0 * y
                        self.kf_f = f_pred + K1 * y

                        # 오차 공분산 업데이트
                        self.P[0][0] = (1.0 - K0) * P00_pred
                        self.P[0][1] = (1.0 - K0) * P01_pred
                        self.P[1][0] = -K1 * P00_pred + P10_pred
                        self.P[1][1] = -K1 * P01_pred + P11_pred

                        # ==========================================
                        # 후처리 (화면 및 토픽 출력용)
                        # ==========================================
                        precise_weight = round(self.kf_w, 3)
                        
                        diff = abs(precise_weight - self.published_weight)
                        if diff > self.noise_window:
                            self.published_weight = precise_weight
                            
                        if abs(self.published_weight) <= self.zero_deadband:
                            self.published_weight = 0.0

                        if self.published_weight < 0.0:
                            self.published_weight = 0.0

                        if self.published_weight != self.last_printed_weight:
                            # 💡 터미널에 깔끔해진 무게와 내부적으로 추정한 유량을 함께 출력합니다!
                            self.get_logger().info(f"⚖️ [칼만 무게]: {self.published_weight:.3f} g | 🌊 [내부 추정 유량]: {self.kf_f:.3f} g/s")
                            self.last_printed_weight = self.published_weight

                        msg = Float32()
                        msg.data = self.published_weight 
                        self.publisher_.publish(msg)
                        
                    except ValueError:
                        pass 
                        
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
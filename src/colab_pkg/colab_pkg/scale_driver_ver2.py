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
        
        # 1. 설정 변수
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.is_active = False
        self.ser = None 
        
        # 💡 [캘리브레이션 보정 계수] 
        self.cal_ratio = 190.0 / 187.8  
        
        # 💡 [초정밀 필터링 설정]
        self.lpf_alpha = 0.1            
        self.filtered_weight = None     
        self.published_weight = 0.0     
        
        self.noise_window = 0.3         
        self.zero_deadband = 0.2        
        
        # 🚀 [추가] 계단식 점프(Jump) 감지 임계값
        # 한 번에 5g 이상 훅 변하면 필터를 무시하고 즉각(계단식) 반영!
        self.jump_threshold = 5.0       
        
        self.last_printed_weight = None
        
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
        
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            time.sleep(0.5)

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'✅ 아두이노 연결 및 영점 조절 시작: {self.port}')
            
            time.sleep(2) 
            self.ser.reset_input_buffer()
            
            self.filtered_weight = None 
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
                        
                        # ====================================================
                        # 💡 [1단계] 점프 감지 + Low Pass Filter
                        # ====================================================
                        if self.filtered_weight is None:
                            self.filtered_weight = raw_weight
                        else:
                            # 🚀 원본 값과 필터값의 차이가 너무 크면(고체를 올리거나 뺐을 때)
                            if abs(raw_weight - self.filtered_weight) > self.jump_threshold:
                                self.filtered_weight = raw_weight  # 지연 없이 즉시 계단식 점프!
                            else:
                                # 평소(물 붓기, 미세 진동)에는 부드럽게 필터 적용
                                self.filtered_weight = (self.lpf_alpha * raw_weight) + ((1.0 - self.lpf_alpha) * self.filtered_weight)
                        
                        # [2단계] 소수점 3째 자리까지만 반올림
                        precise_weight = round(self.filtered_weight, 3)
                        
                        # [3단계] Hysteresis (동적 데드밴드) 
                        diff = abs(precise_weight - self.published_weight)
                        if diff > self.noise_window:
                            self.published_weight = precise_weight
                            
                        # [4단계] 절대 영점 락
                        if abs(self.published_weight) <= self.zero_deadband:
                            self.published_weight = 0.0

                        # [5단계] 마이너스 값 원천 차단
                        if self.published_weight < 0.0:
                            self.published_weight = 0.0

                        if self.published_weight != self.last_printed_weight:
                            self.get_logger().info(f"⚖️ [현재 확정 무게]: {self.published_weight:.3f} g")
                            self.last_printed_weight = self.published_weight

                        # 퍼블리시
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
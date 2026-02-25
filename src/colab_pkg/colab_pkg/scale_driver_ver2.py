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
        
        # 💡 [적응형 필터 설정] lpf_alpha 변수가 사라지고 동적으로 계산됩니다.
        self.filtered_weight = None     
        self.published_weight = 0.0     
        
        # 💡 계단 현상 방지: 0.05g 이상의 미세한 변화도 물 흐르듯 통과시킵니다.
        self.noise_window = 0.05         
        
        # 💡 영점 유령 데이터 철벽 방어: 초기 0.55g 이하의 노이즈는 무조건 0으로 묶습니다.
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
                        
                        if self.filtered_weight is None:
                            self.filtered_weight = raw_weight
                        else:
                            # 🚀 [핵심] 찰나의 순간 변화량(instant_diff) 측정
                            instant_diff = abs(raw_weight - self.filtered_weight)
                            
                            # 🧠 적응형 알파(Adaptive Alpha) 지능형 판단 로직
                            if instant_diff > self.jump_threshold:
                                # 1. 고체 타격 (10g 이상): 딜레이 0초, 필터 없이 100% 즉각 반영
                                current_alpha = 1.0  
                            elif instant_diff > 0.4:
                                # 2. 액체 쾌속 투입 (0.4g 이상 변화): 딜레이 최소화 최우선!
                                # 알파를 0.8로 대폭 끌어올려 0.1초의 지연도 없이 PD 제어기에 값을 꽂아 넣습니다.
                                current_alpha = 0.8  
                            elif instant_diff > 0.1:
                                # 3. 액체 미세 투입 (0.1g 이상 변화): 딜레이와 노이즈의 완벽한 타협점
                                current_alpha = 0.3  
                            else:
                                # 4. 정지 상태 (노이즈 구간): 지연은 상관없으니 숫자를 바위처럼 고정!
                                # 알파를 0.05로 짓눌러서 0.5g짜리 파도도 잔잔하게 만들어 버립니다.
                                current_alpha = 0.05 

                            # 결정된 알파 값으로 필터링 적용 (current_alpha가 계속 변함)
                            self.filtered_weight = (current_alpha * raw_weight) + ((1.0 - current_alpha) * self.filtered_weight)
                        
                        precise_weight = round(self.filtered_weight, 3)
                        
                        diff = abs(precise_weight - self.published_weight)
                        if diff > self.noise_window:
                            self.published_weight = precise_weight
                            
                        if abs(self.published_weight) <= self.zero_deadband:
                            self.published_weight = 0.0

                        if self.published_weight < 0.0:
                            self.published_weight = 0.0

                        if self.published_weight != self.last_printed_weight:
                            self.get_logger().info(f"⚖️ [현재 확정 무게]: {self.published_weight:.3f} g (알파 자동 조절 중)")
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
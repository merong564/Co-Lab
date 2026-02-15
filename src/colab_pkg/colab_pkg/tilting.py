import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import DR_init
import time
import threading

# 무게값 메시지 타입 임포트
from std_msgs.msg import Float32

# [수정] UI 메시지 타입 임포트 제거
# from colab_interfaces.msg import UiInput 

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40
ACC = 60

# P제어 게인
P_GAIN = 0.03
MAX_TILT_STEP = 3.0 
WEIGHT_TOLERANCE = 1.0

# [추가] 목표 도달 전 미리 멈추고 복귀할 임계값 (단위: g)
# 예: 목표가 200g이고 이 값이 10.0이면, 190g에서 붓기를 멈추고 원복함
STOP_THRESHOLD = 40.0

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 디지털 출력 상태
ON, OFF = 1, 0

# ==========================================
# 2. ROS 2 노드
# ==========================================
class TiltingController(Node):
    def __init__(self):
        super().__init__('tilting_real_node', namespace=ROBOT_ID)
        
        # 콜백 그룹 설정 (멀티스레드 실행을 위해)
        self.callback_group = ReentrantCallbackGroup()
        
        # 데이터 저장용 변수
        self.current_weight = 0.0
        
        # [유지] 1) 로드셀 무게 구독 (/load_cell/weight)
        self.sub_weight = self.create_subscription(
            Float32,
            '/load_cell/weight',
            self.weight_callback,
            10,
            callback_group=self.callback_group
        )
        
        # [수정] 2) UI 명령 구독 제거
        # self.sub_ui = ... (삭제됨)
        
        self.get_logger().info("Setup: Real Mode (Weight Topic Only)")

    # 무게 콜백 함수
    def weight_callback(self, msg):
        self.current_weight = msg.data
        # 디버깅이 필요하면 주석 해제 (너무 자주 출력될 수 있음)
        # self.get_logger().info(f"Weight: {self.current_weight}")

    # [수정] UI 콜백 함수 제거
    # def ui_callback(self, msg): ... (삭제됨)

# ==========================================
# 3. 로봇 제어 및 계산 함수
# ==========================================
def initialize_robot():
    """로봇 초기화"""
    from DSR_ROBOT2 import set_tool, set_tcp, get_tool, get_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS, set_robot_mode

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1)
    
    print(f"Robot Initialized: {ROBOT_ID}")

def calculate_tilt_angle(current_w, target_w):
    """P제어 각도 계산"""
    error = target_w - current_w
    delta_angle = error * P_GAIN
    
    # 안전장치 (Clamping)
    if delta_angle > MAX_TILT_STEP:
        delta_angle = MAX_TILT_STEP
    elif delta_angle < -MAX_TILT_STEP:
        delta_angle = -MAX_TILT_STEP
        
    return delta_angle, error

def perform_task_real(node):
    from DSR_ROBOT2 import movej, get_current_posj, movel, posx, wait, set_digital_output, get_digital_input, posj
    
    # 디지털 입력 신호 대기 함수
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            # print("Waiting for digital input...")

    # Release 동작
    def release():
        print("Releasing...")
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait_digital_input(2)

    # Grip 동작
    def grip():
        print("Gripping...")
        # release()
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait_digital_input(1)

    # 1. 초기 위치 정의
    pour_ready_pos = posx(585.44, 157.76, 242.63, 91.92, 97.36, 88.55) # 큰 시험관 위치
    
    # 2. [수정] 사용자 입력(input) 받기
    try:
        input_val = input("Enter Target Weight (g): ")
        target_weight = float(input_val)
    except ValueError:
        print("[ERROR] Invalid input. Setting default to 300.0")
        target_weight = 300.0
        
    print(f"[SYSTEM] Task Start! Target: {target_weight}g")
    
    # # 3. 초기 위치로 이동
    # release()
    # wait(0.5)
    # # 홈 위치로 이동
    # p0 = posj(0,0,90,0,90,0)
    # movej(p0,vel=100,acc=50) 

    # 틸팅 위치로 이동
    movel(pour_ready_pos, vel=100, acc=100)
    print("[SYSTEM] Moved to ready position.")
    wait(1.0)

    step_count = 0

    while rclpy.ok():
        # 실시간 무게값 갱신 (콜백에 의해 node.current_weight가 계속 변함)
        current_weight = node.current_weight

        # [수정] 목표 무게보다 STOP_THRESHOLD 만큼 덜 찼을 때 미리 멈춤
        stop_target = target_weight - STOP_THRESHOLD
        
        # 목표 무게 도달 여부 확인
        if current_weight >= stop_target:
            print("Returning to Upright Position Immediately...")
            movel(pour_ready_pos, vel=150, acc=150) # 복귀는 조금 더 빠르게 설정
            
            # 최종 무게 확인
            time.sleep(1.0) # 잔량 떨어지는 것 대기 
            final_weight = node.current_weight
            print(f"✅ [Done] Final Weight: {final_weight:.1f}g / Target: {target_weight}g")

            break

        # 4. P제어 알고리즘 수행
        delta, error = calculate_tilt_angle(current_weight, target_weight)
        
        # 5. 로봇 동작
        try:
            current_joints = get_current_posj()
            if current_joints is not None:
                target_joints = list(current_joints)
                
                # 6축(J6) 회전 반영
                target_joints[5] += delta 
                
                # 로봇 이동
                movej(target_joints, vel=VELOCITY, acc=ACC)
                
                # [데이터 로그 출력]
                print(f"[Step {step_count:02d}] Cur: {current_weight:6.1f} | Err: {error:6.1f} | Control(Delta): {delta:5.2f}")
                
                step_count += 1
                
                # 튜닝 요소: (Loop 속도) 실제 로봇 반응 및 센서 딜레이 고려하여 적절한 대기 시간 설정
                time.sleep(0.1)  
            else:
                print("[ERROR] Failed to get current position")
                break
                
        except Exception as e:
            print(f"[ERROR] Move Error: {e}")
            break

# ==========================================
# 4. 메인 실행부
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    
    # 노드 생성 및 연결
    node = TiltingController()
    DR_init.__dsr__node = node
    
    # 멀티스레드 실행기 설정
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 백그라운드 스레드에서 spin 실행 (토픽 수신용)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        initialize_robot()
        
        # 노드를 인자로 전달하여 실행
        perform_task_real(node)

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
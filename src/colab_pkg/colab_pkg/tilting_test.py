import rclpy
from rclpy.node import Node
import DR_init
import time
import threading

# 무게값 메시지 타입 임포트
from std_msgs.msg import Float32

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 40
ACC = 60

# P제어 게인 (테스트를 위해 조금 민감하게 설정 가능)
P_GAIN = 0.05 
MAX_TILT_STEP = 5.0 
WEIGHT_TOLERANCE = 1.0

# [테스트용 상수]
TEST_TARGET_WEIGHT = 300.0  # 목표 무게 300g
TEST_START_WEIGHT = 0.0     # 시작 무게 0g
SIMULATED_POUR_AMOUNT = 15.0 # 한 번 기울일 때마다 15g씩 찬다고 가정 (시뮬레이션)

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==========================================
# 2. ROS 2 노드 (껍데기만 유지)
# ==========================================
class TiltingController(Node):
    def __init__(self):
        super().__init__('tilting_test_node', namespace=ROBOT_ID)
        # 토픽 구독(Subscriber) 부분은 모두 제거했습니다.
        self.get_logger().info("Setup: Standalone Testing Mode (No Topics)")

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

def perform_task_simulation():
    from DSR_ROBOT2 import movej, get_current_posj, movel, posx, wait
    
    # 1. 초기 위치 정의 (루프 밖에서 정의)
    pour_ready_pos = posx(585.44, 157.76, 242.63, 91.92, 97.36, 88.55) # 큰 시험관 위치
    
    # 2. 사용자 입력 받기 (이전 요청 반영)
    try:
        input_val = input("Enter Target Weight (g): ")
        target_weight = float(input_val)
    except ValueError:
        print("[ERROR] Invalid input. Setting default to 300.0")
        target_weight = 300.0

    current_weight = TEST_START_WEIGHT
    
    print(f"[SYSTEM] Simulation Start. Target: {target_weight}g")
    
    # 3. 초기 위치로 이동 (루프 진입 전 1회 수행)
    movel(pour_ready_pos, vel=100, acc=100)
    print("[SYSTEM] Moved to ready position.")
    wait(1.0)

    step_count = 0

    while rclpy.ok():
        # 목표 무게 도달 여부 확인
        if current_weight >= (target_weight - WEIGHT_TOLERANCE):
            print(f"[Done] Final Weight: {current_weight}g / Target: {target_weight}g")
            break

        # 4. P제어 알고리즘 수행
        delta, error = calculate_tilt_angle(current_weight, target_weight)
        
        # 5. 시뮬레이션 동작
        # 실제로는 error가 줄어들수록 delta(기울이는 양)가 줄어들어야 정상적인 P제어입니다.
        try:
            current_joints = get_current_posj()
            if current_joints is not None:
                target_joints = list(current_joints)
                
                # 6축(J6) 회전 반영
                target_joints[5] += delta 
                
                # 로봇 이동
                movej(target_joints, vel=VELOCITY, acc=ACC)
                
                # [데이터 로그 출력]
                # 이 로그를 보고 P제어가 먹히는지 판단합니다.
                # Error가 줄어듦에 따라 Delta(Tilt Step)도 같이 줄어드는지 확인하세요.
                print(f"[Step {step_count:02d}] Cur: {current_weight:6.1f} | Err: {error:6.1f} | Control(Delta): {delta:5.2f}")

                # 시뮬레이션: 무게 증가 (가정)
                # 실제 환경과 비슷하게 하려면, 기울인 각도가 클수록 무게가 많이 차게 로직을 짤 수도 있지만
                # 지금은 P제어 동작(Error -> Delta) 확인이 목적이므로 고정값 증가도 괜찮습니다.
                current_weight += SIMULATED_POUR_AMOUNT
                
                step_count += 1
                time.sleep(0.5)
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
    
    # 빈 노드 생성 (DR_init 연결용)
    node = TiltingController()
    DR_init.__dsr__node = node
    
    # 멀티스레드 불필요 (단독 실행이므로)
    
    try:
        initialize_robot()
        
        # 시뮬레이션 함수 실행
        perform_task_simulation()

    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
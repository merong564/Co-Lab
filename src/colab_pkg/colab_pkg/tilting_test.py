import rclpy
from rclpy.node import Node
import DR_init
import time
import threading

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
    """틸팅 로직 단독 테스트 함수"""
    from DSR_ROBOT2 import movej, get_current_posj, movel, posx, wait
    
    # 1. 테스트 변수 설정
    current_weight = TEST_START_WEIGHT
    target_weight = TEST_TARGET_WEIGHT
    
    print(f"🚀 Simulation Start! Target: {target_weight}g")
    time.sleep(1)

    

    while rclpy.ok():
        # --- 좌표 정의 ---

        # 1. 비커 앞 대기 위치 (수직 상태)
        pour_ready_pos = posx(604.44, 157.76, 242.63, 91.92, 97.36, 88.55)

        # 2. 따르는 위치 (기울인 상태)
        pour_action_pos = posx(632.66, 159.7, 213.75, 87.33, 99.03, 158.27)


        # --- 실행 로직 ---

        # 1. 정위치로 이동
        movel(pour_ready_pos, vel=100, acc=100)
        print(f"pour ready pos")
        wait(1.0) 

        # 티칭 펜던트 로그에 출력
        #tp_log("작업 후 잔여 무게(Fz): {} N".format(current_weight))

        if current_weight < (target_weight - WEIGHT_TOLERANCE):
            
            # 3. 각도 계산 (P제어)
            delta, error = calculate_tilt_angle(current_weight, target_weight)
            
            print(f"[Sim] Cur: {current_weight:.1f}g / Target: {target_weight}g -> Error: {error:.1f} -> Tilt J6: {delta:.2f} deg")
            
            # 4. 로봇 구동
            try:
                current_joints = get_current_posj()
                if current_joints is not None:
                    target_joints = list(current_joints)
                    
                    # 6축(Index 5) 회전
                    target_joints[5] += delta 
                    
                    # 실제 로봇 이동
                    movej(target_joints, vel=VELOCITY, acc=ACC)
                    
                    # [중요] 시뮬레이션: 로봇이 움직였으니 무게가 찼다고 '가정'하고 값을 올림
                    # 실제 센서가 없으므로 이렇게 안 하면 무한히 회전함
                    current_weight += SIMULATED_POUR_AMOUNT
                    
                    # 너무 빠르지 않게 대기
                    time.sleep(0.5)
                else:
                    print("Error: Failed to get current position")
                    break
                    
            except Exception as e:
                print(f"Move Error: {e}")
                break
            
        else:
            print(f"✅ Simulation Complete! Final Weight: {current_weight}g")
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
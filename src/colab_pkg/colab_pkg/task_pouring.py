import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import DR_init
import time
import threading

from colab_interfaces.srv import RobotCommand
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
P_GAIN = 0.03
MAX_TILT_STEP = 3.0 
STOP_THRESHOLD = 40.0

# ==========================================
# 2. 통신 전담 노드 (서비스 & 토픽)
# ==========================================
class TaskPouring(Node):
    def __init__(self):
        # 이 노드는 로봇 제어와 무관하게 "통신"만 담당하므로 가볍습니다.
        super().__init__('task_pouring', namespace=ROBOT_ID)
        
        self.callback_group = ReentrantCallbackGroup()
        self.current_weight = 0.0

        # 서비스 서버
        self.srv_pouring = self.create_service(
            RobotCommand,
            'execute_pouring',
            self.execute_pouring_callback,
            callback_group=self.callback_group
        )
        
        # 무게 구독
        self.sub_weight = self.create_subscription(
            Float32,
            'load_cell/weight',
            self.weight_callback,
            10,
            callback_group=self.callback_group
        )
            
    def execute_pouring_callback(self, request, response):
        self.get_logger().info(f"[Service] Request Received. Target: {request.target_weight}g")
        
        # 로봇 제어 함수 호출 (self를 넘겨서 현재 무게를 읽을 수 있게 함)
        success = perform_task(self, request.target_weight)
        
        response.success = success
        response.message = "Pouring Completed" if success else "Pouring Failed"
        return response

    def weight_callback(self, msg):
        self.current_weight = msg.data

# ==========================================
# 3. 로봇 제어 로직 (DSR 라이브러리 사용)
# ==========================================
def initialize_robot():
    # 이 함수는 robot_node가 DR_init에 의해 세팅된 후 실행됩니다.
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    
    time.sleep(3.0) # 안전하게 노드 연결 대기
    try:
        print("[Thread] Initializing Robot settings...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print(f"✅ [Thread] Robot Initialized: {ROBOT_ID}")
    except Exception as e:
        print(f"❌ [Thread] Init Failed: {e}")

def calculate_tilt_angle(current_w, target_w):
    error = target_w - current_w
    delta_angle = error * P_GAIN
    if delta_angle > MAX_TILT_STEP: delta_angle = MAX_TILT_STEP
    elif delta_angle < -MAX_TILT_STEP: delta_angle = -MAX_TILT_STEP
    return delta_angle, error

def perform_task(node, target_weight):
    # DSR_ROBOT2 함수들은 DR_init에 등록된 'robot_node'를 통해 명령을 보냅니다.
    from DSR_ROBOT2 import movej, get_current_posj, movel, posx, wait

    print(f"[SYSTEM] Task Start! Target: {target_weight}g")
    pour_ready_pos = posx(585.44, 157.76, 242.63, 91.92, 97.36, 88.55)

    try:
        movel(pour_ready_pos, vel=100, acc=100)
        wait(1.0) # 이동 완료 대기
    except Exception as e:
        print(f"[ERROR] Move Failed: {e}")
        return False

    task_success = False
    
    while rclpy.ok():
        # 무게는 node에서 실시간으로 업데이트됨
        current_weight = node.current_weight
        stop_target = target_weight - STOP_THRESHOLD
        
        if current_weight >= stop_target:
            movel(pour_ready_pos, vel=150, acc=150)
            time.sleep(1.0)
            print(f"✅ [Done] Final: {current_weight:.1f}g")
            task_success = True
            break

        delta, error = calculate_tilt_angle(current_weight, target_weight)
        
        try:
            current_joints = get_current_posj()
            if current_joints:
                target_joints = list(current_joints)
                target_joints[5] += delta 
                movej(target_joints, vel=VELOCITY, acc=ACC)
                print(f"Cur: {current_weight:.1f} | Delta: {delta:.2f}")
                time.sleep(0.1)
            else:
                break
        except Exception:
            break
            
    return task_success

# ==========================================
# 4. 메인 (노드 분리 전략 적용)
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    
    # [전략 핵심] 노드 2개 생성
    # 1. 로봇 제어용 노드 (두산 라이브러리가 독점)
    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    
    # 2. 통신용 노드 (서비스 및 무게 수신)
    task_node = TaskPouring()
    
    # [연결] 무거운 짐(DR_init)은 robot_node에게 떠넘김
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = robot_node 
    
    # Executor에 두 노드 모두 등록 (병렬 실행)
    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    # 초기화 스레드 시작
    init_thread = threading.Thread(target=initialize_robot, daemon=True)
    init_thread.start()
    
    print("==========================================")
    print(" [Ready] Service Server Started (Multi-Node) ")
    print("==========================================")
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        task_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
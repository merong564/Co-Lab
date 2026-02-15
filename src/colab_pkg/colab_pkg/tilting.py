import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import DR_init
import time
import threading

# 메시지 타입 임포트
from std_msgs.msg import Float32
from colab_interfaces.msg import UiInput

# 로봇 설정 상수 (필요에 따라 수정)
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도 (필요에 따라 수정)
VELOCITY = 40
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

P_GAIN = 0.05
# 안전을 위한 한 번에 움직이는 최대 각도 제한 (도)
MAX_TILT_STEP = 5.0 

# 무게 오차 허용 범위 (g) - 이 범위 안에 들어오면 정지
WEIGHT_TOLERANCE = 1.0

class TiltingController(Node):
    def __init__(self):
        super().__init__('tilting_node', namespace=ROBOT_ID)

        # [중요] 콜백 그룹 설정: 서로 다른 콜백(센서, 로봇이동)이 동시에 실행되도록 함
        self.callback_group = ReentrantCallbackGroup()
        
        # 전역 변수처럼 쓸 데이터 저장소
        self.current_weight = 0.0
        self.target_weight = 0.0
        self.is_confirmed = False
        
        # 1) 로드셀 무게 구독 (/load_cell/weight)
        self.sub_weight = self.create_subscription(
            Float32,
            '/load_cell/weight',
            self.weight_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 2) UI 명령 구독 (/ui/command)
        self.sub_ui = self.create_subscription(
            UiInput,
            '/ui/command',
            self.ui_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Tilting Controller Node Started")

    def weight_callback(self, msg):
        self.current_weight = msg.data
        # 디버깅용 (너무 자주 뜨면 주석 처리)
        # self.get_logger().info(f"Current Weight: {self.current_weight}")

    def ui_callback(self, msg):
        self.target_weight = msg.target_weight
        self.is_confirmed = msg.is_confirmed
        self.get_logger().info(f"UI Command Received -> Target: {self.target_weight}, Confirmed: {self.is_confirmed}")


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)

def calculate_tilt_angle(current_w, target_w):
    """
    P제어를 통해 기울일 각도(delta)를 계산하는 함수
    :return: 더 기울여야 할 각도 (float)
    """
    error = target_w - current_w
    
    # 목표 무게보다 현재 무게가 더 많으면(오버필) 멈추거나 반대로 들어야 함
    # 여기서는 붓는 과정이므로 오차만큼 더 기울이는(양수) 로직
    
    # P제어: 오차 * 게인
    delta_angle = error * P_GAIN
    
    # 안전장치: 한 번에 너무 확 기울이지 않도록 제한 (Clamping)
    if delta_angle > MAX_TILT_STEP:
        delta_angle = MAX_TILT_STEP
    elif delta_angle < -MAX_TILT_STEP:
        delta_angle = -MAX_TILT_STEP
        
    return delta_angle, error


def perform_task(ros_node):
    """로봇이 수행할 작업"""
    print("Performing task...")
    from DSR_ROBOT2 import posx,movej,get_current_posj
    # # 초기 위치 및 목표 위치 설정
    # JReady = [0, 0, 90, 0, 90, 0]
    # pos1 = posx([500, 80, 200, 150, 179, 150])

    # # 반복 동작 수행
    # while True:       
    #     # 이동 명령 실행
    #     print("movej")
    #     movej(JReady, vel=VELOCITY, acc=ACC)
    #     print("movel")
    #     movel(pos1, vel=VELOCITY, acc=ACC)

    print("Waiting for UI confirmation...")
    
    # 작업 루프
    while rclpy.ok():
        # 1. 사용자가 시작을 눌렀는지 확인
        if ros_node.is_confirmed:
            
            # 2. 목표 무게 도달 여부 확인
            # (목표값 - 허용오차) 보다 현재 무게가 작으면 계속 붓기
            if ros_node.current_weight < (ros_node.target_weight - WEIGHT_TOLERANCE):
                
                # 3. P제어로 각도 계산
                delta, error = calculate_tilt_angle(ros_node.current_weight, ros_node.target_weight)
                
                print(f"[Tilting] Error: {error:.2f}g -> Moving J6 by {delta:.2f} deg")
                
                # 4. 현재 관절 각도 가져오기
                current_joints = get_current_posj() # [J1, J2, J3, J4, J5, J6]
                
                # 5. 6축(Index 5)만 회전
                # 주의: 로봇 설정에 따라 +가 붓는 방향인지 -가 붓는 방향인지 확인 필요
                # 여기서는 +를 붓는 방향(Down)으로 가정
                target_joints = list(current_joints)
                target_joints[5] += delta 
                
                # 6. 로봇 이동 (movej 사용)
                # P제어이므로 부드럽게 연결되려면 속도를 적절히 조절하거나 movej 대신 amovej 등을 고려할 수 있음
                movej(target_joints, vel=VELOCITY, acc=ACC)
                
            elif ros_node.current_weight >= (ros_node.target_weight - WEIGHT_TOLERANCE):
                print(f"✅ Target Reached! ({ros_node.current_weight} / {ros_node.target_weight})")
                # 목표 도달 시 루프를 탈출하거나 대기 (여기서는 1초 대기 후 상태 유지)
                time.sleep(1)
                # 필요하다면 is_confirmed를 False로 바꿔서 다시 대기 상태로 갈 수 있음
                # ros_node.is_confirmed = False 
                
        else:
            # UI 입력 대기 중
            time.sleep(0.5)
    

def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    # rclpy.init(args=args)
    # node = rclpy.create_node("move_basic", namespace=ROBOT_ID)

    # # DR_init에 노드 설정
    # DR_init.__dsr__node = node

    rclpy.init(args=args)
    
    # 1. ROS 노드 생성
    node = TiltingController()
    DR_init.__dsr__node = node # 두산 로봇에 노드 연결

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 2. ROS 콜백을 처리할 별도 스레드 시작
    # (이게 없으면 perform_task의 while 루프 때문에 토픽을 못 받아옴)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        # perform_task()
        perform_task(node)

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
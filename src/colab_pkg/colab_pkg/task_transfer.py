#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

# [추가] 멀티스레딩 및 서비스 관련 모듈 임포트
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from colab_interfaces.srv import RobotCommand

# ===============================
# 1. 설정 및 상수
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 속도/가속도 (안전하게 설정)
VEL, ACC = 60, 60
DO_OPEN = 1
DO_CLOSE = 2

# DSR_init 설정 (전역)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# [추가] 통신 전담 노드 클래스 생성 (요청사항 5)
class TaskTransfer(Node):
    def __init__(self):
        super().__init__('task_transfer', namespace=ROBOT_ID)
        
        # [추가] 콜백 그룹 설정 (요청사항 3)
        self.callback_group = ReentrantCallbackGroup()
        
        # [추가] 서비스 서버 생성 (요청사항 1)
        self.srv_transfer = self.create_service(
            RobotCommand,
            'execute_transfer',
            self.execute_transfer_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("TaskTransfer Ready. Service: execute_transfer")

    # [추가] 서비스 콜백 함수 (요청사항 1, 2, 6)
    def execute_transfer_callback(self, request, response):
        mode = (getattr(request, "mode", "") or "").strip().upper()

        # [수정사항] 추후 UI에서 tube_type이 필요하면 아래와 같이 처리
        # tube_type = (getattr(request, "tube_type", "") or "LARGE").strip().upper()
        
        
        # CLI 환경에서 사용자 입력을 받아 tube_type 결정
        user_input = input("시험관 크기를 입력하세요 (0: SMALL, 1: LARGE): ").strip()
        if user_input == "0":
            tube_type = "SMALL"
        elif user_input == "1":
            tube_type = "LARGE"
        else:
            self.get_logger().error("잘못된 입력입니다. 기본값 LARGE로 설정합니다.")
            tube_type = "LARGE"
        
        self.get_logger().info(f"[Service] Request Received. Mode: {mode}, Tube: {tube_type}")
        
        # [수정] 콜백 내부에서 perform_task 실행 (요청사항 2, 6)
        perform_task(mode, tube_type)
        
        response.success = True
        response.message = f"{mode} Completed"
        return response

# ===============================
# 2. 로봇 제어 로직
# ===============================
def get_poses(posx_func):
    return {
        # [수정] SMALL 시험관 좌표 추가
        "SMALL": {
            "PICK_DOWN": posx_func(555.786, -78.524, 126.047, 90.674, 92.519, 93.656),
            "PICK_UP":   posx_func(555.784, -78.523, 259.725, 90.674, 92.518, 93.657),
            "POUR_READY": posx_func(604.441, 157.760, 242.631, 91.920, 97.360, 88.550),
        },
        "LARGE": {
            "PICK_DOWN": posx_func(306.636, -66.725,  89.141, 91.356, 91.786, 90.102),
            "PICK_UP":   posx_func(306.636, -66.725, 257.898, 91.356, 91.786, 90.102),
            "POUR_READY": posx_func(585.440, 157.760, 242.631, 91.920, 97.360, 88.550),
        }
    }

def initialize_robot():
    """로봇 초기화 (grip_simple 예제와 동일한 방식)"""
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, get_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    
    # 1. 매뉴얼 모드 전환
    set_robot_mode(ROBOT_MODE_MANUAL)
    
    # 2. 툴/TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    # 3. 자율 모드 전환
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0) # 모드 변경 안정화 대기

    # 상태 출력
    print("#" * 50)
    print(f" Robot Initialized (Mode: {get_robot_mode()})")
    print("#" * 50)

def perform_task(mode, tube_type):
    """작업 수행 로직"""
    from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_BASE

    # 좌표 데이터 로드
    try:
        POSES = get_poses(posx)
    except Exception:
        print(" Error: DSR Library not ready")
        return

    # 안전한 초기 자세 (Joint)
    J_READY = [0, 0, 90, 0, 90, 0]

    def gripper(action):
        if action == "OPEN":
            set_digital_output(DO_CLOSE, 0)
            set_digital_output(DO_OPEN, 1)
            wait(0.5)
        elif action == "CLOSE":
            set_digital_output(DO_OPEN, 0)
            set_digital_output(DO_CLOSE, 1)
            wait(0.5)

    try:
        P = POSES[tube_type]

        # [안전 장치] 작업 시작 전 movej로 관절을 풀어줌 (특이점 회피)
        print(f" Moving to Ready Pose (J)")
        movej(J_READY, vel=VEL, acc=ACC)
        wait(0.5)

        # 작업 수행
        if mode == "PICKUP":
            print(f"[Action] PICKUP Start ({tube_type})")
            gripper("OPEN")
            
            # sync=True는 기본값이지만 명시적으로 확인
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
            gripper("CLOSE")
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)
            print("[Action] PICKUP Done")

        elif mode == "RETURN":
            print(f"[Action] RETURN Start ({tube_type})")
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
            gripper("OPEN")
            wait(0.5)
            movel(P["PICK_UP"],   vel=VEL, acc=ACC, ref=DR_BASE)
            
            # 복귀
            movej(J_READY, vel=VEL, acc=ACC)
            print("[Action] RETURN Done")

    except Exception as e:
        print(f" [Action] Failed: {e}")

# ===============================
# 3. 메인 (grip_simple 구조 따름)
# ===============================
def main(args=None):
    rclpy.init(args=args)

    # [수정] 멀티 노드 생성 (요청사항 3, 4, 5)
    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    task_node = TaskTransfer()

    # [수정] DSR_init에 robot_node 주입 (요청사항 4)
    DR_init.__dsr__node = robot_node

    # [추가] Executor에 노드 등록 (요청사항 3)
    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    # [수정] 초기화를 스레드로 실행하도록 변경
    init_thread = threading.Thread(target=initialize_robot, daemon=True)
    init_thread.start()

    print("\n=== Service Server Started (Multi-Node) ===")

    try:
        # [수정] 서비스 대기를 위해 spin 실행
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # [수정] 노드 종료 처리
        task_node.destroy_node()
        robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
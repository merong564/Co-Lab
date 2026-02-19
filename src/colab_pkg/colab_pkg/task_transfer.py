#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

# 멀티스레딩 및 서비스/토픽 관련
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from colab_interfaces.srv import RobotCommand
from std_msgs.msg import String

# ===============================
# 1. 설정 및 상수 (원본 유지)
# ===============================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VEL, ACC = 60, 60
DO_OPEN = 1
DO_CLOSE = 2

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ===============================
# [추가] STOP → 에러로 정지
# ===============================
CRASH_ON_STOP = True         # True: STOP 즉시 raise로 노드 터뜨림
STOP_REQUESTED = False       # STOP 플래그


class TaskTransfer(Node):
    def __init__(self):
        super().__init__('task_transfer', namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()

        # [추가] STOP 토픽 구독 (/dsr01/stop)
        self.sub_stop = self.create_subscription(
            String,
            'stop',
            self.stop_callback,
            10,
            callback_group=self.callback_group
        )

        self.srv_transfer = self.create_service(
            RobotCommand,
            'execute_transfer',
            self.execute_transfer_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("TaskTransfer Ready. Service: execute_transfer")

    # [추가] STOP 콜백: 플래그 올리고, 옵션에 따라 일부러 에러 발생
    def stop_callback(self, msg: String):
        global STOP_REQUESTED
        if (msg.data or "").strip().upper() != "STOP":
            return

        STOP_REQUESTED = True
        self.get_logger().warn("🚨 STOP received")

        if CRASH_ON_STOP:
            # ✅ 의도적으로 에러 띄워서 정지(노드 종료)
            raise RuntimeError("EMERGENCY STOP (intentional crash)")

    def execute_transfer_callback(self, request, response):
        global STOP_REQUESTED
        STOP_REQUESTED = False  # 작업 시작 시 초기화

        mode = (getattr(request, "mode", "") or "").strip().upper()

        user_input = input("시험관 크기를 입력하세요 (0: SMALL, 1: LARGE): ").strip()
        if user_input == "0":
            tube_type = "SMALL"
        elif user_input == "1":
            tube_type = "LARGE"
        else:
            self.get_logger().error("잘못된 입력입니다. 기본값 LARGE로 설정합니다.")
            tube_type = "LARGE"

        self.get_logger().info(f"[Service] Request Received. Mode: {mode}, Tube: {tube_type}")

        # [추가] perform_task에서 STOP이면 raise로 끊기도록 처리됨
        try:
            perform_task(mode, tube_type)
            response.success = True
            response.message = f"{mode} Completed"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response


# ===============================
# 2. 로봇 제어 로직 (원본 유지 + STOP 체크만 추가)
# ===============================
def get_poses(posx_func):
    return {
        "SMALL": {
            "PICK_DOWN": posx_func(555.786, -78.524, 126.047, 90.674, 92.519, 93.656),
            "PICK_UP":   posx_func(555.784, -78.523, 259.725, 90.674, 92.518, 93.657),
            "POUR_READY": posx_func(604.441, 157.760, 242.631, 91.920, 97.360, 88.550),
        },
        "LARGE": {
            "PICK_DOWN": posx_func(306.636, -66.725,  109.141, 91.356, 91.786, 90.102),
            "PICK_UP":   posx_func(306.636, -66.725, 257.898, 91.356, 91.786, 90.102),
            "POUR_UP": posx_func(585.440, 157.760, 242.631, 91.920, 97.360, 88.550),
            "POUR_READY": posx_func(585.440, 157.760, 180.631, 91.920, 97.360, 88.550),
        }
    }


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, get_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

    print("#" * 50)
    print(f" Robot Initialized (Mode: {get_robot_mode()})")
    print("#" * 50)


def perform_task(mode, tube_type):
    global STOP_REQUESTED
    from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_BASE

    # 좌표 데이터 로드
    try:
        POSES = get_poses(posx)
    except Exception:
        print(" Error: DSR Library not ready")
        return

    J_READY = [0, 0, 90, 0, 90, 0]
    ON, OFF = 1, 0

    # [추가] STOP 체크 함수(최소 추가)
    def _check_stop(tag=""):
        global STOP_REQUESTED
        if STOP_REQUESTED:
            raise RuntimeError(f"STOP (intentional error) at: {tag}")

    def gripper_open():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(2.0)

    def gripper_close():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2.0)

    try:
        P = POSES[tube_type]

        if mode == "PICKUP":
            print(f"[Action] PICKUP Start ({tube_type})")

            _check_stop("before movej ready")
            movej(J_READY, vel=VEL, acc=ACC)
            wait(0.5)

            _check_stop("before gripper_open")
            gripper_open()

            _check_stop("before movel PICK_UP")
            movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

            _check_stop("before movel PICK_DOWN")
            movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)

            _check_stop("before gripper_close")
            gripper_close()

            _check_stop("before movel PICK_UP(2)")
            movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

            _check_stop("before movel PICK_UP(2)")
            movel(P["POUR_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

            _check_stop("before movel POUR_READY")
            movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)

            print("[Action] PICKUP Done")

        elif mode == "RETURN":
            print(f"[Action] RETURN Start ({tube_type})")

            _check_stop("before movel PICK_UP")
            movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

            _check_stop("before movel PICK_DOWN")
            movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)

            _check_stop("before gripper_open")
            gripper_open()

            _check_stop("before movel PICK_UP(2)")
            movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

            _check_stop("before movej ready")
            movej(J_READY, vel=VEL, acc=ACC)

            print("[Action] RETURN Done")

    except Exception as e:
        print(f" [Action] Failed: {e}")
        # 여기서 다시 raise 하면 서비스가 실패로 응답됨(원하는 “에러로 정지” 유지)
        raise


# ===============================
# 3. 메인 (원본 유지)
# ===============================
def main(args=None):
    rclpy.init(args=args)

    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    task_node = TaskTransfer()

    DR_init.__dsr__node = robot_node

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    init_thread = threading.Thread(target=initialize_robot, daemon=True)
    init_thread.start()

    print("\n=== Service Server Started (Multi-Node) ===")

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

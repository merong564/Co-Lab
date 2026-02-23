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
        self.get_logger().warn("[WARN] STOP received") 

        if CRASH_ON_STOP:
            # 의도적으로 에러 띄워서 정지(노드 종료)
            raise RuntimeError("EMERGENCY STOP (intentional crash)")

    def execute_transfer_callback(self, request, response):
        global STOP_REQUESTED
        STOP_REQUESTED = False  # 작업 시작 시 초기화

        mode = (getattr(request, "mode", "") or "").strip().upper()

        # [수정] 서비스 request 데이터에서 시험관 종류를 동적으로 파악 (배열 인터페이스 반영)
        targets_list = getattr(request, "targets", ["LARGE"])
        tube_type = targets_list[0].strip().upper() if targets_list else "LARGE"

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
# 2. 로봇 제어 로직
# ===============================
def get_poses(posx_func):
    return {
        "LARGE": {
            "PICK_DOWN": posx_func(306.636, -66.725,  109.141, 91.356, 91.786, 90.102),
            "PICK_UP":   posx_func(306.636, -66.725, 257.898, 91.356, 91.786, 90.102),
            "POUR_UP": posx_func(585.440, 157.760, 242.631, 91.920, 97.360, 88.550),
            "POUR_READY": posx_func(585.440, 157.760, 180.631, 91.920, 97.360, 88.550),
            "RETURN_READY": posx_func(303.736, 81.616, 230.386, 91.920, 97.360, 88.550),
            "RETURN_UP": posx_func(417.368, 608.704, 260.356, 90.362, 91.682, 89.077),
            "RETURN_DOWN": posx_func(417.368, 608.704, 104.231, 90.362, 91.682, 89.077)
        },
        "SMALL1": {
            "PICK_DOWN": posx_func(333.096, 373.067, 138.164, 91.215, 89.984, 92.903),
            "PICK_UP": posx_func(333.096, 373.067, 224.104, 91.215, 89.984, 92.903),
            "POUR_UP": posx_func(585.440, 157.760, 190.631, 91.920, 97.360, 88.550),
            "POUR_READY": posx_func(585.440, 144.760, 160.631, 91.920, 97.360, 88.550)
        },
        "SMALL2": {
            "PICK_DOWN": posx_func(217.794, 377.263, 133.564, 121.034, 93.617, 92.329),
            "PICK_UP": posx_func(216.423, 384.357, 282.484, 120.725, 94.227, 91.915),
            "POUR_UP": posx_func(585.440, 157.760, 190.631, 91.920, 97.360, 88.550),
            "POUR_READY": posx_func(585.440, 144.760, 160.631, 91.920, 97.360, 88.550)
        },
        "BEAKER": {
            "PICK_DOWN": posx_func(303.736, 81.616, 86.386, 170.495, -178.848, 167.281),
            "PICK_UP": posx_func(303.736, 81.616, 230.386, 170.495, -178.848, 167.281),
            "RETURN_UP": posx_func(368.058, 473.059, 230.706, 19.522, 178.596, 15.563),
            "RETURN_DOWN": posx_func(368.058, 473.059, 82.706, 19.522, 178.596, 15.563)
        }
    }


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, get_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

    print("#" * 50)
    print(f" Robot Initialized (Mode: {get_robot_mode()})")
    print("#" * 50)


def perform_task(mode, tube_type):
    global STOP_REQUESTED
    from DSR_ROBOT2 import movej, movel, posx, wait, set_digital_output, DR_BASE
    from DSR_ROBOT2 import set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(0.5)

    POSES = get_poses(posx)
    J_READY = [0, 0, 90, 0, 90, 0]
    ON, OFF = 1, 0

    def _check_stop(tag=""):
        global STOP_REQUESTED
        if STOP_REQUESTED:
            raise RuntimeError(f"STOP (intentional error) at: {tag}")

    def gripper_open():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(2.0)

    def gripper_large_open():
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(3, ON)
        set_digital_output(4, OFF)
        wait(2.0)

    def gripper_close():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2.0)
        
    def target_gripper_open():
        if tube_type == "LARGE":
            gripper_large_open()
        else:
            gripper_open()

    # [추가] 모듈화된 작업 함수들 정의
    def _pickup_tube_common(P):
        _check_stop("before movej ready")
        movej(J_READY, vel=VEL, acc=ACC)
        wait(0.5)
        _check_stop("before target_gripper_open")
        target_gripper_open()
        _check_stop("before movel PICK_UP")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel PICK_DOWN")
        movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before gripper_close")
        gripper_close()
        _check_stop("before movel PICK_UP(2)")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel POUR_UP")
        movel(P["POUR_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel POUR_READY")
        movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)

    def pickup_large(P):
        _pickup_tube_common(P)

    def pickup_small(P):
        _pickup_tube_common(P)

    def pickup_beaker(P):
        # POUR 위치가 없는 비커의 기본 픽업
        _check_stop("before movej ready")
        movej(J_READY, vel=VEL, acc=ACC)
        wait(0.5)
        _check_stop("before target_gripper_open")
        target_gripper_open()
        _check_stop("before movel PICK_UP")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel PICK_DOWN")
        movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before gripper_close")
        gripper_close()
        _check_stop("before movel PICK_UP(2)")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

    def return_large(P):
        _check_stop("before movel POUR_READY to POUR_UP")
        movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)
        movel(P["POUR_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        # [추가] 비커의 PICK_UP 위치로 이동
        _check_stop("before movel BEAKER PICK_UP")
        movel(P["RETURN_READY"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel RETURN_UP")
        movel(P["RETURN_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel RETURN_DOWN")
        movel(P["RETURN_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before target_gripper_open")
        target_gripper_open()
        _check_stop("before movel RETURN_UP(2)")
        movel(P["RETURN_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movej ready")
        movej(J_READY, vel=VEL, acc=ACC)

    def return_small(P):
        _check_stop("before movel POUR_READY to POUR_UP")
        movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)
        movel(P["POUR_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel PICK_UP")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel PICK_DOWN")
        movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before target_gripper_open")
        target_gripper_open()
        _check_stop("before movel PICK_UP(2)")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movej ready")
        movej(J_READY, vel=VEL, acc=ACC)

    def return_beaker(P):
        # 지시해주신 비커 전용 단일 시퀀스 
        _check_stop("before target_gripper_open")
        target_gripper_open()
        _check_stop("before movel PICK_UP")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel PICK_DOWN")
        movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before gripper_close")
        gripper_close()
        _check_stop("before movel PICK_UP(2)")
        movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel RETURN_UP")
        movel(P["RETURN_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movel RETURN_DOWN")
        movel(P["RETURN_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before target_gripper_open")
        target_gripper_open()
        _check_stop("before movel RETURN_UP(2)")
        movel(P["RETURN_UP"], vel=VEL, acc=ACC, ref=DR_BASE)
        _check_stop("before movej ready")
        movej(J_READY, vel=VEL, acc=ACC)

    try:
        P = POSES[tube_type]

        # [추가] 간소화된 분기 제어문
        if mode == "PICKUP":
            print(f"[Action] PICKUP Start ({tube_type})")
            if tube_type == "LARGE":
                pickup_large(P)
            elif tube_type in ["SMALL1", "SMALL2"]:
                pickup_small(P)
            elif tube_type == "BEAKER":
                pickup_beaker(P)
            print("[Action] PICKUP Done")

        elif mode == "RETURN":
            print(f"[Action] RETURN Start ({tube_type})")
            if tube_type == "LARGE":
                return_large(P)
            elif tube_type in ["SMALL1", "SMALL2"]:
                return_small(P)
            elif tube_type == "BEAKER":
                return_beaker(P)
            print("[Action] RETURN Done")

    except Exception as e:
        print(f" [Action] Failed: {e}")
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
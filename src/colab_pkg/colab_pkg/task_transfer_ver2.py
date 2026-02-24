#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from colab_interfaces.srv import RobotCommand
from std_msgs.msg import String

# ===============================
# 1. 설정 및 상수
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
# STOP 처리
#  - STOP 받으면 언제든 중단
#  - 중단되면 서비스 응답 success=False
#  - STOP 플래그는 RESET에서만 해제 (서비스 시작에서 지우지 않음)
# ===============================
STOP_REQUESTED = False


class TaskTransfer(Node):
    def __init__(self):
        super().__init__('task_transfer', namespace=ROBOT_ID)
        self.callback_group = ReentrantCallbackGroup()

        # /dsr01/stop 구독
        self.sub_stop = self.create_subscription(
            String,
            'stop',
            self.stop_callback,
            10,
            callback_group=self.callback_group
        )

        # /dsr01/execute_transfer 서비스
        self.srv_transfer = self.create_service(
            RobotCommand,
            'execute_transfer',
            self.execute_transfer_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("TaskTransfer Ready. Service: execute_transfer")

    def stop_callback(self, msg: String):
        global STOP_REQUESTED
        cmd = (msg.data or "").strip().upper()

        if cmd == "STOP":
            STOP_REQUESTED = True
            self.get_logger().warn("[STOP] received -> flag set (node stays alive)")

            # 가능하면 즉시 모션 정지 시도
            try:
                from DSR_ROBOT2 import stop, DR_QSTOP
                stop(DR_QSTOP)
                self.get_logger().warn("[STOP] Robot stop() called (DR_QSTOP)")
            except Exception:
                try:
                    from DSR_ROBOT2 import stop, DR_SSTOP
                    stop(DR_SSTOP)
                    self.get_logger().warn("[STOP] Robot stop() called (DR_SSTOP)")
                except Exception:
                    pass

        elif cmd == "RESET":
            STOP_REQUESTED = False
            self.get_logger().info("[RESET] received -> flag cleared")

    def execute_transfer_callback(self, request, response):
        global STOP_REQUESTED

        # ✅ 서비스 시작 시 STOP이면 즉시 실패 응답 (플래그 지우지 않음)
        if STOP_REQUESTED:
            self.get_logger().info(f"##### Service Response = False ######")

            response.success = False
            response.message = "STOP already requested"
            return response

        mode = (getattr(request, "mode", "") or "").strip().upper()

        targets_list = getattr(request, "targets", ["LARGE"])
        tube_type = targets_list[0].strip().upper() if targets_list else "LARGE"

        self.get_logger().info(f"[Service] Request Received. Mode: {mode}, Tube: {tube_type}")

        try:
            perform_task(mode, tube_type)

            # ✅ 작업이 정상 종료된 것처럼 보이더라도 STOP이 들어왔으면 실패 처리
            if STOP_REQUESTED:
                raise RuntimeError("STOP requested during task")

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
    from DSR_ROBOT2 import (
        set_tool, set_tcp,
        set_robot_mode, get_robot_mode,
        ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    )

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

    print("#" * 50)
    print(f" Robot Initialized (Mode: {get_robot_mode()})")
    print("#" * 50)


def perform_task(mode, tube_type):
    """
    STOP 처리 보장:
      - 시작 시 STOP이면 즉시 중단
      - 작업 진행 중에도 _check_stop()로 계속 감지
      - STOP이면 RuntimeError -> 서비스 콜백에서 success=False 응답
    """
    global STOP_REQUESTED

    # [추가] amovel, amovej, check_motion 임포트
    from DSR_ROBOT2 import movej, movel, amovel, amovej, check_motion, posx, wait, set_digital_output, DR_BASE
    from DSR_ROBOT2 import set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS

    def _check_stop(tag=""):
        global STOP_REQUESTED
        if STOP_REQUESTED:
            # 가능한 경우, 추가 정지 시도
            try:
                from DSR_ROBOT2 import stop, DR_QSTOP
                stop(DR_QSTOP)
            except Exception:
                pass
            raise RuntimeError(f"STOP at: {tag}")

    # ✅ 시작하자마자 STOP이면 바로 중단
    _check_stop("before task start")

    # [추가] 모션 및 대기 중 STOP 플래그를 실시간으로 감시하는 커스텀 함수 정의
    def custom_movel(*args, **kwargs):
        amovel(*args, **kwargs)
        time.sleep(0.1) # 모션 시작 대기
        while check_motion() == 1:
            _check_stop("during movel")
            time.sleep(0.05)

    def custom_movej(*args, **kwargs):
        amovej(*args, **kwargs)
        time.sleep(0.1) # 모션 시작 대기
        while check_motion() == 1:
            _check_stop("during movej")
            time.sleep(0.05)

    def custom_wait(wait_time):
        start = time.time()
        while time.time() - start < wait_time:
            _check_stop("during wait")
            time.sleep(0.05)

    # [추가] 기존 블로킹 함수들을 커스텀 함수로 덮어씌움
    movel = custom_movel
    movej = custom_movej
    wait = custom_wait

    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(0.5)

    POSES = get_poses(posx)
    J_READY = [0, 0, 90, 0, 90, 0]
    ON, OFF = 1, 0

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

    def _pickup_tube_common(P):
        _check_stop("before movej ready")
        custom_movej(J_READY, vel=VEL, acc=ACC)
        wait(0.5)

        _check_stop("before target_gripper_open")
        target_gripper_open()

        _check_stop("before movel PICK_UP")
        custom_movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before movel PICK_DOWN")
        custom_movel(P["PICK_DOWN"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before gripper_close")
        gripper_close()

        _check_stop("before movel PICK_UP(2)")
        custom_movel(P["PICK_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before movel POUR_UP")
        custom_movel(P["POUR_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before movel POUR_READY")
        custom_movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)

    def pickup_beaker(P):
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
        _check_stop("before movel POUR_READY")
        movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before movel POUR_UP")
        movel(P["POUR_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before movel RETURN_READY")
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
        _check_stop("before movel POUR_READY")
        movel(P["POUR_READY"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before movel POUR_UP")
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

        _check_stop("before target_gripper_open(2)")
        target_gripper_open()

        _check_stop("before movel RETURN_UP(2)")
        movel(P["RETURN_UP"], vel=VEL, acc=ACC, ref=DR_BASE)

        _check_stop("before movej ready")
        movej(J_READY, vel=VEL, acc=ACC)

    # 실행
    if tube_type not in POSES:
        raise RuntimeError(f"Unknown tube_type: {tube_type} (valid: {list(POSES.keys())})")

    P = POSES[tube_type]

    if mode == "PICKUP":
        print(f"[Action] PICKUP Start ({tube_type})")
        if tube_type == "BEAKER":
            pickup_beaker(P)
        else:
            _pickup_tube_common(P)
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

    else:
        raise RuntimeError(f"Unknown mode: {mode} (use PICKUP or RETURN)")


# ===============================
# 3. 메인
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
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

# 멀티스레딩 및 서비스 관련 모듈 임포트
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

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ===============================
# 2. 서비스 노드 클래스
# ===============================
class TaskMixing(Node):
    def __init__(self):
        super().__init__('task_mixing', namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()

        self.srv_mixing = self.create_service(
            RobotCommand,
            'execute_mixing',
            self.execute_mixing_callback,
            callback_group=self.callback_group
        )

        self.pos_home = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        self.pos_beaker_pick       = [617.838, 138.024, 120.460, 142.800, 179.222, 142.231]
        self.pos_beaker_pick_safe  = [617.838, 138.024, 226.696, 142.800, 179.222, 142.231]
        self.pos_beaker_place_safe = [406.705, 111.093, 203.088, 3.086, 178.554, -0.335]
        self.pos_beaker_place      = [303.736, 81.616, 86.386, 170.495, -178.848, 167.281]

        self.pos_mixer_pick        = [87.752, 443.877, 236.217, 114.003, 179.135, 113.295]
        self.pos_mixer_pick_safe   = [87.752, 190.136, 236.217, 114.003, 179.135, 113.295]
        self.pos_mixer_mix_safe    = [349.592, 93.050, 233.490, 123.441, 179.314, 122.717]
        self.pos_mixer_mix_down    = [349.592, 93.050, 135.172, 123.441, 179.314, 122.717]

        self.get_logger().info("TaskMixing Ready. Service: execute_mixing")

    def execute_mixing_callback(self, request, response):
        mode = (getattr(request, "mode", "") or "").strip().upper()
        mixing_duration = float(getattr(request, "mixing_duration", 10.0))

        self.get_logger().info(f"[Service] Request Received. Mode: {mode}, Duration: {mixing_duration}s")

        try:
            self.perform_task(mixing_duration)
            response.success = True
            response.message = f"{mode} Mixing Completed Successfully"
        except Exception as e:
            self.get_logger().error(f"Task failed: {e}")
            response.success = False
            response.message = f"{mode} Mixing Failed: {str(e)}"

        return response

    def perform_task(self, mixing_duration=10.0):
        from DSR_ROBOT2 import (
            movej, movel, posj, posx,
            set_digital_output, wait, set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            move_periodic, get_current_posx,
            DR_BASE, DR_TOOL
        )

        def log(msg: str):
            self.get_logger().info(msg)

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        J_VEL, J_ACC = 40, 40
        L_VEL, L_ACC = 100, 100
        ON, OFF = 1, 0

        def gripper_open():
            log("그리퍼 열기")
            set_digital_output(1, OFF)
            set_digital_output(2, ON)
            wait(2.0)

        def gripper_close():
            log("그리퍼 닫기")
            set_digital_output(1, ON)
            set_digital_output(2, OFF)
            wait(2.0)

        def pick_and_place_beaker():
            log("[1] 비커 이동 작업 시작")
            gripper_open()
            movel(posx(self.pos_beaker_pick_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(posx(self.pos_beaker_pick), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_close()
            movel(posx(self.pos_beaker_pick_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)

            movel(posx(self.pos_beaker_place_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(posx(self.pos_beaker_place), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_open()
            movel(posx(self.pos_beaker_place_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            log("비커 이동 작업 완료")

        def pick_and_ready_mixer():
            log("[2] 믹서 픽업 및 대기 위치 이동 시작")
            movel(posx(self.pos_mixer_pick_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(posx(self.pos_mixer_pick), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_close()
            movel(posx(self.pos_mixer_pick_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(posx(self.pos_mixer_mix_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            log("믹서 대기 위치 이동 완료")

        def return_mixer():
            log("[4] 믹서 원위치 반환 시작")
            movel(posx(self.pos_mixer_mix_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(posx(self.pos_mixer_pick_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(posx(self.pos_mixer_pick), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_open()
            movel(posx(self.pos_mixer_pick_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            log("믹서 원위치 반환 완료")

        # ✅ 위로(올릴 때)는 천천히 / 아래로(털려고 내릴 때)는 빠르게
        def shake_off_before_return(
            lift_mm=70.0,
            tap_mm=25.0,
            tap_count=12,
            up_vel=60, up_acc=60,        # 위로 갈 때(천천히)
            down_vel=350, down_acc=500,  # 아래로 갈 때(빠르게)
            ref=DR_BASE
        ):
            log("[3-1] 믹서 털기(위=천천히/아래=빠르게) 시작")

            ret = get_current_posx(ref=ref)
            cur = ret[0] if isinstance(ret, tuple) else ret  # [x,y,z,rx,ry,rz]
            x, y, z, rx, ry, rz = map(float, cur)

            # 1) 털기 기준 높이로 천천히 들어올리기
            up = posx(x, y, z + lift_mm, rx, ry, rz)
            movel(up, vel=up_vel, acc=up_acc, ref=ref)
            wait(0.05)

            base_z = z + lift_mm
            down = posx(x, y, base_z - tap_mm, rx, ry, rz)

            # 2) 탁탁 반복: 내려갈 때 빠르게(탁), 올라갈 때 천천히(복귀)
            for _ in range(int(tap_count)):
                movel(down, vel=down_vel, acc=down_acc, ref=ref)
                wait(0.01)
                movel(up, vel=up_vel, acc=up_acc, ref=ref)
                wait(0.01)

            log("[3-1] 믹서 털기 완료")

        # ✅ 액체용 믹싱: 외력/순응제어 제거 + move_periodic으로 "베이스 기준" 위아래 혼합
        def mixer_descend_and_wiggle(end_pos_list, fz_trigger=7.0, down_force=-20.0):
            # fz_trigger/down_force는 시그니처 유지용(미사용)

            log("[3] (액체) 믹싱 다운 위치로 하강")
            movel(posx(end_pos_list), vel=10, acc=10, ref=DR_BASE)
            wait(0.1)

            log(f"[3] (액체) 베이스 기준 위아래 혼합 시작 ({mixing_duration}초)")
            period_s = 1.2
            repeat_calc = max(1, int(mixing_duration / period_s))

            move_periodic(
                amp=[0, 0, 15, 0, 0, 0],   # Z 15mm 위아래 (10~25 튜닝)
                period=period_s,
                atime=0.2,
                repeat=repeat_calc,
                ref=DR_BASE                # ✅ 핵심: 바닥(Z축) 기준으로 수직 위아래
            )

            log("[3] (액체) 혼합 완료 후 안전 위치로 상승")
            movel(posx(self.pos_mixer_mix_safe), vel=60, acc=60, ref=DR_BASE)
            wait(0.1)

        # =========================
        # 전체 흐름 제어
        # =========================
        pick_and_place_beaker()

        pick_and_ready_mixer()
        wait(0.2)

        # 액체 혼합 (외력 없이, 베이스 기준 위아래 move_periodic)
        mixer_descend_and_wiggle(end_pos_list=self.pos_mixer_mix_down)

        # ✅ 위로는 천천히 / 아래로는 빠르게 (탁탁)
        shake_off_before_return(
            lift_mm=70.0,
            tap_mm=25.0,
            tap_count=12,
            up_vel=60, up_acc=60,
            down_vel=600, down_acc=900
        )

        return_mixer()

        movej(posj(self.pos_home), vel=J_VEL, acc=J_ACC)
        wait(0.2)

        log("Task 완료")

# ===============================
# 3. 로봇 초기화 (독립 스레드용)
# ===============================
def initialize_robot():
    from DSR_ROBOT2 import (
        set_tool, set_tcp, set_robot_mode, get_robot_mode,
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

# ===============================
# 5. 메인
# ===============================
def main(args=None):
    rclpy.init(args=args)

    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    DR_init.__dsr__node = robot_node

    task_node = TaskMixing()

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
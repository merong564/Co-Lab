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

        # home pose (joint)
        self.pos_home = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        # beaker
        self.pos_beaker_pick       = [617.838, 138.024, 120.460, 142.800, 179.222, 142.231]
        self.pos_beaker_pick_safe  = [617.838, 138.024, 226.696, 142.800, 179.222, 142.231]
        self.pos_beaker_place_safe = [406.705, 111.093, 203.088, 3.086, 178.554, -0.335]
        self.pos_beaker_place      = [303.736, 81.616, 86.386, 170.495, -178.848, 167.281]

        # mixer
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
            task_compliance_ctrl, release_compliance_ctrl,
            set_desired_force, release_force, DR_FC_MOD_REL,
            move_periodic, get_tool_force, get_current_posx, amovel,
            DR_BASE, DR_TOOL
        )

        def log(msg: str):
            self.get_logger().info(msg)

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)

        # ===============================
        # 속도/가속 (기본)
        # ===============================
        J_VEL, J_ACC = 40, 40
        L_VEL, L_ACC = 100, 100
        ON, OFF = 1, 0

        # ===============================
        # 그리퍼
        # ===============================
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

        # ===============================
        # 공정 함수
        # ===============================
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

        # ===============================
        # (A) 내려가면서 좌우 회전(선 꼬임 방지)
        # ===============================
        def descend_while_yaw_oscillate(
            target_z: float,
            dz: float = 1.5,          # 내려가는 스텝(mm)
            yaw_amp: float = 25.0,    # rz 좌우 진폭(deg)
            vel: float = 15.0,
            acc: float = 15.0,
            ref=DR_BASE,
            max_steps: int = 250,
            timeout_s: float = 20.0,
            fz_abort: float = 35.0,   # 과힘 안전 중단(N)
        ):
            """
            현재 pose에서 target_z까지 내려가며 rz를 rz0±yaw_amp로 왕복.
            (회전 누적 없음 → 케이블 꼬임 최소)
            """
            t0 = time.time()
            steps = 0
            sign = 1

            ret = get_current_posx(ref=ref)
            cur = ret[0] if isinstance(ret, tuple) else ret
            x, y, z, rx, ry, rz0 = map(float, cur)

            while z > target_z:
                if steps >= max_steps:
                    log("[중단] max_steps 도달 (하강/회전 루프 강제 종료)")
                    break
                if (time.time() - t0) > timeout_s:
                    log("[중단] timeout (하강/회전 루프 강제 종료)")
                    break

                # 과힘 방지
                f = get_tool_force(DR_TOOL)
                if isinstance(f, (list, tuple)) and len(f) >= 3:
                    fz = abs(float(f[2]))
                    if fz >= fz_abort:
                        log(f"[중단] 과힘 감지 fz={fz:.2f}N >= {fz_abort}N")
                        break

                z_next = max(target_z, z - dz)
                rz_next = rz0 + (yaw_amp * sign)

                movel(posx(x, y, z_next, rx, ry, rz_next), vel=vel, acc=acc, ref=ref)

                z = z_next
                sign *= -1
                steps += 1

        # ===============================
        # (B) 강한 탁탁(툴축 기준 + 2단 탭 + 비틀기)
        # ===============================
        def shake_off_before_return_strong(
            lift_mm=80.0,
            tap1_mm=25.0,
            tap2_mm=55.0,
            tap_count=14,
            up_vel=320, up_acc=700,
            down1_vel=350, down1_acc=800,
            down2_vel=650, down2_acc=1400,
            twist_deg=10.0,          # 탭 사이에 짧은 비틀기(좌우)
            ref=DR_TOOL,             # ✅ 툴축 기준으로 탁탁(체감 ↑)
            fz_abort=45.0,           # 과힘 안전 중단(N)
        ):
            log("[3-1] 믹서 털기(강화: 툴축/2단탭/비틀기) 시작")

            ret = get_current_posx(ref=ref)
            cur = ret[0] if isinstance(ret, tuple) else ret
            x, y, z, rx, ry, rz = map(float, cur)

            # 기준 높이로 올리기
            up = posx(x, y, z + lift_mm, rx, ry, rz)
            movel(up, vel=up_vel, acc=up_acc, ref=ref)
            wait(0.02)

            base_z = z + lift_mm
            down1 = posx(x, y, base_z - tap1_mm, rx, ry, rz)
            down2 = posx(x, y, base_z - tap2_mm, rx, ry, rz)

            # 비틀기 포즈(동일 높이에서 rz만 좌우)
            tw_p = posx(x, y, base_z, rx, ry, rz + twist_deg)
            tw_n = posx(x, y, base_z, rx, ry, rz - twist_deg)

            for i in range(int(tap_count)):
                # 과힘 방지
                f = get_tool_force(DR_TOOL)
                if isinstance(f, (list, tuple)) and len(f) >= 3:
                    fz = abs(float(f[2]))
                    if fz >= fz_abort:
                        log(f"[중단] 탁탁 중 과힘 fz={fz:.2f}N >= {fz_abort}N")
                        break

                # 2단 탭 (마지막 구간 더 빠르게)
                movel(down1, vel=down1_vel, acc=down1_acc, ref=ref)
                movel(down2, vel=down2_vel, acc=down2_acc, ref=ref)

                # 짧은 비틀기(점성 액체 잘 떨어짐) - 누적 회전 아님
                movel(tw_p if (i % 2 == 0) else tw_n, vel=220, acc=450, ref=ref)

                # 리바운드 (너무 느리면 탁 느낌 약해짐)
                movel(up, vel=up_vel, acc=up_acc, ref=ref)
                wait(0.01)

            log("[3-1] 믹서 털기 완료")

        # ===============================
        # (C) 순응 기반 혼합: 외력 감지 → 내려가며 좌우 회전 → 추가 혼합
        # ===============================
        def mixer_descend_and_mix_oscillate(end_pos_list, fz_trigger=7.0, down_force=-10.0):
            def _get_fz():
                ret = get_tool_force(DR_TOOL)
                if ret is None or isinstance(ret, int) or len(ret) < 3:
                    return 0.0
                return abs(float(ret[2]))

            target_z = float(end_pos_list[2])

            # 순응 + 원하는 힘(아래로)
            task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
            set_desired_force(fd=[0, 0, down_force, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            wait(0.1)

            # 1) 힘 감지 모드로 하강 시작 (비커/액체 접촉 외력 대기)
            log("1. 힘 감지 모드로 하강 시작 (외력 대기)")
            amovel(posx(end_pos_list), vel=10, acc=10, ref=DR_BASE)

            t0 = time.time()
            timeout_wait_contact = 12.0
            while True:
                fz = _get_fz()
                log(f'현재 fz: {fz:.2f} N, 목표: {fz_trigger} N')
                if fz >= fz_trigger:
                    log(f"[감지] 외력 도달 ({fz_trigger}N). 내려가며 좌우 회전 시작")
                    break
                if (time.time() - t0) > timeout_wait_contact:
                    log("[중단] 외력 대기 timeout - 강제로 다음 단계 진행(안전 주의)")
                    break
                wait(0.1)

            # 2) 내려가면서 좌우 회전 (선 꼬임 방지)
            log("2. 내려가며 좌우 회전(누적X)로 혼합")
            descend_while_yaw_oscillate(
                target_z=target_z,
                dz=1.5,
                yaw_amp=28.0,
                vel=15,
                acc=15,
                ref=DR_BASE,
                max_steps=260,
                timeout_s=22.0,
                fz_abort=35.0
            )

            # 3) 목표 도달 후 추가 혼합(짧은 주기 왕복)
            log(f"3. 목표 도달 후 추가 혼합 ({mixing_duration}초)")
            repeat_calc = max(1, int(mixing_duration / 1.8))
            move_periodic(
                amp=[0, 0, -5, 0, 0, 18],  # 왕복 성분(누적 회전 아님)
                period=1.8,
                atime=0.2,
                repeat=repeat_calc,
                ref=DR_TOOL
            )

            release_force()
            release_compliance_ctrl()
            wait(0.2)

        # =========================
        # 전체 흐름 제어
        # =========================
        pick_and_place_beaker()

        pick_and_ready_mixer()
        wait(0.2)

        # 잡은 상태에서 시작하고 싶은 경우
        # movel(posx(self.pos_mixer_mix_safe), vel=L_VEL, acc=L_ACC, ref=DR_BASE)

        log("[3] 순응 제어 기반 혼합 시작")
        mixer_descend_and_mix_oscillate(
            end_pos_list=self.pos_mixer_mix_down,
            fz_trigger=7.0,
            down_force=-10.0
        )

        # ✅ 강화 탁탁 (툴축/2단/비틀기/리바운드 빠르게)
        shake_off_before_return_strong(
            lift_mm=80.0,
            tap1_mm=25.0,
            tap2_mm=55.0,
            tap_count=14,
            up_vel=320, up_acc=700,
            down1_vel=350, down1_acc=800,
            down2_vel=650, down2_acc=1400,
            twist_deg=10.0,
            ref=DR_TOOL,
            fz_abort=45.0
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
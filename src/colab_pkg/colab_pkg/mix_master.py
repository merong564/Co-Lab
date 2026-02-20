#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

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
# 2. 로봇 초기화
# ===============================
def initialize_robot():
    """로봇 초기화 (툴/TCP/모드 세팅)"""
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


# ===============================
# 3. 작업 수행 로직 (단독 실행)
# ===============================
def perform_task(mixing_duration=0.0, logger=None):
    from DSR_ROBOT2 import (
        movej, movel,
        posj, posx,
        set_digital_output, wait,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        task_compliance_ctrl, release_compliance_ctrl,
        move_periodic, get_tool_force,
        DR_BASE, DR_TOOL
    )

    def log(msg: str):
        if logger is not None:
            logger.info(msg)
        else:
            print(msg)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    # 속도/가속
    J_VEL, J_ACC = 40, 40
    L_VEL, L_ACC = 100, 100

    # 그리퍼 DO
    ON, OFF = 1, 0

    def gripper_open():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(2.0)

    def gripper_close():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2.0)

    # 좌표 (posx)
    P = {
        "PICK_DOWN":  posx(306.636, -66.725, 109.141, 91.356, 91.786, 90.102),
        "PICK_UP":    posx(306.636, -66.725, 257.898, 91.356, 91.786, 90.102),
        "POUR_UP":    posx(585.440, 157.760, 242.631, 91.920, 97.360, 88.550),
        "POUR_READY": posx(585.440, 157.760, 180.631, 91.920, 97.360, 88.550),
    }

    # 홈 (posj)
    P0_HOME = posj(0, 0, 90, 0, 90, 0)

    # ✅ 비커 넣기 전/후 조인트 좌표
    BEAKER_BEFORE_J = posj(10.61, -0.20, 79.78, 178.91, -98.90, 62.01)
    BEAKER_AFTER_J  = posj(10.61,  1.40, 99.77, 178.91, -79.40, 62.01)

    # =========================
    # ✅ 요청 동작:
    # - BEFORE->AFTER 천천히 이동(분할 movej)
    # - 내려가다가 외력(Fz) 감지되면 그 순간부터:
    #     move_periodic(진폭 회전+Z진폭)로 섞으며 내려감
    # - AFTER 조인트 도달 성공하면 3초 더 periodic 후 종료
    # =========================
    def beaker_insert_with_force_trigger(
        before_j,
        after_j,
        fz_trigger=3.0,          # 외력 감지 임계값(N)
        j_vel_slow=10, j_acc_slow=10,
        steps=50,                # BEFORE->AFTER 분할수(클수록 천천히/부드럽게)
        amp_rz_deg=18.0,         # 회전 진폭(좌우 흔들기)
        amp_z_mm=-2.0,           # Z(툴) 진폭(살짝 눌러가며)
        period=1.0,              # periodic 주기
        extra_spin_sec=7.0,      # AFTER 도달 후 추가 섞기 시간
        timeout_sec=12.0         # force가 안 들어와도 무한대기 방지
    ):
        def _safe_get_fz():
            ret = get_tool_force(DR_TOOL)
            f = ret[0] if isinstance(ret, tuple) else ret
            if isinstance(f, (list, tuple)) and len(f) >= 3:
                return abs(float(f[2]))
            return None

        # 숫자 배열로 보간할 before/after 조인트 값(요청값 고정)
        b = [10.61, -0.20, 79.78, 178.91, -98.90, 62.01]
        a = [10.61,  1.40, 99.77, 178.91, -79.40, 62.01]

        # 1) before로 천천히 정렬
        movej(before_j, vel=j_vel_slow, acc=j_acc_slow)
        wait(0.2)

        triggered = False
        t0 = time.time()

        # 2) BEFORE -> AFTER 천천히 내려가며 이동
        for i in range(1, steps + 1):
            t = i / float(steps)
            j = [b[k] + (a[k] - b[k]) * t for k in range(6)]
            movej(posj(*j), vel=j_vel_slow, acc=j_acc_slow)
            wait(0.05)

            fz = _safe_get_fz()
            if fz is not None:
                log(f"[force] fz={fz:.2f}")
                if fz >= fz_trigger:
                    triggered = True
                    log(f"[force] TRIGGERED at fz={fz:.2f} (>= {fz_trigger})")
                    break
            else:
                log("[force] invalid/empty")

            if (time.time() - t0) > timeout_sec:
                log("[force] timeout while approaching -> continue without trigger")
                break

        # 3) 트리거 시: 순응 ON + periodic 섞기
        if triggered:
            task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])

            amp = [0, 0, amp_z_mm, 0, 0, amp_rz_deg]

            # 트리거 직후 섞기 조금
            move_periodic(
                amp=amp,
                period=period,
                atime=0.2,
                repeat=3,
                ref=DR_TOOL
            )

            # AFTER 조인트로 천천히 "도달" (성공 조건)
            movej(after_j, vel=j_vel_slow, acc=j_acc_slow)
            wait(0.2)

            # AFTER 도달 후 3초 더 섞기(진폭 회전)
            repeat_extra = max(1, int(extra_spin_sec / max(0.1, period)))
            move_periodic(
                amp=amp,
                period=period,
                atime=0.2,
                repeat=repeat_extra,
                ref=DR_TOOL
            )

            release_compliance_ctrl()
            wait(0.2)

        else:
            # 트리거 못 걸면 그냥 AFTER로 이동만
            movej(after_j, vel=j_vel_slow, acc=j_acc_slow)
            wait(0.2)

    # =========================
    # 시퀀스
    # =========================
    # log("[1] Go PICK_DOWN")
    # gripper_open()
    # movel(P["PICK_DOWN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # log("[2] Gripper CLOSE (pick)")
    # gripper_close()

    # log("[3] Go PICK_UP (lift)")
    # movel(P["PICK_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # log("[4] Go POUR_UP (move)")
    # movel(P["POUR_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # log("[5] Go POUR_READY (down)")
    # movel(P["POUR_READY"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # ✅ 여기서 "외력 감지 후 periodic 섞기" 수행
    log("[6] Beaker insert: slow approach -> force trigger -> periodic mix -> reach AFTER -> +3s mix")
    beaker_insert_with_force_trigger(
        BEAKER_BEFORE_J,
        BEAKER_AFTER_J,
        fz_trigger=4.5,
        j_vel_slow=10, j_acc_slow=10,
        steps=50,
        amp_rz_deg=18.0,
        amp_z_mm=-2.0,
        period=1.0,
        extra_spin_sec=3.0,
        timeout_sec=12.0
    )

    # =========================
    # 이후 복귀 시퀀스 유지
    # =========================
    # log("[7] Return to POUR_UP (safe lift before moving back)")
    # movel(P["POUR_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # log("[8] Move back to PICK_UP (safe height at pick area)")
    # movel(P["PICK_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # log("[9] Go PICK_DOWN (place back)")
    # movel(P["PICK_DOWN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # log("[10] Gripper OPEN (release at original place)")
    # gripper_open()

    # log("[11] Return HOME")
    # movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    # wait(0.2)

    # log("종료")


# ===============================
# 4. main: ros2 run 하면 즉시 1회 실행 후 종료
# ===============================
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("mix_master", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task(mixing_duration=0.0, logger=node.get_logger())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
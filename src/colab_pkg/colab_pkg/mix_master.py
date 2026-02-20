#!/usr/bin/env python3
import time
import rclpy
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
def perform_task(logger=None):
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

    # 홈 (posj)  ✅ 초기화 후 여기로 먼저 이동
    P0_HOME = posj(0, 0, 90, 0, 90, 0)

    # 픽업 좌표 (posx)  ✅ 너 코드 그대로 유지
    P_PICK_DOWN = posx(448.18, -180.31, 118.12, 110.37, -179.05, 110.13)
    P_PICK_UP   = posx(448.58, -179.86, 207.30, 104.66, -179.86, 104.49)

    # ✅ 비커 넣기 전/후 조인트 좌표
    BEAKER_BEFORE_J = posj(10.61, -0.20, 79.78, 178.91, -98.90, 62.01)
    BEAKER_AFTER_J  = posj(10.61,  1.40, 99.77, 178.91, -79.40, 62.02)

    # =========================
    # 비커 삽입: BEFORE->AFTER를 끝까지 분할 movej로 천천히 이동
    # - 트리거 없으면: AFTER까지 갔다가 -> BEFORE -> HOME
    # - 트리거 있으면: "트리거 순간부터" 회전하면서 내려가서 AFTER 도달,
    #                 트리거 시점 기준 총 7초가 찰 때까지 계속 회전 -> BEFORE -> HOME
    # =========================
    def beaker_insert_flow(
        before_j,
        after_j,
        fz_trigger=4.5,
        j_vel_slow=10, j_acc_slow=10,
        steps=80,
        amp_rz_deg=33.0,     # 18 + 15
        amp_z_mm=-2.0,
        period=1.0,
        extra_spin_sec=7.0
    ) -> bool:
        def _safe_get_fz():
            ret = get_tool_force(DR_TOOL)
            f = ret[0] if isinstance(ret, tuple) else ret
            if isinstance(f, (list, tuple)) and len(f) >= 3:
                return abs(float(f[2]))
            return None

        # 보간용 조인트 값(요청 고정)
        b = [10.61, -0.20, 79.78, 178.91, -98.90, 62.01]
        a = [10.61,  1.40, 99.77, 178.91, -79.40, 62.02]

        # 1) BEFORE 정렬
        movej(before_j, vel=j_vel_slow, acc=j_acc_slow)
        wait(0.2)

        triggered = False
        trig_t0 = None
        compliance_on = False

        # periodic 파라미터 (진폭)
        amp = [0, 0, amp_z_mm, 0, 0, amp_rz_deg]

        try:
            # 2) BEFORE -> AFTER : 끝까지 "분할 movej"
            for i in range(1, steps + 1):
                t = i / float(steps)
                j = [b[k] + (a[k] - b[k]) * t for k in range(6)]

                # (a) 내려가기(movej)
                movej(posj(*j), vel=j_vel_slow, acc=j_acc_slow)

                # (b) 트리거 체크(아직 안 걸렸으면)
                if not triggered:
                    fz = _safe_get_fz()
                    if fz is not None and fz >= fz_trigger:
                        triggered = True
                        trig_t0 = time.time()
                        log(f"[force] TRIGGERED at step {i}/{steps} (fz={fz:.2f} >= {fz_trigger})")

                        # 트리거 순간부터 compliance + periodic 시작
                        task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
                        compliance_on = True

                # (c) 트리거 이후: 내려가는 동안에도 계속 periodic을 이어붙여 수행
                if triggered:
                    move_periodic(
                        amp=amp,
                        period=period,
                        atime=0.2,
                        repeat=1,      # 한 사이클씩 계속 이어붙이기
                        ref=DR_TOOL
                    )

                wait(0.05)

            # 3) AFTER 도착 후: 트리거 시점부터 총 extra_spin_sec가 찰 때까지 계속 회전
            if triggered and trig_t0 is not None:
                while (time.time() - trig_t0) < extra_spin_sec:
                    move_periodic(
                        amp=amp,
                        period=period,
                        atime=0.2,
                        repeat=1,
                        ref=DR_TOOL
                    )
                    wait(0.05)

        finally:
            # compliance는 트리거 된 경우에만 해제
            if compliance_on:
                release_compliance_ctrl()
                wait(0.2)

        return triggered

    # =========================
    # ✅ 전체 흐름 (요청 반영)
    # 0) 초기화 후 HOME 먼저
    # 1) 픽업
    # 2) 붓는 위치 이동(pour_up/pour_ready) 삭제
    # 3) 바로 비커 삽입 흐름
    # 4) (트리거 여부 상관없이) BEFORE -> HOME
    # =========================
    log("[0] Go HOME first")
    movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    wait(0.2)

    log("[1] Pick: OPEN -> PICK_DOWN")
    gripper_open()
    movel(P_PICK_DOWN, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    log("[2] Pick: CLOSE")
    gripper_close()

    log("[3] Pick: LIFT to PICK_UP")
    movel(P_PICK_UP, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    log("[4] Go BEAKER_BEFORE_J (directly, no pour positions)")
    movej(BEAKER_BEFORE_J, vel=10, acc=10)
    wait(0.2)

    log("[5] Beaker insert: steady approach -> (if force) rotate while descending + until 7s total")
    triggered = beaker_insert_flow(
        BEAKER_BEFORE_J,
        BEAKER_AFTER_J,
        fz_trigger=4.5,
        j_vel_slow=10, j_acc_slow=10,
        steps=80,
        amp_rz_deg=33.0,
        amp_z_mm=-2.0,
        period=1.0,
        extra_spin_sec=7.0
    )

    if triggered:
        log("[6] Triggered: return BEFORE -> HOME")
    else:
        log("[6] No force until AFTER: return BEFORE -> HOME")

    movej(BEAKER_BEFORE_J, vel=10, acc=10)
    wait(0.2)

    movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    wait(0.2)

    log("종료")


# ===============================
# 4. main: ros2 run 하면 즉시 1회 실행 후 종료
# ===============================
def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("mix_master", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()
        perform_task(logger=node.get_logger())
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
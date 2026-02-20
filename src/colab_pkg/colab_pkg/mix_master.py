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

    # 홈 (조인트 홈은 movej 유지)
    P0_HOME = posj(0, 0, 90, 0, 90, 0)

    # 픽업 좌표 (posx) - 베이스 기준
    P_PICK_DOWN = posx(448.18, -180.31, 118.12, 110.37, -179.05, 110.13)
    P_PICK_UP   = posx(448.58, -179.86, 207.30, 104.66, -179.86, 104.49)


    # ✅ 너가 준 "테스크(posx, DR_BASE 기준)"으로 교체할 자리
    # 지금은 아직 안 줘서 placeholder. 받는 즉시 여기만 바꿔주면 됨.
    BEAKER_AFTER_X = posx(340.96, 84.35, 136.51, 26.49, -179.49, 26.20)  # TODO: 너가 줄 BEFORE 테스크 좌표
    BEAKER_BEFORE_X  = posx(340.96, 84.35, 276.83, 26.49, -179.49, 26.20)  # TODO: 너가 줄 AFTER  테스크 좌표

    # =========================
    # 비커 삽입: BEFORE_X -> AFTER_X
    # - 분할 movel로 천천히 접근
    # - force 트리거 걸리면: 그 순간부터 compliance + periodic
    # - AFTER 도달 후: 트리거 시점 기준 extra_spin_sec까지 periodic 유지
    # =========================
    def beaker_insert_flow(
        before_x,
        after_x,
        fz_trigger=4.5,
        vel_slow=30, acc_slow=30,
        steps=80,
        amp_rz_deg=60.0,
        amp_z_mm=-2.0,
        period=1.0,
        extra_spin_sec=20.0
    ) -> bool:

        def _safe_get_fz():
            ret = get_tool_force(DR_TOOL)
            f = ret[0] if isinstance(ret, tuple) else ret
            if isinstance(f, (list, tuple)) and len(f) >= 3:
                return abs(float(f[2]))
            return None

        # posx 보간용 6값 추출
        try:
            b = [before_x[0], before_x[1], before_x[2], before_x[3], before_x[4], before_x[5]]
            a = [after_x[0],  after_x[1],  after_x[2],  after_x[3],  after_x[4],  after_x[5]]
        except Exception:
            log("[ERR] posx 인덱싱 불가. (환경에 따라 posx가 list처럼 안 될 수 있음)")
            raise

        # 1) BEFORE 정렬
        movel(before_x, vel=vel_slow, acc=acc_slow, ref=DR_BASE)
        wait(0.2)

        triggered = False
        trig_t0 = None
        compliance_on = False

        amp = [0, 0, amp_z_mm, 0, 0, amp_rz_deg]

        try:
            # 2) BEFORE -> AFTER : 분할 movel
            for i in range(1, steps + 1):
                t = i / float(steps)
                x = [b[k] + (a[k] - b[k]) * t for k in range(6)]

                movel(posx(*x), vel=vel_slow, acc=acc_slow, ref=DR_BASE)

                if not triggered:
                    fz = _safe_get_fz()
                    if fz is not None and fz >= fz_trigger:
                        triggered = True
                        trig_t0 = time.time()
                        log(f"[force] TRIGGERED at step {i}/{steps} (fz={fz:.2f} >= {fz_trigger})")

                        task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
                        compliance_on = True

                if triggered:
                    move_periodic(
                        amp=amp,
                        period=period,
                        atime=0.2,
                        repeat=1,
                        ref=DR_TOOL
                    )

                wait(0.05)

            # 3) AFTER 도착 후: 트리거 시점 기준 extra_spin_sec까지 유지
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
            if compliance_on:
                release_compliance_ctrl()
                wait(0.2)

        return triggered

    # =========================
    # ✅ 전체 흐름
    # =========================
    log("[0] Go HOME first (movej)")
    movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    wait(0.2)

    log("[1] Pick: OPEN -> PICK_DOWN (movel)")
    gripper_open()
    movel(P_PICK_DOWN, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    log("[2] Pick: CLOSE")
    gripper_close()

    log("[3] Pick: LIFT to PICK_UP (movel)")
    movel(P_PICK_UP, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    # ✅ BEAKER 접근도 "단순 이동=movel"로 통일
    # (정렬 목적: 조인트 before로 먼저 맞추고 싶으면 movej로 1번 찍어도 되는데, 네 요청이 movel이라 movel로 감)
    log("[4] Go BEAKER_BEFORE_X (movel, DR_BASE)")
    movel(BEAKER_BEFORE_X, vel=30, acc=30, ref=DR_BASE)
    wait(0.2)

    log("[5] Beaker insert (movel interpolation + force trigger + periodic)")
    triggered = beaker_insert_flow(
        BEAKER_BEFORE_X,
        BEAKER_AFTER_X,
        fz_trigger=4.5,
        vel_slow=30, acc_slow=30,
        steps=80,
        amp_rz_deg=60.0,
        amp_z_mm=-2.0,
        period=1.0,
        extra_spin_sec=20.0
    )

    if triggered:
        log("[6] Triggered: return BEFORE -> release")
    else:
        log("[6] No force until AFTER: return BEFORE -> release")

    movel(BEAKER_BEFORE_X, vel=30, acc=30, ref=DR_BASE)
    wait(0.2)

    # ✅ 마무리: PICK_UP -> PICK_DOWN -> OPEN -> HOME
    movel(P_PICK_UP, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    movel(P_PICK_DOWN, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    gripper_open()

    movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    wait(0.2)

    log("종료")


# ===============================
# 4. main
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
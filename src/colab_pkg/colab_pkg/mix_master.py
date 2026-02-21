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

        # compliance/force
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        DR_FC_MOD_REL,

        # periodic / force read
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

    # ✅ 네가 준 방식 그대로: 순응+힘제어+periodic+stable 체크
    def compliance_wiggle(
        force_z=-20,                 # fd z
        amp=[0, 0, -5, 0, 0, 15],     # 네 값 그대로
        period=1.0,
        atime=0.2,
        repeat=10,
        stable_need=5,
        stable_dt=0.5,
        stable_min=10,
        stable_max=80,
        ref_force=DR_TOOL,
        ref_periodic=DR_TOOL,
    ):
        def _safe_get_fz():
            ret = get_tool_force(ref_force)
            f = ret[0] if isinstance(ret, tuple) else ret
            if isinstance(f, (list, tuple)) and len(f) >= 3:
                return abs(float(f[2]))
            return None

        # 순응 활성화
        task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
        wait(0.1)

        # Z축 기준 힘 인가(네 방식)
        fd = [0, 0, float(force_z), 0, 0, 0]
        fctrl_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        wait(0.1)

        # periodic (네 값 그대로)
        move_periodic(
            amp=amp,
            period=period,
            atime=atime,
            repeat=repeat,
            ref=ref_periodic
        )

        # stable 체크 (네 로직 그대로)
        stable = 0
        while stable < stable_need:
            fz = _safe_get_fz()
            if fz is None:
                log("[WARN] get_tool_force failed/None")
                stable = 0
            else:
                log(f"fz:{fz:.2f}")
                if (fz >= stable_min) and (fz <= stable_max):
                    stable += 1
                else:
                    stable = 0

            wait(stable_dt)

        # 종료
        release_force()
        release_compliance_ctrl()
        wait(0.2)

    # 홈 (조인트 홈은 movej 유지)
    P0_HOME = posj(0, 0, 90, 0, 90, 0)

    # 픽업 좌표 (posx) - 베이스 기준
    P_PICK_DOWN = posx(448.18, -180.31, 118.12, 110.37, -179.05, 110.13)
    P_PICK_UP   = posx(448.58, -179.86, 207.30, 104.66, -179.86, 104.49)

    # 비커 BEFORE/AFTER (posx) - 네 placeholder 유지
    BEAKER_AFTER_X   = posx(340.96, 84.35, 136.51, 26.49, -179.49, 26.20)
    BEAKER_BEFORE_X  = posx(340.96, 84.35, 276.83, 26.49, -179.49, 26.20)

    # =========================
    # 비커 삽입 flow (네 기존 유지)
    # - 단, periodic 방식은 "네 compliance 방식"으로 바꾸고 싶으면
    #   triggered 시점/AFTER 도착 후에 compliance_wiggle() 호출하면 됨.
    # =========================
    def beaker_insert_flow(
        before_x,
        after_x,
        fz_trigger=4.5,
        vel_slow=30, acc_slow=30,
        steps=80
    ) -> bool:
        def _safe_get_fz():
            ret = get_tool_force(DR_TOOL)
            f = ret[0] if isinstance(ret, tuple) else ret
            if isinstance(f, (list, tuple)) and len(f) >= 3:
                return abs(float(f[2]))
            return None

        b = [before_x[0], before_x[1], before_x[2], before_x[3], before_x[4], before_x[5]]
        a = [after_x[0],  after_x[1],  after_x[2],  after_x[3],  after_x[4],  after_x[5]]

        movel(before_x, vel=vel_slow, acc=acc_slow, ref=DR_BASE)
        wait(0.2)

        triggered = False

        for i in range(1, steps + 1):
            t = i / float(steps)
            x = [b[k] + (a[k] - b[k]) * t for k in range(6)]
            movel(posx(*x), vel=vel_slow, acc=acc_slow, ref=DR_BASE)

            if not triggered:
                fz = _safe_get_fz()
                if fz is not None and fz >= fz_trigger:
                    triggered = True
                    log(f"[force] TRIGGERED at step {i}/{steps} (fz={fz:.2f} >= {fz_trigger})")

            wait(0.05)

        # ✅ AFTER 도착 후 회전방식 = 네 compliance 방식으로 수행하고 싶으면 여기서 실행
        # (원하면 triggered 조건 걸고, 아니면 항상 실행)
        if triggered:
            compliance_wiggle(
                force_z=-20,
                amp=[0, 0, -5, 0, 0, 15],
                period=1.0,
                atime=0.2,
                repeat=10,
                stable_need=5,
                stable_dt=0.5,
                stable_min=10,
                stable_max=80,
                ref_force=DR_TOOL,
                ref_periodic=DR_TOOL
            )

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

    # ✅ 여기!! 너가 요청한 부분:
    # P_PICK_UP에서 P_PICK_DOWN으로 내려갈 때 "네 compliance 회전 방식" 적용
    log("[1.5] Go PICK_UP (movel)")
    movel(P_PICK_UP, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    log("[1.6] Down to PICK_DOWN (movel) -> compliance_wiggle (your rotation method)")
    movel(P_PICK_DOWN, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    compliance_wiggle(
        force_z=-20,
        amp=[0, 0, -5, 0, 0, 15],   # ✅ 네 각도/시간 그대로
        period=1.0,
        atime=0.2,
        repeat=10,
        stable_need=5,
        stable_dt=0.5,
        stable_min=10,
        stable_max=80,
        ref_force=DR_TOOL,
        ref_periodic=DR_TOOL
    )

    log("[2] Pick: CLOSE")
    gripper_close()

    log("[3] Pick: LIFT to PICK_UP (movel)")
    movel(P_PICK_UP, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    log("[4] Go BEAKER_BEFORE_X (movel, DR_BASE)")
    movel(BEAKER_BEFORE_X, vel=30, acc=30, ref=DR_BASE)
    wait(0.2)

    log("[5] Beaker insert (movel interpolation + force trigger)")
    triggered = beaker_insert_flow(
        BEAKER_BEFORE_X,
        BEAKER_AFTER_X,
        fz_trigger=4.5,
        vel_slow=30, acc_slow=30,
        steps=80
    )

    log(f"[6] insert done. triggered={triggered}")

    movel(BEAKER_BEFORE_X, vel=30, acc=30, ref=DR_BASE)
    wait(0.2)

    # 마무리: PICK_UP -> PICK_DOWN -> OPEN -> HOME
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
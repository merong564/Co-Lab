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

    # 순응+힘제어+periodic+stable 체크
    def compliance_wiggle(
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
        ref_periodic=DR_TOOL,
    ):
        def _safe_get_fz():
            ret = get_tool_force(ref_force)
            f = ret[0] if isinstance(ret, tuple) else ret
            if isinstance(f, (list, tuple)) and len(f) >= 3:
                return abs(float(f[2]))
            return None

        task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
        wait(0.1)

        fd = [0, 0, float(force_z), 0, 0, 0]
        fctrl_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        wait(0.1)

        # [추가] Z축 하강 중 외력(바닥 접촉)이 감지될 때까지 Wiggle 대기 로직 추가
        log("하강 중... 바닥 접촉 대기")
        while True:
            contact_fz = _safe_get_fz()
            if contact_fz is not None and contact_fz >= 10.0:  # 10N을 접촉 기준으로 설정
                log(f"외력 감지 (fz: {contact_fz:.2f}). Wiggle 시작.")
                break
            wait(0.1)

        move_periodic(
            amp=amp,
            period=period,
            atime=atime,
            repeat=repeat,
            ref=ref_periodic
        )

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

        release_force()
        release_compliance_ctrl()
        wait(0.2)

    P0_HOME = posj(0, 0, 90, 0, 90, 0)

    # [수정] 요청된 비커 이동 좌표 반영
    beaker_pick_x = posx(619.419, 137.199, 113.598, 148.997, 179.125, 148.536)
    beaker_up_x   = posx(406.705, 111.093, 203.088, 3.086, 178.554, -0.335)
    beaker_down_x = posx(303.736, 81.616, 86.386, 170.495, -178.848, 167.281)

    # [수정] 요청된 믹서 이동 좌표 반영
    mixer_pick_x        = posx(87.752, 443.877, 236.217, 114.003, 179.135, 113.295)
    mixer_forward_x     = posx(87.752, 190.136, 236.217, 114.003, 179.135, 113.295)
    mixer_beaker_up_x   = posx(349.592, 93.050, 233.490, 123.441, 179.314, 122.717)
    mixer_beaker_down_x = posx(349.592, 93.050, 125.172, 123.441, 179.314, 122.717)


    # =========================
    # 전체 흐름
    # =========================
    # log("[0] Go HOME first (movej)")
    # movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    # wait(0.2)

    # [추가] 1. 비커 잡고 혼합 위치로 옮기기 활성화 및 로직 수정
    log("[1] 비커 잡고 혼합 위치로 옮기기")
    gripper_open()
    movel(beaker_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_close()
    wait(2.0)

    movel(beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(beaker_down_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)

    gripper_open()
    wait(2.0)
    movel(beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)

    # [수정] 2. 믹서 잡고 혼합 위치로 옮기기
    log("[2] 믹서 잡고 혼합 위치로 옮기기")
    movel(mixer_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_close()
    wait(2.0)
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_beaker_down_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    # --- 3. 혼합하기 ---
    log("[3] 순응 제어 기반 혼합 시작")
    
    # [기존 로직 보존을 위한 주석 처리]
    # compliance_wiggle(
    #     force_z=-20,
    #     amp=[0, 0, -5, 0, 0, 15],
    #     period=1.0,
    #     atime=0.2,
    #     repeat=10,
    #     stable_need=5,
    #     stable_dt=0.5,
    #     stable_min=10,
    #     stable_max=80,
    #     ref_force=DR_TOOL,
    #     ref_periodic=DR_TOOL
    # )

    # [추가] 회전량 45도로 증가, 속도(주기) 0.5초로 단축, 짧아진 주기에 맞춰 repeat 횟수 20회로 증가
    compliance_wiggle(
        force_z=-20,
        amp=[0, 0, -5, 0, 0, 45],
        period=0.5,
        atime=0.2,
        repeat=20,
        stable_need=5,
        stable_dt=0.5,
        stable_min=10,
        stable_max=80,
        ref_force=DR_TOOL,
        ref_periodic=DR_TOOL
    )

    # [수정] 4. 믹서 원위치 및 종료
    log("[4] 믹서 원위치 및 종료")
    movel(mixer_beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_open()
    wait(2.0)
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
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
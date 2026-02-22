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
        set_digital_output, get_digital_input, wait,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS, 

        # compliance/force
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force,
        DR_FC_MOD_REL,

        # periodic / force read
        move_periodic, get_tool_force, get_current_posx, amovel,

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
        log("그리퍼 열기")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(2.0)

    def gripper_close():
        log("그리퍼 닫기")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2.0)

   # [수정] 복잡한 보간 로직을 제거하고, 순응 제어 하강과 좌표 확인 방식으로 재구성
    def mixer_descend_and_wiggle(
        end_posx,
        fz_trigger=7.0,    # tool에 가해지는 힘이 이 값 이상이면 혼합 시작
        down_force=-10.0  # Z축으로 누르며 내려갈 힘]
    ):
        def _get_fz():     # 툴에 걸리는 z축 방향 힘 읽어오는 함수
            ret = get_tool_force(DR_TOOL)

            # 통신 딜레이로 인한 None 또는 에러 코드 반환 시 0.0으로 안전하게 예외 처리
            if ret is None or isinstance(ret, int) or len(ret) < 3:
                return 0.0
            
            return abs(float(ret[2]))

        def _get_current_z():   # 현재 Z축 위치 읽어오는 함수
            ret = get_current_posx(DR_BASE)
            pos = ret[0] if isinstance(ret, tuple) else ret
            return float(pos[2])

        target_z = end_posx[2]

        # 1. 순응 제어 및 힘 제어 시작 (설정된 힘만큼 스스로 밀며 하강)
        task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
        set_desired_force(fd=[0, 0, down_force, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        wait(0.1)

        log("1. 힘 감지 모드로 하강 시작 (외력 대기)")
        
        amovel(posx(end_posx), vel=10, acc=10, ref=DR_BASE)
        
        # 2. 설정된 외력(fz_trigger)이 감지될 때까지 대기
        while True:
            fz = _get_fz()
            log(f'현재 fz: {fz:.2f} N, 목표: {fz_trigger} N')
            if _get_fz() >= fz_trigger:
                log(f"[감지] 외력 도달 ({fz_trigger}N). Wiggle을 시작합니다.")
                break
            wait(0.1)   # 0.1초마다 외력 체크

        ret_pos = get_current_posx(DR_BASE)
        curr_pos = list(ret_pos[0]) if isinstance(ret_pos, tuple) else list(ret_pos)

        # 3. 외력 감지 이후: 목표 Z 높이(pos_mixer_mix_down)에 도달할 때까지 Wiggle 수행
        log("2. 아래로 힘주며 하강 및 Wiggle 동시 수행")
        while True:
            curr_z = _get_current_z()
            if curr_z <= target_z:
                log(f"[도달] 목표 높이 도달 (현재 Z: {curr_z:.2f})")
                break
                
            # [추가] 2mm씩 Z축 하강
            # curr_pos[2] -= 2.0
            # if curr_pos[2] < target_z:
            #     curr_pos[2] = target_z
                
            # movel(posx(curr_pos), vel=10, acc=10, ref=DR_BASE)
            
            # Wiggle 1회전 수행 (이 코드가 도는 동안에도 힘 제어에 의해 계속 하강 중임)
            move_periodic(amp=[0, 0, -5, 0, 0, 15],    # 회전 각도 45도
                          period=1.0, 
                          atime=0.2, 
                          repeat=1, 
                          ref=DR_TOOL)

        # 4. 목표 도달 후 10번 제자리 추가 회전
        log(f"3. 목표 도달 후 추가 혼합")
        move_periodic(amp=[0, 0, -5, 0, 0, 15], 
                        period=2.0, 
                        atime=0.2, 
                        repeat=10, 
                        ref=DR_TOOL)

        # 5. 제어 해제
        release_force()
        release_compliance_ctrl()
        wait(0.2)

    P0_HOME = posj(0, 0, 90, 0, 90, 0)

    # 비커 관련 좌표값
    pos_beaker_pick       = posx(617.838, 138.024, 120.460, 142.800, 179.222, 142.231)  # 비커 잡는 위치
    pos_beaker_pick_safe  = posx(617.838, 138.024, 226.696, 142.800, 179.222, 142.231)  # 비커 잡기 전후로 Z축 높이만 안전하게 올린 좌표
    pos_beaker_place_safe = posx(406.705, 111.093, 203.088, 3.086, 178.554, -0.335)     # 비커 놓기 전후로 안전한 대기 자세 좌표
    pos_beaker_place      = posx(303.736, 81.616, 86.386, 170.495, -178.848, 167.281)   # 비커 놓는 위치

    # 믹서 관련 좌표값
    pos_mixer_pick        = posx(87.752, 443.877, 236.217, 114.003, 179.135, 113.295)     # 믹서 잡는 위치
    pos_mixer_pick_safe   = posx(87.752, 190.136, 236.217, 114.003, 179.135, 113.295)     # 믹서 잡기 전후로 y축으로만 이동하는 좌표
    pos_mixer_mix_safe    = posx(349.592, 93.050, 233.490, 123.441, 179.314, 122.717)     # 믹서 잡은 상태에서 비커 위로 이동한 좌표
    pos_mixer_mix_down    = posx(349.592, 93.050, 155.172, 123.441, 179.314, 122.717)     # 혼합 시 믹서를 최대로 내리는 좌표


    # 비커 Pick & Place 전용 함수
    def pick_and_place_beaker():
        log("[1] 비커 이동 작업 시작")
        gripper_open()
        
        # Pick 동작
        movel(pos_beaker_pick_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        movel(pos_beaker_pick, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        gripper_close()
        movel(pos_beaker_pick_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        
        # Place 동작
        movel(pos_beaker_place_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        movel(pos_beaker_place, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        gripper_open()
        movel(pos_beaker_place_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        log("비커 이동 작업 완료")
    
    # 믹서 Pick & 준비 위치 이동 전용 함수
    def pick_and_ready_mixer():
        log("[2] 믹서 픽업 및 대기 위치 이동 시작")
        movel(pos_mixer_pick_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        movel(pos_mixer_pick, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        gripper_close()
        movel(pos_mixer_pick_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        movel(pos_mixer_mix_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        log("믹서 대기 위치 이동 완료")

    # 믹서 원위치 반환 전용 함수
    def return_mixer():
        log("[4] 믹서 원위치 반환 시작")
        movel(pos_mixer_mix_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        movel(pos_mixer_pick_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        movel(pos_mixer_pick, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        gripper_open()
        movel(pos_mixer_pick_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        log("믹서 원위치 반환 완료")

    # =========================
    # 전체 흐름
    # =========================

    # 1. 비커 혼합 위치로 이동
    pick_and_place_beaker()

    # # 2. 믹서를 비커 위 준비 위치로 이동
    pick_and_ready_mixer()
    wait(0.2)

    # 믹서 잡은 채로 대기한 상태에서 시작하고 싶을 경우
    # movel(pos_mixer_mix_safe, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    # wait(0.2)

    # 3. 혼합하기
    log("[3] 순응 제어 기반 혼합 시작")

    # [추가] 분할 하강 및 Wiggle 동시 제어 로직 실행
    mixer_descend_and_wiggle(
        end_posx=pos_mixer_mix_down,
        fz_trigger=7.0,
        down_force=-10.0
    )

    # 4. 믹서 원위치 및 종료
    return_mixer()
    
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
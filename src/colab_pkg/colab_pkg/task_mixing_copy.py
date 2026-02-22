#!/usr/bin/env python3
import time
import math
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

# DSR_init 설정 (전역)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ===============================
# 2. 서비스 노드 클래스
# ===============================
class TaskMixing(Node):
    def __init__(self):
        super().__init__('task_mixing', namespace=ROBOT_ID) # 이름 명시
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.srv_mixing = self.create_service(
            RobotCommand,
            'execute_mixing',
            self.execute_mixing_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info("TaskMixing Ready. Service: execute_mixing")

    def execute_mixing_callback(self, request, response):
        mode = (getattr(request, "mode", "") or "").strip().upper()
        # UI에서 전달된 mixing_duration 추출 (기본값 설정)
        mixing_duration = float(getattr(request, "mixing_duration", 10.0))

        self.get_logger().info(f"[Service] Request Received. Mode: {mode}, Duration: {mixing_duration}s")
        
        # 믹싱 작업 수행
        try:
            perform_task(mixing_duration, logger=self.get_logger())
            response.success = True
            response.message = f"{mode} Mixing Completed Successfully"
        except Exception as e:
            self.get_logger().error(f"Task failed: {e}")
            response.success = False
            response.message = f"{mode} Mixing Failed: {str(e)}"
            
        return response

# ===============================
# 3. 로봇 초기화
# ===============================
def initialize_robot():
    """로봇 초기화"""
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
# 4. 핵심 작업 수행 로직 (외력 감지 + 믹싱)
# ===============================
def perform_task(mixing_duration=10.0, logger=None):
    from DSR_ROBOT2 import (
        movej, movel, posj, posx,
        set_digital_output, wait, set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        task_compliance_ctrl, release_compliance_ctrl,
        set_desired_force, release_force, DR_FC_MOD_REL,
        move_periodic, get_tool_force,
        DR_BASE, DR_TOOL
    )

    def log(msg: str):
        if logger is not None:
            logger.info(msg)
        else:
            print(msg)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    J_VEL, J_ACC = 40, 40          
    L_VEL, L_ACC = 100, 100          

    ON, OFF = 1, 0

    def gripper_open():
        set_digital_output(2, ON)
        set_digital_output(1, OFF)
        wait(2.0)

    def gripper_close():
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(2.0)

    # 좌표 설정
    beaker_pick_x = posx(619.419, 137.199, 113.598, 148.997, 179.125, 148.536)
    beaker_up_x   = posx(406.705, 111.093, 203.088, 3.086, 178.554, -0.335)
    beaker_down_x = posx(303.736, 81.616, 86.386, 170.495, -178.848, 167.281)

    mixer_pick_x        = posx(87.752, 443.877, 236.217, 114.003, 179.135, 113.295)
    mixer_forward_x     = posx(87.752, 190.136, 236.217, 114.003, 179.135, 113.295)
    mixer_beaker_up_x   = posx(349.592, 93.050, 233.490, 123.441, 179.314, 122.717)
    mixer_beaker_down_x = posx(349.592, 93.050, 125.172, 123.441, 179.314, 122.717)

    # 안전하게 Z축 외력 읽어오는 헬퍼 함수
    def _safe_get_fz():
        ret = get_tool_force(DR_TOOL)
        f = ret[0] if isinstance(ret, tuple) else ret
        if isinstance(f, (list, tuple)) and len(f) >= 3:
            return abs(float(f[2]))
        return None

    # -------------------------------------------------------------------
    # [추가됨] 특이점(Singularity) 회피를 위해 관절 각도 기반 Home 위치로 먼저 이동
    P0_HOME = posj(0, 0, 90, 0, 90, 0)
    log("[0] 초기 위치(Home)로 먼저 이동합니다. (특이점 회피)")
    movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    wait(1.0)
    # -------------------------------------------------------------------

    # --- 1. 비커 이동 ---
    log("[1] 비커 잡고 혼합 위치로 옮기기")
    gripper_open()
    movel(beaker_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_close()
    
    movel(beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(beaker_down_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_open()
    movel(beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)

    # --- 2. 믹서 이동 ---
    log("[2] 믹서 잡고 혼합 위치로 옮기기")
    movel(mixer_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_close()
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    
    # 믹서를 천천히 내리기 위한 세팅
    movel(mixer_beaker_down_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    # --- 3. 순응 제어 기반 외력 감지 및 혼합 ---
    log("[3] 순응 제어 활성화 및 비즈 외력 대기 (시뮬레이션 모드)")
    task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
    wait(0.1)

    # Z축으로 누르는 힘 설정 (비즈에 닿기 위해)
    fd = [0, 0, -20, 0, 0, 0]
    fctrl_dir = [0, 0, 1, 0, 0, 0]
    set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
    wait(0.1)

    # [수정] 시뮬레이션용 외력 가짜 생성 (3초 후 5N 발생)
    contact_threshold = 3.0
    sim_start_time = time.time()
    
    while True:
        # 실제 로봇에서는 아래 줄 사용: 
        # contact_fz = _safe_get_fz()
        
        # 가상환경 테스트용: 3초 대기 후 임의의 힘(5.0N) 발생
        if time.time() - sim_start_time > 3.0:
            contact_fz = 5.0
        else:
            contact_fz = 0.0

        if contact_fz is not None and contact_fz >= contact_threshold:
            log(f"비즈 외력 감지 성공 (fz: {contact_fz:.2f}N). 혼합을 시작합니다.")
            break
        wait(0.1)

    # 믹싱 동작 (period=0.5초 기준, UI에서 받은 duration에 맞춰 반복 횟수 계산)
    period = 0.5
    repeat_count = max(1, int(mixing_duration / period))
    log(f"총 {mixing_duration}초 간 혼합 진행 (반복 횟수: {repeat_count}회)")
    
    move_periodic(
        amp=[0, 0, -5, 0, 0, 45], # 45도 회전
        period=period,
        atime=0.2,
        repeat=repeat_count,
        ref=DR_TOOL
    )

    # 힘 제어 및 순응 제어 해제
    release_force()
    release_compliance_ctrl()
    wait(0.2)

    # --- 4. 믹서 원위치 ---
    log("[4] 믹서 원위치 및 종료")
    movel(mixer_beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_open()
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    
    log("Task 완료됨")

# ===============================
# 5. 메인
# ===============================
def main(args=None):
    rclpy.init(args=args)

    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    task_node = TaskMixing() 

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
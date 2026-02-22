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
        self.get_logger().info("TaskMixing Ready. Service: execute_mixing")

    def execute_mixing_callback(self, request, response):
        mode = (getattr(request, "mode", "") or "").strip().upper()
        mixing_duration = float(getattr(request, "mixing_duration", 10.0))

        self.get_logger().info(f"[Service] Request Received. Mode: {mode}, Duration: {mixing_duration}s")
        
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
# 4. 핵심 작업 수행 로직 (팀원 수정사항 반영)
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

    # [수정] 팀원 코드의 그리퍼 로직 반영 (DO 1, 2 사용)
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

    # [좌표 확인] 팀원 코드와 동일하게 유지
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

    # 특이점 회피를 위한 Home 이동
    P0_HOME = posj(0, 0, 90, 0, 90, 0)
    log("[0] 초기 위치(Home)로 이동")
    movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    wait(0.2)

    # --- 1. 비커 이동 (팀원 코드에서 비활성화된 경우 필요시 주석 해제) ---
    log("[1] 비커 잡고 혼합 위치로 옮기기")
    gripper_open()
    movel(beaker_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_close()
    movel(beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(beaker_down_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_open()
    wait(2.0)
    movel(beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)

    # --- 2. 믹서 이동 ---
    log("[2] 믹서 잡고 혼합 위치로 옮기기")
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_close()
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    wait(0.2)

    # --- 3. 순응 제어 기반 외력 감지 및 혼합 ---
    log("[3] 순응 제어 활성화 및 바닥 접촉 대기")
    task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
    wait(0.1)

    # Z축 하강 힘 설정 (-20N)
    fd = [0, 0, -20, 0, 0, 0]
    set_desired_force(fd, dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
    
    # [수정] 팀원 업데이트 반영: 접촉 임계값 10.0N
    contact_threshold = 10.0
    while True:
        contact_fz = _safe_get_fz()
        if contact_fz is not None and contact_fz >= contact_threshold:
            log(f"외력 감지 성공 (fz: {contact_fz:.2f}). 믹싱 시작.")
            break
        wait(0.1)

    # 힘 제어 일시 해제 후 믹싱 모션 진입
    release_force()
    release_compliance_ctrl()
    wait(0.2)

    # [수정] 팀원 업데이트 반영: 회전량 45도, 주기 0.5s
    # 서비스로 받은 mixing_duration에 맞춰 repeat 횟수 계산
    period = 0.5
    repeat_calc = max(1, int(mixing_duration / period))
    
    log(f"믹싱 시작: {mixing_duration}초 ({repeat_calc}회 반복)")
    move_periodic(
        amp=[0, 0, -5, 0, 0, 45],
        period=period,
        atime=0.2,
        repeat=repeat_calc,
        ref=DR_TOOL
    )

    # --- 4. 믹서 원위치 ---
    log("[4] 믹서 원위치 및 종료")
    movel(mixer_beaker_up_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movel(mixer_pick_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    gripper_open()
    movel(mixer_forward_x, vel=L_VEL, acc=L_ACC, ref=DR_BASE)
    movej(P0_HOME, vel=J_VEL, acc=J_ACC)
    
    log("Task 완료")

# ===============================
# 5. 메인 (MultiThreadedExecutor 유지)
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
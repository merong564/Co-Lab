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
        
        # [수정] 콜백 그룹 설정
        self.callback_group = ReentrantCallbackGroup()
        
        self.srv_mixing = self.create_service(
            RobotCommand,
            'execute_mixing',
            self.execute_mixing_callback,
            callback_group=self.callback_group
        )
        
        # [추가] 좌표값 클래스 멤버 변수로 초기화 (안전성을 위해 리스트로 저장)
        self.pos_home = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        
        self.pos_beaker_pick       = [617.838, 138.024, 120.460, 142.800, 179.222, 142.231]
        self.pos_beaker_pick_safe  = [617.838, 138.024, 226.696, 142.800, 179.222, 142.231]
        self.pos_beaker_place_safe = [406.705, 111.093, 203.088, 3.086, 178.554, -0.335]
        self.pos_beaker_place      = [303.736, 81.616, 86.386, 170.495, -178.848, 167.281]

        self.pos_mixer_pick        = [87.752, 443.877, 236.217, 114.003, 179.135, 113.295]
        self.pos_mixer_pick_safe   = [87.752, 190.136, 236.217, 114.003, 179.135, 113.295]
        self.pos_mixer_mix_safe    = [349.592, 93.050, 233.490, 123.441, 179.314, 122.717]
        self.pos_mixer_mix_down    = [349.592, 93.050, 125.172, 123.441, 179.314, 122.717]

        self.get_logger().info("TaskMixing Ready. Service: execute_mixing")

    def execute_mixing_callback(self, request, response):
        mode = (getattr(request, "mode", "") or "").strip().upper()
        mixing_duration = float(getattr(request, "mixing_duration", 10.0))

        self.get_logger().info(f"[Service] Request Received. Mode: {mode}, Duration: {mixing_duration}s")
        
        try:
            # [수정] self 객체를 전달하여 내부에서 멤버 변수에 접근하도록 구성
            self.perform_task(mixing_duration)
            response.success = True
            response.message = f"{mode} Mixing Completed Successfully"
        except Exception as e:
            self.get_logger().error(f"Task failed: {e}")
            response.success = False
            response.message = f"{mode} Mixing Failed: {str(e)}"
            
        return response

    # [수정] 함수 위치 이동 (클래스 내부 메서드로 통합)
    def perform_task(self, mixing_duration=10.0):
        from DSR_ROBOT2 import (
            movej, movel, posj, posx,
            set_digital_output, wait, set_robot_mode, ROBOT_MODE_AUTONOMOUS,
            task_compliance_ctrl, release_compliance_ctrl,
            set_desired_force, release_force, DR_FC_MOD_REL,
            move_periodic, get_tool_force, get_current_posx,
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

        def mixer_descend_and_wiggle(end_pos_list, fz_trigger=10.0, down_force=-10.0):
            def _get_fz():     
                ret = get_tool_force(DR_TOOL)
                if ret is None or isinstance(ret, int) or len(ret) < 3:
                    return 0.0
                return abs(float(ret[2]))

            def _get_current_z():   
                ret = get_current_posx(DR_BASE)
                pos = ret[0] if isinstance(ret, tuple) else ret
                return float(pos[2])

            target_z = end_pos_list[2]
            
            task_compliance_ctrl(stx=[3000, 3000, 100, 100, 100, 100])
            set_desired_force(fd=[0, 0, down_force, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
            wait(0.1)

            log("1. 힘 감지 모드로 하강 시작 (외력 대기)")
            while True:
                if _get_fz() >= fz_trigger:
                    log(f"[감지] 외력 도달 ({fz_trigger}N). Wiggle을 시작합니다.")
                    break
                wait(0.1)

            log("2. 아래로 힘주며 하강 및 Wiggle 동시 수행")
            while True:
                curr_z = _get_current_z()
                if curr_z <= target_z:
                    log(f"[도달] 목표 높이 도달 (현재 Z: {curr_z:.2f})")
                    break
                move_periodic(amp=[0, 0, 0, 0, 0, 45.0], period=0.5, atime=0.2, repeat=1, ref=DR_TOOL)

            log(f"3. 목표 도달 후 추가 혼합 ({mixing_duration}초)")
            repeat_calc = max(1, int(mixing_duration / 0.5))
            move_periodic(amp=[0, 0, 0, 0, 0, 45.0], period=0.5, atime=0.2, repeat=repeat_calc, ref=DR_TOOL)

            release_force()
            release_compliance_ctrl()
            wait(0.2)

        # =========================
        # 전체 흐름 제어
        # =========================
        pick_and_place_beaker()
        
        pick_and_ready_mixer()
        wait(0.2)

        log("[3] 순응 제어 기반 혼합 시작")
        mixer_descend_and_wiggle(
            end_pos_list=self.pos_mixer_mix_down,
            fz_trigger=10.0,
            down_force=-10.0
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
    
    # [수정] 순서 보장: DR_init 세팅 후 TaskMixing 인스턴스화
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
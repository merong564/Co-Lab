#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init
import math

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

        self.get_logger().info(f"[Service] Request Received. Mode: {mode}")
        
        # [수정] 불필요한 tube_type 파라미터 제거 및 mixing 전용 task 호출
        perform_task()
        
        response.success = True
        response.message = f"{mode} Mixing Completed"
        return response

# ===============================
# 2. 로봇 제어 로직
# ===============================
def initialize_robot():
    """로봇 초기화"""
    from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, get_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
    
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(1.0)

    print("#" * 50)
    print(f" Robot Initialized (Mode: {get_robot_mode()})")
    print("#" * 50)

# [수정] 제공해주신 mixing 전용 로직으로 대체
def perform_task():
    from DSR_ROBOT2 import (
        movej, movel,
        posj, posx,
        get_current_posx,
        set_digital_output, wait,
        set_robot_mode, ROBOT_MODE_AUTONOMOUS,
        DR_BASE
    )

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    J_VEL, J_ACC = 60, 60          
    L_VEL, L_ACC = 60, 60          

    DO_OPEN = 1
    DO_CLOSE = 2

    def gripper_open():
        set_digital_output(DO_CLOSE, 0)
        set_digital_output(DO_OPEN, 1)

    def gripper_close():
        set_digital_output(DO_OPEN, 0)
        set_digital_output(DO_CLOSE, 1)

    beaker_approach_j = posj(-17.561, 71.881, 35.950, 76.814, 105.046, -21.195)
    beaker_pick_j     = posj(-15.252, 70.070, 39.779, 77.202, 102.451, -22.726)

    gripper_open()
    wait(0.2)

    movej(beaker_approach_j, vel=J_VEL, acc=J_ACC)
    wait(0.5)
    movej(beaker_pick_j,     vel=J_VEL, acc=J_ACC)

    gripper_close()
    wait(0.5)

    lift_j = posj(-15.228, 63.002, 32.623, 80.680, 105.138, -8.335)
    movej(lift_j, vel=J_VEL, acc=J_ACC)
    wait(0.2)

    REF = DR_BASE
    cur, _ = get_current_posx(ref=REF)
    cx, cy, cz, rx0, ry0, rz0 = cur   

    R = 50.0        
    TURNS = 3      
    STEPS = 60    
    BLEND = 2.0    

    movel(posx(cx + R, cy, cz, rx0, ry0, rz0),
          vel=L_VEL, acc=L_ACC, ref=REF, radius=0.0)

    total_steps = TURNS * STEPS
    for i in range(1, total_steps + 1):
        th = 2.0 * math.pi * (i / STEPS)   

        x = cx + R * math.cos(th)
        y = cy + R * math.sin(th)

        rad = 0.0 if i == total_steps else BLEND  
        movel(posx(x, y, cz, rx0, ry0, rz0),
              vel=L_VEL, acc=L_ACC, ref=REF, radius=rad)
        
# ===============================
# 3. 메인
# ===============================
def main(args=None):
    rclpy.init(args=args)

    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    task_node = TaskMixing() # [수정] TaskMixing 노드로 변경

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
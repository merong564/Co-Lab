#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
import DR_init

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
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

STOP_REQUESTED = False


class TaskRecovery(Node):
    def __init__(self):
        super().__init__('task_recovery', namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()

        # STOP 토픽 구독 (복구 중 2차 비상 정지 대비)
        self.sub_stop = self.create_subscription(String, 'stop', self.stop_callback, 10, callback_group=self.callback_group)

        # 복구용 서비스 서버
        self.srv = self.create_service(
            RobotCommand,
            'execute_recovery',
            self.execute_recovery_callback,
            callback_group=self.callback_group
        )

        # 4번 테스트: Z 올리고 홈 복귀 높이
        self.Z_UP_MM = 120.0

        # 로봇 기본 세팅
        try:
            from DSR_ROBOT2 import set_tool, set_tcp, set_robot_mode, ROBOT_MODE_AUTONOMOUS
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            set_tool(ROBOT_TOOL)
            set_tcp(ROBOT_TCP)
        except Exception as e:
            self.get_logger().warn(f"Robot init setting failed (continue): {e}")

        self.get_logger().info("TaskRecovery Ready. Service: /dsr01/execute_recovery")

    def stop_callback(self, msg: String):
        global STOP_REQUESTED
        cmd = (msg.data or "").strip().upper()
        if cmd == "STOP":
            STOP_REQUESTED = True
            self.get_logger().warn("[RECOVERY-STOP] received -> halting recovery motion!")
            # try:
            #     from DSR_ROBOT2 import stop
            #     stop(0)
            # except Exception:
            #     pass
        elif cmd == "RESET":
            STOP_REQUESTED = False
            self.get_logger().info("[RECOVERY-RESET] received -> unlocked")

    def execute_recovery_callback(self, request, response):
        global STOP_REQUESTED
        
        # 컨트롤러에서 RESET을 쐈으므로 False여야 정상. 만약 아직 True라면 거부
        if STOP_REQUESTED:
            response.success = False
            response.message = "Cannot recover while STOP_REQUESTED is True"
            return response

        # 들고 있는 물체 식별
        target = ""
        if getattr(request, "targets", None) and len(request.targets) > 0:
            target = str(request.targets[0]).strip().upper()

        self.get_logger().info(f"=== [RECOVERY STARTED] Target in hand: '{target}' ===")

        try:
            self.perform_task(target)

            if STOP_REQUESTED:
                raise RuntimeError("STOP requested during recovery task")

            response.success = True
            response.message = "RECOVERY_DONE"
        except Exception as e:
            self.get_logger().error(f"[RECOVERY FAILED]: {e}")
            response.success = False
            response.message = str(e)

        return response

    def perform_task(self, target):
        from DSR_ROBOT2 import (
            wait, get_current_posx, movel, movej, amovel, amovej, check_motion, 
            set_digital_output, posx, posj, DR_BASE,
            release_force, release_compliance_ctrl
        )
        global STOP_REQUESTED

        # --- 1) 안전 모션 커스텀 함수 정의 ---
        def _check_stop(tag=""):
            global STOP_REQUESTED
            if STOP_REQUESTED:
                raise RuntimeError(f"STOP at: {tag}")

        def custom_movel(*args, **kwargs):
            while check_motion() == 1:
                _check_stop("wait previous motion end")
                time.sleep(0.05)
            amovel(*args, **kwargs)
            wait_start = time.time()
            while check_motion() == 0 and (time.time() - wait_start) < 1.0:
                _check_stop("wait motion start")
                time.sleep(0.05)
            idle_count = 0
            while True:
                if check_motion() == 0: idle_count += 1
                else: idle_count = 0
                if idle_count >= 3: break
                _check_stop("during movel")
                time.sleep(0.05)

        def custom_movej(*args, **kwargs):
            while check_motion() == 1:
                _check_stop("wait previous motion end")
                time.sleep(0.05)
            amovej(*args, **kwargs)
            wait_start = time.time()
            while check_motion() == 0 and (time.time() - wait_start) < 1.0:
                _check_stop("wait motion start")
                time.sleep(0.05)
            while check_motion() == 1:
                _check_stop("during movej")
                time.sleep(0.05)

        movel = custom_movel
        movej = custom_movej

        L_VEL, L_ACC = 100, 100
        J_VEL, J_ACC = 40, 40
        ON, OFF = 1, 0
        home_j = posj(0, 0, 90.0, 0, 90.0, 0)

        def gripper_open():
            set_digital_output(3, OFF)
            set_digital_output(4, OFF)
            set_digital_output(2, ON)
            set_digital_output(1, OFF)
            time.sleep(2.0)

        def gripper_large_open():
            set_digital_output(1, OFF)
            set_digital_output(2, OFF)
            set_digital_output(3, ON)
            set_digital_output(4, OFF)
            time.sleep(2.0)

        def get_poses(posx_func):
            return {
                "LARGE": {
                    "RETURN_UP": posx_func(398.040, 351.050, 313.608, 90.329, 93.577, 89.530),
                    "RETURN_DOWN": posx_func(389.408, 563.410, 50.182, 91.859, 98.698, 89.033),
                    "AFTER_RETURN": posx_func(371.852, 520.254, 207.882, 89.661, 92.225, 89.338),
                    "AFTER_RETURN_UP": posx_func(355.186, 533.297, 400.718, 89.688, 92.114, 88.361),
                    "FINAL_POS": posx_func(309.545, 313.500, 128.890, 89.844, 90.996, 92.951)
                },
                "SMALL1": {
                    "PICK_DOWN": posx_func(333.096, 373.067, 128.164, 91.215, 89.984, 92.903),
                    "PICK_UP": posx_func(333.096, 373.067, 224.104, 91.215, 89.984, 92.903),
                    "FINAL_POS": posx_func(309.545, 313.500, 128.890, 89.844, 90.996, 92.951)
                },
                "SMALL2": {
                    "PICK_DOWN": posx_func(217.794, 377.263, 133.564, 121.034, 93.617, 92.329),
                    "PICK_UP": posx_func(216.423, 384.357, 282.484, 120.725, 94.227, 91.915),
                    "FINAL_POS": posx_func(309.545, 313.500, 128.890, 89.844, 90.996, 92.951)
                },
                "BEAKER": {
                    "RETURN_UP": posx_func(368.058, 423.059, 230.706, 19.522, 178.596, 15.563),
                    "RETURN_DOWN": posx_func(368.058, 423.059, 82.706, 19.522, 178.596, 15.563)
                },
                "MIXER": {
                    "PICK_SAFE": posx_func(87.752, 190.136, 236.217, 114.003, 179.135, 113.295),
                    "PICK": posx_func(87.752, 443.877, 236.217, 114.003, 179.135, 113.295)
                }
            }
        
        POSES = get_poses(posx)

        # release_force()
        # release_compliance_ctrl()


        # [추가] 로봇 모드 전환을 통한 제어기 하드웨어 락 강제 해제
        self.get_logger().info(">>> [STEP 1-1] Resetting Robot Mode to clear locks...")
        try:
            from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
            set_robot_mode(ROBOT_MODE_MANUAL)
            time.sleep(0.5)
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().warn(f"Mode reset failed: {e}")

        self.get_logger().info(">>> [STEP 1] Starting Recovery Motion Sequence...")
        _check_stop("before Z-UP")
        
        self.get_logger().info(">>> [STEP 2] Calling get_current_posx()...")
        cur, _ = get_current_posx(ref=DR_BASE)
        x, y, z, rx, ry, rz = cur
        self.get_logger().info(f">>> [STEP 3] Current Z: {z:.1f}. Moving Z-UP (+{self.Z_UP_MM}mm)...")
        
        movel(posx(x, y, z + self.Z_UP_MM, rx, ry, rz), vel=L_VEL, acc=L_ACC, ref=DR_BASE)
        
        self.get_logger().info(">>> [STEP 4] Z-UP Finished.")


        # [수정] 타겟이 없을 때만 홈으로 이동
        if not target:
            self.get_logger().info(">>> [STEP 5] Hand is empty. Moving to HOME...")
            _check_stop("before HOME")
            movej(home_j, vel=J_VEL, acc=J_ACC)
            self.get_logger().info("[RECOVERY] Recovery completed at HOME.")
            return

        if target not in POSES:
            self.get_logger().warn(f"[RECOVERY] Unknown target '{target}'. Keeping it in hand and moving HOME.")
            _check_stop("before HOME")
            movej(home_j, vel=J_VEL, acc=J_ACC)
            return

        # [수정] 타겟이 있을 경우 중간 홈 이동 생략(삭제) 후 바로 반환 슬롯으로 이동
        P = POSES[target]
        self.get_logger().info(f">>> [STEP 5] Target '{target}' in hand. Moving directly to return slot...")

        if target == "LARGE":
            movel(P["RETURN_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(P["RETURN_DOWN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_large_open()
            movel(P["AFTER_RETURN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(P["AFTER_RETURN_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(P["FINAL_POS"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            
        elif target in ["SMALL1", "SMALL2"]:
            movel(P["PICK_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(P["PICK_DOWN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_open()
            movel(P["PICK_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(P["FINAL_POS"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            
        elif target == "BEAKER":
            movel(P["RETURN_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(P["RETURN_DOWN"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_open()
            movel(P["RETURN_UP"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            
        elif target == "MIXER":
            movel(P["PICK_SAFE"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            movel(P["PICK"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)
            gripper_open()
            movel(P["PICK_SAFE"], vel=L_VEL, acc=L_ACC, ref=DR_BASE)

        self.get_logger().info(">>> [STEP 6] Target dropped. Returning to HOME...")
        _check_stop("before final HOME") # [추가] 마지막 복귀 전 정지 확인
        movej(home_j, vel=J_VEL, acc=J_ACC)
        self.get_logger().info(f"[RECOVERY] {target} securely dropped. Robot at HOME.")

def main(args=None):
    rclpy.init(args=args)

    robot_node = rclpy.create_node("dsr_bridge_recovery", namespace=ROBOT_ID)
    DR_init.__dsr__node = robot_node

    task_node = TaskRecovery()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    print("\n=== Recovery Service Server Started ===")
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
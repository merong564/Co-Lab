#!/usr/bin/env python3
import time
import threading
import csv # [추가] CSV 로깅용 모듈 임포트
import os # [추가] 파일 존재 여부 확인용 모듈 임포트
from datetime import datetime # [추가] 실험 일시 기록용 모듈 임포트

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import DR_init

from colab_interfaces.srv import RobotCommand
from std_msgs.msg import Float32, String

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

NEW_TCP_NAME = "CustomTCP" # [추가] 새 TCP 이름
NEW_TCP_OFFSET = [0.0, 0.0, 228.0, 0.0, 0.0, 0.0] # [추가] 새 TCP 오프셋 [X, Y, Z, Rx, Ry, Rz]

FINGER_TCP_NAME = "FingerTCP" # [추가] 집게 끝단 기준 새 TCP 이름
FINGER_TCP_OFFSET = [-32.0, 0.0, 228.0, 0.0, 0.0, 0.0] # [추가] 비커 쪽 집게 위치 오프셋 (실제 거리에 맞춰 Y 또는 X축 값 수정 필요)

VELOCITY = 40
ACC = 60
P_GAIN = 0.01
MAX_TILT_STEP = 1.0
STOP_THRESHOLD = 20.0

# STOP 신호 플래그 (STOP 토픽 받으면 True)
STOP_REQUESTED = False

# ==========================================
# [추가] 정량 지표 계산 함수
# ==========================================
def calc_metrics(log_t, log_w, target_w, final_w):
    if not log_w:
        return
    
    max_w = max(log_w)
    overshoot = max(0.0, max_w - target_w)
    
    rise_w = target_w * 0.9
    rise_t = 0.0
    for t, w in zip(log_t, log_w):
        if w >= rise_w:
            rise_t = t
            break
            
    err_bound = target_w * 0.02
    set_t = 0.0
    for i in range(len(log_w)-1, -1, -1):
        if abs(log_w[i] - target_w) > err_bound:
            set_t = log_t[min(i+1, len(log_t)-1)]
            break
            
    ss_err = abs(target_w - final_w)
    
    print("--- [Metrics] ---")
    print(f"Overshoot: {overshoot:.2f} g")
    print(f"Rise Time (90%): {rise_t:.2f} s")
    print(f"Settling Time (2%): {set_t:.2f} s")
    print(f"SS Error: {ss_err:.2f} g")
    print("-----------------")

    # [추가] 실험 메타데이터 및 정량 지표 CSV 저장 로직 시작
    file_path = "pouring_metrics.csv" # [추가]
    file_exists = os.path.isfile(file_path) # [추가]
    
    with open(file_path, mode='a', newline='') as f: # [추가]
        writer = csv.writer(f) # [추가]
        if not file_exists: # [추가] 헤더가 없을 경우 생성
            writer.writerow(["Timestamp", "Target_W", "Final_W", "P_GAIN", "MAX_TILT_STEP", "STOP_THRESHOLD", "Overshoot", "Rise_Time", "Settling_Time", "SS_Error"]) # [추가]
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S") # [추가]
        writer.writerow([timestamp, target_w, final_w, P_GAIN, MAX_TILT_STEP, STOP_THRESHOLD, round(overshoot, 2), round(rise_t, 2), round(set_t, 2), round(ss_err, 2)]) # [추가]

# ==========================================
# 2. 통신 전담 노드 (서비스 & 토픽)
# ==========================================
class TaskPouring(Node):
    def __init__(self):
        super().__init__("task_pouring", namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()
        self.current_weight = 0.0

        # 서비스 서버
        self.srv_pouring = self.create_service(
            RobotCommand,
            "execute_pouring",
            self.execute_pouring_callback,
            callback_group=self.callback_group,
        )

        # 무게 구독
        self.sub_weight = self.create_subscription(
            Float32,
            "load_cell/weight",
            self.weight_callback,
            10,
            callback_group=self.callback_group,
        )

        # STOP 토픽 구독 (/dsr01/stop)
        self.sub_stop = self.create_subscription(
            String,
            "stop",
            self.stop_callback,
            10,
            callback_group=self.callback_group,
        )

    def stop_callback(self, msg: String):
        global STOP_REQUESTED
        if (msg.data or "").strip().upper() != "STOP":
            return

        STOP_REQUESTED = True
        self.get_logger().warn("🚨 STOP received -> will return to ready pose then finish")

    def execute_pouring_callback(self, request, response):
        self.get_logger().info(f"[Service] Request Received. Target: {request.target_weight}g")

        success = perform_task(self, float(request.target_weight))

        response.success = bool(success)
        response.message = "Pouring Completed" if success else "Pouring Failed"
        return response

    def weight_callback(self, msg: Float32):
        self.current_weight = float(msg.data)

# ==========================================
# 3. 로봇 제어 로직 (DSR 라이브러리 사용)
# ==========================================
def initialize_robot():
    from DSR_ROBOT2 import (set_tool, set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,)
    from DSR_ROBOT2 import add_tcp # [추가] 새로운 TCP 등록을 위한 함수 임포트

    time.sleep(3.0)  # 노드 연결 대기(안전)
    try:
        print("[Thread] Initializing Robot settings...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)

        add_tcp(NEW_TCP_NAME, NEW_TCP_OFFSET) # [추가] 신규 TCP 정의
        set_tcp(NEW_TCP_NAME) # [추가] 정의된 신규 TCP로 변경

        add_tcp(FINGER_TCP_NAME, FINGER_TCP_OFFSET) # [추가] 집게 끝단 TCP 정의
        set_tcp(FINGER_TCP_NAME) # [추가] 회전 중심을 집게 끝단으로 변경

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print(f" [Thread] Robot Initialized: {ROBOT_ID}")
    except Exception as e:
        print(f" [Thread] Init Failed: {e}")

def calculate_tilt_angle(current_w: float, target_w: float):
    error = target_w - current_w
    delta_angle = error * P_GAIN

    if delta_angle > MAX_TILT_STEP:
        delta_angle = MAX_TILT_STEP
    elif delta_angle < -MAX_TILT_STEP:
        delta_angle = -MAX_TILT_STEP

    return float(delta_angle), float(error)

def perform_task(node: TaskPouring, target_weight: float) -> bool:
    from DSR_ROBOT2 import movej, get_current_posj, movel, posx, wait
    from DSR_ROBOT2 import set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS # [추가] 모드 변경 함수 임포트
    
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(0.5)

    global STOP_REQUESTED
    global P_GAIN, MAX_TILT_STEP, STOP_THRESHOLD # [추가] 전역 변수 선언

    # [추가] 목표 무게에 따른 파라미터 분기
    if target_weight < 100.0:
        P_GAIN = 0.03
        MAX_TILT_STEP = 3.0
        STOP_THRESHOLD = 10.0
    else:
        P_GAIN = 0.01
        MAX_TILT_STEP = 1.0
        STOP_THRESHOLD = 20.0

    print(f"[SYSTEM] Task Start! Target: {target_weight}g")

    start_t = time.time() # [추가] 시작 시간 기록
    log_t = [] # [추가] 경과 시간 로깅 리스트
    log_w = [] # [추가] 현재 무게 로깅 리스트
    log_d = [] # [추가] 제어 입력(delta) 로깅 리스트

    pour_ready_pos = posx(585.440, 157.760, 160.631, 91.920, 97.360, 88.550)

    # 시작 자세로 이동
    try:
        movel(pour_ready_pos, vel=100, acc=100)
        wait(1.0)

        set_robot_mode(ROBOT_MODE_MANUAL) # [추가]
        set_tcp(FINGER_TCP_NAME) # [추가]
        set_robot_mode(ROBOT_MODE_AUTONOMOUS) # [추가]
        time.sleep(0.5)

    except Exception as e:
        print(f"[ERROR] Move Failed: {e}")
        return False

    stop_target = target_weight - STOP_THRESHOLD

    while rclpy.ok():
        # STOP 들어오면: 자세 복귀 후 종료(노드 유지)
        if STOP_REQUESTED:
            try:
                movel(pour_ready_pos, vel=150, acc=150)
                wait(1.0)
            except Exception as e:
                print(f"[ERROR] Return Move Failed after STOP: {e}")
                STOP_REQUESTED = False
                return False

            print(" [STOP] Returned to ready pose. Finishing task.")
            STOP_REQUESTED = False
            return True  # STOP을 '정상 종료'로 볼지 여부(원하면 False로)

        current_weight = float(node.current_weight)

        cur_t = time.time() - start_t # [추가] 현재 경과 시간 계산
        log_t.append(cur_t) # [추가] 시간 로깅
        log_w.append(current_weight) # [추가] 무게 로깅

        # 목표 근처 도달하면 복귀 자세로 이동 후 종료
        if current_weight >= stop_target:
            try:
                movel(pour_ready_pos, vel=150, acc=150)
                wait(1.0)
            except Exception as e:
                print(f"[ERROR] Return Move Failed: {e}")
                return False
            
            time.sleep(3.0)
            final_settled_weight = float(node.current_weight)
            
            calc_metrics(log_t, log_w, target_weight, final_settled_weight) # [추가] 정량 지표 계산 및 출력
            
            print(f" [Done] Final: {final_settled_weight:.1f}g (stop_target={stop_target:.1f}g)")
            return True

        delta, error = calculate_tilt_angle(current_weight, target_weight)
        log_d.append(delta) # [추가] 제어 입력 로깅

        try:
            # current_joints = get_current_posj()
            # if not current_joints:
            #     print("[ERROR] get_current_posj() returned empty.")
            #     return False

            # target_joints = list(current_joints)
            # target_joints[5] += delta  # J6

            # movej(target_joints, vel=VELOCITY, acc=ACC) # [기존 코드 주석 처리]
            
            rel_pos = posx(0.0, 0.0, 0.0, 0.0, 0.0, delta) # [추가] 툴 좌표계 기준 Z축 회전량(delta) 설정
            movel(rel_pos, vel=VELOCITY, acc=ACC, ref=1, mod=1) # [추가] ref=1(툴 좌표계), mod=1(상대 이동) 적용하여 직교 제어

            print(f"Cur: {current_weight:.1f} | Delta: {delta:.2f} | Err: {error:.1f}")
            #time.sleep(0.1)

        except Exception as e:
            print(f"[ERROR] Tilt Move Failed: {e}")
            return False

    return False

# ==========================================
# 4. 메인 (노드 분리 전략 적용)
# ==========================================
def main(args=None):
    rclpy.init(args=args)

    # 1) 로봇 제어용 노드 (DSR이 독점)
    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)

    # 2) 통신용 노드 (서비스/무게/STOP)
    task_node = TaskPouring()

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = robot_node

    executor = MultiThreadedExecutor()
    executor.add_node(robot_node)
    executor.add_node(task_node)

    init_thread = threading.Thread(target=initialize_robot, daemon=True)
    init_thread.start()

    print("==========================================")
    print(" [Ready] Service Server Started (Multi-Node) ")
    print(f" - service: /{ROBOT_ID}/execute_pouring")
    print(f" - weight : /{ROBOT_ID}/load_cell/weight (Float32)")
    print(f" - stop   : /{ROBOT_ID}/stop (String, data='STOP')")
    print("==========================================")

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
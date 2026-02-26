#!/usr/bin/env python3
import time
import threading
import csv # [추가] CSV 로깅용 모듈 임포트
import os # [추가] 파일 존재 여부 확인용 모듈 임포트
from datetime import datetime # [추가] 실험 일시 기록용 모듈 임포트
import math # [추가] 기구학 연산용 모듈 임포트

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import DR_init

from colab_interfaces.srv import RobotCommand
from std_msgs.msg import Float32, String
from colab_interfaces.msg import ControlLive, ControlResult # [수정] 메시지 분리 임포트

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

FINGER_TCP_NAME = "FingerTCP" # [추가] 집게 끝단 기준 새 TCP 이름
FINGER_TCP_OFFSET = [-32.0, 0.0, 228.0, 0.0, 0.0, 0.0] # [추가] 비커 쪽 집게 위치 오프셋 (실제 거리에 맞춰 Y 또는 X축 값 수정 필요)

VELOCITY = 40
ACC = 60

prev_error = 0.0 # [추가] 이전 오차 저장용 변수

# STOP 신호 플래그 (STOP 토픽 받으면 True)
STOP_REQUESTED = False

TUBE_TUNING = {
    "LARGE": {
        "P_GAIN": 0.015, "D_GAIN": 0.08, "MAX_TILT_STEP": 1.0, "STOP_THRESHOLD": 0.5,
        "INIT_TILT_STEP": 1.5, "TILT_BACK_ANGLE": -1.0 # [추가] 초기 틸팅 및 후퇴 각도 튜닝값
    },
    "SMALL1": {
        "P_GAIN": 0.015, "D_GAIN": 0.15, "MAX_TILT_STEP": 0.2, "STOP_THRESHOLD": 0.5, # [수정] 오버슛 방지를 위한 상향
        "INIT_TILT_STEP": 0.8, "TILT_BACK_ANGLE": -1.5 # [추가] 초기 틸팅 및 후퇴 각도 튜닝값
    },
    "SMALL2": {
        "P_GAIN": 0.015, "D_GAIN": 0.15, "MAX_TILT_STEP": 0.2, "STOP_THRESHOLD": 0.5, # [수정] 오버슛 방지를 위한 상향
        "INIT_TILT_STEP": 0.8, "TILT_BACK_ANGLE": -1.5 # [추가] 초기 틸팅 및 후퇴 각도 튜닝값
    }
}

# ==========================================
# [추가] 실시간 기구학 데이터 계산 함수
# ==========================================
def calc_kinematics(prev_tcp_vel, prev_time):
    from DSR_ROBOT2 import get_current_velx
    
    try:
        vel_data = get_current_velx()
        vx, vy, vz = vel_data[0], vel_data[1], vel_data[2]
        tcp_vel = math.sqrt(vx**2 + vy**2 + vz**2)
        pour_speed = abs(vel_data[5]) # Rz 기준 회전(틸팅) 속도
    except Exception:
        tcp_vel = 0.0
        pour_speed = 0.0

    current_time = time.time()
    dt = current_time - prev_time

    if dt > 0:
        tcp_acc = (tcp_vel - prev_tcp_vel) / dt
    else:
        tcp_acc = 0.0

    return tcp_vel, tcp_acc, pour_speed, current_time

# ==========================================
# [추가] 정량 지표 계산 함수
# ==========================================
def calc_metrics(log_t, log_w, target_w, final_w, p_gain, d_gain, max_tilt_step, stop_thresh, init_tilt_step, tilt_back_angle, tube_type, actual_cycle_time):
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
    
    error_rate = (ss_err / target_w) * 100 if target_w > 0 else 0.0
    p_d_ratio = p_gain / d_gain if d_gain > 0 else 0.0
    avg_pouring_rate = (final_w / rise_t) if rise_t > 0 else 0.0
    cycle_time = actual_cycle_time 
    overhead_time = cycle_time - set_t 
    
    print("--- [Metrics] ---")
    print(f"Overshoot: {overshoot:.2f} g")
    print(f"Rise Time (90%): {rise_t:.2f} s")
    print(f"Settling Time (2%): {set_t:.2f} s")
    print(f"SS Error: {ss_err:.2f} g")
    print(f"Error Rate: {error_rate:.2f} %") 
    print(f"Avg Pouring Rate: {avg_pouring_rate:.2f} g/s") 
    print("-----------------")

    file_path = "pouring_metrics.csv"
    file_exists = os.path.isfile(file_path)
    
    with open(file_path, mode='a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow([
                "Timestamp", "Target_W", "Final_W", "P_GAIN", "D_GAIN", "MAX_TILT_STEP", 
                "STOP_THRESHOLD", "INIT_TILT_STEP", "TILT_BACK_ANGLE", "Overshoot(g)", "Rise_Time", "Settling_Time", "SS_Error(g)", # [수정] 신규 튜닝 요소 컬럼 추가
                "Material", "Error_Rate(%)", "P_D_Ratio", "Avg_Pouring_Rate(g/s)", "Cycle_Time", "Overhead_Time"
            ]) 
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        writer.writerow([
            timestamp, 
            round(target_w, 2), 
            round(final_w, 2), 
            round(p_gain, 3), 
            round(d_gain, 3), 
            round(max_tilt_step, 2), 
            round(stop_thresh, 2), 
            round(init_tilt_step, 2), # [추가] INIT_TILT_STEP 기록
            round(tilt_back_angle, 2), # [추가] TILT_BACK_ANGLE 기록
            round(overshoot, 2), 
            round(rise_t, 2), 
            round(set_t, 2), 
            round(ss_err, 2),
            tube_type, 
            round(error_rate, 2), 
            round(p_d_ratio, 2), 
            round(avg_pouring_rate, 2), 
            round(cycle_time, 2), 
            round(overhead_time, 2) 
        ])

    return overshoot, rise_t, set_t, ss_err, error_rate, p_d_ratio
# ==========================================
# 2. 통신 전담 노드 (서비스 & 토픽)
# ==========================================
class TaskPouring(Node):
    def __init__(self):
        super().__init__("task_pouring", namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()
        self.current_weight = 0.0

        # [추가] 디버그 모드 파라미터 선언 및 획득
        self.declare_parameter("debug_mode", False)
        self.debug_mode = self.get_parameter("debug_mode").value

        # [수정] 목적에 따른 퍼블리셔 분리
        self.pub_live = self.create_publisher(ControlLive, "log_control_live", 10)
        self.pub_result = self.create_publisher(ControlResult, "log_control_result", 10)

        # [추가] 디버그 모드일 경우 rqt_plot용 퍼블리셔 생성
        if self.debug_mode:
            self.pub_debug_delta = self.create_publisher(Float32, "debug/delta", 10)
            self.pub_debug_weight = self.create_publisher(Float32, "debug/weight", 10)
            self.get_logger().info("[SYSTEM] Debug mode ON: Publishing rqt_plot topics.")

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
        self.sub_stop = self.create_subscription(String, "stop", self.stop_callback, 10, callback_group=self.callback_group)

    def stop_callback(self, msg: String):
            global STOP_REQUESTED
            cmd = (msg.data or "").strip().upper()

            if cmd == "STOP":
                STOP_REQUESTED = True
                self.get_logger().warn("[STOP] received -> flag set (node stays alive)")

            elif cmd == "RESET":
                STOP_REQUESTED = False
                self.get_logger().info("[RESET] received -> flag cleared")

    def execute_pouring_callback(self, request, response):
        # [수정] 배열 형태로 전달된 targets 및 target_weights에서 값 추출
        target_w = request.target_weights[0] if request.target_weights else 0.0
        tube_type = request.targets[0].strip().upper() if request.targets else "LARGE" # [추가] 시험관 종류 추출
        
        self.get_logger().info(f"[Service] Request Received. Tube: {tube_type}, Target: {target_w}g")

        success = perform_task(self, float(target_w), tube_type) # [수정] 시험관 종류 파라미터 추가 전달

        response.success = bool(success)
        response.message = "Pouring Completed" if success else "Pouring Failed"
        response.held_object = tube_type
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

        add_tcp(FINGER_TCP_NAME, FINGER_TCP_OFFSET) # [추가] 집게 끝단 TCP 정의

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print(f" [Thread] Robot Initialized: {ROBOT_ID}")
    except Exception as e:
        print(f" [Thread] Init Failed: {e}")

def calculate_tilt_angle_pd(current_w: float, target_w: float, p_gain: float, d_gain: float, max_tilt_step: float): # [추가] 순수 PD 제어기
    global prev_error
    
    error = target_w - current_w # [추가]
    
    # [추가] PD 제어 계산 (단일 P_GAIN 유지)
    p_term = error * p_gain
    d_term = (error - prev_error) * d_gain
    prev_error = error # [추가] 현재 오차를 다음 사이클을 위해 저장
    
    delta_angle = p_term + d_term # [추가]
    
    if delta_angle > max_tilt_step: # [추가] 동적으로 전달받은 max_tilt_step 적용
        delta_angle = max_tilt_step 
    elif delta_angle < -max_tilt_step: 
        delta_angle = -max_tilt_step

    return float(delta_angle), float(error) # [추가]

def perform_task(node: TaskPouring, target_weight: float, tube_type: str = "LARGE") -> bool:
    from DSR_ROBOT2 import movej, get_current_posj, amovel, movel, check_motion, posx, wait
    from DSR_ROBOT2 import set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS # [추가] 모드 변경 함수 임포트
    
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(0.5)

    global STOP_REQUESTED
    global P_GAIN, MAX_TILT_STEP, STOP_THRESHOLD # [추가] 전역 변수 선언
    global prev_error

    # [추가] amovel, amovej, check_motion 임포트 (DSR_ROBOT2)
    def _check_stop(tag=""):
        global STOP_REQUESTED
        if STOP_REQUESTED:
            raise RuntimeError(f"STOP at: {tag}")

    def custom_movel(*args, **kwargs):
            while check_motion() == 1:
                print('이전 모션 중', flush=True)
                _check_stop("wait previous motion end")
                time.sleep(0.05)
            amovel(*args, **kwargs)
            wait_start = time.time()
            while check_motion() == 0 and (time.time() - wait_start) < 1.0:
                print('movel 모션 중인데 안 움직이는 중', flush=True)
                _check_stop("wait motion start")
                time.sleep(0.05)
            idle_count = 0
            while True:
                if check_motion() == 0:
                    idle_count += 1
                else:
                    idle_count = 0
                if idle_count >= 3:
                    break
                _check_stop("during movel")
                time.sleep(0.05)

    # [추가] movej, wait도 동일하게 커스텀 함수 작성 후 덮어쓰기
    # movel = custom_movel

    # [추가] 타겟 종류에 맞는 튜닝 파라미터 설정
    tuning = TUBE_TUNING[tube_type]
    active_p_gain = tuning["P_GAIN"]
    active_d_gain = tuning["D_GAIN"]
    active_max_tilt_step = tuning["MAX_TILT_STEP"]
    active_stop_thresh = tuning["STOP_THRESHOLD"]
    active_init_tilt_step = tuning.get("INIT_TILT_STEP", 1.5) # [추가] 동적 초기 틸팅값 로드
    active_tilt_back_angle = tuning.get("TILT_BACK_ANGLE", -1.0) # [추가] 동적 후퇴 각도 로드

    start_t = time.time()
    log_t = [] # [추가] 경과 시간 로깅 리스트
    log_w = [] # [추가] 현재 무게 로깅 리스트
    log_d = [] # [추가] 제어 입력(delta) 로깅 리스트

    # [추가] 속도 및 가속도 계산을 위한 이전 상태 변수
    prev_time = time.time()
    prev_tcp_vel = 0.0
    prev_pour_speed = 0.0

    # [추가] 초기 무게 감지 플래그 추가
    weight_detected = False
    is_dribble_mode = False # [추가] 미세 제어 1회 진입 확인용 플래그

    stop_target = target_weight - active_stop_thresh
    

    print(f"[SYSTEM] Task Start! Target: {target_weight}g | Tube: {tube_type} | Final Stop: {stop_target}g")

    if tube_type == "LARGE":
        pour_ready_pos = posx(561.045, 144.760, 175.965, 91.920, 97.358, 88.558)
    else: # "SMALL1", "SMALL2"
        pour_ready_pos = posx(585.440, 144.760, 160.631, 91.920, 97.360, 88.550)

    # 시작 자세로 이동
    try:
        # [수정] 액체 출렁임 방지를 위해 이동 속도 하향 (vel=50, acc=50)
        movel(pour_ready_pos, vel=50, acc=50)
        wait(1.0)

        set_robot_mode(ROBOT_MODE_MANUAL) # [추가]
        set_tcp(FINGER_TCP_NAME) # [추가]
        time.sleep(0.5) # [수정] 제어기 TCP 변경 적용 대기
        set_robot_mode(ROBOT_MODE_AUTONOMOUS) # [추가]
        time.sleep(0.5) # [수정] 모드 전환 완료 대기

    except Exception as e:
        print(f"[ERROR] Move Failed: {e}")
        return False

    prev_error = target_weight - float(node.current_weight)

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
            return False

        current_weight = float(node.current_weight)

        cur_t = time.time() - start_t # [추가] 현재 경과 시간 계산
        log_t.append(cur_t) # [추가] 시간 로깅
        log_w.append(current_weight) # [추가] 무게 로깅

        # 목표 근처 도달하면 복귀 자세로 이동 후 종료
        if current_weight >= stop_target:
            try:
                movel(pour_ready_pos, vel=100, acc=100)
                wait(1.0)
            except Exception as e:
                print(f"[ERROR] Return Move Failed: {e}")
                return False
            
            time.sleep(3.0)
            final_settled_weight = float(node.current_weight)

            actual_cycle_time = time.time() - start_t
            
            # [수정] active_init_tilt_step, active_tilt_back_angle 인자 추가 전달
            metrics_result = calc_metrics(log_t, log_w, target_weight, final_settled_weight, 
                                          active_p_gain, active_d_gain, active_max_tilt_step, active_stop_thresh, 
                                          active_init_tilt_step, active_tilt_back_angle,
                                          tube_type, actual_cycle_time)   
            
            # [수정] 종료 후 결과 퍼블리시 (ControlResult)
            if metrics_result:
                overshoot, rise_t, set_t, ss_err, error_rate, p_d_ratio = metrics_result
                
                msg_result = ControlResult()
                msg_result.p_gain = float(active_p_gain)
                msg_result.d_gain = float(active_d_gain)
                msg_result.max_tilt_step = float(active_max_tilt_step)
                msg_result.stop_threshold = float(active_stop_thresh)
                msg_result.p_d_ratio = float(p_d_ratio)
                msg_result.overshoot = float(overshoot)
                msg_result.rise_time = float(rise_t)
                msg_result.settling_time = float(set_t)
                msg_result.ss_error = float(ss_err)
                msg_result.error_rate = float(error_rate)
                msg_result.final_settled_weight = float(final_settled_weight)
                node.pub_result.publish(msg_result)

            print(f" [Done] Final: {final_settled_weight:.1f}g (stop_target={stop_target:.1f}g)")
            return True

        # [수정] 초기 틸팅 및 Tilt-back 로직에 동적 튜닝 파라미터 적용
        if not weight_detected:
            if current_weight >= 1.0: 
                weight_detected = True
                print("[SYSTEM] Liquid detected. Executing initial Tilt-back to cut flow.")
                
                try:
                    # [수정] 하드코딩된 -1.0 대신 active_tilt_back_angle 적용
                    tilt_back_pos = posx(0.0, 0.0, 0.0, 0.0, 0.0, active_tilt_back_angle)
                    movel(tilt_back_pos, vel=20, acc=20, ref=1, mod=1)
                    wait(0.5) 
                except Exception as e:
                    print(f"[ERROR] Initial Tilt-back Move Failed: {e}")
                    return False
                
                continue
            else:
                # [수정] 하드코딩된 1.5 대신 active_init_tilt_step 적용
                delta = active_init_tilt_step
                error = target_weight - current_weight
        else:
            delta, error = calculate_tilt_angle_pd(current_weight, target_weight, active_p_gain, active_d_gain, active_max_tilt_step)
        
        log_d.append(delta) # [추가] 제어 입력 로깅

        # [추가] 실시간 기구학 데이터 계산
        tcp_vel, tcp_acc, pour_speed, current_time = calc_kinematics(prev_tcp_vel, prev_time)
        prev_tcp_vel = tcp_vel
        prev_pour_speed = pour_speed
        prev_time = current_time

        # [수정] 실시간 제어 지표 퍼블리시 (ControlLive)
        msg_live = ControlLive()
        msg_live.tcp_vel = float(tcp_vel)
        msg_live.tcp_acc = float(tcp_acc)
        msg_live.pour_speed = float(pour_speed)
        node.pub_live.publish(msg_live)

        # [추가] 디버깅 모드일 경우 rqt_plot을 위한 데이터 퍼블리시
        if getattr(node, 'debug_mode', False):
            msg_delta = Float32()
            msg_delta.data = float(delta)
            node.pub_debug_delta.publish(msg_delta)

            msg_weight = Float32()
            msg_weight.data = float(current_weight)
            node.pub_debug_weight.publish(msg_weight)

        try:
            rel_pos = posx(0.0, 0.0, 0.0, 0.0, 0.0, delta) # [추가] 툴 좌표계 기준 Z축 회전량(delta) 설정
            movel(rel_pos, vel=VELOCITY, acc=ACC, ref=1, mod=1) # [추가] ref=1(툴 좌표계), mod=1(상대 이동) 적용하여 직교 제어

            print(f"Cur: {current_weight:.1f} | Delta: {delta:.2f} | Err: {error:.1f}")

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
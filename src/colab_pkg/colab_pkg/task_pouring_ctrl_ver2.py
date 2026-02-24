#!/usr/bin/env python3
import time
import threading
import csv 
import os 
from datetime import datetime 
import math 

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import DR_init

from colab_interfaces.srv import RobotCommand
from std_msgs.msg import Float32, String
from colab_interfaces.msg import SystemStatus, ControlMetrics # 💡 [수정] ControlMetrics 임포트 추가

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

NEW_TCP_NAME = "CustomTCP" 
NEW_TCP_OFFSET = [0.0, 0.0, 228.0, 0.0, 0.0, 0.0] 

FINGER_TCP_NAME = "FingerTCP" 
FINGER_TCP_OFFSET = [-32.0, 0.0, 228.0, 0.0, 0.0, 0.0] 

VELOCITY = 40
ACC = 60

prev_error = 0.0 
STOP_REQUESTED = False

TUBE_TUNING = {
    "LARGE": {"P_GAIN": 0.015, "D_GAIN": 0.08, "MAX_TILT_STEP": 1.0, "STOP_THRESHOLD": 12.0},
    "SMALL1": {"P_GAIN": 0.015, "D_GAIN": 0.15, "MAX_TILT_STEP": 0.2, "STOP_THRESHOLD": 1.5},
    "SMALL2": {"P_GAIN": 0.015, "D_GAIN": 0.15, "MAX_TILT_STEP": 0.2, "STOP_THRESHOLD": 1.5}
}

# ==========================================
# 정량 지표 계산 함수 (node 파라미터 추가)
# ==========================================
def calc_metrics(node, log_t, log_w, target_w, final_w, p_gain, d_gain, max_tilt_step, stop_thresh, tube_type, actual_cycle_time):
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

    # 💡 [수정] 시스템 상태 토픽 발행 (제어 지표 제외)
    node.publish_system_status(
        phase="Ready", 
        err_rate=error_rate, 
        cycle_t=cycle_time
    )

    # 💡 [추가] 제어 성능 데이터 전용 토픽 발행 (DB 저장용)
    node.publish_control_metrics(
        p_gain=p_gain, d_gain=d_gain, max_tilt_step=max_tilt_step, stop_thresh=stop_thresh,
        p_d_ratio=p_d_ratio, overshoot=overshoot, rise_time=rise_t, settling_time=set_t, ss_error=ss_err
    )

    file_path = "pouring_metrics.csv"
    file_exists = os.path.isfile(file_path)
    
    with open(file_path, mode='a', newline='') as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow([
                "Timestamp", "Target_W", "Final_W", "P_GAIN", "D_GAIN", "MAX_TILT_STEP", 
                "STOP_THRESHOLD", "Overshoot(g)", "Rise_Time", "Settling_Time", "SS_Error(g)", 
                "Material", "Error_Rate(%)", "P_D_Ratio", "Avg_Pouring_Rate(g/s)", "Cycle_Time", "Overhead_Time"
            ]) 
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        writer.writerow([
            timestamp, round(target_w, 2), round(final_w, 2), round(p_gain, 3), round(d_gain, 3), 
            round(max_tilt_step, 2), round(stop_thresh, 2), round(overshoot, 2), round(rise_t, 2), 
            round(set_t, 2), round(ss_err, 2), tube_type, round(error_rate, 2), round(p_d_ratio, 2), 
            round(avg_pouring_rate, 2), round(cycle_time, 2), round(overhead_time, 2)
        ])


# ==========================================
# 2. 통신 전담 노드 (서비스 & 토픽)
# ==========================================
class TaskPouring(Node):
    def __init__(self):
        super().__init__("task_pouring", namespace=ROBOT_ID)

        self.callback_group = ReentrantCallbackGroup()
        self.current_weight = 0.0
        self.total_count = 0
        self.success_count = 0

        self.pub_status = self.create_publisher(SystemStatus, "system_status", 10, callback_group=self.callback_group)
        # 💡 [추가] ControlMetrics 발행기 추가
        self.pub_metrics = self.create_publisher(ControlMetrics, "control_metrics", 10, callback_group=self.callback_group)

        self.srv_pouring = self.create_service(
            RobotCommand,
            "execute_pouring",
            self.execute_pouring_callback,
            callback_group=self.callback_group,
        )

        self.sub_weight = self.create_subscription(
            Float32,
            "load_cell/weight",
            self.weight_callback,
            10,
            callback_group=self.callback_group,
        )

        self.sub_stop = self.create_subscription(
            String,
            "stop",
            self.stop_callback,
            10,
            callback_group=self.callback_group,
        )

    # 💡 [수정] SystemStatus에서 제어 지표(p, d, overshoot 등) 관련 파라미터 전부 제거
    def publish_system_status(self, phase="Ready", tcp_vel=0.0, tcp_acc=0.0, pour_speed=0.0, err_rate=0.0, cycle_t=0.0):
        msg = SystemStatus()
        msg.phase = phase
        msg.tcp_vel = float(tcp_vel)
        msg.tcp_acc = float(tcp_acc)
        msg.pour_speed = float(pour_speed)
        
        msg.total_count = self.total_count
        msg.success_count = self.success_count
        msg.error_rate = float(err_rate)
        msg.last_cycle_time = float(cycle_t)
        
        self.pub_status.publish(msg)

    # 💡 [추가] ControlMetrics 전용 발행 함수
    def publish_control_metrics(self, p_gain, d_gain, max_tilt_step, stop_thresh, p_d_ratio, overshoot, rise_time, settling_time, ss_error):
        msg = ControlMetrics()
        msg.p_gain = float(p_gain)
        msg.d_gain = float(d_gain)
        msg.max_tilt_step = float(max_tilt_step)
        msg.stop_threshold = float(stop_thresh)
        msg.p_d_ratio = float(p_d_ratio)
        msg.overshoot = float(overshoot)
        msg.rise_time = float(rise_time)
        msg.settling_time = float(settling_time)
        msg.ss_error = float(ss_error)
        
        self.pub_metrics.publish(msg)

    def stop_callback(self, msg: String):
        global STOP_REQUESTED
        if (msg.data or "").strip().upper() != "STOP":
            return

        STOP_REQUESTED = True
        self.get_logger().warn("[WARN] STOP received -> will return to ready pose then finish")

    def execute_pouring_callback(self, request, response):
        target_w = request.target_weights[0] if request.target_weights else 0.0
        tube_type = request.targets[0].strip().upper() if request.targets else "LARGE" 
        
        self.get_logger().info(f"[Service] Request Received. Tube: {tube_type}, Target: {target_w}g")

        self.total_count += 1
        
        # 💡 [수정] 이동 시작 단계 알림 (p_gain 제거)
        self.publish_system_status(phase="Transfer")

        # 💡 [데드락 방지] 로봇 제어 로직을 별도 스레드에서 실행
        res_container = [False]
        def run_task():
            res_container[0] = perform_task(self, float(target_w), tube_type)
        
        t = threading.Thread(target=run_task)
        t.start()
        t.join() # 작업 스레드가 끝날 때까지 대기

        success = res_container[0]
        if success:
            self.success_count += 1 

        response.success = bool(success)
        response.message = "Pouring Completed" if success else "Pouring Failed"
        return response

    def weight_callback(self, msg: Float32):
        self.current_weight = float(msg.data)

# ==========================================
# 3. 로봇 제어 로직
# ==========================================
def initialize_robot():
    from DSR_ROBOT2 import (set_tool, set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,)
    from DSR_ROBOT2 import add_tcp

    time.sleep(3.0) 
    try:
        print("[Thread] Initializing Robot settings...")
        set_robot_mode(ROBOT_MODE_MANUAL)
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)

        add_tcp(NEW_TCP_NAME, NEW_TCP_OFFSET) 
        set_tcp(NEW_TCP_NAME) 

        add_tcp(FINGER_TCP_NAME, FINGER_TCP_OFFSET) 
        set_tcp(FINGER_TCP_NAME) 

        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
        print(f" [Thread] Robot Initialized: {ROBOT_ID}")
    except Exception as e:
        print(f" [Thread] Init Failed: {e}")

def calculate_tilt_angle_pd(current_w: float, target_w: float, p_gain: float, d_gain: float, max_tilt_step: float): 
    global prev_error
    
    error = target_w - current_w 
    p_term = error * p_gain
    d_term = (error - prev_error) * d_gain
    prev_error = error 
    
    delta_angle = p_term + d_term 
    
    if delta_angle > max_tilt_step: 
        delta_angle = max_tilt_step 
    elif delta_angle < -max_tilt_step: 
        delta_angle = -max_tilt_step

    return float(delta_angle), float(error) 

def perform_task(node: TaskPouring, target_weight: float, tube_type: str = "LARGE") -> bool:
    from DSR_ROBOT2 import movej, get_current_posj, movel, posx, wait, get_current_velx, get_current_velj
    from DSR_ROBOT2 import set_tcp, set_robot_mode, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS 
    
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tcp(ROBOT_TCP)
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(0.5)

    global STOP_REQUESTED
    global prev_error

    if tube_type not in TUBE_TUNING:
        print(f"[ERROR] Invalid tube type: {tube_type}")
        return False

    tuning = TUBE_TUNING[tube_type]
    active_p_gain = tuning["P_GAIN"]
    active_d_gain = tuning["D_GAIN"]
    active_max_tilt_step = tuning["MAX_TILT_STEP"]
    active_stop_thresh = tuning["STOP_THRESHOLD"]

    print(f"[SYSTEM] Task Start! Target: {target_weight}g | Tube: {tube_type} | Threshold: {active_stop_thresh}g")

    start_t = time.time() 
    log_t = [] 
    log_w = [] 
    log_d = [] 

    if tube_type == "LARGE":
        pour_ready_pos = posx(585.440, 157.760, 160.631, 91.920, 97.360, 88.550)
    else: 
        pour_ready_pos = posx(585.440, 144.760, 160.631, 91.920, 97.360, 88.550)

    try:
        movel(pour_ready_pos, vel=100, acc=100)
        wait(1.0)

        set_robot_mode(ROBOT_MODE_MANUAL) 
        set_tcp(FINGER_TCP_NAME) 
        set_robot_mode(ROBOT_MODE_AUTONOMOUS) 
        time.sleep(0.5)

        if tube_type in ["SMALL1", "SMALL2"]:
            print(f"[SYSTEM] Initial 80 deg fast tilt for {tube_type}") 
            init_tilt_pos = posx(0.0, 0.0, 0.0, 0.0, 0.0, 85.0) 
            movel(init_tilt_pos, vel=60, acc=80, ref=1, mod=1) 
            wait(0.5) 

    except Exception as e:
        print(f"[ERROR] Move Failed: {e}")
        return False

    stop_target = target_weight - active_stop_thresh
    prev_error = target_weight - float(node.current_weight)

    prev_tcp_vel = 0.0
    prev_time = time.time()

    while rclpy.ok():
        if STOP_REQUESTED:
            # 💡 [Phase: Return] 중단 시 복귀 단계 알림
            node.publish_system_status(phase="Return")
            try:
                movel(pour_ready_pos, vel=150, acc=150)
                wait(1.0)
            except Exception as e:
                print(f"[ERROR] Return Move Failed after STOP: {e}")
                STOP_REQUESTED = False
                return False

            print(" [STOP] Returned to ready pose. Finishing task.")
            STOP_REQUESTED = False
            return True  

        current_weight = float(node.current_weight)
        current_time = time.time()
        dt = current_time - prev_time
        
        log_t.append(current_time - start_t) 
        log_w.append(current_weight) 

        # ----------------------------------------------------
        # 💡 [변수명 통일] .msg 규격에 맞춘 tcp_vel, tcp_acc, pour_speed 실시간 센서값 추출
        # ----------------------------------------------------
        try:
            # 1. TCP 직교 속도 및 가속도
            velx = get_current_velx() # [vx, vy, vz, rx, ry, rz]
            tcp_vel = math.sqrt(velx[0]**2 + velx[1]**2 + velx[2]**2)
            tcp_acc = (tcp_vel - prev_tcp_vel) / dt if dt > 0 else 0.0
            
            # 2. 붓기 속도 (J6 관절 속도)
            velj = get_current_velj() # [v1, v2, v3, v4, v5, v6]
            pour_speed = abs(velj[5])
        except Exception as e:
            tcp_vel, tcp_acc, pour_speed = 0.0, 0.0, 0.0

        prev_tcp_vel = tcp_vel
        prev_time = current_time

        # 터미널 확인용 로그
        print(f"[API Check] TCP Vel: {tcp_vel:.1f} | Acc: {tcp_acc:.1f} | J6 Vel: {pour_speed:.1f}")

        if current_weight >= stop_target:
            # 💡 [Phase: Return] 목표 도달 시 복귀 단계 알림
            node.publish_system_status(phase="Return")
            try:
                movel(pour_ready_pos, vel=150, acc=150)
                wait(1.0)
            except Exception as e:
                print(f"[ERROR] Return Move Failed: {e}")
                return False
            
            time.sleep(3.0)
            final_settled_weight = float(node.current_weight)
            actual_cycle_time = time.time() - start_t
            
            calc_metrics(node, log_t, log_w, target_weight, final_settled_weight, 
                         active_p_gain, active_d_gain, active_max_tilt_step, active_stop_thresh, 
                         tube_type, actual_cycle_time)            
            return True

        delta, error = calculate_tilt_angle_pd(current_weight, target_weight, active_p_gain, active_d_gain, active_max_tilt_step) 
        log_d.append(delta) 

        # 💡 [수정] 붓기 중 실시간 상태 발행 (p, d 제어값 제거)
        node.publish_system_status(
            phase="Pouring", 
            tcp_vel=tcp_vel, 
            tcp_acc=tcp_acc, 
            pour_speed=pour_speed
        )

        try:
            rel_pos = posx(0.0, 0.0, 0.0, 0.0, 0.0, delta) 
            movel(rel_pos, vel=VELOCITY, acc=ACC, ref=1, mod=1) 
            print(f"Cur: {current_weight:.1f} | Delta: {delta:.2f} | Err: {error:.1f} | Real Vel: {tcp_vel:.1f}")
        except Exception as e:
            print(f"[ERROR] Tilt Move Failed: {e}")
            return False

    return False

# ==========================================
# 4. 메인
# ==========================================
def main(args=None):
    rclpy.init(args=args)

    robot_node = rclpy.create_node("dsr_bridge_hidden", namespace=ROBOT_ID)
    task_node = TaskPouring()

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = robot_node

    # 💡 [수정] 중복된 add_node를 확실하게 제거하고 15개 스레드로 안정적 실행
    executor = MultiThreadedExecutor(num_threads=15)
    executor.add_node(robot_node)
    executor.add_node(task_node)

    init_thread = threading.Thread(target=initialize_robot, daemon=True)
    init_thread.start()

    print("==========================================")
    print(" [Ready] Service Server Started (Multi-Node) ")
    print(f" - service: /{ROBOT_ID}/execute_pouring")
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
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] user_interface.py
[Version] 260227_v36 (7분 30초 풀 시나리오 시뮬레이션)
"""

import rclpy
from rclpy.node import Node
import firebase_admin
from firebase_admin import credentials, db
import time
import datetime
import re  
import math
import random

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, String

try:
    from colab_interfaces.srv import RobotCommand
    from colab_interfaces.msg import SystemStatus
    from colab_interfaces.msg import ControlResult  
    IMPORT_SUCCESS = True
except ImportError:
    IMPORT_SUCCESS = False
    
ROBOT_ID = "dsr01"

class UserInterface(Node):
    def __init__(self):
        super().__init__('user_interface', namespace=ROBOT_ID)
        
        # 💡 시뮬레이션 모드 활성화 
        self.simulation_mode = True 
        
        try:
            cred = credentials.Certificate("/home/rokey/Co-Lab/serviceAccountKey.json")
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://colab1-78afc-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.get_logger().info("🔥 Firebase Connected (7m 30s Sim Mode)!")
            db.reference('commands').set({}) 
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        # 0.1초마다 루프 실행 (화면 렌더링 부드럽게)
        self.timer = self.create_timer(0.1, self.loop_callback)
        self.last_command_timestamp = 0
        
        self.latest_weight = 0.0
        self.latest_system_status = {'phase': 'Ready', 'tcp_vel': 0.0, 'pour_speed': 0.0}
        self.current_target_weight = 220.0
        self.current_material = "Lab Recipe (물 200/에탄올 10/아세톤 10)"
        self.is_cycle_running = False         
        
        self.base_accumulated_weight = 0.0 
        self.last_display_weight = 0.0
        self.final_accumulated_weight = 0.0
        self.sim_start_time = 0
        
        self.sim_error_multipliers = [1.0, 1.0, 1.0] 
        self.phase_durations = [] 

    def loop_callback(self):
        self.check_firebase_commands()
        if self.simulation_mode and self.is_cycle_running:
            self.run_step_simulation()
        self.upload_to_firebase()

    def check_firebase_commands(self):
        try:
            cmd_ref = db.reference('commands')
            cmd_data = cmd_ref.get()
            if cmd_data and cmd_data.get('timestamp', 0) > self.last_command_timestamp:
                self.last_command_timestamp = cmd_data['timestamp']
                
                # 🛑 [긴급 정지] 버튼 처리
                if cmd_data.get('type') == 'emergency_stop':
                    self.get_logger().warn("🚨 웹에서 긴급 중단 명령 수신! 시뮬레이션 강제 종료.")
                    self.is_cycle_running = False
                    self.set_sim_state('Stop', 0.0, 0.0, 0.0)
                
                # ▶️ [작업 시작] 버튼 처리
                elif cmd_data.get('type') == 'start_pouring':
                    self.current_material = cmd_data.get('material', 'Unknown')
                    self.current_target_weight = float(cmd_data.get('target_weight', 0.0))
                    self.start_new_cycle()
        except: pass

    def start_new_cycle(self):
        self.sim_start_time = time.time()
        self.is_cycle_running = True
        self.base_accumulated_weight = 0.0
        self.last_display_weight = 0.0
        
        # 실제와 유사한 1% 내외의 랜덤 오차 생성
        self.sim_error_multipliers = [1.0 + random.uniform(0.005, 0.012) for _ in range(3)]
        
        # 💡 [핵심] 7분 30초 (약 450초) 시나리오 정밀 분배
        self.phase_durations = [
            16.0,  # [0] Transfer 1: 물 시험관 Pick & 이동
            42.0,  # [1] Pouring 1: 물 200g 붓기
            59.0,  # [2] Transfer 2: 물 반납, 에탄올 Pick & 이동
            33.0,  # [3] Pouring 2: 에탄올 10g 정밀 붓기
            57.0,  # [4] Transfer 3: 에탄올 반납, 아세톤 Pick & 이동
            51.0,  # [5] Pouring 3: 아세톤 10g 정밀 붓기
            120.0, # [6] Mixing: 아세톤 반납, 비커 Pick, Mixing 도구 세팅 및 긴 혼합
            56.0   # [7] Return: 믹싱 비커 시험대 안착 및 로봇 원위치
        ]
        
        total_expected = sum(self.phase_durations)
        self.get_logger().info(f"📈 7분 30초 롱텀 사이클 시작 (예상 시간: {total_expected:.1f}s)")

    def run_step_simulation(self):
        t = time.time() - self.sim_start_time
        d = self.phase_durations
        m = self.sim_error_multipliers
        
        # 각 페이즈가 끝나는 누적 시간(Boundary) 계산
        c = [sum(d[:i+1]) for i in range(len(d))]

        if t < c[0]: 
            self.set_sim_state('Transfer', 250.0, 0.0, 0.0)
            
        elif t < c[1]: 
            prog = (t - c[0]) / d[1]
            # y = sqrt(x) 형태로 액체가 초반에 빠르게 차오르고 후반에 섬세해지는 연출
            self.latest_weight = (200 * m[0]) * math.sqrt(prog)
            self.set_sim_state('Pouring', 0.0, self.latest_weight, 15.0 * (1-prog))
            
        elif t < c[2]: 
            self.base_accumulated_weight = 200 * m[0]
            self.latest_weight = 0.0
            self.set_sim_state('Transfer', 200.0, 0.0, 0.0)
            
        elif t < c[3]: 
            prog = (t - c[2]) / d[3]
            self.latest_weight = (10 * m[1]) * math.sqrt(prog)
            self.set_sim_state('Pouring', 0.0, self.latest_weight, 8.0 * (1-prog))
            
        elif t < c[4]: 
            self.base_accumulated_weight = (200 * m[0]) + (10 * m[1])
            self.latest_weight = 0.0
            self.set_sim_state('Transfer', 200.0, 0.0, 0.0)
            
        elif t < c[5]: 
            prog = (t - c[4]) / d[5]
            self.latest_weight = (10 * m[2]) * math.sqrt(prog)
            self.set_sim_state('Pouring', 0.0, self.latest_weight, 5.0 * (1-prog))
            
        elif t < c[6]: 
            if self.latest_system_status['phase'] == 'Pouring':
                # 마지막 붓기가 끝나는 순간 최종 무게를 '피신'시켜 보존
                self.final_accumulated_weight = self.last_display_weight
            
            # 비커를 들어올리는 동작이므로 로드셀 무게는 0이 되어야 함
            self.base_accumulated_weight = 0.0
            self.latest_weight = 0.0
            self.set_sim_state('Mixing', 80.0, 0.0, 0.0)
            
        elif t < c[7]: 
            self.set_sim_state('Return', 150.0, 0.0, 0.0)
            
        else:
            # 시나리오 완벽 종료
            self.is_cycle_running = False
            self.latest_system_status['phase'] = 'Ready'
            self.save_experiment_history(t)
            self.save_simulated_metrics()

    def set_sim_state(self, phase, vel, weight, p_speed):
        self.latest_system_status['phase'] = phase
        self.latest_system_status['tcp_vel'] = vel
        self.latest_weight = weight
        self.latest_system_status['pour_speed'] = round(p_speed, 1)

    def upload_to_firebase(self):
        try:
            # 💡 [프론트엔드 무게 누적 흉내]
            display_w = round(max(self.base_accumulated_weight + self.latest_weight, self.last_display_weight), 2)
            
            # 믹싱이나 복귀 단계에선 비커가 로드셀 위에 없으므로 0으로 덮어씀
            if self.latest_system_status['phase'] in ['Mixing', 'Return']: 
                display_w = 0.0
                self.last_display_weight = 0.0 # 영점 리셋
            else:
                self.last_display_weight = display_w

            db.reference().update({
                'sensor_data/weight': display_w,
                'sensor_data/timestamp': int(time.time() * 1000),
                'system_stats': self.latest_system_status,
                'robot_status/phase': self.latest_system_status['phase']
            })
        except: pass

    # 💡 직관적인 네이밍 룰 (experiment_history)
    def save_experiment_history(self, final_time):
        final_w = round(self.final_accumulated_weight, 2)
        target_w = self.current_target_weight
        err_rate = round(abs(target_w - final_w) / target_w * 100, 2)
        
        history_data = {
            'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'material': self.current_material,
            'target_weight': target_w,
            'final_weight': final_w,
            'error_rate': err_rate,
            'success': True,
            'cycle_time': round(final_time, 1)
        }
        
        now = datetime.datetime.now()
        short_time_str = now.strftime('%y%m%d_%H%M%S') # 260227 형식
        
        match = re.search(r'에탄올(\d+(\.\d+)?)/아세톤(\d+(\.\d+)?)/물(\d+(\.\d+)?)', self.current_material)
        if match:
            ethanol_amt = match.group(1)
            acetone_amt = match.group(3)
            water_amt = match.group(5)
            custom_key = f"{short_time_str}_recipe_water{water_amt}_ethanol{ethanol_amt}_acetone{acetone_amt}"
        else:
            safe_material = str(self.current_material).replace('/', '-').replace(' ', '_').replace('(', '').replace(')', '')
            custom_key = f"{short_time_str}_{safe_material or 'Unknown'}"
            
        db.reference('experiment_history').child(custom_key).set(history_data)
        self.get_logger().info(f"💾 [DB 저장 완료] {custom_key}")

    def save_simulated_metrics(self):
        try:
            targets = [200.0, 10.0, 10.0]
            materials = ["Water", "Ethanol", "Acetone"]
            
            for i in range(3):
                target_w = targets[i]
                final_w = round(target_w * self.sim_error_multipliers[i], 2)
                ss_err = round(abs(target_w - final_w), 2)
                err_rate = round((ss_err / target_w) * 100, 2)
                
                # 물(대용량)과 화학시약(소용량)의 제어 파라미터 차이 반영
                p_gain = 0.015
                d_gain = 0.08 if target_w > 50 else 0.15
                max_tilt = 1.0 if target_w > 50 else 0.2
                stop_th = 12.0 if target_w > 50 else 1.5
                
                pour_duration = self.phase_durations[i*2 + 1]
                
                metrics_data = {
                    'timestamp': int(time.time() * 1000) + i, 
                    'date': datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'pour_speed': 0.0,  
                    'error_rate': err_rate,   
                    'p_gain': p_gain,
                    'd_gain': d_gain,
                    'max_tilt_step': max_tilt,
                    'stop_threshold': stop_th,
                    'p_d_ratio': round(p_gain / d_gain, 2),
                    'overshoot': max(0.0, round(final_w - target_w, 2)),
                    'rise_time': round(pour_duration * 0.9, 2), 
                    'settling_time': round(pour_duration, 2),
                    'ss_error': ss_err,
                    'final_settled_weight': final_w
                }
                
                now = datetime.datetime.now()
                time_str = now.strftime('%Y%m%d_%H%M%S')
                ms = int(time.time() * 100) % 100 + i
                custom_key = f"{time_str}_{ms}_Metrics_{materials[i]}"
                db.reference('control_metrics_history').child(custom_key).set(metrics_data) 
                
        except Exception as e: pass

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()
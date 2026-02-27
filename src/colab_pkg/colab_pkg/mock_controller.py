#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
[Project] CO-LAB
[File] mock_controller.py
[Description] 하드웨어 없이 UI 로직을 테스트하기 위한 완벽한 가상 시뮬레이터 (Count at the end)
"""

import rclpy
from rclpy.node import Node
import time
import threading

from colab_interfaces.srv import RobotCommand
from colab_interfaces.msg import SystemStatus, ControlMetrics
from std_msgs.msg import Float32, String

class MockController(Node):
    def __init__(self):
        super().__init__('mock_controller', namespace='dsr01')
        
        self.status_pub = self.create_publisher(SystemStatus, 'system_status', 10)
        self.weight_pub = self.create_publisher(Float32, 'load_cell/weight', 10)
        self.metrics_pub = self.create_publisher(ControlMetrics, 'log_control_metrics', 10)
        
        self.srv = self.create_service(RobotCommand, 'start_process', self.handle_start_process)
        
        self.create_subscription(String, 'stop/ui', self.stop_cb, 10)
        self.create_subscription(String, 'stop', self.stop_cb, 10)
        
        self.is_running = False
        self.stop_requested = False
        self.current_weight = 0.0
        self.total_count = 0
        self.success_count = 0 # 💡 추가: 성공 횟수도 따로 관리
        
        self.get_logger().info("🤖 [가상 제어기] 실행 완료! UI에서 '작업 시작'을 눌러주세요.")

    def stop_cb(self, msg):
        if msg.data.strip().upper() == "STOP":
            self.stop_requested = True

    def handle_start_process(self, request, response):
        if self.is_running:
            response.success = False
            response.message = "Already running"
            return response
            
        self.get_logger().info(f"▶️ 작업 지시 수신: {request.targets}, 목표 무게들: {request.target_weights}g")
        
        self.is_running = True
        self.stop_requested = False
        # 💡 [삭제됨] 여기서 카운트를 미리 올리지 않습니다!
        
        thread = threading.Thread(target=self.run_simulation, args=(request.targets, request.target_weights))
        thread.start()
        
        response.success = True
        response.message = "Simulation started"
        return response
        
    def pub_status(self, phase):
        msg = SystemStatus()
        msg.phase = phase
        if hasattr(msg, 'total_count'): msg.total_count = self.total_count
        if hasattr(msg, 'success_count'): msg.success_count = self.success_count
        self.status_pub.publish(msg)
        
    def pub_weight(self, w):
        msg = Float32()
        msg.data = float(w)
        self.weight_pub.publish(msg)
        self.current_weight = float(w)

    def pub_metrics(self, speed, error):
        msg = ControlMetrics()
        if hasattr(msg, 'pour_speed'): msg.pour_speed = float(speed)
        if hasattr(msg, 'error_rate'): msg.error_rate = float(error)
        self.metrics_pub.publish(msg)

    def run_simulation(self, targets, weights):
        try:
            self.current_weight = 0.0
            self.pub_weight(0.0)
            
            for i in range(len(targets)):
                if self.stop_requested: break
                
                target_name = targets[i]
                target_w = weights[i]
                
                self.get_logger().info(f"🔄 [{target_name}] 이동 중... (무게 누적 유지)")
                self.pub_status('Transfer')
                for _ in range(20): 
                    if self.stop_requested: break
                    time.sleep(0.1)
                if self.stop_requested: break
                
                self.get_logger().info(f"💧 [{target_name}] {target_w}g 붓기 시작!")
                self.pub_status('Pouring')
                
                start_w = self.current_weight
                steps = 30 
                for step in range(1, steps + 1):
                    if self.stop_requested: break
                    current = start_w + (target_w * (step / steps))
                    self.pub_weight(current)
                    time.sleep(0.1)
                    
                if self.stop_requested: break
                
                self.pub_metrics(speed=5.0, error=1.2)
                self.get_logger().info(f"✅ [{target_name}] 완료. 비커 누적 무게: {self.current_weight:.1f}g")
                time.sleep(1)

            # 🚨 [비상 종료 처리] 카운트는 올리되(1회 수행했으므로) 성공 횟수는 안 올림
            if self.stop_requested:
                self.total_count += 1
                self.get_logger().warn(f"🚨 비상 정지됨! 총 횟수: {self.total_count}")
                self.pub_status('Emergency')
                return

            self.get_logger().info(f"🌀 [Mixing] 비커를 들어올림! (물리적 무게 0.0g)")
            self.pub_status('Mixing')
            self.pub_weight(0.0) 
            for _ in range(30):
                if self.stop_requested: break
                time.sleep(0.1)

            if self.stop_requested:
                self.total_count += 1
                self.pub_status('Emergency')
                return

            self.get_logger().info(f"🔙 [Return] 홈 복귀 중...")
            self.pub_status('Return')
            time.sleep(2)
            
            # 🏁 [정상 완벽 종료] 여기서 최종적으로 카운트와 성공 횟수를 동시에 올림!
            self.total_count += 1
            self.success_count += 1
            self.get_logger().info(f"🏁 사이클 완벽 종료! 총 횟수: {self.total_count}")
            self.pub_status('Ready')

        except Exception as e:
            self.get_logger().error(f"Simulation error: {e}")
        finally:
            self.is_running = False

def main(args=None):
    rclpy.init(args=args)
    node = MockController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
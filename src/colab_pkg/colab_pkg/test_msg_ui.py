#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import time
import random

# 우리가 만든 커스텀 메시지 임포트
from colab_interfaces.msg import SystemStatus, ControlMetrics

class MockUIPublisher(Node):
    def __init__(self):
        super().__init__('mock_ui_publisher')
        
        self.pub_status = self.create_publisher(SystemStatus, '/dsr01/system_status', 10)
        self.pub_metrics = self.create_publisher(ControlMetrics, '/dsr01/log_control_metrics', 10)
        
        # 2초마다 실시간 데이터 발송
        self.timer = self.create_timer(2.0, self.publish_mock_data)
        
        self.tick = 0        # 타이머 반복 횟수
        self.real_count = 0  # 실제 완료된 공정 횟수
        
        self.get_logger().info("🚀 [스마트 테스트 모드] 실제 공정 속도에 맞춰 가짜 데이터를 발사합니다...")

    def publish_mock_data(self):
        msg_status = SystemStatus()
        
        # 1. 상태 변화 (10초 주기로 1사이클 순환)
        phases = ["Ready", "Transfer", "Pouring", "Mixing", "Return"]
        current_phase_index = self.tick % 5
        msg_status.phase = phases[current_phase_index] 
        
        # 2. 실시간 로봇 데이터 (2초마다 무작위로 계속 변함)
        msg_status.tcp_vel = 100.0 + random.uniform(-10.0, 10.0)
        msg_status.tcp_acc = 50.0 + random.uniform(-5.0, 5.0)
        
        if msg_status.phase == "Pouring":
            msg_status.pour_speed = 10.0 + random.uniform(-2.0, 2.0)
        else:
            msg_status.pour_speed = 0.0 # 붓기 상태가 아닐 땐 속도 0
        
        # 3. 공정 완료 감지 및 통계 데이터 업데이트 로직
        # "Return" 단계가 끝나고 다시 "Ready"로 넘어갈 때(사이클 완료 시점) 카운트 증가
        if current_phase_index == 0 and self.tick > 0:
            self.real_count += 1
            self.publish_metrics_data() # 공정이 끝났을 때만 제어 지표를 한 번 쏨!
            
        msg_status.total_count = self.real_count
        msg_status.success_count = self.real_count
        msg_status.error_rate = random.uniform(0.5, 3.0)
        msg_status.last_cycle_time = 45.0 + random.uniform(-2.0, 2.0)
        
        self.pub_status.publish(msg_status)
        self.get_logger().info(f"⏱️ Phase: {msg_status.phase} | TCP Vel: {msg_status.tcp_vel:.1f} | TCP Acc: {msg_status.tcp_acc:.1f}")
        
        self.tick += 1

    def publish_metrics_data(self):
        # 공정(사이클)이 한 번 끝날 때마다 호출되는 함수
        msg_metrics = ControlMetrics()
        
        if self.real_count % 2 == 0:
            msg_metrics.max_tilt_step = 1.0  # 자갈 세팅
            msg_metrics.stop_threshold = 12.0
        else:
            msg_metrics.max_tilt_step = 0.2  # 비즈 세팅
            msg_metrics.stop_threshold = 1.5

        msg_metrics.p_gain = 0.015
        msg_metrics.d_gain = 0.08
        msg_metrics.p_d_ratio = 0.18 # [추가] 누락되었던 p_d_ratio 추가
        msg_metrics.overshoot = random.uniform(0.0, 2.0)
        msg_metrics.rise_time = 5.0
        msg_metrics.settling_time = 6.5
        msg_metrics.ss_error = random.uniform(0.0, 1.5)
        
        self.pub_metrics.publish(msg_metrics)
        self.get_logger().info(f"📊 [{self.real_count}회차 완료] 제어 지표(ControlMetrics) DB 전송 완료!")

def main(args=None):
    rclpy.init(args=args)
    node = MockUIPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 테스트 퍼블리셔를 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
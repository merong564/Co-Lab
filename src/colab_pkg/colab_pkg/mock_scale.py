#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from colab_interfaces.msg import SystemStatus

class MockScale(Node):
    def __init__(self):
        super().__init__('mock_scale', namespace='dsr01')
        self.pub = self.create_publisher(Float32, 'load_cell/weight', 10)
        self.sub = self.create_subscription(SystemStatus, 'system_status', self.status_cb, 10)
        self.timer = self.create_timer(0.1, self.timer_cb)
        
        self.current_weight = 0.0
        self.is_pouring = False

    def status_cb(self, msg):
        current_phase = msg.phase.lower()
        self.is_pouring = (current_phase == 'pouring')
        
        # 믹싱하러 비커를 들어올리면 하드웨어적으로 무게가 0이 되는 것을 모사
        if current_phase in ['mixing', 'return']:
            self.current_weight = 0.0

    def timer_cb(self):
        # 붓는 중일 때만 0.1초당 0.5g씩 (초당 5g) 무게가 차오름
        if self.is_pouring:
            self.current_weight += 0.5 
            
        msg = Float32()
        msg.data = self.current_weight
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = MockScale()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
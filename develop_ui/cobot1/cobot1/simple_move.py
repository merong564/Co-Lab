import rclpy
import DR_init
import sys

def main(args=None):
    
    #모든 작업을 시작하기 전에 로봇의 ID와 모델명을 지정
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
		
		#ROS 2와 네트워크를 통해 통신할 수 있게 설정 
    rclpy.init(args=args)
    node = rclpy.create_node('example_py', namespace=ROBOT_ID)
    DR_init.__dsr__node = node


		#로봇 제어를 위한 핵심 기능 구현
		#DSR_ROBOT2가져오기는 ROS 2 노드( DR_init.__dsr__node)를 초기화한 후 사용해야 함.
    from DSR_ROBOT2 import movej, posj, set_robot_mode, ROBOT_MODE_AUTONOMOUS

		#로봇의 작동 모드 및 속도 설정
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

		#위치 정의
    target_pos = posj(0, 0, 90.0, 0, 90.0, 0)

    movej(target_pos, vel=100, acc=100)

    print("Example complete")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
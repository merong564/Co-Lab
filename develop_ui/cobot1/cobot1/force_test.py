import time
import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
VELOCITY = 60
ACC = 60

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool, set_tcp,get_tool,get_tcp,ROBOT_MODE_MANUAL,ROBOT_MODE_AUTONOMOUS  # 필요한 기능만 임포트
    from DSR_ROBOT2 import get_robot_mode,set_robot_mode

    # Tool과 TCP 설정시 매뉴얼 모드로 변경해서 진행
    set_robot_mode(ROBOT_MODE_MANUAL)
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    time.sleep(2)  # 설정 안정화를 위해 잠시 대기
    # 설정된 상수 출력
    print("#" * 50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {get_tcp()}") 
    print(f"ROBOT_TOOL: {get_tool()}")
    print(f"ROBOT_MODE 0:수동, 1:자동 : {get_robot_mode()}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#" * 50)


def perform_task():
    """로봇이 수행할 작업"""
    print("Performing force control task...")
    from DSR_ROBOT2 import (
        release_compliance_ctrl,release_force,
        check_force_condition,
        task_compliance_ctrl,
        set_desired_force,
        set_ref_coord,
        movej,
        movel,wait,
        DR_FC_MOD_REL,
        DR_AXIS_Z,
        DR_BASE,
        posx
    )

    # 초기 위치 및 목표 위치 설정
    JReady = [0, 0, 90, 0, 90, 0]
    pos = posx([500, 0, 200, 90, 180, 90])
    
    # 초기 위치로 이동
    print("초기 위치로 이동...")
    movej(JReady, vel=VELOCITY, acc=ACC)

    # 힘 제어 시작
    print("힘 제어 시작...")

    set_ref_coord(1) # Tool 좌표계 설정
    task_compliance_ctrl(stx=[1000, 1000, 200, 200, 200, 200])
    wait(0.5) # 안정화 대기(필수)
    set_desired_force(fd=[0, 0, 15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)

    
    # # 힘 조건 확인
    while True:
        ret = check_force_condition(DR_AXIS_Z, min=0, max=15)
        print("Z축 힘이 15N 이상이 될 때까지 대기...", ret)
        if ret == -1:
            print("Z축 힘이 15N 이상 감지됨.",ret)
            break
        wait(0.5)

    # 힘 제어 해제
    print("힘 제어 해제...")
    release_force()
    release_compliance_ctrl()


def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("force_test", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()

        # 작업 수행 (한 번만 호출)
        perform_task()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
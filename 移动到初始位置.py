import rtde_control
import rtde_receive
import time
import numpy as np

# 初始化RTDE接口
robot = rtde_control.RTDEControlInterface("192.168.1.100")
info = rtde_receive.RTDEReceiveInterface("192.168.1.100")

velocity = 0.05
acceleration = 0.1
dt = 1.0 / 25 #设置控制循环的时间间隔
lookahead_time = 0.1 #设置路径跟踪中的前瞻时间，用于平滑路径
gain = 100 #设置控制增益，用于调节控制系统的响应


SAFE_SPACE = {
    'x': (0.305, 0.672),
    'y': (-0.226, 0.3),
    'z': (0.08, 0.3),
}


def is_target_position_safe(target_position):
    x, y, z = target_position
    return (SAFE_SPACE['x'][0] <= x <= SAFE_SPACE['x'][1] and
            SAFE_SPACE['y'][0] <= y <= SAFE_SPACE['y'][1] and
            SAFE_SPACE['z'][0] <= z <= SAFE_SPACE['z'][1])

# 初始位置定义
initial_position =  [0.5014534966766954, 0.13052814932282858, 0.13362233136541152, 1.2140002809099426, 1.2101572264570504, 1.215203142219635]
Target_TCP = info.getActualTCPPose() 

# 函数：将机械臂移动到初始位置
def move_to_initial_position(robot, initial_position, velocity, acceleration):

    print("Moving to initial position...")
    robot.moveL(initial_position, velocity, acceleration)        
    print("Reached initial position.")

# 获取当前TCP位置（可以用于调试和确认位置）
def get_current_tcp_position(info):
    current_TCP = info.getActualTCPPose()
    print(f"Current TCP position: {current_TCP}")
    return current_TCP

if __name__ == '__main__':
    # 移动机械臂到初始位置
    move_to_initial_position(robot, initial_position, velocity, acceleration)

    # 检查当前TCP位置
    current_TCP = get_current_tcp_position(info)



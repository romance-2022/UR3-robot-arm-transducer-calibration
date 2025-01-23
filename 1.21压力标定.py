import rtde_control
import rtde_receive
import time
import numpy as np

# 初始化RTDE接口
robot = rtde_control.RTDEControlInterface("192.168.1.100")
info = rtde_receive.RTDEReceiveInterface("192.168.1.100")

velocity = 0.1
acceleration = 0.1
dt = 1.0 / 25 #设置控制循环的时间间隔
lookahead_time = 0.1 #设置路径跟踪中的前瞻时间，用于平滑路径
gain = 100 #设置控制增益，用于调节控制系统的响应

motion_state = "PLANE"



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


def move_check(Target_TCP, velocity, acceleration):
    # 检查目标位置是否在安全范围内
    if is_target_position_safe(Target_TCP[:3]):
        robot.moveL(np.array(Target_TCP), velocity, acceleration)
    else:
        print("Target position out of safe range!")

# 函数：将机械臂移动到初始位置
def move_to_initial_position(robot, initial_position, velocity, acceleration):

    print("Moving to initial position...")
    robot.moveL(initial_position, velocity, acceleration)        
    print("Reached initial position.")


def task(Target_TCP_init):
    # 点之间的距离
    dx = 0.006  # 6mm in meters
    dy = 0.006  # 6mm in meters
    # 按压深度
    depths = [0.001, 0.002, 0.003, 0.004]  # 1mm, 2mm, 3mm, 4mm in meters
    # 停留时间
    wait_time1 = 0.5  # 
    wait_time2 = 1.5  # 

    # 5x5 array
    for i in range(5):
        for j in range(5):
            # 计算当前点的位置
            x = Target_TCP_init[0] + i * dx
            y = Target_TCP_init[1] + j * dy
            z = Target_TCP_init[2]
            Target_TCP = [x, y, z +0.005, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
            move_check(Target_TCP, velocity, acceleration)
            for depth in depths:
                # 移动到按压深度
                press_TCP = [x, y, z - depth, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
                move_check(press_TCP, velocity, acceleration)
                # 停留一段时间
                time.sleep(wait_time1)

                # 回到初始 z 位置
                lift_TCP = [x, y, z, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
                move_check(lift_TCP, velocity, acceleration)
                time.sleep(wait_time2)

if __name__ == "__main__":

    # 初始位置，使用时替换为具体数值
    Target_TCP_init = info.getActualTCPPose()

    # 移动机械臂到初始位置
    move_to_initial_position(robot, Target_TCP_init, velocity, acceleration) 

    task(Target_TCP_init)
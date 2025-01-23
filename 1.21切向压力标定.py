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

def main(Target_TCP_init):
    # 点之间的距离
    dx = 0.006  # 6mm in meters
    dy = 0.006  # 6mm in meters
    # 按压深度
    press_depth = 0.002  # 2mm in meters
    # 切向移动距离
    tangential_distances = [0.0005, 0.001, 0.0015]  # 0.5mm, 1mm, 1.5mm in meters
    # 停留时间
    wait_time1 = 0.5  # 2.5 seconds
    wait_time2 = 1.5  # 2.5 seconds

    # 选择 3x3 阵列的起始索引
    start_i = 1  # 从 5x5 阵列的第二行开始
    start_j = 1  # 从 5x5 阵列的第二列开始
    end_i = 3  # 到第四行结束
    end_j = 3  # 到第四列结束

    # 3x3 array
    for i in range(start_i, end_i + 1):
        for j in range(start_j, end_j + 1):
            # 计算当前点的位置
            x = Target_TCP_init[0] + i * dx
            y = Target_TCP_init[1] + j * dy
            z = Target_TCP_init[2]
            Target_TCP = [x, y, z+0.008, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
            move_check(Target_TCP, velocity, acceleration)
            # 垂直按压
            press_TCP = [x, y, z - press_depth, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
            move_check(press_TCP, velocity, acceleration)
            for dist in tangential_distances:
                # 向 x 轴方向切向移动
                tangential_TCP = [x + dist, y, z - press_depth, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
                move_check(tangential_TCP, velocity, acceleration)
                # 停留一段时间
                time.sleep(wait_time1)
                # 回到按压位置
                press_TCP = [x, y, z - press_depth, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
                move_check(press_TCP, velocity, acceleration)
                time.sleep(wait_time2)                
                
            # 抬起
            lift_TCP = [x, y, z, Target_TCP_init[3], Target_TCP_init[4], Target_TCP_init[5]]
            move_check(lift_TCP, velocity, acceleration)


if __name__ == "__main__":
    # 初始位置，使用时替换为具体数值
    Target_TCP_init = info.getActualTCPPose()

    # 移动机械臂到初始位置
    move_to_initial_position(robot, Target_TCP_init, velocity, acceleration) 

    main(Target_TCP_init)
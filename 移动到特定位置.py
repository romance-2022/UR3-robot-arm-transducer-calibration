import rtde_control
import rtde_receive
import time
import numpy as np


velocity = 0.05
acceleration = 0.1
dt = 1.0 / 25 #设置控制循环的时间间隔
lookahead_time = 0.1 #设置路径跟踪中的前瞻时间，用于平滑路径
gain = 100 #设置控制增益，用于调节控制系统的响应

motion_state = "PLANE"
Target_TCP = [0, 0, 0, 0, 0, 0]
Ordinary_speed = 0.1
Ordinary_angular_speed = 0.1

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


# 初始化RTDE接口
robot = rtde_control.RTDEControlInterface("192.168.1.100")
info = rtde_receive.RTDEReceiveInterface("192.168.1.100")


def move_check(Target_TCP,velocity, acceleration):
    
        # 检查目标位置是否在安全范围内
    if is_target_position_safe(Target_TCP[:3]):
        robot.moveL(np.array(Target_TCP), velocity, acceleration)
    else:
        print("Target position out of safe range!")


# 给定的类别标签字典
label_value = {
    "1*1 从左往右": 0,


}

value_label = {v: l for l, v in label_value.items()}

class Yjq_sequence():
    def __init__(self):
        self.manipulate_sequenve = [0]


Ordinary_speed = Ordinary_speed
Ordinary_angular = Ordinary_angular_speed

translation = 1  # 将移动距离设为1个单位，速度已经设定为0.02m/s

interval = 3  # 保持不变

if __name__ == '__main__':
    yjq_sequence = Yjq_sequence()

   
    Target_TCP_init = [0.5014534966766954, 0.13052814932282858, 0.13362233136541152, 1.2140002809099426, 1.2101572264570504, 1.215203142219635]    
    robot.moveL(np.array(Target_TCP_init), velocity, acceleration)    

    Target_TCP = info.getActualTCPPose()  # 更新当前位置为初始位置
    # while True:
    #     temp_Target = info.getActualTCPPose()
    #     print(temp_Target)
    #     print("x:{}, y:{}, z:{}".format(temp_Target[0], temp_Target[1], temp_Target[2]))
    #     time.sleep(5)
    # robot.moveL([0.20090365290280396, -0.15511181621869916, -0.14853890623323837, 2.2726801144798316, 2.163293291575628, -0.014349955146889231], velocity, acceleration)
    for idx, num in enumerate(yjq_sequence.manipulate_sequenve):

        if num == label_value["1*1 从左往右"]: # Left2Right
            Target_TCP[2] = 0.11170911554195352
            move_check(Target_TCP,velocity, acceleration)  


        print("COMMAND: {}".format(value_label[num]))

        print("running")

        time.sleep(interval)

    robot.moveL(np.array(Target_TCP_init), velocity, acceleration)
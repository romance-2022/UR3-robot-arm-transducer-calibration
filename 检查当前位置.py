import rtde_control
import rtde_receive
import time
import numpy as np

# 初始化RTDE接口
robot = rtde_control.RTDEControlInterface("192.168.1.100")
info = rtde_receive.RTDEReceiveInterface("192.168.1.100")

# 运动参数
velocity = 0.2  # 移动速度
acceleration = 1.0  # 加速度
dt = 1.0 / 25  # 控制循环时间间隔
lookahead_time = 0.1  # 路径跟踪前瞻时间
gain = 100  # 控制增益

# 安全范围定义
SAFE_SPACE = {
    'x': (0.205, 0.399),
    'y': (-0.226, 0.029),
    'z': (-0.163, 0.058),
}

def is_target_position_safe(target_position):
    x, y, z = target_position
    return (SAFE_SPACE['x'][0] <= x <= SAFE_SPACE['x'][1] and
            SAFE_SPACE['y'][0] <= y <= SAFE_SPACE['y'][1] and
            SAFE_SPACE['z'][0] <= z <= SAFE_SPACE['z'][1])


# 获取当前TCP位置（可以用于调试和确认位置）
def get_current_tcp_position(info):
    current_TCP = info.getActualTCPPose()
    print(f"Current TCP position: {current_TCP}")
    return current_TCP

if __name__ == '__main__':

    # 检查当前TCP位置
    current_TCP = get_current_tcp_position(info)



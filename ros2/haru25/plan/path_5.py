import planning_casadi as pl
from planning_casadi import Wall
import os
import numpy as np

# 始点 (x, y, yaw)
start = [3.112, 0.29, 0.0]

# 終点
# end = [2.937, 6.501, 0.0]
end = [0.980, 6.384, -np.pi/2]

# 最大加速度
max_a = 2.0

# 最大並進速度
max_vx = 1.0

# 最大回転速度
max_vt = 0.1

# 位置 x最小最大/y最小最大
pos_range = [[None, None], [None, None]]

# 角度 最小/最大
angle_range = [-np.pi, np.pi]

# 到達予想時間　最小/最大
time_range = [2.0, 8.5]

# 考慮する壁
walls = []
# 横壁
# 上と下の壁
walls.append(Wall(0.0, 4.0, 6.8, 6.8))
# walls.append(Wall(0.0, 4.0, 0.0, 0.0))
# 左から生えてる壁
walls.append(Wall(0.588, 2.5, 5.7, 5.7))
# walls.append(Wall(0.588, 1.6, 2.5, 2.5))

# 右から生えてる壁
# walls.append(Wall(1.5, 3.5, 5.5, 5.5))
# walls.append(Wall(1.5, 3.5, 3.5, 3.5))
# walls.append(Wall(1.5, 3.5, 1.5, 1.5))

# 縦壁
# walls.append(Wall(3.27, 3.27, 4, 7))
# walls.append(Wall(0.588, 0.588, 1.4, 5.5))
walls.append(Wall(2.564, 2.564, 1.4, 5.5))

# ロボットの大きさ
robot_edge = 0.62

# 半径
wheel_base = 0.25

# lidar
lidar_x = -0.207
lidar_y = -0.207
lidar_radius = 0.

# 経路点の数
N = 100

# 前回の初期値
pre_path = os.path.join(os.path.dirname(__file__), "output", "X5.npz")

pl.planning(start, end, max_a, max_vx, max_vt, pos_range, angle_range, time_range, walls, robot_edge, wheel_base, N, pre_path, lidar_x, lidar_y, lidar_radius)

pl.plot(pre_path)
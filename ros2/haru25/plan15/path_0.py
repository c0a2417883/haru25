import planning_casadi as pl
from planning_casadi import Wall
import os
import numpy as np

# 始点 (x, y, yaw)
start = [3.112, 0.29, 0.0]

# 終点 6.501→6.03
end = [2.937, 6.45, 0.0]

# 最大加速度
max_a = 2.0

# 最大並進速度
max_vx = 1.5

# 最大回転速度
max_vt = 0.2

# 位置 x最小最大/y最小最大
pos_range = [[None, None], [None, None]]

# 角度 最小/最大 -np.pi/2, np.pi/2
angle_range = [0,0]

# 到達予想時間　最小/最大
time_range = [2.0, 5.8]

# 考慮する壁
walls = []
# 横壁

# 縦壁
walls.append(Wall(3.27, 3.27, 4, 7))
# walls.append(Wall(2.214, 2.214, 0, 2.576))

# ロボットの大きさ
robot_edge = 0.65

# 半径
wheel_base = 0.25

# lidar
lidar_x = -0.207
lidar_y = -0.207
lidar_radius = 0.

# 経路点の数
N = 100

# 前回の初期値
pre_path = os.path.join(os.path.dirname(__file__), "output", "X0.npz")

pl.planning(start, end, max_a, max_vx, max_vt, pos_range, angle_range, time_range, walls, robot_edge, wheel_base, N, pre_path, lidar_x, lidar_y, lidar_radius)

pl.plot(pre_path)
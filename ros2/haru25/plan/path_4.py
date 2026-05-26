import planning_casadi as pl
from planning_casadi import Wall
import os
import numpy as np

# 始点 (x, y, yaw)
start = [0.98, 6.38, -np.pi/2]

# 終点
end = [0.938, 0.4, -np.pi/2]

# 最大加速度
max_a = 2.0

# 最大並進速度
max_vx = 1.0

# 最大回転速度
max_vt = 0.0

# 位置 x最小最大/y最小最大
pos_range = [[None, None], [None, None]]

# 角度 最小/最大
angle_range = [-np.pi, np.pi]

# 到達予想時間　最小/最大
time_range = [5.0, 9.5]

# 考慮する壁
walls = []
# 横壁
# 上と下の壁
walls.append(Wall(0.0, 4.0, 6.8, 6.8))
walls.append(Wall(0.0, 4.0, 0.0, 0.0))
# 左から生えてる壁
walls.append(Wall(0.588, 1.6, 4.5, 4.5))
walls.append(Wall(0.588, 1.6, 2.5, 2.5))

# 右から生えてる壁
walls.append(Wall(1.5, 3.5, 5.5, 5.5))
walls.append(Wall(1.5, 3.5, 3.5, 3.5))
walls.append(Wall(1.5, 3.5, 1.6, 1.6))

# 縦壁
walls.append(Wall(0.588, 0.588, 1.4, 5.5))
walls.append(Wall(2.564, 2.564, 1.4, 5.5))

# ロボットの大きさ
robot_edge = 0.6

# 半径
wheel_base = 0.25

# lidar
lidar_x = 0.207
lidar_y = -0.207
lidar_radius = 0.2

# 経路点の数
N = 100

# ヨーを動かないようにする
yaw_lock = True

# 前回の初期値
pre_path = os.path.join(os.path.dirname(__file__), "output", "X4.npz")

pl.planning(start, end, max_a, max_vx, max_vt, pos_range, angle_range, time_range, walls, robot_edge, wheel_base, N, pre_path, lidar_x, lidar_y, lidar_radius, yaw_lock)

pl.plot(pre_path)
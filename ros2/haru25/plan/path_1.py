import planning_casadi as pl
from planning_casadi import Wall
import os
import numpy as np

# 始点 (x, y, yaw)　6.501→6.581 0.08m上にいくはず
start = [2.937, 6.501, 0.0]

# 終点
# end = [2.937, 6.501, -np.pi/6] 
end = [2.782, 6.224, -0.385238907352447]

# 最大加速度
max_a = 2.0

# 最大並進速度
max_vx = 1.0

# 最大回転速度
max_vt = 0.2

# 位置 x最小最大/y最小最大
pos_range = [[0.038, 3.5], [None, None]]

# 角度 最小/最大
angle_range = [-np.pi/2, np.pi/2]

# 到達予想時間　最小/最大
time_range = [0.0, 3.6]

# 考慮する壁
walls = []
# 横壁
#walls.append(Wall(0.588, 4.0, 5.5, 5.5))
# walls.append(Wall(0.588, 4.0, 5.5, 5.5))
# walls.append(Wall(0.0, 4.0, 6.924, 6.924))

# 縦壁
# walls.append(Wall(0.038, 0.038, 0.0, 7.0))
# walls.append(Wall(2.564, 2.564, 1.462, 5.5))

# ロボットの大きさ
robot_edge = 0.5

# 半径
wheel_base = 0.25

# lidar
lidar_x = -0.207
lidar_y = -0.207
lidar_radius = 0.

# 経路点の数
N = 100

# 前回の初期値
pre_path = os.path.join(os.path.dirname(__file__), "output", "X1.npz")

pl.planning(start, end, max_a, max_vx, max_vt, pos_range, angle_range, time_range, walls, robot_edge, wheel_base, N, pre_path, lidar_x, lidar_y, lidar_radius)

pl.plot(pre_path)
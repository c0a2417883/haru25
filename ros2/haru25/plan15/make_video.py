import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import matplotlib.patches as patches
import math
import os

# 経路の番号
PATH_NUM = 5

# データ読み込み
PRE_PATH = os.path.join(os.path.dirname(__file__), "output", f'X{PATH_NUM}.npz')

pre_data = np.load(PRE_PATH)
X = pre_data['X']
U = pre_data['U']
T = pre_data['T']

OUTPUT_PATH = os.path.join(os.path.dirname(__file__), "video", f'X{PATH_NUM}.mp4')

x = X[0,:]
y = X[1,:]
t = X[2,:]
vx = X[3,:]
vy = X[4,:]
vt = X[5,:]

ax = U[0,:]
ay = U[1,:]
at = U[2,:]

# フィールドサイズ
field_x = 3.5
filed_y = 6.962

# robot
robot = 0.4
robot_left_down = np.array([-robot/2, -robot/2])
if PATH_NUM != 4:
    lidar_xy = np.array([-0.207, -0.207])
else:
    lidar_xy = np.array([0.207, -0.207])
lidar_radius = 0.2

# 図の大きさ
fig = plt.figure(figsize=(6, 6))
# 枠を追加するため，軸を追加
ax1 = fig.add_subplot()
# 枠の大きさを指定　[枠の左辺, 枠の下辺, 枠の横幅, 枠の高さ]
ax1.set_position([0.1, 0.05, field_x/8, filed_y/8])

# 描画範囲を指定
ax1.set_xlim((0, field_x))
ax1.set_ylim((0, filed_y))

def plot_map():
    # 横線 (板の中心のy, xmin, xmax)
    ax1.hlines(-0.019, 0.038, 3.462, color='k')
    ax1.hlines(6.943, 0.038, 3.462, color='k')
    ax1.hlines(1.481, 1.526, 2.526, color='k')
    ax1.hlines(2.481, 0.626, 1.526, color='k')
    ax1.hlines(2.981, 0.038, 0.588, color='k')
    ax1.hlines(3.481, 1.526, 2.526, color='k')
    ax1.hlines(4.481, 0.626, 1.526, color='k')
    ax1.hlines(5.481, 1.526, 2.526, color='k')
    ax1.hlines(3.481, 1.526, 2.526, color='k')

    # 縦線 (板の中心のx, ymin, ymax)
    ax1.vlines(0.019, -0.038, 6.962, color='k')
    ax1.vlines(3.481, -0.038, 6.962, color='k')
    ax1.vlines(0.607, 1.462, 5.5, color='k')
    ax1.vlines(2.545, 1.462, 5.5, color='k')

frames = len(ax)
fps = int(frames / T)
metadata = dict(title='', artist='', comment='')
writer = FFMpegWriter(fps=fps, metadata=metadata)

ax1.plot(X[:, 0])

with writer.saving(fig, OUTPUT_PATH, frames):
    for i in range(len(ax)):
        ax1.cla()
        # for stopping simulation with the esc key.
        ax1.plot(x[i], y[i])
        #ax1.axis("equal")
        # 横線 (板の中心のy, xmin, xmax)
        ax1.hlines(-0.019, 0.038, 3.462, color='k')
        ax1.hlines(6.943, 0.038, 3.462, color='k')
        ax1.hlines(1.481, 1.526, 2.526, color='k')
        ax1.hlines(2.481, 0.626, 1.526, color='k')
        ax1.hlines(2.981, 0.038, 0.588, color='k')
        ax1.hlines(3.481, 1.526, 2.526, color='k')
        ax1.hlines(4.481, 0.626, 1.526, color='k')
        ax1.hlines(5.481, 1.526, 2.526, color='k')
        ax1.hlines(3.481, 1.526, 2.526, color='k')

        # 縦線 (板の中心のx, ymin, ymax)
        ax1.vlines(0.019, -0.038, 6.962, color='k')
        ax1.vlines(3.481, -0.038, 6.962, color='k')
        ax1.vlines(0.607, 1.462, 5.5, color='k')
        ax1.vlines(2.545, 1.462, 5.5, color='k')
        
        #経路 
        ax1.plot(x, y)

        # ロボット
        c = math.cos(t[i])
        s = math.sin(t[i])
        rotate = np.array([[c, -s], [s, c]])
        left_down = rotate@robot_left_down
        lidar = rotate@lidar_xy
        ax1.add_patch(patches.Rectangle(xy=(x[i]+left_down[0], y[i]+left_down[1]), width=robot, height=robot, fill=False, facecolor='none', edgecolor='k', angle=np.rad2deg(t[i])))
        ax1.add_patch(patches.Circle(xy = (x[i]+lidar[0], y[i]+lidar[1]), radius=lidar_radius, fill=False, facecolor='none', edgecolor='k'))
        # 加速度
        ax1.plot([x[i], x[i] + ax[i]*0.2], [y[i], y[i] + ay[i]*0.2], color="red")
        fig.suptitle(f'time: {T:.3}s')
        writer.grab_frame()
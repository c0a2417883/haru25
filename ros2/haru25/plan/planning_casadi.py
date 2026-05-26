from casadi import *
import matplotlib.pyplot as plt
import numpy as np

# 壁の情報
# x1, y1(始点) -> x2, y2(終点)
class Wall:
    x1 = 0
    x2 = 0
    y1 = 0
    y2 = 0
    '''
    x1: 始点 x座標
    x2: 終点 x座標
    y1: 始点 y座標
    y2: 終点 y座標
    '''
    def __init__(self, x1, x2, y1, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2

# マップを表示
def plotMap(ax):
    # 横線 (板の中心のy, xmin, xmax)
    ax.hlines(-0.019, 0.038, 3.462, color='k')
    ax.hlines(6.943, 0.038, 3.462, color='k')
    ax.hlines(1.481, 1.526, 2.526, color='k')
    ax.hlines(2.481, 0.626, 1.526, color='k')
    ax.hlines(2.981, 0.038, 0.588, color='k')
    ax.hlines(3.481, 1.526, 2.526, color='k')
    ax.hlines(4.481, 0.626, 1.526, color='k')
    ax.hlines(5.481, 1.526, 2.526, color='k')
    ax.hlines(3.481, 1.526, 2.526, color='k')

    # 縦線 (板の中心のx, ymin, ymax)
    ax.vlines(0.019, -0.038, 6.962, color='k')
    ax.vlines(3.481, -0.038, 6.962, color='k')
    ax.vlines(0.607, 1.462, 5.5, color='k')
    ax.vlines(2.545, 1.462, 5.5, color='k')

'''
start: 始点
end: 終点
max_v: 最大速度
max_a: 最大加速度
robot_edge: ロボットの大きさ
wheel_base: ロボットの中心からタイヤまでの距離
wall: 考慮する壁
N: 経路点数
pre_path: 前の計算結果
'''
def planning(start:list, end:list, max_a:float, max_vx:float, max_vt:float, pos_range:list, angle_range:list, time_range:list, walls:list, robot_edge:float, wheel_base:float, N:int, pre_path:str, lidar_x:float, lidar_y:float,lidar_radius:float, yaw_lock=False):
    opti = Opti()

    # 状態量 x, y, t, vx, vy, omega
    X = opti.variable(6,N+1)
    x = X[0,:]
    y = X[1,:]
    t = X[2, :]
    vx = X[3, :]
    vy = X[4, :]
    vt = X[5, :]

    # 入力
    U = opti.variable(3,N)
    ax = U[0, :]
    ay = U[1, :]
    at = U[2, :]

    # 終端時間
    T = opti.variable()

    # 時間を最小化する
    opti.minimize(T)

    # 微小時間
    dt = T/N

    # 移動による拘束条件
    for k in range(N):
        opti.subject_to(x[k+1]==x[k] + dt*vx[k])
        opti.subject_to(y[k+1]==y[k] + dt*vy[k])
        opti.subject_to(t[k+1]==t[k] + dt/wheel_base*vt[k])
        opti.subject_to(vx[k+1]==vx[k] + dt*ax[k])
        opti.subject_to(vy[k+1]==vy[k] + dt*ay[k])
        opti.subject_to(vt[k+1]==vt[k] + dt*at[k])

    # ロボットの大きさ
    robot_2 = robot_edge / 2
    robot2 = (robot_edge / 2) **2
    robot_edges = np.array([[robot_2, robot_2], [-robot_2, robot_2], [-robot_2, -robot_2], [robot_2, -robot_2]])

    # 障害物
    c = cos(t)
    s = sin(t)

    robot_edge_1 = [x + c * robot_edges[0, 0] - s * robot_edges[0, 1], y + s * robot_edges[0, 0] + c * robot_edges[0, 1]]
    robot_edge_2 = [x + c * robot_edges[1, 0] - s * robot_edges[1, 1], y + s * robot_edges[1, 0] + c * robot_edges[1, 1]]
    robot_edge_3 = [x + c * robot_edges[2, 0] - s * robot_edges[2, 1], y + s * robot_edges[2, 0] + c * robot_edges[2, 1]] 
    robot_edge_4 = [x + c * robot_edges[3, 0] - s * robot_edges[3, 1], y + s * robot_edges[3, 0] + c * robot_edges[3, 1]]
    lidar_xy = [x + c * lidar_x - s * lidar_y, y + s * lidar_x + c * lidar_y]

    def hlines(y, x_min, x_max, edge1, edge2):
        a1 = edge1[0] - edge2[0]
        b1 = edge1[1] - edge2[1]
        s1 = a1 * (y-edge1[1]) - b1*(x_min-edge1[0])
        t1 = a1 * (y-edge1[1]) - b1*(x_max-edge1[0])
        s2 = edge1[1] - y
        t2 = edge2[1] - y
        opti.subject_to(fmax(s1*t1, s2*t2) > 0)
        if lidar_radius > 0:
            dx1 = (lidar_xy[0] - x_min)
            dx2 = (lidar_xy[0] - x_max)
            dy1 = (lidar_xy[1] - y)
            r2 = lidar_radius*lidar_radius - dy1*dy1
            opti.subject_to(dx1*dx1 > r2)
            opti.subject_to(dx2*dx2 > r2)
        
    def vlines(x, y_min, y_max, edge1, edge2):
        a1 = edge1[0] - edge2[0]
        b1 = edge1[1] - edge2[1]
        s1 = a1 * (y_min-edge1[1]) - b1*(x-edge1[0])
        t1 = a1 * (y_max-edge1[1]) - b1*(x-edge1[0])
        s2 = edge1[0] - x
        t2 = edge2[0] - x
        opti.subject_to(fmax(s1*t1, s2*t2) > 0)
        if lidar_radius > 0:
            dx1 = (lidar_xy[0] - x)
            dy1 = (lidar_xy[1] - y_min)
            dy2 = (lidar_xy[1] - y_max)
            r2 = lidar_radius*lidar_radius - dx1*dx1
            opti.subject_to(dy1*dy1 > r2)
            opti.subject_to(dy2*dy2 > r2)
        
    for wall in walls:
        if wall.y1 == wall.y2:
           # 水平の壁
            hlines(wall.y1, wall.x1, wall.x2, robot_edge_1, robot_edge_2)
            hlines(wall.y1, wall.x1, wall.x2, robot_edge_2, robot_edge_3)
            hlines(wall.y1, wall.x1, wall.x2, robot_edge_3, robot_edge_4)
            hlines(wall.y1, wall.x1, wall.x2, robot_edge_4, robot_edge_1)
        else:
            # 垂直の壁
            vlines(wall.x1, wall.y1, wall.y2, robot_edge_1, robot_edge_2)
            vlines(wall.x1, wall.y1, wall.y2, robot_edge_2, robot_edge_3)
            vlines(wall.x1, wall.y1, wall.y2, robot_edge_3, robot_edge_4)
            vlines(wall.x1, wall.y1, wall.y2, robot_edge_4, robot_edge_1)
    
    # 加速度の制約
    opti.subject_to(opti.bounded(0, ax**2 + ay**2 + at**2, max_a**2))
    
    # 並進速度の制約
    if max_vx != None:
        opti.subject_to(opti.bounded(0, vx**2+vy**2, max_vx**2))
    
    # 回転速度の制約
    if max_vt != None:
        opti.subject_to(opti.bounded(-max_vt, vt, max_vt))
    
    # 角度の制約
    if yaw_lock:
        opti.subject_to(at == 0)
        #opti.subject_to(vt == 0)
        #opti.subject_to(t == 0)
    else:
        if angle_range[0] == None:
            angle_range[0] = -np.pi
        if angle_range[1] == None:
            angle_range[1] = np.pi
        opti.subject_to(opti.bounded(angle_range[0], t, angle_range[1]))
    
    #位置の制約
    if pos_range[0][0] != None:
        opti.subject_to(pos_range[0][0] <= x)
    if pos_range[0][1] != None:
        opti.subject_to(pos_range[0][1] >= x)
    if pos_range[1][0] != None:
        opti.subject_to(pos_range[1][0] <= y)
    if pos_range[1][1] != None:
        opti.subject_to(pos_range[1][1] >= y)
        
    # 時間の制約
    if time_range[0] != None:
        opti.subject_to(time_range[0] <= T)
    if time_range[1] != None:
        opti.subject_to(time_range[1] >= T)

    # 初期条件
    opti.subject_to(x[0]==start[0])
    opti.subject_to(y[0]==start[1])
    opti.subject_to(t[0]==start[2])
    opti.subject_to(vx[0]==0)
    opti.subject_to(vy[0]==0)
    opti.subject_to(vt[0]==0)
    #opti.subject_to(ax[0]==0)
    #opti.subject_to(ay[0]==0)
    

    # 終端条件
    opti.subject_to(vx[-1]==0)
    opti.subject_to(vy[-1]==0)
    opti.subject_to(vt[-1]==0)
    #opti.subject_to(ax[-1]==0)
    #opti.subject_to(ay[-1]==0)
    opti.subject_to(x[-1]==end[0])
    opti.subject_to(y[-1]==end[1])
    opti.subject_to(t[-1]==end[2])
    
    # 時間的拘束
    #opti.subject_to(T > 1)

    # 初期値入力
    if os.path.isfile(pre_path):
        pre_data = np.load(pre_path)
        opti.set_initial(X, pre_data['X'])
        opti.set_initial(U, pre_data['U'])
        opti.set_initial(T, pre_data['T'])
    else:
        opti.set_initial(T, 3.0)

    opti.solver("ipopt")
    sol = opti.solve()
    
    # 結果を保存
    np.savez_compressed(pre_path, X=sol.value(X), U=sol.value(U), T=sol.value(T))
        
def plot(pre_path):
    pre_data = np.load(pre_path)
    X = pre_data['X']
    U = pre_data['U']
    T = pre_data['T']
    
    # 状態量 x, y, t, vx, vy, omega
    x = X[0,:]
    y = X[1,:]
    t = X[2, :]
    vx = X[3, :]
    vy = X[4, :]
    vt = X[5, :]

    # 入力
    ax = U[0, :]
    ay = U[1, :]
    at = U[2, :]
    fig = plt.figure(figsize=(3.5*3*2, 3.6*2))
    ax1 = fig.add_subplot(1, 3, 1)
    ax2 = fig.add_subplot(1, 3, 2)
    ax3 = fig.add_subplot(1, 3, 3)
    plotMap(ax1)
    ax1.plot(x,y, label="path")
    ax1.legend(loc="upper left")
    #ax1.set_xlim((0, 3.5))
    #ax1.set_ylim((0, 3.6))

    ax2.plot(vx, label="vx")
    ax2.plot(vy, label="vy")
    ax2.plot(vt, label="vt")
    ax2.plot(t, label="t")
    ax2.legend(loc="upper left")

    ax3.plot(ax, label="ax")
    ax3.plot(ay, label="ay")
    ax3.plot(at, label="at")
    ax3.legend(loc="upper left")

    fig.suptitle(f'time: {T:.3}s')
    #plt.savefig(os.path.join(os.path.dirname(__file__), "output", "X1.png"), format="png", dpi=300)
    plt.show()

if __name__ == "__main__":
    fig = plt.figure(figsize=(3.5*2, 3.6*2))
    ax1 = fig.add_subplot(1, 1, 1)
    plotMap(ax1)
    plt.show()
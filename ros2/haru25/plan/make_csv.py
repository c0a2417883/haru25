import os
import numpy as np
import glob

# 出力されたCSVのパス
SAVE_CSV_PATH = os.path.join(os.path.dirname(__file__), 'csv', 'plan.csv')

# 事前計算された経路のディレクトリ
PRE_DATA_DIR = os.path.join(os.path.dirname(__file__), "output")
# 事前計算された経路のパス（正規表現）
PRE_DATA_PATH = os.path.join(PRE_DATA_DIR, 'X*.npz')

# ファイルのパスを配列に格納
files =  glob.glob(PRE_DATA_PATH)
# 経路の数
path_num = len(files)

# 経路
plan = None

for i in range(path_num):
    # 経路ファイルパス
    file_path = os.path.join(PRE_DATA_DIR, f'X{i}.npz') 
    print(file_path)
    # .npzファイルを読み込む
    pre_data = np.load(file_path)
    # Xっていう行列(x, y, t, vx, vy, vt)を取り出す, tは角度
    array_x = pre_data['X']
    if plan is None:    # 一周目
        # planがNoneの場合はXを代入
        plan = array_x
    else:
        # planの末行にXを連結　（vertical stack　縦連結）
        plan = np.vstack((plan, array_x))

# numpy arayをcsvファイルに保存
np.savetxt(SAVE_CSV_PATH, plan, delimiter=',', fmt='%f')
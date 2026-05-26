import os
import glob

print(__file__)
#/home/yui/ros2_ws/src/haru25/ros2/haru25/plan/test.py
print(os.path.dirname(__file__))
#/home/yui/ros2_ws/src/haru25/ros2/haru25/plan
print(os.path.join(os.path.dirname(__file__), 'csv', 'plan.csv'))
#/home/yui/ros2_ws/src/haru25/ros2/haru25/plan/csv/plan.csv
print(glob.glob(os.path.join(os.path.dirname(__file__), "output", 'X*.npz')))
#['/home/yui/ros2_ws/src/haru25/ros2/haru25/plan/output/X2.npz', '/home/yui/ros2_ws/src/haru25/ros2/haru25/plan/output/X3.npz', '/home/yui/ros2_ws/src/haru25/ros2/haru25/plan/output/X0.npz', '/home/yui/ros2_ws/src/haru25/ros2/haru25/plan/output/X1.npz'


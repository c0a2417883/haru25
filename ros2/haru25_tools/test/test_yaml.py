import yaml

file_path = '/home/sasa/ros2_ws/src/haru25/ros2/haru25/config/haru25.yaml'

with open(file_path) as file:
    y = yaml.load(file)
    print(y)

y['motor_type'][0] = 1

with open(file_path, 'w') as file:
    yaml.dump(y, file, default_flow_style=True, allow_unicode=True)


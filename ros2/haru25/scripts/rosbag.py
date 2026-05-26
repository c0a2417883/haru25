import glob
import re
import datetime

def get_rosbag_path():
    rosbag_path = '/home/humble/rosbag/'
    # show directory name
    try:
        dir_names = glob.glob(f'{rosbag_path}[0-9]_[0-9][0-9]/', recursive=False)
        dir_names.sort(reverse=True)

        # 3_33
        dir_name = dir_names[0]
        rosbag_names = glob.glob(f'{dir_name}**/', recursive=False)
        rosbag_names = [re.findall(r'(?<=auto)[0-9]*', s) for s in rosbag_names]
        max_index = -1

        for rosbag_name in rosbag_names:
            for index in rosbag_name:
                index = int(index)
                if max_index < index:
                    max_index = index
        return f'{dir_name}auto{max_index+1}'
    except:
        print("rosbag error")
        now = datetime.datetime.now()
        date = now.strftime('%Y%m%d_%H%M%S')
        return f'{rosbag_path}{date}'
    
if __name__ == '__main__':
    print(get_rosbag_path())
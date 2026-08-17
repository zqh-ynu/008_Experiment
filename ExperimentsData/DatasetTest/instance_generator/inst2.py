# 本文件生成实验二所需数据：变化的无人机数
# 无人机数从 5 变化至 25
# 用户数：300

import os
import sys


# 获取当前脚本的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取父目录 (DatasetTest)
parent_dir = os.path.dirname(current_dir)
# 将父目录加入系统路径，这样才能找到 uav_deployment.py
sys.path.append(parent_dir)
# -----------------------------------
from uav_deployment import UAVDeployment
# 用户坐标文件目录
USER_DIR_NAME = r'/home/pc/qinghui/ExperimentsData/data/variable_user_num/3000u_num/user_data/'

output_dir = r"/home/pc/qinghui/ExperimentsData/data/variable_uav_num/"
uav_nums = [5, 10, 15, 20, 25]


for uav_num in uav_nums:
    output_data_dir = output_dir + str(uav_num)
    print(USER_DIR_NAME)
    print(output_data_dir)
    if not os.path.exists(output_data_dir):
        os.makedirs(output_data_dir)

    deployment = UAVDeployment(USER_DIR_NAME, output_data_dir, uav_num)

    deployment.process_all_files()
    pass




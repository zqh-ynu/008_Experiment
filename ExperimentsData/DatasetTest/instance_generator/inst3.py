
import sys
import os
# 本文件生成实验一所需数据：变化的用户数
# 用户数变化从1000-5000
# 用户数：1000,2000,3000,4000,5000


# 获取当前脚本的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取父目录 (DatasetTest)
parent_dir = os.path.dirname(current_dir)
# 将父目录加入系统路径，这样才能找到 uav_deployment.py
sys.path.append(parent_dir)
# 输入CSV文件所在的文件夹
from uav_deployment import UAVDeployment
from generate_user_data import *


INPUT_DIR_NAME = parent_dir + '/data/generated_user_loc/'

DATA_DIR = r"/home/pc/qinghui/ExperimentsData/data/variable_hard_user_ratio/"
hard_ratio_list = [0, 2, 4, 6, 8, 10]


def generate_selected_files(input_dir, f_num):
    # 2. 获取输入目录下所有CSV文件
    if not os.path.exists(input_dir):
        print(f"错误: 输入目录不存在 -> {input_dir}")
        return

    # 使用 glob 获取所有 .csv 文件
    all_csv_files = glob.glob(os.path.join(input_dir, "*.csv"))
    total_files = len(all_csv_files)

    if total_files == 0:
        print(f"错误: 在目录 {input_dir} 下未找到CSV文件。")
        return

    print(f"目录中共有 {total_files} 个CSV文件。计划随机抽取 {f_num} 个。")

    # 3. 随机抽取 f_num 个文件
    if f_num >= total_files:
        print("注意: 计划抽取数大于或等于文件总数，将处理所有文件。")
        selected_files = all_csv_files
    else:
        selected_files = random.sample(all_csv_files, f_num)

    return selected_files


selected_files = generate_selected_files(INPUT_DIR_NAME, 50)

# for hard_ratio in hard_ratio_list:
#     user_config_file = DATA_DIR + str(hard_ratio) + "/u_data_config.json"
#     generator = QoSTrafficGenerator(user_config_file)
#     generator.run_batch_simulation(
#         selected_files=selected_files,
#         output_dir=DATA_DIR + str(hard_ratio) + r"/user_data/"
#     )
#     pass


user_num = 3000

for hard_ratio in hard_ratio_list:
    # 设置输入输出目录
    input_data_dir = DATA_DIR + fr"{hard_ratio}/user_data"
    output_data_dir = DATA_DIR + fr"{hard_ratio}/uav_data"
    print(input_data_dir)
    print(output_data_dir)
    if not os.path.exists(output_data_dir):
        os.makedirs(output_data_dir)
    deployment = UAVDeployment(input_data_dir, output_data_dir)

    deployment.process_all_files()

    pass
from generate_user_data import *

# 本文件生成实验一所需数据：变化的用户数
# 用户数变化从1000-5000
# 用户数：1000,2000,3000,4000,5000

# 输入CSV文件所在的文件夹
INPUT_DIR_NAME = '../data/generated_user_loc/'

output_dir = r"E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\data\variable_user_num\\"
user_nums = [1000, 2000, 3000, 4000, 5000]


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

for unum in user_nums:
    config_file = output_dir + str(unum) + r"u_num\\" + str(unum) + "u_data_config.json"
    generator = QoSTrafficGenerator(config_file)
    generator.run_batch_simulation(
        selected_files=selected_files,
        output_dir=output_dir + str(unum) + r"u_num\user_data\\"
    )
    pass



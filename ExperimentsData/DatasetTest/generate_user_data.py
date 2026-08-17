import pandas as pd
import numpy as np
import random
import os
import json
import glob

# ================= 文件路径配置 =================

# 输入CSV文件所在的文件夹
INPUT_DIR_NAME = './data/generated_user_loc/'

# 配置文件和输出文件所在的根目录
# 注意：使用 raw string (r'') 防止 Windows 路径中的反斜杠被转义
CONFIG_DIR_NAME = r'E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\data\County_loc_Type_req\\'

# 输出文件夹
OUTPUT_DIR_NAME = os.path.join(CONFIG_DIR_NAME, 'simulated_data')
# 配置文件路径
CONFIG_FILE_PATH = os.path.join(CONFIG_DIR_NAME, 'config.json')


# ================= 核心处理类 =================

class QoSTrafficGenerator:
    def __init__(self, config_path):
        self.config = self._load_config(config_path)

    def _load_config(self, path):
        """读取并解析JSON配置文件"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"未找到配置文件: {path}")
        except json.JSONDecodeError:
            raise ValueError(f"配置文件格式错误: {path}")

    def _pick_app_config(self, config_list):
        """通用函数：根据概率列表随机选择一个应用配置对象"""
        probs = [item['prob'] for item in config_list]
        probs = np.array(probs)
        probs /= probs.sum()  # 归一化
        chosen_idx = np.random.choice(range(len(config_list)), p=probs)
        return config_list[chosen_idx]

    def generate_hard_qos(self):
        """生成 Hard QoS 数据"""
        app_conf = self._pick_app_config(self.config['hard_qos_config'])
        r_min_low, r_min_high = app_conf['r_min_range']
        r_min = np.random.uniform(r_min_low, r_min_high)
        p_out = app_conf['p_out']
        return round(r_min, 4), p_out, app_conf['name']

    def generate_elastic_qos(self):
        """生成 Elastic QoS 数据"""
        app_conf = self._pick_app_config(self.config['elastic_qos_config'])
        r_min_low, r_min_high = app_conf['r_min_range']
        r_min = np.random.uniform(r_min_low, r_min_high)
        r_max_low, r_max_high = app_conf['r_max_range']
        r_max = np.random.uniform(r_max_low, r_max_high)
        return round(r_min, 4), round(r_max, 4), app_conf['name']

    def process_single_file(self, input_full_path, output_full_path, n_num, hard_ratio):
        """
        处理单个文件的核心逻辑
        """
        # 1. 读取 CSV
        try:
            df_raw = pd.read_csv(input_full_path)
        except Exception as e:
            print(f"读取失败: {input_full_path}, 错误: {e}")
            return

        # 2. 随机采样用户 (n_num)
        if len(df_raw) >= n_num:
            df = df_raw.sample(n=n_num).reset_index(drop=True)
        else:
            # print(f"警告: 文件 {os.path.basename(input_full_path)} 数据不足 {n_num} 条，使用全部数据。")
            df = df_raw.copy()
            n_num = len(df)  # 更新实际处理的数量

        # 3. 分配用户类型
        hard_count = int(n_num * hard_ratio)
        user_types = ['hard'] * hard_count + ['elastic'] * (n_num - hard_count)
        random.shuffle(user_types)

        # 4. 生成需求数据
        results = {
            'user_type': [], 'user_weight': [],
            'user_requirement_1': [], 'user_requirement_2': [], 'app_category': []
        }

        for u_type in user_types:
            results['user_type'].append(u_type)


            if u_type == 'hard':
                r1, r2, app = self.generate_hard_qos()
                weight = random.randint(6, 10)
            else:
                r1, r2, app = self.generate_elastic_qos()
                weight = random.randint(1, 5)

            results['user_weight'].append(weight)
            results['user_requirement_1'].append(r1)
            results['user_requirement_2'].append(r2)
            results['app_category'].append(app)

        # 5. 合并数据
        for key, val in results.items():
            df[key] = val

        # 6. 输出
        final_cols = [
            'user_id', 'longitude', 'latitude',
            'user_type', 'user_weight',
            'user_requirement_1', 'user_requirement_2', 'app_category'
        ]

        try:
            df[final_cols].to_csv(output_full_path, index=False)
            print(f"  -> 生成完毕: {os.path.basename(output_full_path)} (用户数: {n_num})")
        except KeyError as e:
            print(f"  -> 错误: 输入文件 {os.path.basename(input_full_path)} 缺少必要列: {e}")



    def run_batch_simulation(self, selected_files, output_dir):
        """
        批量处理入口
        """
        # 1. 读取配置参数
        n_num = self.config['simulation_settings']['n_num']
        hard_ratio = self.config['simulation_settings']['hard_ratio']
        # 实例数量
        f_num = self.config['simulation_settings'].get('f_num', 5)  # 默认5个，如果配置文件没写

        # 4. 确保输出目录存在
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"已创建输出目录: {output_dir}")

        # 5. 循环处理
        print("=" * 30)
        for idx, file_path in enumerate(selected_files):
            # 获取原始文件名 (不带扩展名)
            base_name = os.path.splitext(os.path.basename(file_path))[0]

            # 构建新文件名： 原名_序号.csv
            new_file_name = f"{idx + 1}_{n_num}users_data_{base_name}.csv"
            output_full_path = os.path.join(output_dir, new_file_name)

            # 调用处理函数
            self.process_single_file(file_path, output_full_path, n_num, hard_ratio)

        print("=" * 30)
        print("所有任务处理完成。")




# ================= 辅助：生成测试环境 (仅首次运行需要) =================
def setup_test_env():
    # 模拟输入数据文件夹
    if not os.path.exists(INPUT_DIR_NAME):
        os.makedirs(INPUT_DIR_NAME)
        # 生成几个模拟的 csv 文件
        for i in range(10):
            df = pd.DataFrame({
                'user_id': range(100),
                'longitude': np.random.uniform(100, 101, 100),
                'latitude': np.random.uniform(30, 31, 100)
            })
            df.to_csv(os.path.join(INPUT_DIR_NAME, f'raw_data_{i}.csv'), index=False)

    # 模拟配置文件夹
    config_dir_folder = os.path.dirname(CONFIG_FILE_PATH)
    if not os.path.exists(config_dir_folder):
        os.makedirs(config_dir_folder)

    if not os.path.exists(CONFIG_FILE_PATH):
        default_config = {
            "simulation_settings": {
                "n_num": 50,
                "hard_ratio": 0.3,
                "f_num": 3  # 这里添加了 f_num
            },
            "hard_qos_config": [
                {"name": "MCPTT_Voice", "prob": 0.35, "r_min_range": [12.2, 32.0], "p_out": 0.001},
                {"name": "Vital_Signs", "prob": 0.20, "r_min_range": [1.0, 10.0], "p_out": 0.001},
                {"name": "HD_Video", "prob": 0.15, "r_min_range": [1000.0, 4000.0], "p_out": 0.01},
                {"name": "Remote_Control", "prob": 0.10, "r_min_range": [100.0, 500.0], "p_out": 0.00001},
                {"name": "Position_Report", "prob": 0.20, "r_min_range": [0.5, 2.0], "p_out": 0.1}
            ],
            "elastic_qos_config": [
                {"name": "Text_Msg", "prob": 0.30, "r_min_range": [1.0, 2.0], "r_max_range": [8.0, 16.0]},
                {"name": "Email", "prob": 0.25, "r_min_range": [16.0, 32.0], "r_max_range": [256.0, 512.0]},
                {"name": "Offline_Map", "prob": 0.15, "r_min_range": [64.0, 128.0], "r_max_range": [1000.0, 2000.0]},
                {"name": "Social_Media", "prob": 0.20, "r_min_range": [32.0, 64.0], "r_max_range": [512.0, 1000.0]},
                {"name": "Sensor_Log", "prob": 0.10, "r_min_range": [0.5, 1.0], "r_max_range": [10.0, 20.0]}
            ]
        }
        with open(CONFIG_FILE_PATH, 'w', encoding='utf-8') as f:
            json.dump(default_config, f, indent=2)
        print(f"已生成默认配置文件: {CONFIG_FILE_PATH}")


# ================= 主入口 =================

if __name__ == "__main__":

    # 1. 初始化环境 (如果你的文件已经存在，这步会自动跳过生成配置文件的过程)
    # 第一次运行时建议保留，用来生成包含 f_num 的默认配置
    # setup_test_env()

    # 2. 实例化生成器
    # 注意：确保配置文件确实存在于 CONFIG_FILE_PATH 指定的位置
    try:
        # 通过配置文件加载实例配置信息
        generator = QoSTrafficGenerator(CONFIG_FILE_PATH)

        # 3. 运行批量处理
        # 从 INPUT_DIR_NAME 读取，输出到 OUTPUT_DIR_NAME
        generator.run_batch_simulation(
            input_dir=INPUT_DIR_NAME,
            output_dir=OUTPUT_DIR_NAME
        )

    except Exception as e:
        print(f"程序运行出错: {e}")
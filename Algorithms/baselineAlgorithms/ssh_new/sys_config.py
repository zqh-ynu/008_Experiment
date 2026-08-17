"""
UAV Bandwidth Allocation System - Configuration Management
配置管理模块

包含全局参数定义、配置文件加载等功能
"""
import sys
# 插入SystemModel目录到搜索路径最前面（对应错误日志里的目录）
sys.path.insert(0, r'../SystemModel')


import math
import json
from typing import Dict, Any, Tuple
from dataclasses import dataclass
import os

# ============================================
# 全局物理常量和参数
# ============================================

# 数学常量
PI = 3.1415926535
EARTH_RADIUS = 6371000.0  # 地球半径 (米)

# 环境物理量（这些值将从配置文件加载）
freq_hz: float = 2.4e9              # 频率 (Hz)
freq_ghz: float = 2.4
speed_light: float = 299792458            # 光速 (m/s)
path_loss_exp: float = 2.0          # 路径损耗指数

# 几何参数
uav_alt: float = 300.0              # UAV高度 (m)
uav_trans_power: float = 10.0        # UAV发射功率 (W)

# 信道模型参数
noise_dbm: float = -105.0           # 噪声功率 (dBm)
los_loss_db: float = 1.0            # LoS额外损耗 (dB)
nlos_loss_db: float = 20.0          # NLoS额外损耗 (dB)
param_a: float = 9.611725               # LoS概率参数a
param_b: float = 0.158062               # LoS概率参数b
MAX_COVERAGE: int = 600   # 最大通信距离 (m)

USERS_PER_CLUSTER: int = 50

# 硬件参数
gain_bs_db: float = 0.0             # 基站天线增益 (dB)
gain_uav_db: float = 5.0            # UAV天线增益 (dB)

# 约束参数
min_se_threshold: float = 0.1       # 最小频谱效率


def load_global_channel_config(config_file: str) -> bool:
    """
    从JSON配置文件加载全局信道参数

    Args:
        config_file: 配置文件路径

    Returns:
        是否加载成功
    """
    global freq_hz, freq_ghz, speed_light, path_loss_exp
    global uav_alt, uav_trans_power
    global noise_dbm, los_loss_db, nlos_loss_db, param_a, param_b
    global MAX_COVERAGE, gain_bs_db, gain_uav_db, min_se_threshold

    try:
        with open(config_file, 'r', encoding='utf-8') as f:
            data = json.load(f)

        # 读取环境参数
        if 'environmental_constants' in data:
            env = data['environmental_constants']
            los_loss_db = env.get('los_loss_db', los_loss_db)
            nlos_loss_db = env.get('nlos_loss_db', nlos_loss_db)
            noise_dbm = env.get('noise_dbm', noise_dbm)
            param_a = env.get('param_a', param_a)
            param_b = env.get('param_b', param_b)
            speed_light = env.get('speed_light', speed_light)
            MAX_COVERAGE = env.get('maximum_comm_distance', MAX_COVERAGE)

            print("=== 环境参数 ===")
            print(f"LoS 损耗: {los_loss_db} dB")
            print(f"NLoS 损耗: {nlos_loss_db} dB")
            print(f"噪声功率: {noise_dbm} dBm")
            print(f"参数 A: {param_a}")
            print(f"参数 B: {param_b}")
            print(f"光速: {speed_light} m/s")
            print(f"最大通信距离: {MAX_COVERAGE} m")

        # 读取无人机参数
        if 'uav_constants' in data:
            uav = data['uav_constants']
            uav_alt = uav.get('uav_alt', uav_alt)
            freq_ghz = uav.get('freq_hz', freq_ghz)
            freq_hz = uav.get('freq_hz', 2.0) * 1e9  # 配置文件中以GHz为单位
            uav_trans_power = uav.get('trans_power', uav_trans_power)
            gain_uav_db = uav.get('gain_uav_db', gain_uav_db)

            print("\n=== 无人机参数 ===")
            print(f"高度: {uav_alt} m")
            print(f"频率: {freq_hz/1e9:.2f} GHz")
            print(f"传输功率: {uav_trans_power} W")
            print(f"增益: {gain_uav_db} dB")

        return True

    except FileNotFoundError:
        print(f"错误: 配置文件不存在: {config_file}")
        return False
    except json.JSONDecodeError as e:
        print(f"JSON解析错误: {e}")
        return False
    except Exception as e:
        print(f"加载配置文件时出错: {e}")
        return False


def get_matched_file_pairs(dir_path: str,
                           pattern_a: str,
                           pattern_b: str,
                           max_count: int) -> Tuple[list, list]:
    """
    获取文件夹下按ID配对的文件路径列表

    Args:
        dir_path: 数据文件夹路径
        pattern_a: 第一类文件的关键词 (例如 "1000users_data")
        pattern_b: 第二类文件的关键词 (例如 "10uavs_loc")
        max_count: 最大ID数量 (例如 50，则查找 1~50 的文件)

    Returns:
        (文件A列表, 文件B列表) 元组，按ID顺序
    """
    if not os.path.exists(dir_path) or not os.path.isdir(dir_path):
        print(f"[Error] Directory not found: {dir_path}")
        return [], []

    # 使用字典暂存文件：Key=ID, Value=FilePath
    map_a = {}
    map_b = {}

    # 遍历文件夹
    for filename in os.listdir(dir_path):
        filepath = os.path.join(dir_path, filename)

        if not os.path.isfile(filepath):
            continue

        # 解析文件ID (假设文件名格式为 "数字_关键词...")
        try:
            underscore_pos = filename.find('_')
            if underscore_pos == -1:
                continue

            # 提取开头的数字ID
            file_id = int(filename[:underscore_pos])

            # 如果ID超过了我们需要的范围，跳过
            if file_id > max_count:
                continue

            # 根据包含的关键词进行分类
            if pattern_a in filename:
                map_a[file_id] = filepath
            elif pattern_b in filename:
                map_b[file_id] = filepath

        except (ValueError, IndexError):
            continue  # 忽略无法解析ID的文件

    # 按顺序 (1 到 max_count) 提取配对成功的文件
    out_files_a = []
    out_files_b = []

    for i in range(1, max_count + 1):
        # 只有当 User 和 UAV 文件在同一个 ID 下都存在时，才加入列表
        if i in map_a and i in map_b:
            out_files_a.append(map_a[i])
            out_files_b.append(map_b[i])

    return out_files_a, out_files_b

def print_config():
    """打印当前配置"""
    print("\n" + "="*80)
    print("System Configuration")
    print("="*80)
    print(f"Frequency: {freq_ghz} GHz")
    print(f"LoS loss: {los_loss_db} dB")
    print(f"NLoS loss: {nlos_loss_db} dB")
    print(f"UAV altitude: {uav_alt} m")
    print(f"UAV transmit power: {uav_trans_power} W ({10*math.log10(uav_trans_power*1000):.1f} dBm)")
    print(f"UAV antenna gain: {gain_uav_db} dBi")
    print(f"User antenna gain: {gain_bs_db} dBi")
    print(f"Noise power: {noise_dbm:.2f} dBm")
    print(f"Max coverage distance: {MAX_COVERAGE} m")
    print("="*80)


if __name__ == "__main__":
    load_global_channel_config("def_config.json")
    print_config()
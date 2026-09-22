"""EXP5真实请求UAV数量实验入口；旧EXP5_Drawer.py仍专用于定位误差实验。"""
from EXP1_Drawer import plot_experiment_results_final, plot_targets

# 手动指定批次；只处理新真实请求EXP5，不读取旧定位误差实验结果。
BATCH_NAME = "run_01"
TARGET_EXPERIMENTS = ("EXP5_real_uav_num",)

# 仅直接运行时调用公共绘图；导入不读取结果、不生成图片，也不触发其他实验。
if __name__ == "__main__":
    plot_targets(BATCH_NAME, TARGET_EXPERIMENTS)

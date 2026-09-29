"""新hard用户比例实验入口：复用EXP1公共绘图，不改结果文件，导入不生成图片。"""
from EXP1_Drawer import plot_experiment_results_final, plot_targets

# 明确指定批次与目标，不自动查找最新目录；旧EXP5_Drawer仍专用于定位误差。
BATCH_NAME = "run_02"
TARGET_EXPERIMENTS = ("EXP3_hard_ratio",)

if __name__ == "__main__":
    plot_targets(BATCH_NAME, TARGET_EXPERIMENTS)

"""EXP4真实请求用户数实验入口：复用EXP1公共绘图，不改结果文件，导入不生成图片。"""
from EXP1_Drawer import plot_experiment_results_final, plot_targets

# 手动指定批次；路径由公共实现从脚本位置解析，不依赖启动时的工作目录。
BATCH_NAME = "run_02"
TARGET_EXPERIMENTS = ("EXP4_real_user_num",)

# 仅直接运行时绘制本实验的三个PDF；导入本模块不会读取实验结果或创建图件。
if __name__ == "__main__":
    plot_targets(BATCH_NAME, TARGET_EXPERIMENTS)

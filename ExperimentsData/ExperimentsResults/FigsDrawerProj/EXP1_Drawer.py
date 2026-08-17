import os
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ==========================================
# 1. 用户自定义配置区
# ==========================================

# 全局 X 轴标签名
X_LABEL_NAME = "Number of Users"

# 算法名称映射：{"CSV中的原始列名": "想要显示的正式名称"}
ALGO_NAME_MAP = {
    "ApproBetter": "ApproBetter",
    "ApproFast": "ApproFast",
    "AlgDRL": "AlgDRL",
    "AlgMatching": "AlgMatching",
    "AlgHardFirst": "AlgHardFirst",
    "AlgSADA": "AlgSADD"
}

# Y 轴标签映射：{"文件名": "Y轴显示的标题"}
Y_LABEL_MAP = {
    "Total_Utility": "Network Utility",
    "Run_time_ms": "Running Time (ms)",
    "Hard_Bandwidth": "Hard Bandwidth Allocation (Kbps)",
    "Total_Throughput": "System Throughput (Kbps)",
    # 如果没列出的文件，程序会自动处理文件名
}


# ==========================================
# 2. 核心绘图逻辑
# ==========================================

def plot_experiment_results_final(summary_dir):
    base_path = Path(summary_dir)
    save_dir = base_path / "figs"
    save_dir.mkdir(exist_ok=True)

    # 学术风格设置
    plt.rcParams.update({
        'pdf.fonttype': 42,  # 关键：使用 Type 42 字体 (TrueType)，确保字体嵌入
        'ps.fonttype': 42,  # 同样适用于 PostScript 格式
        'mathtext.fontset': 'stix',  # 使科学计数法的指数部分也接近 Times 风格
        'font.family': 'serif',  # 使用衬线字体
        'font.serif': ['Times New Roman'],  # 指定主字体为 Times New Roman
        'font.size': 24,
        'axes.labelsize': 26,
        'legend.fontsize': 16,
        'figure.figsize': (8, 6),
        'axes.grid': True,
        'grid.linestyle': '--',
        'grid.alpha': 0.6,
        'savefig.dpi': 300,
        'savefig.bbox': 'tight'
    })

    markers = ['o', 's', '^', 'v', 'D', '*', 'p', 'h', 'x']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22']

    csv_files = list(base_path.glob("*.csv"))

    for file_path in csv_files:
        print(f"Processing: {file_path.name}")
        df = pd.read_csv(file_path)

        x_col = df.columns[0]
        x_values = df[x_col]
        algorithms = df.columns[1:]

        plt.figure()

        # 绘图
        for i, alg in enumerate(algorithms):
            display_name = ALGO_NAME_MAP.get(alg, alg)
            plt.plot(x_values, df[alg],
                     label=display_name,
                     marker=markers[i % len(markers)],
                     color=colors[i % len(colors)],
                     linewidth=2,
                     markersize=7)

        # --- 核心改进：设置 X 轴范围和刻度 ---
        # 1. 强制 X 轴范围为数据的最小值到最大值，消除空白
        plt.xlim(x_values.min(), x_values.max())

        # 2. 强制刻度仅显示数据点
        plt.xticks(x_values)

        # 设置标签
        plt.xlabel(X_LABEL_NAME)
        file_stem = file_path.stem
        y_label = Y_LABEL_MAP.get(file_stem, file_stem.replace('_', ' '))
        plt.ylabel(y_label)

        # 超过10^4的数用科学计数法
        plt.ticklabel_format(style='sci', axis='y', scilimits=(0, 4))
        plt.gca().yaxis.get_offset_text().set_family('serif')

        plt.legend(loc='best', frameon=True)
        plt.tight_layout()

        # 处理运行时间文件的特殊情况
        if "Run_time" in file_path.stem:
            # 如果耗时差异巨大，可开启对数坐标
            plt.yscale('log')
            pass

        # 保存
        save_path = save_dir / f"{file_stem}.pdf"
        plt.savefig(save_path)
        plt.close()

    print(f"\n✅ 绘图完成！已消除边缘空白。")


# --- 运行 ---
target_dir = r"E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\ExperimentsResults\EXP1_user_num\summary"
# target_dir = r"E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\ExperimentsResults\EXP2_uav_num\summary"
plot_experiment_results_final(target_dir)
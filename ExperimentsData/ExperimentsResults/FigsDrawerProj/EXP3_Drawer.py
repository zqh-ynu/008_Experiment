"""EXP3_hard_user_ratio：只读绘制最新完整批次的独立图及横向组图，输出至 summary/figs。
保留旧图名称与统计口径，不修改算法、结果 CSV 或历史批次。
"""

import json
import math
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ==========================================
# 1. 用户自定义配置区
# ==========================================

# 全局 X 轴标签名
X_LABEL_NAME = "The Ratio of Users with Hard QoS Requirements "

# 算法名称映射：{"CSV中的原始列名": "想要显示的正式名称"}
ALGO_NAME_MAP = {
    "ApproBetter": "ApproBetter",
    "ApproFast": "ApproFast",
    "AlgDRL": "AlgDRL",
    "AlgMatching": "AlgMatching",
    "AlgHardFirst": "AlgHardFirst",
    "AlgSADA": "AlgSADD",
    # 仅恢复旧稿件显示名；AlgDRL 对应松弛舍入，不代表 DRL 实现。
    "AlgRelaxRound": "AlgDRL",
    "AlgSwapMatching": "AlgMatching",
    "AlgSA-DD": "AlgSADD"
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

METHOD_ORDER = [
    "ApproBetter", "ApproFast", "AlgRelaxRound",
    "AlgSwapMatching", "AlgHardFirst", "AlgSA-DD"
]
EXPECTED_CONDITIONS = [0,2,4,6,8,10]
EXPECTED_ALGORITHM_VERSION = "ton-proposed-fast-faster-v1"

def plot_experiment_results_final(summary_dir):
    """校验并绘制指定最新汇总目录的独立图及横向组图，返回 None。

    参数 summary_dir 为完整批次的 summary 目录；输出两个独立 PDF 和一个组图至 figs。
    缺文件、版本不符、条件不全或数值非法时抛出异常，不回退到旧数据。
    """
    base_path = Path(summary_dir)
    with (base_path.parent / "run_info.json").open(encoding="utf-8-sig") as handle:
        run_info = json.load(handle)
    if (run_info.get("experiment") != "EXP3_hard_user_ratio"
            or run_info.get("algorithm_version") != EXPECTED_ALGORITHM_VERSION
            or run_info.get("summary", {}).get("status") != "complete"
            or run_info.get("instance_count") != 10
            or run_info.get("summary", {}).get("instance_count") != 10
            or run_info.get("methods") != METHOD_ORDER):
        raise ValueError(f"结果批次不完整或与当前绘图设置不符：{base_path.parent}")

    # 先检查全部输入，避免在第二项指标缺失时已经发布第一张图。
    frames = {}
    for file_stem in ("Run_time_ms", "Total_Utility"):
        file_path = base_path / f"{file_stem}.csv"
        frame = pd.read_csv(file_path, encoding="utf-8-sig", float_precision="round_trip")
        if (list(frame.columns) != ["User_Scale", *METHOD_ORDER]
                or frame["User_Scale"].tolist() != EXPECTED_CONDITIONS):
            raise ValueError(f"方法列或实验条件不完整：{file_path}")
        values = frame[METHOD_ORDER].to_numpy(dtype=float)
        if not all(math.isfinite(value) for value in values.flat):
            raise ValueError(f"指标包含缺失或非有限值：{file_path}")
        if file_stem == "Run_time_ms" and (values <= 0).any():
            raise ValueError(f"对数时间轴要求所有耗时大于零：{file_path}")
        frames[file_stem] = frame

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
        'axes.labelsize': 24,
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

    for file_stem, df in frames.items():
        print(f"Processing: {file_stem}.csv")
        x_col = df.columns[0]
        # CSV 存储 0--10 的档位编码；仅横轴显示换算为 0--1 的实际比例。
        x_values = df[x_col] / 10.0
        algorithms = METHOD_ORDER

        plt.figure()

        # 绘图
        for i, alg in enumerate(algorithms):
            display_name = ALGO_NAME_MAP.get(alg, alg)
            plt.plot(x_values, df[alg],
                     label=display_name,
                     marker=markers[i % len(markers)],
                     color=colors[i % len(colors)],
                     linewidth=2,
                     # 实心小方块叠在大空心圆内，重合点仍能辨识两种提出方法。
                     linestyle='--' if alg == "ApproFast" else '-',
                     markersize=10 if alg == "ApproBetter" else (5 if alg == "ApproFast" else 7),
                     markerfacecolor='none' if alg == "ApproBetter" else colors[i % len(colors)],
                     markeredgewidth=1.5 if alg == "ApproBetter" else 1,
                     zorder=4 if alg == "ApproBetter" else (5 if alg == "ApproFast" else 2))

        # 少量边距避免端点标记被裁切；刻度仍对应真实实验条件。
        plt.margins(x=0.03)
        plt.xticks(x_values, [f"{value:g}" for value in x_values])

        # 设置标签
        plt.xlabel(X_LABEL_NAME)
        y_label = Y_LABEL_MAP.get(file_stem, file_stem.replace('_', ' '))
        plt.ylabel(y_label)

        # 先设置坐标尺度，再计算布局，避免对数轴使用线性格式器。
        if file_stem == "Run_time_ms":
            plt.yscale('log')
        else:
            plt.ticklabel_format(style='sci', axis='y', scilimits=(0, 4))
            plt.gca().yaxis.get_offset_text().set_family('serif')

        # 图例置于绘图区上方，避免遮挡数据；按行保持既有六方法顺序。
        handles, labels = plt.gca().get_legend_handles_labels()
        legend_order = [0, 3, 1, 4, 2, 5]
        plt.legend([handles[index] for index in legend_order],
                   [labels[index] for index in legend_order],
                   loc='lower center', bbox_to_anchor=(0.5, 1.10),
                   ncol=3, frameon=True, columnspacing=1.2)
        plt.tight_layout()

        # 保存
        save_path = save_dir / f"{file_stem}.pdf"
        plt.savefig(save_path)
        plt.close()

    # 复用已校验的两项数据绘制横向组图，不改变独立图的生成方式。
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    for ax, file_stem in zip(axes, ("Run_time_ms", "Total_Utility")):
        df = frames[file_stem]
        x_col = df.columns[0]
        # 与独立图一致，仅将 CSV 中的 0--10 档位编码显示为 0--1 比例。
        x_values = df[x_col] / 10.0

        for i, alg in enumerate(METHOD_ORDER):
            display_name = ALGO_NAME_MAP.get(alg, alg)
            ax.plot(x_values, df[alg],
                    label=display_name,
                    marker=markers[i % len(markers)],
                    color=colors[i % len(colors)],
                    linewidth=2,
                    linestyle='--' if alg == "ApproFast" else '-',
                    markersize=10 if alg == "ApproBetter" else (5 if alg == "ApproFast" else 7),
                    markerfacecolor='none' if alg == "ApproBetter" else colors[i % len(colors)],
                    markeredgewidth=1.5 if alg == "ApproBetter" else 1,
                    zorder=4 if alg == "ApproBetter" else (5 if alg == "ApproFast" else 2))

        ax.margins(x=0.03)
        ax.set_xticks(x_values)
        ax.set_xticklabels([f"{value:g}" for value in x_values])
        ax.set_xlabel(X_LABEL_NAME)
        ax.set_ylabel(Y_LABEL_MAP.get(file_stem, file_stem.replace('_', ' ')))
        if file_stem == "Run_time_ms":
            ax.set_yscale('log')
        else:
            ax.ticklabel_format(style='sci', axis='y', scilimits=(0, 4))
            ax.yaxis.get_offset_text().set_family('serif')

    handles, labels = axes[0].get_legend_handles_labels()
    # 单行图例直接使用 METHOD_ORDER 的绘制顺序，无需独立图的两行重排。
    fig.legend(handles, labels,
               loc="upper center", bbox_to_anchor=(0.5, 0.995),
               ncol=len(METHOD_ORDER), frameon=True, columnspacing=1.2)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(save_dir / "EXP3_Combined.pdf")
    plt.close(fig)

    print(f"\n绘图完成：{save_dir}（两个独立图及一个横向组图）")


# 从脚本位置定位当前批次；导入模块时不自动创建图片。
if __name__ == '__main__':
    target_dir = (Path(__file__).resolve().parent.parent
                  / "ToN" / "EXP3_hard_user_ratio" / "run_ton_01" / "summary")
    plot_experiment_results_final(target_dir)

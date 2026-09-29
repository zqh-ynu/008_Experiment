"""EXP5：只读绘制定位误差下的四项网络均值，并导出独立图及 2x2 组合图。

输入为当前完整 run_ton_01 的 overall_means.csv；不重新聚合、运行算法或改写 CSV。
完整批次仍校验六种方法，但图中只绘制当前批次标记的两种本文算法 ApproBetter 和 ApproFast。
utility_loss 沿用各网络相对自身零误差参照的损失，保留负值与 NA。
违约比例以已接纳 Hard 用户为分母；平均相对速率缺口还包含达标者的零缺口，
仅在已接纳且最低速率为正的 Hard 用户中平均，不是仅对违约用户平均。
"""

import json
import math
import warnings
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

# 输入批次必须包含的完整方法顺序；绘图时不会删改这些方法对应的 CSV 行。
INPUT_METHOD_ORDER = [
    "ApproBetter", "ApproFast", "AlgRelaxRound",
    "AlgSwapMatching", "AlgHardFirst", "AlgSA-DD"
]

# 当前 EXP5 图中仅保留 run_info.json proposed_algorithms 标记的两种本文算法。
METHOD_ORDER = [
    "ApproBetter", "ApproFast"
]
# AlgDRL 仅是松弛舍入方法的历史显示别名；其余映射为旧图兼容保留。
ALGO_NAME_MAP = {
    "ApproBetter": "ApproBetter",
    "ApproFast": "ApproFast",
    "AlgDRL": "AlgDRL",
    "AlgMatching": "AlgMatching",
    "AlgHardFirst": "AlgHardFirst",
    "AlgSADA": "AlgSADD",
    "AlgRelaxRound": "AlgDRL",
    "AlgSwapMatching": "AlgMatching",
    "AlgSA-DD": "AlgSADD"
}
RMSE_LEVELS = [0, 1, 2, 5, 10, 20, 50]
EXPECTED_ALGORITHM_VERSION = "ton-proposed-fast-faster-v1"
X_LABEL_NAME = "Localization RMSE (m)"
MARKERS = ['o', 's', '^', 'v', 'D', '*']
COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

# 每项为：现有指标列名、英文纵轴标签、显示倍率、输出文件名。
# 仅后三项乘 100 转成百分数，不修改其统计总体或分母。
METRICS = [
    ("realized_total_utility", "Realized Network Utility", 1.0, "Realized_Total_Utility.pdf"),
    ("utility_loss", "Utility Loss (%)", 100.0, "Utility_Loss.pdf"),
    ("hard_admission_violation_ratio", "Admitted Hard-QoS\nViolation Rate (%)",
     100.0, "Hard_Admission_Violation_Ratio.pdf"),
    ("hard_mean_shortfall", "Mean Relative Rate\nShortfall (%)",
     100.0, "Hard_Mean_Shortfall.pdf")
]


def load_experiment_results(run_dir):
    """读取并校验完整 EXP5 批次，返回指标到 RMSE×方法 DataFrame 的字典。

    参数 run_dir 为批次根目录。均值直接取自 overall_means.csv；
    缺文件、缺条件、重复行、版本不符或非法计数时报错，不寻找旧批次。
    合法 NA 原样保留为 NaN，并通过警告报告有效网络数不足的指标。
    """
    base_path = Path(run_dir)
    for name in ("run_info.json", "overall_means.csv", "network_means.csv", "raw_results.csv"):
        if not (base_path / name).is_file():
            raise FileNotFoundError(f"EXP5 缺少必需文件：{base_path / name}")
    with (base_path / "run_info.json").open(encoding="utf-8-sig") as handle:
        run_info = json.load(handle)
    settings = run_info.get("settings", {})
    if (run_info.get("summary", {}).get("status") != "complete"
            or run_info.get("summary", {}).get("completed_calls") != 780
            or settings.get("algorithm_version") != EXPECTED_ALGORITHM_VERSION
            or settings.get("instance_count") != 10
            or settings.get("expected_calls") != 780
            or settings.get("error_repeats") != 2
            or settings.get("methods") != INPUT_METHOD_ORDER
            or set(settings.get("proposed_algorithms", {}).keys()) != set(METHOD_ORDER)
            or settings.get("rmse_m") != RMSE_LEVELS):
        raise ValueError(f"EXP5 批次不完整或与当前绘图设置不符：{base_path}")

    summary = pd.read_csv(base_path / "overall_means.csv", encoding="utf-8-sig",
                          float_precision="round_trip")
    expected_columns = ["rmse_m", "method", "metric", "mean", "valid_networks", "expected_networks"]
    if list(summary.columns) != expected_columns:
        raise ValueError("EXP5 overall_means.csv 的字段与当前格式不符")

    frames = {}
    for metric, _, _, _ in METRICS:
        rows = summary.loc[summary["metric"] == metric].copy()
        if (len(rows) != len(RMSE_LEVELS) * len(INPUT_METHOD_ORDER)
                or set(rows["method"]) != set(INPUT_METHOD_ORDER)
                or set(rows["rmse_m"]) != set(RMSE_LEVELS)
                or rows.duplicated(["rmse_m", "method"]).any()):
            raise ValueError(f"EXP5 指标缺条件、缺方法或包含重复行：{metric}")
        if (not rows["expected_networks"].eq(10).all()
                or not rows["valid_networks"].isin(range(11)).all()):
            raise ValueError(f"EXP5 指标的网络计数非法：{metric}")
        values = pd.to_numeric(rows["mean"], errors="raise")
        if (not all(pd.isna(value) or math.isfinite(value) for value in values)
                or not values.isna().eq(rows["valid_networks"].eq(0)).all()):
            raise ValueError(f"EXP5 指标值与有效网络计数不一致：{metric}")
        if rows["valid_networks"].lt(10).any():
            warnings.warn(f"{metric} 存在少于 10 个有效网络的点；NA 保留为曲线缺口。",
                          RuntimeWarning, stacklevel=2)
        rows["mean"] = values
        # 不在绘图层平均或归一化；重排后每个位置仍是原 CSV 的同一均值。
        frames[metric] = rows.pivot(index="rmse_m", columns="method", values="mean").reindex(
            index=RMSE_LEVELS, columns=METHOD_ORDER)
    return frames


def set_plot_style():
    """设置与原 EXP1--3 一致的字体、配色之外的全局样式；无参数，返回 None。"""
    plt.rcParams.update({
        'pdf.fonttype': 42,
        'ps.fonttype': 42,
        'mathtext.fontset': 'stix',
        'font.family': 'serif',
        'font.serif': ['Times New Roman'],
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


def draw_metric(ax, frame, metric_config, compact=False):
    """在 ax 上绘制一项均值，返回两条本文算法曲线的 Line2D 列表。

    frame 的索引为实际设定的 RMSE、列为当前绘图的两种本文算法；metric_config 给出
    指标、标签及单位倍率。compact=True 仅缩小组合图字号，不改变任何坐标数据。
    """
    metric, y_label, scale, _ = metric_config
    lines = []
    for index, method in enumerate(METHOD_ORDER):
        line, = ax.plot(
            RMSE_LEVELS, frame[method].to_numpy(dtype=float) * scale,
            label=ALGO_NAME_MAP[method], marker=MARKERS[index], color=COLORS[index],
            linewidth=2, linestyle='--' if method == "ApproFast" else '-',
            markersize=10 if method == "ApproBetter" else (5 if method == "ApproFast" else 7),
            markerfacecolor='none' if method == "ApproBetter" else COLORS[index],
            markeredgewidth=1.5 if method == "ApproBetter" else 1,
            zorder=4 if method == "ApproBetter" else (5 if method == "ApproFast" else 2))
        lines.append(line)

    # 数值轴保留不等间距采样；小误差点全部绘制，但不挤放 0/1/2 的大字号刻度。
    ax.set_xscale("linear")
    ax.set_yscale("linear")
    ax.set_xticks([0, 10, 20, 30, 40, 50])
    ax.set_xticks([1, 2, 5], minor=True)
    ax.margins(x=0.03, y=0.08)
    ax.set_xlabel(X_LABEL_NAME, fontsize=18 if compact else 26)
    ax.set_ylabel(y_label, fontsize=18 if compact else 26)
    ax.tick_params(axis="both", which="major", labelsize=16 if compact else 24)
    ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
    if metric == "realized_total_utility":
        ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 4))
    else:
        ax.ticklabel_format(style="plain", axis="y", useOffset=False)
    ax.yaxis.get_offset_text().set_family("serif")
    ax.yaxis.get_offset_text().set_fontsize(16 if compact else 24)
    return lines


def plot_experiment_results_final(run_dir):
    """绘制完整 EXP5 批次，返回四张独立 PDF 和组合 PDF 的路径列表。

    参数 run_dir 为最新批次根目录。先校验全部输入，再在其 figs 子目录输出；
    独立图与组合图复用同一份 frames，保证数据、方法、单位和统计口径一致。
    """
    base_path = Path(run_dir)
    frames = load_experiment_results(base_path)
    set_plot_style()
    save_dir = base_path / "figs"
    save_dir.mkdir(exist_ok=True)
    output_paths = []

    for metric_config in METRICS:
        metric, _, _, filename = metric_config
        fig, ax = plt.subplots()
        draw_metric(ax, frames[metric], metric_config)
        # 独立图同样将图例放到轴外，避免自动位置遮住两种本文算法曲线。
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles, labels,
                  loc="lower center", bbox_to_anchor=(0.5, 1.10),
                  ncol=len(METHOD_ORDER), frameon=True, columnspacing=1.2)
        fig.tight_layout()
        save_path = save_dir / filename
        fig.savefig(save_path)
        plt.close(fig)
        output_paths.append(save_path)

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    for panel, ax, metric_config in zip("abcd", axes.flat, METRICS):
        draw_metric(ax, frames[metric_config[0]], metric_config, compact=True)
        # 子图编号放在右上方，避开左上方效用轴的科学计数法倍率。
        ax.text(1.0, 1.03, f"({panel})", transform=ax.transAxes,
                ha="right", va="bottom", fontsize=20)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    # 组合图沿用当前两种本文算法的绘制顺序，图例不再包含基准算法。
    fig.legend(handles, labels,
               loc="upper center", bbox_to_anchor=(0.53, 0.995),
               ncol=len(METHOD_ORDER), frameon=False, fontsize=16)
    fig.subplots_adjust(left=0.115, right=0.98, bottom=0.09, top=0.855,
                        wspace=0.36, hspace=0.45)
    save_path = save_dir / "EXP5_Combined.pdf"
    fig.savefig(save_path)
    plt.close(fig)
    output_paths.append(save_path)
    print(f"绘图完成：{save_dir}（四张独立图及一张组合图）")
    return output_paths


if __name__ == '__main__':
    target_dir = (Path(__file__).resolve().parent.parent
                  / "ToN" / "EXP5_location_error" / "run_ton_01")
    plot_experiment_results_final(target_dir)

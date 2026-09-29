"""新五组multi-hard实验的公共读取与绘图；直接运行本文件仅绘制EXP1主体请求用户数实验。

只读四个结果文件，不聚合原始记录、不改CSV或元数据。未完成目标跳过，导入不出图。
"""
import json
import math
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

# 手动选择结果批次，不自动发现“最新”目录；其余入口各自也显式配置此名称。
BATCH_NAME = "run_02"
TARGET_EXPERIMENTS = ("EXP1_user_num",)
METHOD_ORDER = ["ApproBetter", "ApproFast", "AlgRelaxRound", "AlgSwapMatching", "AlgHardFirst", "AlgSA-DD"]
# 仅作旧图例显示映射；AlgDRL实际对应松弛舍入，不表示新增/执行了DRL。
ALGO_NAME_MAP = dict(zip(METHOD_ORDER, ["ApproBetter", "ApproFast", "AlgDRL", "AlgMatching", "AlgHardFirst", "AlgSADD"]))
# 与run_02元数据严格一致：八候选HardFirst及Better弹性重优化v3，基线采用最低档。
EXPECTED_ALGORITHM_VERSION = "ton-multihard-lowest-baselines-hardfirst-hard-only-rectangular-hungarian-single-slot-multistart8-v1-better-elastic-reopt-v3"
EXPERIMENTS = {
    "EXP1_user_num": ([1000, 2000, 3000, 4000, 5000], "Number of Users", 1, "EXP1"),
    "EXP2_uav_num": ([5, 10, 15, 20], "Number of UAVs", 1, "EXP2"),
    "EXP3_hard_ratio": ([0, 2, 4, 6, 8, 10], "The Ratio of Users with Hard QoS Requirements ", 10, "EXP3"),
    "EXP4_real_user_num": ([1000, 2000, 3000, 4000, 5000], "Number of Users", 1, "EXP4"),
    "EXP5_real_uav_num": ([5, 10, 15, 20], "Number of UAVs", 1, "EXP5"),
}
Y_LABEL_MAP = {"Run_time_ms": "Running Time (ms)", "Total_Utility": "Network Utility"}
MARKERS = ["o", "s", "^", "v", "D", "*"]
COLORS = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


def _read_info(path):
    """以UTF-8读取运行元数据；坏JSON明确报错，不修复或回退旧数据。"""
    with path.open(encoding="utf-8-sig") as handle:
        value = json.load(handle)
    if not isinstance(value, dict):
        raise ValueError(f"元数据必须为对象：{path}")
    return value


def _integer(value, label):
    """校验非负整数计数，拒绝bool、空值、非有限值及小数；返回int。"""
    if isinstance(value, bool):
        raise ValueError(f"非法计数：{label}")
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"非法计数：{label}") from None
    if not math.isfinite(number) or number < 0 or number != math.floor(number):
        raise ValueError(f"非法计数：{label}")
    return int(number)


def load_experiment_results(summary_dir):
    """读取完成实验的两张均值表及成功率；返回(info, frames)，未完成返回None。

    frames仅在内存中把零成功均值和零耗时置为NaN，以断开曲线；源文件不修改。
    """
    base = Path(summary_dir)
    info_path = base.parent / "run_info.json"
    if not base.parent.exists() or not info_path.exists():
        print(f"[SKIP] {base.parent.name}：结果目录或元数据尚不存在")
        return None
    info = _read_info(info_path)
    if info.get("summary", {}).get("status") != "complete":
        print(f"[SKIP] {base.parent.name}：汇总尚未完成")
        return None
    experiment = info.get("experiment")
    if (experiment not in EXPERIMENTS or experiment != base.parent.name
            or info.get("schema") != "ton-multihard-run-v1"
            or info.get("input_schema") != "ton-multihard-input-v1"
            or info.get("baseline_hard_policy") != "lowest_level_only"
            or info.get("utility_evaluation") != "original_model"
            or info.get("methods") != METHOD_ORDER):
        raise ValueError(f"实验身份、格式或算法策略不匹配：{base.parent}")
    count = _integer(info.get("instance_count"), "instance_count")
    if count < 1 or _integer(info["summary"].get("instance_count"), "summary.instance_count") != count:
        raise ValueError(f"汇总实例数与当前批次不一致：{base.parent}")
    conditions = EXPERIMENTS[experiment][0]
    if [str(c) for c in conditions] != [c.get("key") for c in info.get("conditions", [])]:
        raise ValueError(f"元数据条件网格不完整：{base.parent}")

    # 按字符串读表以区分真正的空单元格和写入的NaN/Inf文本；后者属于非法非空值。
    tables = {}
    try:
        for filename in ("Success_Rate", "Run_time_ms", "Total_Utility"):
            tables[filename] = pd.read_csv(base / f"{filename}.csv", encoding="utf-8-sig",
                                           dtype=str, keep_default_na=False)
    except (OSError, ValueError, pd.errors.ParserError) as error:
        # 若读取期间刚好启动扩跑，汇总可能暂时不可读；只在元数据确实变化时跳过。
        try:
            changed = _read_info(info_path) != info
        except (OSError, ValueError):
            changed = True
        if changed:
            print(f"[SKIP] {experiment}：汇总正在更新，请稍后手动重试")
            return None
        raise error
    # 读取后再看元数据，拒绝在扩跑/重写过程中混用旧新汇总；不等待、不轮询。
    try:
        after = _read_info(info_path)
    except (OSError, ValueError):
        print(f"[SKIP] {experiment}：读取期间元数据不可用，请待汇总稳定后重试")
        return None
    if after != info:
        print(f"[SKIP] {experiment}：读取期间运行状态或配置已变化")
        return None

    rates = tables["Success_Rate"]
    rate_columns = ["condition", "method", "success_count", "attempt_count", "success_rate"]
    if list(rates.columns) != rate_columns or len(rates) != len(conditions) * len(METHOD_ORDER):
        raise ValueError(f"成功率表结构不完整：{base}")
    successes = {}
    for row in rates.to_dict("records"):
        condition = _integer(row["condition"], "condition")
        key = (condition, row["method"])
        if condition not in conditions or row["method"] not in METHOD_ORDER or key in successes:
            raise ValueError(f"成功率表条件/方法错误或重复：{key}")
        success = _integer(row["success_count"], str(key))
        attempts = _integer(row["attempt_count"], str(key))
        rate = float(row["success_rate"])
        if attempts != count or success > attempts or not math.isfinite(rate) or not math.isclose(rate, success / attempts, rel_tol=1e-10, abs_tol=1e-12):
            raise ValueError(f"成功数、尝试数或成功率矛盾：{key}")
        successes[key] = success
        if success < attempts:
            print(f"[WARN] {experiment}/{condition}/{row['method']}：成功 {success}/{attempts}，均值仅基于成功实例")
    if (_integer(info["summary"].get("attempt_count"), "summary.attempt_count") != len(successes) * count
            or _integer(info["summary"].get("success_count"), "summary.success_count") != sum(successes.values())):
        raise ValueError(f"成功率表与元数据总计不符：{base}")

    frames = {}
    for metric in Y_LABEL_MAP:
        raw = tables[metric]
        if list(raw.columns) != ["User_Scale", *METHOD_ORDER] or [_integer(v, "User_Scale") for v in raw["User_Scale"]] != conditions:
            raise ValueError(f"指标表列名或条件网格错误：{base / (metric + '.csv')}")
        frame = pd.DataFrame({"User_Scale": conditions})
        for method in METHOD_ORDER:
            values = []
            for condition, cell in zip(conditions, raw[method]):
                cell = cell.strip()
                success = successes[(condition, method)]
                if not cell:
                    if success:
                        raise ValueError(f"存在成功实例但均值缺失：{metric}/{condition}/{method}")
                    values.append(float("nan"))
                    continue
                # if success == 0:
                #     raise ValueError(f"零成功实例却有非空均值：{metric}/{condition}/{method}")
                value = float(cell)
                if not math.isfinite(value) or value < 0:
                    raise ValueError(f"非法指标值：{metric}/{condition}/{method}")
                if metric == "Run_time_ms" and value == 0:
                    print(f"[WARN] {experiment}/{condition}/{method}：零耗时无法用于对数轴，仅屏蔽该耗时点")
                    value = float("nan")
                values.append(value)
            frame[method] = values
        frames[metric] = frame
    return info, frames


def set_plot_style():
    """保留原Times New Roman、字号、网格和嵌入字体设置，不引入其他期刊样式。"""
    plt.rcParams.update({
        "pdf.fonttype": 42, "ps.fonttype": 42, "mathtext.fontset": "stix",
        "font.family": "serif", "font.serif": ["Times New Roman"],
        "font.size": 24, "axes.labelsize": 26, "legend.fontsize": 16,
        "figure.figsize": (8, 6), "axes.grid": True,
        "grid.linestyle": "--", "grid.alpha": 0.6,
        "savefig.dpi": 300, "savefig.bbox": "tight",
    })


def draw_metric(ax, frame, metric, experiment):
    """绘制六方法及原图例样式；NaN直接传入matplotlib形成缺口，绝不删除中间行后跨缺口连线。"""
    _, x_label, divisor, _ = EXPERIMENTS[experiment]
    x = frame["User_Scale"].to_numpy(dtype=float) / divisor
    for i, method in enumerate(METHOD_ORDER):
        ax.plot(x, frame[method].to_numpy(dtype=float),
                label=ALGO_NAME_MAP[method], marker=MARKERS[i], color=COLORS[i], linewidth=2,
                linestyle="--" if method == "ApproFast" else "-",
                markersize=10 if method == "ApproBetter" else (5 if method == "ApproFast" else 7),
                markerfacecolor="none" if method == "ApproBetter" else COLORS[i],
                markeredgewidth=1.5 if method == "ApproBetter" else 1,
                zorder=4 if method == "ApproBetter" else (5 if method == "ApproFast" else 2))
    ax.margins(x=0.03)
    ax.set_xticks(x)
    # 原比例图轴标签为24号，其他实验为26号；保留已有字号差异。
    label_size = 24 if experiment == "EXP3_hard_ratio" else 26
    ax.set_xlabel(x_label, fontsize=label_size)
    ax.set_ylabel(Y_LABEL_MAP[metric], fontsize=label_size)
    if metric == "Run_time_ms":
        ax.set_yscale("log")
    else:
        ax.ticklabel_format(style="sci", axis="y", scilimits=(0, 4))
        ax.yaxis.get_offset_text().set_family("serif")


def plot_experiment_results_final(summary_dir):
    """读取summary_dir，写带实验编号的两个独立PDF和一个横向组图；返回路径列表，跳过时返回空列表。"""
    loaded = load_experiment_results(summary_dir)
    if loaded is None:
        return []
    info, frames = loaded
    experiment = info["experiment"]
    set_plot_style()
    save_dir = Path(summary_dir) / "figs"
    save_dir.mkdir(exist_ok=True)
    outputs = []
    for metric, frame in frames.items():
        fig, ax = plt.subplots(figsize=(8, 6))
        try:
            draw_metric(ax, frame, metric, experiment)
            handles, labels = ax.get_legend_handles_labels()
            order = [0, 3, 1, 4, 2, 5]
            # 单图图例放在轴内自动选择的位置，保留原三列布局及显示顺序。
            ax.legend([handles[i] for i in order], [labels[i] for i in order],
                      loc="best", ncol=3,
                      frameon=True, columnspacing=1.2)
            fig.tight_layout()
            path = save_dir / f"{EXPERIMENTS[experiment][3]}_{metric}.pdf"
            fig.savefig(path)
            outputs.append(path)
        finally:
            plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))
    try:
        for ax, metric in zip(axes, Y_LABEL_MAP):
            draw_metric(ax, frames[metric], metric, experiment)
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 0.995),
                   ncol=len(METHOD_ORDER), frameon=True, columnspacing=1.2)
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        path = save_dir / f"{EXPERIMENTS[experiment][3]}_Combined.pdf"
        fig.savefig(path)
        outputs.append(path)
    finally:
        plt.close(fig)
    print(f"[DONE] {experiment}：N={info['instance_count']}，三个PDF -> {save_dir}")
    return outputs


def plot_targets(batch_name, experiments):
    """手动入口的目标循环；未完成跳过，坏数据明确报告，其他完成目标仍处理，最终报错退出。"""
    root = Path(__file__).resolve().parent.parent / "ToN_multiHard" / batch_name
    errors = []
    for experiment in experiments:
        try:
            plot_experiment_results_final(root / experiment / "summary")
        except (OSError, ValueError, KeyError, TypeError) as error:
            errors.append(f"{experiment}: {error}")
            print(f"[ERROR] {experiment}：{error}")
    if errors:
        raise RuntimeError("部分绘图目标失败；未回退旧数据：\n" + "\n".join(errors))


if __name__ == "__main__":
    plot_targets(BATCH_NAME, TARGET_EXPERIMENTS)

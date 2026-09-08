"""生成 ToN 扩展实验使用的成批用户与 UAV 实例。

本入口使用相对于脚本文件的位置解析输入与输出目录，默认生成 30 个县区
重复实例，手动入口固定使用 ``data_ToN/2026-09-07``，供 C++ 先算 ID 1--10。
所有条件的文件编号、表头和实际规模检查通过后才写生成完成配置。导入本模块不会
创建目录；只有直接运行本文件或显式调用 ``generate_all_instances`` 才会写入
完整批次。由于 UAV 网格部署耗时较长，普通语法或小样本检查不应调用该入口。
"""

from __future__ import annotations

import json
import platform
import shutil
import sys
from datetime import date, datetime
from pathlib import Path

import numpy as np
import pandas as pd


INSTANCE_GENERATOR_DIR = Path(__file__).resolve().parent
DATASET_TEST_DIR = INSTANCE_GENERATOR_DIR.parent
EXPERIMENTS_DATA_DIR = DATASET_TEST_DIR.parent
DEFAULT_LOCATION_DIR = DATASET_TEST_DIR / "data" / "generated_user_loc"
DEFAULT_OUTPUT_ROOT = EXPERIMENTS_DATA_DIR / "data_ToN"

# 直接执行本文件时，把 DatasetTest 加入模块搜索路径，以复用相邻生成代码。
if str(DATASET_TEST_DIR) not in sys.path:
    sys.path.insert(0, str(DATASET_TEST_DIR))

from generate_user_data_ToN import (  # noqa: E402
    EXP3_ELASTIC_PROFILES,
    EXP3_HARD_PROFILES,
    FIXED_HARD_QOS,
    INSTANT_MESSAGE_HARD_PROBABILITY,
    INSTANT_MESSAGE_VIDEO_QOS,
    INSTANT_MESSAGE_VOICE_PROBABILITY,
    INSTANT_MESSAGE_VOICE_QOS,
    TABLE_3_APPLICATIONS,
    TABLE_6_SESSION_WEIGHTS,
    USER_COLUMNS,
    generate_exp3_user_sets,
    generate_ton_users,
    load_location_pool,
)
from uav_deployment import UAVDeployment  # noqa: E402


DEFAULT_MASTER_SEED = 20260904
DEFAULT_REPLICATE_COUNT = 30
DEFAULT_UAV_BANDWIDTH = 40
USER_NUMBERS = (1000, 2000, 3000, 4000, 5000)
UAV_NUMBERS = (5, 10, 15, 20)
HARD_RATIOS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)


def _normalize_batch_date(batch_date: str | date | None) -> str:
    """把可选日期转换为严格的 ``YYYY-MM-DD`` 批次目录名。

    参数:
        batch_date: ``None``、日期对象或 ``YYYY-MM-DD`` 字符串。``None`` 表示
            使用当前本机日期。

    返回:
        规范化后的日期字符串；格式错误时抛出 ``ValueError``。
    """

    if batch_date is None:
        return date.today().isoformat()
    if isinstance(batch_date, date):
        return batch_date.strftime("%Y-%m-%d")

    parsed_date = datetime.strptime(str(batch_date), "%Y-%m-%d").date()
    return parsed_date.isoformat()


def _select_source_files(
    input_dir: Path,
    master_seed: int,
    replicate_count: int,
) -> list[Path]:
    """从排序后的县区位置文件中按主种子无放回选择重复实例。

    参数:
        input_dir: ``generated_user_loc`` 县区 CSV 目录。
        master_seed: 用于县区选择的主随机种子。
        replicate_count: 需要选择的县区数量。

    返回:
        按随机选择顺序排列的县区 CSV 路径列表。
    """

    if not input_dir.is_dir():
        raise FileNotFoundError(f"县区位置目录不存在: {input_dir}")
    if replicate_count <= 0:
        raise ValueError("replicate_count 必须为正整数")

    source_files = sorted(input_dir.glob("*.csv"), key=lambda path: path.name)
    if replicate_count > len(source_files):
        raise ValueError(
            f"需要 {replicate_count} 个县区，但 {input_dir} 中只有 "
            f"{len(source_files)} 个 CSV"
        )

    selection_rng = np.random.default_rng(master_seed)
    selected_indices = selection_rng.choice(
        len(source_files), size=replicate_count, replace=False
    )
    return [source_files[int(index)] for index in selected_indices]


def _create_batch_directories(batch_dir: Path) -> None:
    """一次性建立 ToN 批次所需的最小目录树。

    参数:
        batch_dir: 尚不存在的日期批次根目录。

    返回:
        无。若批次目录已存在则由 ``mkdir`` 抛出错误，避免覆盖旧批次。
    """

    batch_dir.mkdir(parents=True, exist_ok=False)

    for user_number in USER_NUMBERS:
        condition_dir = batch_dir / "variable_user_num" / f"{user_number}u_num"
        (condition_dir / "user_data").mkdir(parents=True)
        (condition_dir / "uav_data").mkdir()

    for uav_number in UAV_NUMBERS:
        (
            batch_dir
            / "variable_uav_num"
            / str(uav_number)
            / "uav_data"
        ).mkdir(parents=True)

    for hard_ratio in HARD_RATIOS:
        ratio_label = str(int(round(hard_ratio * 10)))
        condition_dir = batch_dir / "variable_hard_user_ratio" / ratio_label
        (condition_dir / "user_data").mkdir(parents=True)
        (condition_dir / "uav_data").mkdir()


def _write_user_csv(users: pd.DataFrame, output_path: Path) -> None:
    """把一个已生成用户表写入指定的兼容命名路径。

    参数:
        users: 含现有 8 列 schema 的用户数据。
        output_path: 当前新批次中的目标 CSV 路径。

    返回:
        无。目标目录由批次初始化阶段预先建立。
    """

    users.to_csv(output_path, index=False)


def _deploy_uav_directory(
    user_dir: Path,
    uav_dir: Path,
    uav_number: int,
) -> None:
    """调用历史 UAV 部署器处理一个用户条件目录。

    参数:
        user_dir: 含用户实例 CSV 的目录。
        uav_dir: 对应 UAV CSV 输出目录。
        uav_number: 本条件要部署的 UAV 数量。

    返回:
        无。显式覆盖历史类的 UAV 数量和带宽默认值。
    """

    deployment = UAVDeployment(str(user_dir), str(uav_dir))
    deployment.num_uavs = uav_number
    deployment.bandwidth = DEFAULT_UAV_BANDWIDTH
    deployment.process_all_files()


def _copy_uav_files(source_dir: Path, target_dirs: list[Path]) -> None:
    """逐文件复用已经计算的 3000 用户、10-UAV 部署结果。

    参数:
        source_dir: EXP1 的 3000 用户、10-UAV 输出目录。
        target_dirs: EXP2 和各 EXP3 条件的目标 UAV 目录。

    返回:
        无。每个 CSV 使用明确的源、目标路径复制，不执行目录级复制或删除。
    """

    source_files = sorted(source_dir.glob("*_10uavs_loc_*.csv"))
    if not source_files:
        raise RuntimeError(f"没有可复用的 10-UAV 文件: {source_dir}")

    for target_dir in target_dirs:
        for source_file in source_files:
            target_file = target_dir / source_file.name
            if target_file.exists():
                raise FileExistsError(f"拒绝覆盖已有 UAV 文件: {target_file}")
            shutil.copy2(source_file, target_file)


def _build_generation_config(
    batch_date: str,
    input_dir: Path,
    master_seed: int,
    replicate_records: list[dict[str, object]],
) -> dict[str, object]:
    """在输出完整性检查通过后构造生成完成记录，不创建逐文件 manifest 或哈希。

    参数:
        batch_date: 当前批次的日期目录名。
        input_dir: 县区位置输入目录。
        master_seed: 县区选择及有效种子公式使用的主种子。
        replicate_records: 重复实例与源文件、行政代码、有效种子的对应表。

    返回:
        可直接序列化为 JSON 的批次配置字典。
    """

    fixed_hard_qos = {
        application: {"r_min_mbps": qos[0], "p_out": qos[1]}
        for application, qos in FIXED_HARD_QOS.items()
    }
    return {
        "batch_date": batch_date,
        "generation_status": "complete",
        "generator_files": [
            "ExperimentsData/DatasetTest/generate_user_data_ToN.py",
            "ExperimentsData/DatasetTest/instance_generator/generate_instances_ToN.py",
            "ExperimentsData/DatasetTest/uav_deployment.py",
        ],
        "uav_deployment_rule": "max_new_coverage_connected_grid_including_zero_gain_v1",
        "location_input_dir": str(input_dir.resolve()),
        "replicate_count": len(replicate_records),
        "randomness": {
            "master_seed": master_seed,
            "effective_seed_formula": "master_seed + replicate_id - 1",
            "rng": "numpy.random.default_rng",
        },
        "environment": {
            "python": platform.python_version(),
            "numpy": np.__version__,
            "pandas": pd.__version__,
        },
        "replicates": replicate_records,
        "experiments": {
            "EXP1": {"user_numbers": list(USER_NUMBERS), "uav_number": 10},
            "EXP2": {"user_number": 3000, "uav_numbers": list(UAV_NUMBERS)},
            "EXP3": {
                "user_number": 3000,
                "uav_number": 10,
                "hard_ratios": list(HARD_RATIOS),
                "uses_scientific_data_2025": False,
            },
            "EXP4": {
                "user_number": 3000,
                "uav_number": 10,
                "bandwidths": [10, 20, 30, 40, 50],
                "reuses_EXP1_files": True,
            },
        },
        "scientific_data_2025": {
            "usage": "EXP1_EXP2_EXP4_only",
            "table_6_session_weights": TABLE_6_SESSION_WEIGHTS,
            "table_3_applications": {
                category: list(applications)
                for category, applications in TABLE_3_APPLICATIONS.items()
            },
            "within_category_sampling": "uniform",
        },
        "study_modeling_rules": {
            "default_table_3_user_type": "elastic",
            "fixed_hard_qos": fixed_hard_qos,
            "game_qos_note": "model_proxy_parameters",
            "instant_message": {
                "applications": ["QQ", "WeChat"],
                "hard_probability": INSTANT_MESSAGE_HARD_PROBABILITY,
                "hard_voice_probability": INSTANT_MESSAGE_VOICE_PROBABILITY,
                "voice_qos": {
                    "r_min_mbps": INSTANT_MESSAGE_VOICE_QOS[0],
                    "p_out": INSTANT_MESSAGE_VOICE_QOS[1],
                },
                "video_qos": {
                    "r_min_mbps": INSTANT_MESSAGE_VIDEO_QOS[0],
                    "p_out": INSTANT_MESSAGE_VIDEO_QOS[1],
                },
            },
            "elastic_requirement_sentinel": [0.0, 0.0],
            "weight_ranges": {"hard": [1, 10], "elastic": [1, 5]},
            "EXP3_hard_profiles": list(EXP3_HARD_PROFILES),
            "EXP3_elastic_profiles": list(EXP3_ELASTIC_PROFILES),
        },
    }


def _validate_generated_directory(
    directory: Path,
    expected_files: set[str],
    expected_columns: list[str],
    expected_rows: int,
) -> None:
    """核对一个条件目录的完整文件集合、CSV 表头和实际行数。

    参数:
        directory: 本次新生成的用户或 UAV 目录。
        expected_files: 由重复实例 ID 和县区代码构造的准确文件名集合。
        expected_columns: 按读取接口顺序排列的列名。
        expected_rows: 每个文件必须具有的用户数或 UAV 数。

    返回:
        无。缺失/多余文件、不可读 CSV 或规模不符时抛错，不修复或覆盖文件。
    """

    actual_files = {path.name for path in directory.glob("*.csv") if path.is_file()}
    if actual_files != expected_files:
        missing = sorted(expected_files - actual_files)
        unexpected = sorted(actual_files - expected_files)
        raise RuntimeError(
            f"生成不完整: {directory}; 缺失文件={missing}; 多余文件={unexpected}"
        )
    for filename in sorted(expected_files):
        path = directory / filename
        try:
            dataframe = pd.read_csv(path, encoding="utf-8")
        except Exception as error:
            raise RuntimeError(f"生成的 CSV 无法读取: {path}: {error}") from error
        if list(dataframe.columns) != expected_columns or len(dataframe) != expected_rows:
            raise RuntimeError(
                f"生成的 CSV 表头或实际规模不符: {path}; "
                f"期望列={expected_columns}, 行数={expected_rows}; "
                f"实际列={list(dataframe.columns)}, 行数={len(dataframe)}"
            )


def _validate_generated_batch(
    batch_dir: Path,
    replicate_records: list[dict[str, object]],
) -> None:
    """一次检查 EXP1--EXP4 所需输入，防止部署器仅打印错误后仍宣告批次成功。

    参数:
        batch_dir: 当前已写入用户/UAV CSV、尚未写完成配置的批次目录。
        replicate_records: 按 ID 1--N 排列的县区记录，用于检查准确的配对文件名。

    返回:
        无。全部条件通过才返回；任何错误向上传播且不创建完成配置。
    """

    ids = [record["replicate_id"] for record in replicate_records]
    if not ids or ids != list(range(1, len(ids) + 1)):
        raise RuntimeError("生成记录必须包含连续的重复实例 ID 1--N")
    uav_columns = ["uav_id", "longitude", "latitude", "bandwidth"]
    directories = []
    for user_number in USER_NUMBERS:
        root = batch_dir / "variable_user_num" / f"{user_number}u_num"
        directories.append((root / "user_data", f"{user_number}users_data", user_number, USER_COLUMNS))
        directories.append((root / "uav_data", "10uavs_loc", 10, uav_columns))
    for uav_number in UAV_NUMBERS:
        root = batch_dir / "variable_uav_num" / str(uav_number)
        directories.append((root / "uav_data", f"{uav_number}uavs_loc", uav_number, uav_columns))
    for ratio in HARD_RATIOS:
        root = batch_dir / "variable_hard_user_ratio" / str(int(round(ratio * 10)))
        directories.append((root / "user_data", "3000users_data", 3000, USER_COLUMNS))
        directories.append((root / "uav_data", "10uavs_loc", 10, uav_columns))
    # EXP2/EXP4 共用的用户，以及 EXP4 共用的 UAV，已在 EXP1 的 3000-user 条件中检查。
    for directory, pattern, expected_rows, columns in directories:
        expected_files = {
            f"{record['replicate_id']}_{pattern}_{record['adcode']}.csv"
            for record in replicate_records
        }
        _validate_generated_directory(directory, expected_files, list(columns), expected_rows)


def _write_generation_config(
    batch_dir: Path,
    config: dict[str, object],
) -> None:
    """在完整性检查通过后以 UTF-8 JSON 写入简短完成记录，关闭文件后返回。

    参数:
        batch_dir: 当前日期批次根目录。
        config: ``_build_generation_config`` 返回的配置字典。

    返回:
        无。配置写入 ``generation_config_ToN.json``。
    """

    config_path = batch_dir / "generation_config_ToN.json"
    with config_path.open("w", encoding="utf-8", newline="\n") as config_file:
        json.dump(config, config_file, ensure_ascii=False, indent=2)
        config_file.write("\n")


def generate_all_instances(
    batch_date: str | date | None = None,
    master_seed: int = DEFAULT_MASTER_SEED,
    replicate_count: int = DEFAULT_REPLICATE_COUNT,
) -> Path:
    """生成一整批 ToN 用户/UAV 实例并返回日期批次目录。

    参数:
        batch_date: 可选的 ``YYYY-MM-DD`` 批次日期；默认使用当前日期。
        master_seed: 默认 20260904 的主随机种子。
        replicate_count: 无放回选择的县区重复实例数，默认 30。

    返回:
        新建的 ``ExperimentsData/data_ToN/YYYY-MM-DD`` 绝对路径。

    注意:
        该函数会执行耗时的 UAV 网格部署。若同名日期目录已经存在，它会在
        写任何批次文件前失败，且不会覆盖或清理已有数据。全部输出检查通过后
        才写入 generation_status=complete 的配置；失败保留现场，不支持追加生成。
    """

    normalized_date = _normalize_batch_date(batch_date)
    batch_dir = DEFAULT_OUTPUT_ROOT / normalized_date
    if batch_dir.exists():
        raise FileExistsError(f"日期批次已经存在，拒绝覆盖: {batch_dir}")

    selected_sources = _select_source_files(
        DEFAULT_LOCATION_DIR, master_seed, replicate_count
    )
    _create_batch_directories(batch_dir)

    replicate_records: list[dict[str, object]] = []
    for replicate_id, source_file in enumerate(selected_sources, start=1):
        effective_seed = master_seed + replicate_id - 1
        rng = np.random.default_rng(effective_seed)
        adcode = source_file.stem

        location_pool = load_location_pool(source_file, rng, pool_size=5000)
        ton_users = generate_ton_users(location_pool, rng)
        for user_number in USER_NUMBERS:
            user_file = (
                batch_dir
                / "variable_user_num"
                / f"{user_number}u_num"
                / "user_data"
                / f"{replicate_id}_{user_number}users_data_{adcode}.csv"
            )
            _write_user_csv(ton_users.iloc[:user_number].copy(), user_file)

        exp3_sets = generate_exp3_user_sets(location_pool, rng, HARD_RATIOS)
        for hard_ratio, exp3_users in exp3_sets.items():
            ratio_label = str(int(round(hard_ratio * 10)))
            user_file = (
                batch_dir
                / "variable_hard_user_ratio"
                / ratio_label
                / "user_data"
                / f"{replicate_id}_3000users_data_{adcode}.csv"
            )
            _write_user_csv(exp3_users, user_file)

        replicate_records.append(
            {
                "replicate_id": replicate_id,
                "source_file": source_file.name,
                "adcode": adcode,
                "effective_seed": effective_seed,
            }
        )

    # EXP1 分别生成 10-UAV 数据，其中 3000 用户条件只计算一次。
    for user_number in USER_NUMBERS:
        condition_dir = batch_dir / "variable_user_num" / f"{user_number}u_num"
        _deploy_uav_directory(
            condition_dir / "user_data", condition_dir / "uav_data", 10
        )

    exp1_3000_dir = batch_dir / "variable_user_num" / "3000u_num"
    exp2_10_uav_dir = batch_dir / "variable_uav_num" / "10" / "uav_data"
    exp3_uav_dirs = [
        batch_dir
        / "variable_hard_user_ratio"
        / str(int(round(hard_ratio * 10)))
        / "uav_data"
        for hard_ratio in HARD_RATIOS
    ]
    _copy_uav_files(
        exp1_3000_dir / "uav_data", [exp2_10_uav_dir, *exp3_uav_dirs]
    )

    # EXP2 的其余 UAV 数复用相同的 3000 用户输入，但重新计算部署位置。
    for uav_number in (5, 15, 20):
        _deploy_uav_directory(
            exp1_3000_dir / "user_data",
            batch_dir / "variable_uav_num" / str(uav_number) / "uav_data",
            uav_number,
        )

    _validate_generated_batch(batch_dir, replicate_records)
    config = _build_generation_config(
        normalized_date, DEFAULT_LOCATION_DIR, master_seed, replicate_records
    )
    _write_generation_config(batch_dir, config)
    return batch_dir.resolve()


def main() -> None:
    """手动生成固定 2026-09-07 批次的 30 个输入，供 C++ 分两阶段计算。

    参数:
        无。

    返回:
        无。完整性检查与配置写入成功后打印批次路径；失败不打印完成提示。
    """

    # 与 C++ main.cpp 的 input_root 保持同一批次；以后扩算 ID 11--30 时不要重新生成。
    batch_date = "2026-09-07"
    master_seed = 20260904
    replicate_count = 30
    output_dir = generate_all_instances(
        batch_date=batch_date,
        master_seed=master_seed,
        replicate_count=replicate_count,
    )
    print(f"ToN 实验实例生成完成: {output_dir}")


if __name__ == "__main__":
    main()

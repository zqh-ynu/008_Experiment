"""新五组ToN实验的输入构造入口：固定50个空间来源，生成多级请求并复用UAV部署。

直接运行本文件才生成正式输入；导入仅提供函数。新批次拒绝覆盖，旧输入/结果不变。
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import numpy as np
import pandas as pd

GENERATOR_DIR = Path(__file__).resolve().parent
DATASET_DIR = GENERATOR_DIR.parent
DATA_DIR = DATASET_DIR.parent
if str(DATASET_DIR) not in sys.path:
    sys.path.insert(0, str(DATASET_DIR))

from generate_user_data_ToN import (
    LOCATION_COLUMNS, MULTI_USER_COLUMNS, generate_multi_controlled,
    generate_multi_real, load_location_pool,
    EXP3_HARD_PROFILES, EXP3_ELASTIC_PROFILES, TABLE_6_SESSION_WEIGHTS,
)
from uav_deployment import UAVDeployment

DEFAULT_MASTER_SEED = 20260904
DEFAULT_REPLICATE_COUNT = 50
DEFAULT_OUTPUT_ROOT = DATA_DIR / "data_ToN_multiHard"
DEFAULT_LEGACY_ROOT = DATA_DIR / "data_ToN" / "2026-09-07"
DEFAULT_LOCATION_DIR = DATASET_DIR / "data" / "generated_user_loc"
USER_NUMBERS = (1000, 2000, 3000, 4000, 5000)
UAV_NUMBERS = (5, 10, 15, 20)
HARD_RATIOS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0)
INPUT_SCHEMA = "ton-multihard-input-v1"


def multi_conditions() -> list[tuple[str, str, int, int]]:
    """返回五实验的(目录名,条件键,用户数,UAV数)，顺序同时用于确定部署复用来源。"""
    return (
        [("EXP1_user_num", str(n), n, 10) for n in USER_NUMBERS]
        + [("EXP2_uav_num", str(k), 3000, k) for k in UAV_NUMBERS]
        + [("EXP3_hard_ratio", str(int(round(r * 10))), 3000, 10) for r in HARD_RATIOS]
        + [("EXP4_real_user_num", str(n), n, 10) for n in USER_NUMBERS]
        + [("EXP5_real_uav_num", str(k), 3000, k) for k in UAV_NUMBERS]
    )


def plan_multi_sources(
    master_seed: int = DEFAULT_MASTER_SEED,
    legacy_root: Path = DEFAULT_LEGACY_ROOT,
    location_dir: Path = DEFAULT_LOCATION_DIR,
) -> list[dict]:
    """只读确定全部50个来源；前30保留旧5000用户文件顺序，后20按固定种子无放回选择。"""
    legacy_root, location_dir = Path(legacy_root), Path(location_dir)
    with (legacy_root / "generation_config_ToN.json").open(encoding="utf-8") as stream:
        previous = json.load(stream)
    records = previous["replicates"]
    if len(records) != 30 or [r["replicate_id"] for r in records] != list(range(1, 31)):
        raise ValueError("旧批次必须提供连续1—30实例记录")
    selected = []
    excluded = set()
    for record in records:
        i, adcode = record["replicate_id"], str(record["adcode"])
        source = legacy_root / "variable_user_num" / "5000u_num" / "user_data" / f"{i}_5000users_data_{adcode}.csv"
        if not source.is_file():
            raise FileNotFoundError(source)
        excluded.add(adcode)
        selected.append({"replicate_id": i, "adcode": adcode,
                         "source_kind": "legacy_5000_users", "source_path": str(source.resolve())})
    if len(excluded) != 30:
        raise ValueError("旧30实例县区重复")
    available = sorted((p for p in location_dir.glob("*.csv") if p.stem not in excluded), key=lambda p: p.name)
    if len(available) < 20:
        raise ValueError("可补充县区少于20个")
    # 来源抽样使用单独随机流，不消耗请求或位置抽样的随机数。
    rng = np.random.default_rng(np.random.SeedSequence([master_seed, 0]))
    for i, index in enumerate(rng.choice(len(available), 20, replace=False), start=31):
        source = available[int(index)]
        selected.append({"replicate_id": i, "adcode": source.stem,
                         "source_kind": "location_pool", "source_path": str(source.resolve())})
    return selected


def read_multi_locations(record: dict, master_seed: int) -> pd.DataFrame:
    """按来源记录读取5000空间行；旧实例绝不重排，新实例使用独立位置随机流。"""
    source = Path(record["source_path"])
    if record["source_kind"] == "legacy_5000_users":
        table = pd.read_csv(source, encoding="utf-8", dtype={"user_id": str}, usecols=LOCATION_COLUMNS)
        if len(table) != 5000:
            raise ValueError(f"旧空间实例不是5000用户: {source}")
    else:
        rng = np.random.default_rng(np.random.SeedSequence([master_seed, record["replicate_id"], 0]))
        table = load_location_pool(source, rng, pool_size=5000)
    if table["user_id"].isna().any() or table["user_id"].duplicated().any():
        raise ValueError(f"源用户ID缺失或重复: {source}")
    if not np.isfinite(table[["longitude", "latitude"]].to_numpy(dtype=float)).all():
        raise ValueError(f"源坐标非有限值: {source}")
    return table[LOCATION_COLUMNS].reset_index(drop=True)


def validate_multi_profiles(profiles: dict) -> str:
    """核对生成器所需的四种配置，返回版本号；等级数与速率由应用表决定。"""
    version = profiles["config_version"]
    if not isinstance(version, str) or not version:
        raise ValueError("应用配置缺少版本")
    for key in ("voice", "video", "remote_control"):
        item = profiles["profiles"][key]
        rates = np.asarray(item["rate_thresholds_mbps"], dtype=float)
        if item["user_type"] != "hard" or not 0 < item["p_out"] < 1:
            raise ValueError(f"非法hard配置: {key}")
        if rates.ndim != 1 or not len(rates) or not np.isfinite(rates).all() or not (rates > 0).all() or not (np.diff(rates) > 0).all():
            raise ValueError(f"非法等级列表: {key}")
    if profiles["profiles"]["elastic"]["user_type"] != "elastic":
        raise ValueError("非法elastic配置")
    return version


def _write_json(path: Path, value: dict) -> None:
    """将当前新批次的配置/状态写为UTF-8 JSON；错误向上传播，不碰旧批次。"""
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def generate_all_instances(
    batch_name: str = "run_01",
    master_seed: int = DEFAULT_MASTER_SEED,
    replicate_count: int = DEFAULT_REPLICATE_COUNT,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
    legacy_root: Path = DEFAULT_LEGACY_ROOT,
    location_dir: Path = DEFAULT_LOCATION_DIR,
) -> Path:
    """构造新批次并返回其目录；正式默认50个，较小replicate_count仅用于隔离局部检查。

    output_root/legacy_root/location_dir可指向隔离夹具。会运行部署器，不能用于只读来源检查；
    正式运行由用户手动启动。本函数拒绝已有批次，不追加、不清理失败现场。
    """
    if not batch_name or any(c not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-" for c in batch_name):
        raise ValueError("批次名只允许字母、数字、下划线和连字符")
    if not 1 <= replicate_count <= 50:
        raise ValueError("replicate_count必须位于1—50")
    batch = Path(output_root) / batch_name
    if batch.exists():
        raise FileExistsError(f"新批次已存在，拒绝覆盖: {batch}")
    # 来源始终先确定50个；数量较小不会改变前缀。正式配置使用全部50个。
    sources = plan_multi_sources(master_seed, legacy_root, location_dir)[:replicate_count]
    with (DATASET_DIR / "application_profiles.json").open(encoding="utf-8") as stream:
        profiles = json.load(stream)
    version = validate_multi_profiles(profiles)
    with (DATA_DIR / "ExperimentsResults" / "EXP1_user_num" / "def_config.json").open(encoding="utf-8") as stream:
        physical = json.load(stream)
    physical["uav_bandwidth_mhz"] = 40.0

    batch.mkdir(parents=True, exist_ok=False)
    _write_json(batch / "application_profiles.json", profiles)
    _write_json(batch / "physical_config.json", physical)
    info = {
        "input_schema": INPUT_SCHEMA, "generation_status": "generating",
        "batch_name": batch_name, "replicate_count": replicate_count,
        "master_seed": master_seed, "replicates": sources,
        "randomness": "SeedSequence([master_seed, instance_id, stream]); 0=locations,1=controlled,2=real",
        "request_rules": {
            "controlled_hard_probabilities": [p["probability"] for p in EXP3_HARD_PROFILES],
            "controlled_hard_profile_order": ["voice", "video", "remote_control"],
            "controlled_elastic_profiles": list(EXP3_ELASTIC_PROFILES),
            "real_category_session_counts": TABLE_6_SESSION_WEIGHTS,
            "within_category_sampling": "uniform",
            "real_hard_rule": "all_video_all_game_and_20_percent_instant_message",
            "instant_message_hard_voice_probability": 0.5,
            "integer_weight_ranges": {"hard": [1, 10], "elastic": [1, 5]},
        },
        "conditions": [{"experiment": e, "key": key, "user_count": n, "uav_count": k}
                       for e, key, n, k in multi_conditions()],
        "deployment_reuse": "same spatial rows and (user_count,uav_count) reuse one deployment",
    }
    _write_json(batch / "generation_config_ToN.json", info)

    for record in sources:
        i, adcode = record["replicate_id"], record["adcode"]
        locations = read_multi_locations(record, master_seed)
        controlled, ratios = generate_multi_controlled(
            locations, np.random.default_rng(np.random.SeedSequence([master_seed, i, 1])), version)
        real = generate_multi_real(
            locations, np.random.default_rng(np.random.SeedSequence([master_seed, i, 2])), version)
        deployments = {}  # 当前实例的(n,K)->首次生成路径，主体/真实请求共享空间顺序。
        for experiment, key, n, k in multi_conditions():
            root = batch / experiment / key
            user_dir, uav_dir = root / "user_data", root / "uav_data"
            user_dir.mkdir(parents=True, exist_ok=True)
            uav_dir.mkdir(parents=True, exist_ok=True)
            table = (ratios[int(key) / 10] if experiment == "EXP3_hard_ratio"
                     else real.iloc[:n] if experiment.startswith(("EXP4_", "EXP5_"))
                     else controlled.iloc[:n])
            if len(table) != n or list(table.columns) != MULTI_USER_COLUMNS:
                raise ValueError(f"请求表结构错误: {experiment}/{key}/ID{i}")
            user_file = user_dir / f"{i}_{n}users_data_{adcode}.csv"
            table.to_csv(user_file, index=False, encoding="utf-8")
            uav_file = uav_dir / f"{i}_{k}uavs_loc_{adcode}.csv"
            if (n, k) in deployments:
                shutil.copyfile(deployments[(n, k)], uav_file)
            else:
                # 逐文件直接调用，异常不被process_all_files吞掉；每种几何条件只部署一次。
                deployment = UAVDeployment(str(user_dir), str(uav_dir))
                deployment.num_uavs = k
                deployment.bandwidth = physical["uav_bandwidth_mhz"]
                deployment.uav_height = physical["uav_constants"]["uav_alt"]
                deployment.coverage_radius = physical["environmental_constants"]["max_coverage_distance"]
                deployment.process_file(str(user_file))
                uav_table = pd.read_csv(uav_file, encoding="utf-8")
                if list(uav_table.columns) != ["uav_id", "longitude", "latitude", "bandwidth"] or len(uav_table) != k:
                    raise ValueError(f"部署输出不完整: {uav_file}")
                if not (uav_table["bandwidth"] == physical["uav_bandwidth_mhz"]).all():
                    raise ValueError(f"部署带宽不一致: {uav_file}")
                deployments[(n, k)] = uav_file
    info["generation_status"] = "complete"
    _write_json(batch / "generation_config_ToN.json", info)
    return batch.resolve()


def main() -> None:
    """手动入口：一次准备50实例，供C++先运行前10个；不覆盖已有run_01。"""
    output = generate_all_instances(batch_name="run_01", master_seed=20260904, replicate_count=50)
    print(f"新五组实验输入生成完成: {output}")


if __name__ == "__main__":
    main()

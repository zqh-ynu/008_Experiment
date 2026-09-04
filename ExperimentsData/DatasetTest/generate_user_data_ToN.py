"""为 ToN 扩展实验生成用户业务、类型、QoS 和权重属性。

本文件保留现有用户 CSV schema，并把三类信息明确分开：ScientificData2025
Table 3/6 提供的应用标签与 session 权重、本研究定义的 Hard/Elastic 映射，
以及 EXP3 为控制 Hard 用户比例而沿用的历史 QoS 配置。本模块只提供数据生成
函数；导入模块不会创建文件或执行完整实例生成。
"""

from __future__ import annotations

from math import floor
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


LOCATION_COLUMNS = ["user_id", "latitude", "longitude"]
USER_COLUMNS = [
    "user_id",
    "longitude",
    "latitude",
    "user_type",
    "user_weight",
    "user_requirement_1",
    "user_requirement_2",
    "app_category",
]

# ScientificData2025 Table 6 中 Our datasets 一列的原始 session 数。
TABLE_6_SESSION_WEIGHTS = {
    "network-storage": 37236,
    "network-transmission": 57827,
    "video": 11525,
    "game": 14434,
    "instant-message": 57234,
    "web-browsing": 74142,
    "mail-service": 73224,
}

# ScientificData2025 Table 3 中的 7 个大类和 25 个应用实例标签。
TABLE_3_APPLICATIONS = {
    "network-storage": (
        "Baidu Netdisk",
        "Tianyi Cloud Disk",
        "Alibaba Cloud",
        "Hua Weiyun",
    ),
    "network-transmission": ("Thunderbolt", "BT Download", "Emule"),
    "video": (
        "Station B",
        "Tiktok",
        "Tencent Video",
        "IQiyi Video",
        "Youku Video",
        "Mango TV",
        "Tencent Meeting",
    ),
    "game": ("League of Legends", "Honor of Kings"),
    "instant-message": ("QQ", "WeChat"),
    "web-browsing": (
        "Alipay webpage",
        "Today’s Headlines",
        "Zhihu",
        "Baidu Baike",
    ),
    "mail-service": ("163 email", "QQ email", "189 email"),
}

SCIENTIFIC_DATA_APPLICATIONS = tuple(
    application
    for applications in TABLE_3_APPLICATIONS.values()
    for application in applications
)

# 这些类型和 QoS 值是本研究的建模设定，不是 ScientificData2025 的观测值。
FIXED_HARD_QOS = {
    "Tencent Meeting": (1.2, 1e-3),
    # 游戏的可靠速率与 outage 数值只是当前模型可表达的代理参数。
    "League of Legends": (0.08, 1e-3),
    "Honor of Kings": (0.08, 1e-3),
}
INSTANT_MESSAGE_APPS = {"QQ", "WeChat"}
INSTANT_MESSAGE_HARD_PROBABILITY = 0.2
INSTANT_MESSAGE_VOICE_PROBABILITY = 0.5
INSTANT_MESSAGE_VOICE_QOS = (0.032, 1e-2)
INSTANT_MESSAGE_VIDEO_QOS = (1.2, 1e-3)

# EXP3 不使用 ScientificData2025，继续采用原比例实验的业务和 QoS 配置。
EXP3_HARD_PROFILES = (
    {"name": "Voice_Call", "probability": 0.3, "r_min": 0.032, "p_out": 0.01},
    {"name": "Video_Call", "probability": 0.6, "r_min": 1.2, "p_out": 0.001},
    {"name": "Remote_Control", "probability": 0.1, "r_min": 0.08, "p_out": 1e-5},
)
EXP3_ELASTIC_PROFILES = (
    {
        "name": "Text_Msg",
        "probability": 0.5,
        "r_min_range": (0.001, 0.002),
        "r_max_range": (0.008, 0.016),
    },
    {
        "name": "Email",
        "probability": 0.1,
        "r_min_range": (0.016, 0.032),
        "r_max_range": (0.256, 0.512),
    },
    {
        "name": "Offline_Map",
        "probability": 0.1,
        "r_min_range": (0.064, 0.128),
        "r_max_range": (1.0, 2.0),
    },
    {
        "name": "Social_Media",
        "probability": 0.2,
        "r_min_range": (0.032, 0.064),
        "r_max_range": (0.512, 1.0),
    },
    {
        "name": "Sensor_Log",
        "probability": 0.1,
        "r_min_range": (0.0005, 0.001),
        "r_max_range": (0.01, 0.02),
    },
)


def _require_location_columns(dataframe: pd.DataFrame, source_name: str) -> None:
    """检查位置表必需列；缺列时给出简短且可定位的错误信息。

    参数:
        dataframe: 待检查的位置数据。
        source_name: 用于错误信息的输入名称。

    返回:
        无。字段完整时正常返回，否则抛出 ``ValueError``。
    """

    missing_columns = [
        column for column in LOCATION_COLUMNS if column not in dataframe.columns
    ]
    if missing_columns:
        raise ValueError(
            f"位置数据 {source_name} 缺少必需列: {', '.join(missing_columns)}"
        )


def load_location_pool(
    source_csv: str | Path,
    rng: np.random.Generator,
    pool_size: int = 5000,
) -> pd.DataFrame:
    """读取一个县区位置文件并生成稳定随机顺序的用户位置池。

    参数:
        source_csv: 含 ``user_id``、``latitude``、``longitude`` 的 CSV 路径。
        rng: 本重复实例唯一使用的 NumPy 随机数生成器。
        pool_size: 需要保留的位置数量，默认 5000。

    返回:
        仅含三列位置字段、已按 ``rng`` 打乱且重置行索引的数据表。原始
        ``user_id`` 保持不变。
    """

    source_path = Path(source_csv)
    if pool_size <= 0:
        raise ValueError("pool_size 必须为正整数")

    location_data = pd.read_csv(source_path)
    _require_location_columns(location_data, str(source_path))
    if len(location_data) < pool_size:
        raise ValueError(
            f"位置数据 {source_path} 只有 {len(location_data)} 行，少于所需的 "
            f"{pool_size} 行"
        )

    selected_indices = rng.permutation(len(location_data))[:pool_size]
    return (
        location_data.iloc[selected_indices][LOCATION_COLUMNS]
        .copy()
        .reset_index(drop=True)
    )


def generate_ton_users(
    location_pool: pd.DataFrame,
    rng: np.random.Generator,
) -> pd.DataFrame:
    """为 EXP1、EXP2 和 EXP4 生成 ScientificData2025-informed 用户属性。

    参数:
        location_pool: 已确定顺序的用户位置池。
        rng: 与该重复实例位置抽样共用的 NumPy 随机数生成器。

    返回:
        保持现有 8 列 CSV schema 的完整用户数据。大类按 Table 6 session
        权重抽样，类内应用按 Table 3 标签均匀抽样；类型和 QoS 再按本研究
        规则赋值。
    """

    _require_location_columns(location_pool, "location_pool")
    users = location_pool[LOCATION_COLUMNS].copy().reset_index(drop=True)
    user_count = len(users)

    categories = tuple(TABLE_6_SESSION_WEIGHTS)
    category_weights = np.asarray(
        [TABLE_6_SESSION_WEIGHTS[category] for category in categories],
        dtype=float,
    )
    category_probabilities = category_weights / category_weights.sum()
    sampled_categories = rng.choice(
        np.asarray(categories, dtype=object),
        size=user_count,
        p=category_probabilities,
    )

    applications = np.empty(user_count, dtype=object)
    for index, category in enumerate(sampled_categories):
        category_applications = TABLE_3_APPLICATIONS[str(category)]
        application_index = int(rng.integers(0, len(category_applications)))
        applications[index] = category_applications[application_index]

    user_types = np.full(user_count, "elastic", dtype=object)
    requirement_1 = np.zeros(user_count, dtype=float)
    requirement_2 = np.zeros(user_count, dtype=float)

    for index, application in enumerate(applications):
        if application in FIXED_HARD_QOS:
            user_types[index] = "hard"
            requirement_1[index], requirement_2[index] = FIXED_HARD_QOS[application]
        elif (
            application in INSTANT_MESSAGE_APPS
            and rng.random() < INSTANT_MESSAGE_HARD_PROBABILITY
        ):
            user_types[index] = "hard"
            if rng.random() < INSTANT_MESSAGE_VOICE_PROBABILITY:
                requirement_1[index], requirement_2[index] = (
                    INSTANT_MESSAGE_VOICE_QOS
                )
            else:
                requirement_1[index], requirement_2[index] = (
                    INSTANT_MESSAGE_VIDEO_QOS
                )

    hard_mask = user_types == "hard"
    user_weights = np.empty(user_count, dtype=int)
    user_weights[hard_mask] = rng.integers(1, 11, size=int(hard_mask.sum()))
    user_weights[~hard_mask] = rng.integers(1, 6, size=int((~hard_mask).sum()))

    users["user_type"] = user_types
    users["user_weight"] = user_weights
    users["user_requirement_1"] = requirement_1
    users["user_requirement_2"] = requirement_2
    users["app_category"] = applications
    return users[USER_COLUMNS]


def generate_exp3_user_sets(
    location_pool: pd.DataFrame,
    rng: np.random.Generator,
    hard_ratios: Iterable[float],
) -> dict[float, pd.DataFrame]:
    """生成 EXP3 的嵌套 Hard 用户比例条件，不使用 ScientificData2025。

    参数:
        location_pool: 已确定顺序的至少 3000 个用户位置。
        rng: 与该重复实例其他抽样共用的 NumPy 随机数生成器。
        hard_ratios: 要生成的 Hard 用户比例，取值须位于 0 到 1。

    返回:
        以浮点比例为键、完整用户数据表为值的字典。所有条件复用同一
        3000 个位置和同一 Hard 排列，因此比例增大时 Hard 集合严格嵌套；
        同一用户在同一类型下的业务、QoS 和权重也保持不变。
    """

    _require_location_columns(location_pool, "location_pool")
    user_count = 3000
    if len(location_pool) < user_count:
        raise ValueError(
            f"EXP3 需要至少 {user_count} 个位置，当前只有 {len(location_pool)} 个"
        )

    ratios = tuple(float(ratio) for ratio in hard_ratios)
    if any(ratio < 0.0 or ratio > 1.0 for ratio in ratios):
        raise ValueError("hard_ratios 中的比例必须位于 0 到 1")

    base_users = location_pool.iloc[:user_count][LOCATION_COLUMNS].copy()
    base_users.reset_index(drop=True, inplace=True)
    hard_order = rng.permutation(user_count)

    hard_probabilities = np.asarray(
        [profile["probability"] for profile in EXP3_HARD_PROFILES], dtype=float
    )
    hard_profile_indices = rng.choice(
        len(EXP3_HARD_PROFILES), size=user_count, p=hard_probabilities
    )
    hard_names = np.asarray(
        [profile["name"] for profile in EXP3_HARD_PROFILES], dtype=object
    )[hard_profile_indices]
    hard_requirement_1 = np.asarray(
        [profile["r_min"] for profile in EXP3_HARD_PROFILES], dtype=float
    )[hard_profile_indices]
    hard_requirement_2 = np.asarray(
        [profile["p_out"] for profile in EXP3_HARD_PROFILES], dtype=float
    )[hard_profile_indices]
    hard_weights = rng.integers(1, 11, size=user_count)

    elastic_probabilities = np.asarray(
        [profile["probability"] for profile in EXP3_ELASTIC_PROFILES], dtype=float
    )
    elastic_profile_indices = rng.choice(
        len(EXP3_ELASTIC_PROFILES), size=user_count, p=elastic_probabilities
    )
    elastic_names = np.asarray(
        [profile["name"] for profile in EXP3_ELASTIC_PROFILES], dtype=object
    )[elastic_profile_indices]
    elastic_r_min_low = np.asarray(
        [profile["r_min_range"][0] for profile in EXP3_ELASTIC_PROFILES],
        dtype=float,
    )[elastic_profile_indices]
    elastic_r_min_high = np.asarray(
        [profile["r_min_range"][1] for profile in EXP3_ELASTIC_PROFILES],
        dtype=float,
    )[elastic_profile_indices]
    elastic_r_max_low = np.asarray(
        [profile["r_max_range"][0] for profile in EXP3_ELASTIC_PROFILES],
        dtype=float,
    )[elastic_profile_indices]
    elastic_r_max_high = np.asarray(
        [profile["r_max_range"][1] for profile in EXP3_ELASTIC_PROFILES],
        dtype=float,
    )[elastic_profile_indices]
    elastic_requirement_1 = np.round(
        rng.uniform(elastic_r_min_low, elastic_r_min_high), 4
    )
    elastic_requirement_2 = np.round(
        rng.uniform(elastic_r_max_low, elastic_r_max_high), 4
    )
    elastic_weights = rng.integers(1, 6, size=user_count)

    generated_sets: dict[float, pd.DataFrame] = {}
    for ratio in ratios:
        hard_count = floor(user_count * ratio)
        hard_mask = np.zeros(user_count, dtype=bool)
        hard_mask[hard_order[:hard_count]] = True

        users = base_users.copy()
        users["user_type"] = np.where(hard_mask, "hard", "elastic")
        users["user_weight"] = np.where(
            hard_mask, hard_weights, elastic_weights
        ).astype(int)
        users["user_requirement_1"] = np.where(
            hard_mask, hard_requirement_1, elastic_requirement_1
        )
        users["user_requirement_2"] = np.where(
            hard_mask, hard_requirement_2, elastic_requirement_2
        )
        users["app_category"] = np.where(
            hard_mask, hard_names, elastic_names
        )
        generated_sets[ratio] = users[USER_COLUMNS]

    return generated_sets

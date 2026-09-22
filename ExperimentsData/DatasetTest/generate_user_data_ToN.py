"""为 ToN 扩展实验生成用户业务、类型、QoS 和权重属性。

本文件保留旧生成函数供历史代码引用，并新增multi-hard请求构造。三类信息明确分开：ScientificData2025
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

# 新用户格式：hard需求只在应用配置中定义；两列旧需求仅供elastic保留原值。
MULTI_USER_COLUMNS = [
    "user_id", "longitude", "latitude", "user_type", "user_weight",
    "user_requirement_1", "user_requirement_2", "app_label", "service_category",
    "profile_id", "config_version",
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


def generate_multi_controlled(
    locations: pd.DataFrame, rng: np.random.Generator, version: str
) -> tuple[pd.DataFrame, dict[float, pd.DataFrame]]:
    """从5000空间前缀构造受控请求及3000用户比例实验；返回主池与各比例表，不写文件。

    locations保留源ID及坐标顺序；rng只用于主体属性，version对应独立应用配置。
    每1000人精确200个hard；转换类型时使用预生成属性，不重新抽样。
    """
    _require_location_columns(locations, "multi-hard locations")
    if len(locations) != 5000 or locations["user_id"].duplicated().any():
        raise ValueError("受控输入必须有5000个唯一用户")
    n = len(locations)
    hard_choice = rng.choice(3, n, p=[p["probability"] for p in EXP3_HARD_PROFILES])
    hard_names = np.array([p["name"] for p in EXP3_HARD_PROFILES], dtype=object)[hard_choice]
    hard_profiles = np.array(["voice", "video", "remote_control"], dtype=object)[hard_choice]
    hard_categories = np.array(["voice", "video", "remote-control"], dtype=object)[hard_choice]
    hard_weights = rng.integers(1, 11, n)
    elastic_choice = rng.choice(len(EXP3_ELASTIC_PROFILES), n,
                                p=[p["probability"] for p in EXP3_ELASTIC_PROFILES])
    elastic_names = np.array([p["name"] for p in EXP3_ELASTIC_PROFILES], dtype=object)[elastic_choice]
    elastic_categories = np.array(["instant-message", "mail-service", "network-storage",
                                    "web-browsing", "network-transmission"], dtype=object)[elastic_choice]
    # 沿用旧受控elastic需求区间和四位小数精度，但在切换类型前一次性生成。
    low = np.array([p["r_min_range"] for p in EXP3_ELASTIC_PROFILES])[elastic_choice]
    high = np.array([p["r_max_range"] for p in EXP3_ELASTIC_PROFILES])[elastic_choice]
    req1 = np.round(rng.uniform(low[:, 0], low[:, 1]), 4)
    req2 = np.round(rng.uniform(high[:, 0], high[:, 1]), 4)
    elastic_weights = rng.integers(1, 6, n)
    hard_mask = np.zeros(n, dtype=bool)
    for start in range(0, n, 1000):
        hard_mask[start + rng.permutation(1000)[:200]] = True

    def assemble(mask: np.ndarray) -> pd.DataFrame:
        """根据mask选择预生成属性；返回新表，不改变坐标和预生成数组。"""
        count = len(mask)
        table = locations.iloc[:count].copy().reset_index(drop=True)
        table["user_type"] = np.where(mask, "hard", "elastic")
        table["user_weight"] = np.where(mask, hard_weights[:count], elastic_weights[:count])
        table["user_requirement_1"] = np.where(mask, "", req1[:count].astype(str))
        table["user_requirement_2"] = np.where(mask, "", req2[:count].astype(str))
        table["app_label"] = np.where(mask, hard_names[:count], elastic_names[:count])
        table["service_category"] = np.where(mask, hard_categories[:count], elastic_categories[:count])
        table["profile_id"] = np.where(mask, hard_profiles[:count], "elastic")
        table["config_version"] = version
        return table[MULTI_USER_COLUMNS]

    # 前600人正好是主体3000用户的hard集合，后续比例只扩张集合。
    order = np.concatenate((rng.permutation(np.flatnonzero(hard_mask[:3000])),
                            rng.permutation(np.flatnonzero(~hard_mask[:3000]))))
    ratio_sets = {}
    for ratio in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        mask = np.zeros(3000, dtype=bool)
        mask[order[:int(round(3000 * ratio))]] = True
        ratio_sets[ratio] = assemble(mask)
    return assemble(hard_mask), ratio_sets


def generate_multi_real(
    locations: pd.DataFrame, rng: np.random.Generator, version: str
) -> pd.DataFrame:
    """按会话计数和类别内均匀应用抽样生成新请求池；返回表，不写文件。

    video/game全部hard；即时消息20%为hard，其中语音视频各半；elastic保留0/0需求约定。
    """
    _require_location_columns(locations, "real-request locations")
    table = locations.copy().reset_index(drop=True)
    categories = list(TABLE_6_SESSION_WEIGHTS)
    weights = np.array(list(TABLE_6_SESSION_WEIGHTS.values()), dtype=float)
    sampled = rng.choice(categories, len(table), p=weights / weights.sum())
    profiles, labels = [], []
    for category in sampled:
        apps = TABLE_3_APPLICATIONS[category]
        labels.append(apps[int(rng.integers(len(apps)))])
        profile = "elastic"
        if category == "video":
            profile = "video"
        elif category == "game":
            profile = "remote_control"
        elif category == "instant-message" and rng.random() < 0.2:
            profile = "voice" if rng.random() < 0.5 else "video"
        profiles.append(profile)
    mask = np.array(profiles) != "elastic"
    user_weights = np.empty(len(table), dtype=int)
    user_weights[mask] = rng.integers(1, 11, mask.sum())
    user_weights[~mask] = rng.integers(1, 6, (~mask).sum())
    table["user_type"] = np.where(mask, "hard", "elastic")
    table["user_weight"] = user_weights
    table["user_requirement_1"] = np.where(mask, "", "0.0")
    table["user_requirement_2"] = np.where(mask, "", "0.0")
    table["app_label"] = labels
    table["service_category"] = sampled
    table["profile_id"] = profiles
    table["config_version"] = version
    return table[MULTI_USER_COLUMNS]


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

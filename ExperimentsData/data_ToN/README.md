# `ExperimentsData/data_ToN`：ToN 扩展实验实例目录

本目录用于保存 ToN 扩展实验的 **real-data-informed synthetic instances**。
用户空间位置来自本机人口密度处理结果；EXP1、EXP2 和 EXP4 的经验业务组成
来自 ScientificData2025 Table 3/6；Hard/Elastic 映射、QoS 和权重则是本研究
明确规定的建模规则。这些来源不是同一批真实用户的联合观测。

固定 `2026-09-07` 批次已于 **2026-09-08** 完成生成和一次定向修复，全部输入结构检查通过。
当前备好 ID 1–30，共 330 个用户 CSV、450 个 UAV CSV；完成配置中的
`generation_status` 为 `complete`。仅 ID 19 的 20-UAV 文件重算，原 15 行及其他 779 个 CSV
保持不变，原文件另存为 `.before_zero_gain_fix.bak`；原因和实际修复记录写在完成配置中。
这是输入生成完成，不代表 C++ 实验已经运行；后续扩算无需重新运行生成器。
生成规则的完整说明见 [DatasetTest README](../DatasetTest/README.md)。

## 日期批次结构

用户手动运行 `DatasetTest/instance_generator/generate_instances_ToN.py` 后，输出到
`main()` 中固定的批次标识 `2026-09-07`；它与 C++ 的 `input_root` 一致，不随续跑日期变化。
通用结构如下（本轮 `YYYY-MM-DD` 为 `2026-09-07`）：

```text
data_ToN/
  README.md
  YYYY-MM-DD/
    generation_config_ToN.json
    variable_user_num/
      1000u_num/{user_data,uav_data}/
      2000u_num/{user_data,uav_data}/
      3000u_num/{user_data,uav_data}/
      4000u_num/{user_data,uav_data}/
      5000u_num/{user_data,uav_data}/
    variable_uav_num/
      5/uav_data/
      10/uav_data/
      15/uav_data/
      20/uav_data/
    variable_hard_user_ratio/
      0/{user_data,uav_data}/
      2/{user_data,uav_data}/
      4/{user_data,uav_data}/
      6/{user_data,uav_data}/
      8/{user_data,uav_data}/
      10/{user_data,uav_data}/
```

默认生成 30 个重复实例。县区文件先按文件名排序，再以
`master_seed=20260904` 无放回选择；第 `replicate_id` 个实例使用：

```text
effective_seed = master_seed + replicate_id - 1
```

同一日期目录不会被自动覆盖或清理。若目录已经存在，生成入口会在写入批次
文件前停止；需要保留失败现场或由维护者逐项处理，而不是自动删除。

生成器不支持原目录追加。一次备好 30 个输入后，C++ 第一轮各条件只计算 ID 1–10；
后续只把 C++ `instance_count` 改为 30，沿用同一批输入及结果目录补算 ID 11–30，
不要重新运行生成器。初次生成需要 Python 3.10+、NumPy 和 pandas。

## 实例关系与文件格式

- EXP1 生成 1000、2000、3000、4000、5000 用户的嵌套前缀及对应 10-UAV 文件。
- EXP2 复用 EXP1 的 3000 用户，生成 5、10、15、20 UAV 条件；10-UAV 文件直接复用。
- EXP3 复用 3000 个位置和固定排列，生成 Hard 比例
  $0,0.2,0.4,0.6,0.8,1.0$；它完全不使用 ScientificData2025。
- EXP4 复用 EXP1 的 3000 用户、10-UAV 文件，由 C++ 运行时覆盖 UAV 带宽，
  因而不另建带宽数据目录。

用户文件名与 schema 保持现有读取端兼容：

```text
<replicate>_<N>users_data_<adcode>.csv

user_id,longitude,latitude,user_type,user_weight,
user_requirement_1,user_requirement_2,app_category
```

UAV 文件名与 schema 为：

```text
<replicate>_<K>uavs_loc_<adcode>.csv

uav_id,longitude,latitude,bandwidth
```

## 批次记录和版本边界

全部生成结束后只做一次输出完整性检查：各输入目录必须准确包含 ID 1–30 与县区代码
对应的文件，表头及实际用户数/UAV 数必须符合条件。部署器只打印错误、文件缺失、
实际 UAV 数少于文件名等情况均会使生成入口抛错，不写完成配置，不打印批次成功。
检查不会修改部署算法、自动重试或修复已有文件。

检查通过后，日期根目录中的 `generation_config_ToN.json` 记录 `generation_status="complete"`、批次日期标识、
生成器文件名、输入位置目录、Python/NumPy/pandas 版本、主种子、有效种子公式、
30 个县区与重复实例的对应关系，以及 EXP1–EXP4 和 Table 3/6 的生成规则。
它不保存逐 CSV manifest、文件哈希、SemVer 或 Git 提交哈希；日期目录是该批次
的版本标识。

`.gitignore` 忽略本目录下所有派生 `*.csv`，但本 README 和小型 JSON 配置可以
正常纳入版本控制。历史 MASS 实例仍位于 [旧 data 目录](../data/README.md)，
两者不会互相覆盖。

当前 [UAVBandwidthAllocation](../../Algorithms/UAVBandwidthAllocation/README.md)
的 `main()` 已显式选择 `ExperimentsData/data_ToN/2026-09-07`，依次调用 EXP1–EXP4。
生成完成配置缺失、状态不完整或目标 ID 输入缺失时直接报错，绝不退回旧 `data`。
四个实验的结果分别保存到对应实验目录的 `ToN_simple/run_ton_01`，不会导入旧 EXP1 结果。
`ExperimentRunOptions.input_root` 的结构体默认值仍保留旧 `data`，仅用于兼容已有调用。

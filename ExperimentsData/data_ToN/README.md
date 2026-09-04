# `ExperimentsData/data_ToN`：ToN 扩展实验实例目录

本目录用于保存 ToN 扩展实验的 **real-data-informed synthetic instances**。
用户空间位置来自本机人口密度处理结果；EXP1、EXP2 和 EXP4 的经验业务组成
来自 ScientificData2025 Table 3/6；Hard/Elastic 映射、QoS 和权重则是本研究
明确规定的建模规则。这些来源不是同一批真实用户的联合观测。

当前只建立了目录说明和生成器，**尚未执行正式批次生成**，因此本目录下没有
`YYYY-MM-DD` 日期批次、用户 CSV、UAV CSV 或 `generation_config_ToN.json`。
生成规则的完整说明见 [DatasetTest README](../DatasetTest/README.md)。

## 日期批次结构

未来显式运行 `DatasetTest/instance_generator/generate_instances_ToN.py` 后，输出按
实际生成日期隔离：

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

每次成功生成后，日期根目录中的 `generation_config_ToN.json` 记录生成日期、
生成器文件名、输入位置目录、Python/NumPy/pandas 版本、主种子、有效种子公式、
30 个县区与重复实例的对应关系，以及 EXP1–EXP4 和 Table 3/6 的生成规则。
它不保存逐 CSV manifest、文件哈希、SemVer 或 Git 提交哈希；日期目录是该批次
的版本标识。

`.gitignore` 忽略本目录下所有派生 `*.csv`，但本 README 和小型 JSON 配置可以
正常纳入版本控制。历史 MASS 实例仍位于 [旧 data 目录](../data/README.md)，
两者不会互相覆盖。

当前 [UAVBandwidthAllocation](../../Algorithms/UAVBandwidthAllocation/README.md)
中的 EXP1–EXP4 仍读取 `ExperimentsData/data`，不会自动使用这里的日期批次。
正式运行 ToN 实验前，应另行明确目标日期并修改或配置 C++ 读取根目录；本次
生成器修改不改变算法端的默认行为。

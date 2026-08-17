# `DatasetTest/data`：本机数据资产说明

本目录保存人口空间数据处理链所需的原始、裁剪、派生和示例数据。本项目的 Git 快照**刻意不提交**这些数据本体；克隆仓库后，本目录通常只包含本说明文件。需要运行相关脚本时，请从本机冻结备份、独立私有数据快照或具有适当许可的原始来源恢复文件，并保持下列相对路径。

这项安排的目标是同步数据处理逻辑，而不是把大规模栅格、人口抽样坐标库或独立示例数据作为普通 Git blob 上传。

## 不随 Git 同步的资产

| 路径或文件模式 | 角色 | 上游来源或生成方式 | 下游消费者 |
| --- | --- | --- | --- |
| `chn_pop_2025_CN_100m_R2025A_v1.tif` | 全国 100 m 人口栅格原始输入 | 需从有许可的原始人口数据来源恢复 | `../cut_map.py` |
| `china_counties_coords.csv` | 县级中心表 | 由 `../get_county_center.py` 查询后写入 | 栅格裁剪和县区选择 |
| `cut_maps_tif/` 与 `cut_maps_tif.zip` | 县级中心附近约 5 km × 5 km 的人口栅格裁剪结果 | `../cut_map.py` 基于全国栅格生成；ZIP 是历史归档 | 基础用户坐标抽样 |
| `generated_user_loc/` 与 `generated_user_loc.zip` | 按人口权重抽样得到的基础用户坐标中间库 | `../user_location/generate_user_coordinates_simple.py` | `../generate_user_data.py` 和实例生成脚本 |
| `yushu_5km_pop.tif`、`yushu_city_pop_2025.tif` | 玉树相关裁剪/可视化示例 | 历史示例资产 | 数据处理或可视化核对 |
| `Autonomous_Medical_Aid_Dataset.csv` | 无人机救援数据可视化示例输入 | 独立示例数据 | `../main.py`；不是 MASS 主实验实例链 |
| `task_offloading_dataset.csv` | 辅助 task-offloading 数据资产 | 历史辅助数据 | 当前主实验链未见直接调用 |

上述列表说明目录内已知的主要资产类型，并不意味着 Git 提供了下载地址、校验和或再分发许可。恢复数据前，应自行确认来源、许可和版本。

## 主数据处理链

MASS 主实验使用的是基于人口空间分布的合成用户坐标，而不是可识别的真实个人位置。可追踪的处理思路如下：

```text
AMap 区县查询
  → china_counties_coords.csv
  → 全国 100 m 人口栅格裁剪
  → cut_maps_tif/<县区>.tif
  → 人口权重抽样的 generated_user_loc/<县区>.csv
  → 生成 hard/elastic 类型、权重、QoS 和业务类别的 user CSV
  → 100 m 网格贪心部署 UAV
  → ../../data/ 中按实验条件组织的配对 user/UAV 实例
```

对应的处理逻辑保留在仓库中：

1. [get_county_center.py](../get_county_center.py) 查询区县中心并生成县级中心表；
2. [cut_map.py](../cut_map.py) 以中心点为基准裁剪人口栅格；
3. [generate_user_coordinates_simple.py](../user_location/generate_user_coordinates_simple.py) 按像元人口权重抽取基础用户坐标，并在像元内加入扰动；
4. [generate_user_data.py](../generate_user_data.py) 从基础坐标构造带 user type、权重、QoS 和应用类别的用户实例；
5. [uav_deployment.py](../uav_deployment.py) 在归一化区域内采用 100 m 网格贪心策略部署 UAV；
6. 最终 user/UAV CSV 应恢复或生成到 [../../data/README.md](../../data/README.md) 所说明的条件目录中，供 `Algorithms/UAVBandwidthAllocation` 读取。

## 环境变量与依赖边界

`get_county_center.py` 不保存第三方 API 密钥。若要执行县级中心查询，需在本机私密环境中设置 `AMAP_API_KEY`，例如在 PowerShell 会话中设置环境变量后再运行脚本。不要把密钥写入源码、README、日志、`.env` 提交文件或 Git 远端。

不同脚本按需使用 `pandas`、`numpy`、`rasterio`、`geopandas`、`shapely`、`requests`、Matplotlib 或 Plotly 等依赖。请在隔离的 Python 环境中，针对实际要运行的脚本安装依赖。

## 恢复与复现限制

- 本 Git 快照只保留处理逻辑与目录说明，**不包含**全国人口栅格、裁剪结果、基础坐标库或独立示例数据；恢复时应将数据放回本目录的原相对路径。
- `generated_user_loc/` 是人口抽样的中间库，不是 C++ 主实验直接读取的最终配对实例；最终主实验输入位于 `ExperimentsData/data/` 的本机/独立数据快照中。
- 历史脚本中仍有旧的绝对路径 `My paper`，而当前工作区为 `My_paper`；运行前需在副本或受控配置中核对路径。
- `get_county_center.py` 按当前工作目录写出 `china_counties_coords.csv`；应从预期目录启动脚本，或在运行后将输出放回本目录，而不能假定脚本会自动写到脚本相邻的 `data/` 目录。
- `generate_user_data.py` 的顶层历史入口与其当前批处理函数签名存在参数不匹配；`uav_deployment.py` 的当前默认 UAV 数为 15，而既有 `variable_user_num` 主实验文件为 10 UAV。它们保留的是处理逻辑和历史入口，运行前需要修正路径、接口和参数，不能宣称可无配置直接重建既有实例。
- 人口抽样、县区选择、用户抽样、QoS 抽样和 shuffle 没有统一固定并记录随机种子；加上部分脚本的接口漂移，当前代码可以说明生成思路，但不能承诺在不恢复冻结数据的情况下逐字节重建历史实例。
- 当前本机可能只保留 `cut_maps_tif.zip`，而某些脚本期望解压后的 `cut_maps_tif/` 目录。恢复或运行前应检查所需的实际目录是否存在。

若要运行 EXP1–EXP4，请先阅读 [../../data/README.md](../../data/README.md)，恢复其中列出的最终实例目录，再运行主 C++ 工程；不要把本说明文件本身误当作可直接执行实验的数据包。

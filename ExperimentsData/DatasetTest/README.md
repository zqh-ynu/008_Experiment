# `DatasetTest`：实验实例数据生成项目说明

`DatasetTest` 是 `008_Experiment` 中用于构造实验输入实例的 Python 项目。它把行政区中心和人口密度栅格转换为用户空间位置，再为用户生成 Hard/Elastic 类型、权重、QoS 需求和业务标签，最后根据用户位置生成 UAV 部署文件。历史 user/UAV 配对实例由 `ExperimentsData/data` 管理；新的 ToN 扩展实例将在 `ExperimentsData/data_ToN/YYYY-MM-DD` 中按日期批次隔离。

> [!IMPORTANT]
> 本目录保留了存在路径和接口漂移的历史生成链，同时新增了 `generate_user_data_ToN.py` 和 `instance_generator/generate_instances_ToN.py`。ScientificData2025 Table 3/6 规则已经接入这两个新文件，但本次没有执行完整生成，因此**现有历史 CSV 未改变，`data_ToN` 中也尚无正式日期批次**。

`main.py` 读取 `Autonomous_Medical_Aid_Dataset.csv` 并生成 Plotly 医疗救援地图动画。它是一个独立的历史可视化示例，**不是**人口驱动实验实例生成链的主入口。

## 数据处理链

当前项目所表达的主处理逻辑如下：

```text
get_county_center.py
  └─ 获取县级行政区中心坐标
      └─ cut_map.py
          └─ 从全国人口密度栅格裁剪县区中心附近的 5 km × 5 km 栅格
              └─ user_location/generate_user_coordinates_simple.py
                  └─ 按人口密度加权抽样用户经纬度
                      ├─ generate_user_data.py（历史入口）
                      │   └─ 按旧 JSON 分配用户属性
                      └─ generate_user_data_ToN.py
                          └─ 按 Table 3/6 与本研究规则分配用户属性
                              └─ instance_generator/generate_instances_ToN.py
                                  └─ 复用 uav_deployment.py 生成 UAV 坐标
                                      └─ ../data_ToN/YYYY-MM-DD/
                                          └─ 按 EXP1–EXP4 条件组织 ToN 实例
```

这条链描述的是各脚本之间的**逻辑关系**，不代表从仓库根目录依次执行这些文件即可成功重建现有实例。原始数据、中间数据、脚本工作目录、配置文件和历史绝对路径仍需单独核对。

## 目录与脚本职责

| 路径 | 当前职责 | 主要输入与输出 |
| --- | --- | --- |
| `get_county_center.py` | 调用高德行政区 API，提取县级行政区名称、行政代码和中心坐标 | 从本机环境变量 `AMAP_API_KEY` 读取凭据；输出县区中心坐标 CSV |
| `cut_map.py` | 围绕县区中心裁剪人口密度栅格 | 输入全国人口密度 TIF 和县区中心 CSV；输出约 $5\,\mathrm{km}\times5\,\mathrm{km}$ 的县区 TIF |
| `user_location/` | 按人口密度对栅格像元加权抽样，并在像元内部加入随机扰动 | 输出带 `user_id`、`latitude`、`longitude` 的基础用户坐标 CSV；另含历史可视化脚本和产物 |
| `generate_user_data.py` | 历史用户属性生成器 | 读取基础用户 CSV 和旧 JSON 条件配置；保留原行为和已知接口漂移 |
| `generate_user_data_ToN.py` | ToN 用户属性生成核心 | 按 Table 3/6 生成 EXP1/2/4 用户，并为 EXP3 生成嵌套 Hard 用户集合；保持现有 8 列 schema |
| `uav_deployment.py` | 将用户位置归一化到固定区域，在网格候选点上贪心部署 UAV | 读取完整用户实例 CSV；输出 UAV 坐标和带宽 CSV |
| `instance_generator/` | 保存历史入口和新的 ToN 批次入口 | `inst1.py`–`inst3.py` 保留历史行为；`generate_instances_ToN.py` 组织配对实例和日期批次 |
| `../data/**/*.json` | 保存正式实验的历史生成条件 | 包含用户数、`hard_ratio`、实例数量和 Hard/Elastic QoS 配置；不是 C++ 主程序直接读取的运行配置 |
| `../data_ToN/` | 保存新的日期化 ToN 批次 | 详细目录、schema 和 Git 边界见 [data_ToN/README.md](../data_ToN/README.md)；当前尚未生成正式批次 |
| `data/` | 保存本机原始/中间数据的目录约定 | 详细资产、恢复和 Git 边界见 [data/README.md](data/README.md) |
| `uav_visualization.py` | 绘制或检查历史 UAV 部署结果 | 不是实例生成的必要阶段 |
| `main.py` | 生成医疗救援数据的 Plotly 地图动画 | 独立示例，不属于上述人口驱动主链 |

## 历史用户数据生成机制

以下内容仅来自历史 `generate_user_data.py` 中 `QoSTrafficGenerator` 的实现。新的 ToN 行为见后文；新增生成器没有改写这个历史文件。

### 1. 用户抽样与类型分配

对于一个基础用户坐标 CSV：

1. 若输入行数不少于配置中的 `n_num`，使用 `pandas.DataFrame.sample` 随机抽取 `n_num` 行；否则使用全部输入行，并把实际用户数降为输入行数。
2. Hard 用户数量按下式截断为整数：

   $$
   n_H=\left\lfloor n\cdot\texttt{hard\_ratio}\right\rfloor.
   $$

3. 构造 `n_H` 个 `hard` 和其余 `elastic` 标签，再通过 `random.shuffle` 随机打乱。
4. Hard 用户权重从整数区间 $[6,10]$ 均匀抽取，Elastic 用户权重从整数区间 $[1,5]$ 均匀抽取。

### 2. 历史业务标签与 QoS 需求

历史生成器**不使用 ScientificData2025 Table 3 或 Table 6**。它先根据用户类型选择 JSON 中的配置列表，再使用每个配置项的 `prob` 作为相对权重，归一化后随机选择一项；所选配置的 `name` 被写入 `app_category`。

| 用户类型 | `user_requirement_1` | `user_requirement_2` | `app_category` 的当前来源 |
| --- | --- | --- | --- |
| `hard` | 从所选 Hard 配置的 `r_min_range` 均匀抽取 `r_min` | 所选配置的固定 `p_out` | `hard_qos_config[*].name` |
| `elastic` | 从所选 Elastic 配置的 `r_min_range` 均匀抽取 `r_min` | 从 `r_max_range` 均匀抽取 `r_max` | `elastic_qos_config[*].name` |

这里的 `prob` 只控制同一用户类型内部的配置项抽样。它不是 ScientificData2025 Table 3/6 的数据，也不能自动解释为真实用户请求比例。

### 3. 用户 CSV schema

完整用户实例的字段顺序为：

```text
user_id,longitude,latitude,user_type,user_weight,
user_requirement_1,user_requirement_2,app_category
```

其中 `user_requirement_2` 的语义依赖 `user_type`：Hard 用户对应 `p_out`，Elastic 用户对应 `r_max`。下游读取或分析时不能脱离 `user_type` 单独解释该列。

## 当前 UAV 部署机制

`uav_deployment.py` 先以第一个用户为经纬度转换参考点，把用户坐标转换为局部米制坐标，再分别按横、纵坐标范围归一化到 $5000\,\mathrm{m}\times5000\,\mathrm{m}$ 区域。当前构造函数中的默认参数为：

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| 区域边长 `area_size` | 5000 m | 归一化后的方形部署区域 |
| 网格边长 `grid_size` | 100 m | 候选点位于每个网格中心 |
| UAV 高度 `uav_height` | 300 m | 被记录为代码参数；当前覆盖选择只计算水平距离 |
| 覆盖半径 `coverage_radius` | 600 m | 判断用户是否位于某 UAV 的覆盖范围内 |
| 连通距离 `connection_distance` | 800 m | 除第一架 UAV 外，新 UAV 必须与至少一架已部署 UAV 连通 |
| UAV 数量 `num_uavs` | 15 | 当前类的默认部署数量 |
| 单 UAV 容量 `capacity` | 5000 个用户 | 新覆盖用户超过容量时优先保留距离最近者 |
| UAV 带宽 `bandwidth` | 40 | 写入输出 UAV CSV 的默认值 |

部署器遍历 100 m 网格中心，在满足连通约束的候选点中，贪心选择能新增覆盖最多用户的位置。第一架 UAV 因尚无已部署节点而不受连通约束；后续 UAV 必须位于任一已部署 UAV 的 800 m 范围内。部署结束后，坐标被反归一化并转换回经纬度。

UAV CSV 的字段为：

```text
uav_id,longitude,latitude,bandwidth
```

## 与 EXP1–EXP4 的关系

正式实验目录和现有条件矩阵以 [../data/README.md](../data/README.md) 为准：

| 实验 | 主要自变量 | 条件 | `_ToN` 生成器中的数据规则 |
| --- | --- | --- | --- |
| EXP1 | 用户数 | 1000、2000、3000、4000、5000；10 架 UAV | 由 Table 6 权重抽样大类、由 Table 3 均匀抽样类内应用；五种用户数取同一 5000 用户池的嵌套前缀 |
| EXP2 | UAV 数 | 固定 3000 用户；5、10、15、20 架 UAV | 复用 EXP1 同一重复实例的 3000 用户；10-UAV 部署结果也直接复用 |
| EXP3 | Hard 用户比例 $\alpha_H$ | 固定 3000 用户、10 架 UAV；$\alpha_H\in\{0,0.2,0.4,0.6,0.8,1.0\}$ | **不使用 ScientificData2025**；按固定排列生成嵌套 Hard 集合，并使用历史受控 QoS 配置 |
| EXP4 | 每架 UAV 的带宽 | 固定 3000 用户、10 架 UAV；带宽为 10、20、30、40、50 | 复用 EXP1 的 3000 用户和 10-UAV 文件，由 C++ 运行时覆盖带宽，不另存一套 CSV |

这些规则已在两个 `_ToN` 文件中实现，但尚未正式运行，所以现有 `ExperimentsData/data` CSV 并不包含新的 Table 3 标签或 Table 6 抽样结果，`data_ToN` 也尚无日期批次。

## ScientificData2025 Table 3/6 的 ToN 接入规则

### 数据来源及可用信息

ScientificData2025 指论文 [*A Real Network Environment Dataset for Traffic Analysis*](https://doi.org/10.1038/s41597-025-04876-2)。本项目只使用论文表格中的汇总信息，不下载或读取完整流量数据集：

- Nature 官方 [Table 3: Service application classification label](https://www.nature.com/articles/s41597-025-04876-2/tables/3) 提供 7 个应用大类及 25 个应用实例标签；
- Nature 官方 [Table 6: Data quality statistical distribution comparison](https://www.nature.com/articles/s41597-025-04876-2/tables/6) 提供各大类的 `session number`；本项目只取其中 **Our datasets** 一列，不使用对照数据集 ISCXVPN2016 的计数。

Table 3 的完整标签清单如下：

| 应用大类 | 应用实例标签 | 数量 |
| --- | --- | ---: |
| `network-storage` | Baidu Netdisk；Tianyi Cloud Disk；Alibaba Cloud；Hua Weiyun | 4 |
| `network-transmission` | Thunderbolt；BT Download；Emule | 3 |
| `video` | Station B；Tiktok；Tencent Video；IQiyi Video；Youku Video；Mango TV；Tencent Meeting | 7 |
| `game` | League of Legends；Honor of Kings | 2 |
| `instant-message` | QQ；WeChat | 2 |
| `web-browsing` | Alipay webpage；Today’s Headlines；Zhihu；Baidu Baike | 4 |
| `mail-service` | 163 email；QQ email；189 email | 3 |
| **合计** | **7 个大类、25 个应用实例标签** | **25** |

Table 6 中用作大类抽样权重的原始 session 计数如下：

| 应用大类 | `session number` 权重 | 归一化占比（仅供阅读） |
| --- | ---: | ---: |
| `network-storage` | 37,236 | 11.4353% |
| `network-transmission` | 57,827 | 17.7589% |
| `video` | 11,525 | 3.5394% |
| `game` | 14,434 | 4.4327% |
| `instant-message` | 57,234 | 17.5768% |
| `web-browsing` | 74,142 | 22.7693% |
| `mail-service` | 73,224 | 22.4874% |
| **合计** | **325,622** | **100%** |

ToN 生成器直接以七个原始 session 计数作为 `Categorical` 相对权重，不使用四舍五入后的百分比。抽中大类后，再在该类的 Table 3 应用实例中均匀抽样。因此，各类内部每个应用的条件概率依次为 $1/4$、$1/3$、$1/7$、$1/2$、$1/2$、$1/4$ 和 $1/3$。

Table 3 是 `app_category` 的受控词表来源；Table 6 反映这批真实网络流量的**经验业务 session 组成**。二者均不直接提供：

- 每个应用实例在其大类内部的真实出现概率；
- 某个区域内真实活跃用户或应用层请求的组成比例；
- 应用实例到 Hard/Elastic 用户类型的映射；
- `r_min`、`r_max`、`p_out`、用户权重或其他 QoS 参数；
- 本项目所需的随机种子与可复现采样规则。

一条 session 不等于一次完整的用户操作，同一次操作也可能产生多条 session。因此，论文和实验说明应称其为“由真实网络 session 计数得到的经验业务组成”或“业务类别抽样代理”，不能称为严格的“真实用户请求比例”。人口空间数据与 ScientificData2025 也不是同一批真实用户的联合观测；最终实例属于 **real-data-informed synthetic instances**，不是完整的真实网络快照。

### 业务任务到 Hard/Elastic 的映射

对 EXP1、EXP2 和 EXP4，先按 Table 6 权重抽样大类，再按 Table 3 在类内均匀抽样应用，并应用以下映射：

| Table 3 应用 | ToN 采用类型 | 规则性质 |
| --- | --- | --- |
| Baidu Netdisk、Tianyi Cloud Disk、Alibaba Cloud、Hua Weiyun | Elastic | 本研究映射 |
| Thunderbolt、BT Download、Emule | Elastic | 本研究映射 |
| Station B、Tiktok、Tencent Video、IQiyi Video、Youku Video、Mango TV | Elastic | 本研究映射 |
| Tencent Meeting | Hard | 本研究映射 |
| League of Legends、Honor of Kings | Hard | 本研究映射；游戏 QoS 使用当前模型可表达的代理参数 |
| QQ、WeChat | 以 0.2 条件概率为 Hard，否则为 Elastic | 本研究概率设定；不是数据集观测值 |
| Alipay webpage、Today’s Headlines、Zhihu、Baidu Baike | Elastic | 本研究映射 |
| 163 email、QQ email、189 email | Elastic | 本研究映射 |

若 QQ 或 WeChat 被判为 Hard，再以 $0.5/0.5$ 的条件概率选择语音或实时视频配置。`0.2` 与 `0.5/0.5` 都是本研究的固定建模假设，不来自 ScientificData2025。

根据 Table 6 大类权重、Table 3 类内均匀抽样和上述映射，非 EXP3 实例的期望 Hard 比例为：

$$
\frac{14434}{325622}
+\frac{11525}{325622}\times\frac{1}{7}
+\frac{57234}{325622}\times0.2
\approx 8.45\%.
$$

该数值是由本研究规则**推导出的期望值**，不是 ScientificData2025 直接报告的 Hard 用户比例。每个有限规模实例中的实际比例会因随机抽样而波动。

### QoS、单位与权重规则

当前研究保持既有两类效用模型，不为了 Table 3 标签扩展新的 Elastic 效用或时延变量。ToN 生成器采用的 Hard 参数如下：

| Hard 任务 | `r_min`（CSV 中为 Mbps） | `p_out` | 参数来源与边界 |
| --- | ---: | ---: | --- |
| Tencent Meeting | 1.2 | $10^{-3}$ | 复用本项目既有实时视频配置；不是 ScientificData2025 测量值 |
| League of Legends | 0.08 | $10^{-3}$ | 本研究为现有可靠速率模型设置的代理参数 |
| Honor of Kings | 0.08 | $10^{-3}$ | 本研究为现有可靠速率模型设置的代理参数 |
| QQ/WeChat Hard 语音分支 | 0.032 | $10^{-2}$ | 复用本项目既有语音通话配置 |
| QQ/WeChat Hard 视频分支 | 1.2 | $10^{-3}$ | 复用本项目既有实时视频配置 |

游戏业务的主要实际 QoS 还涉及时延和分组错误率，而当前优化模型没有时延变量，分组错误率也不能严格等同于服务中断概率。因此，游戏的 `0.08 Mbps` 和 $10^{-3}$ 只能表述为**模型代理参数**，不能表述成 ScientificData2025 或 3GPP 直接给出的游戏 outage 要求。

对 Elastic 用户，ToN 生成器写入：

```text
user_requirement_1 = 0.0
user_requirement_2 = 0.0
```

它们分别占用现有 CSV 中 `r_min` 和 `r_max` 的位置，但语义均为 `not_applicable` 数值哨兵，不表示对应应用的真实速率需求为 0 Mbps。当前 C++ 读取器忽略 Elastic 的 `user_requirement_2`，当前六个正式实验方法的 Elastic 效用也不使用 Elastic `r_min/r_max`，而是使用权重、分配带宽和信道容量计算对数效用。

历史 `local_search_allocation()` 路径中的 `KktBasedElasticUtility()` 会计算 `weight / log2(r_min + 1)`，因此不能直接处理 Elastic `r_min=0`。该历史路径不属于当前六方法正式实验循环；若以后重新启用，必须先为零值哨兵增加显式保护或重新定义其归一化规则。

用户权重采用无量纲整数：

- Hard：从 $\{1,2,\ldots,10\}$ 均匀抽样；
- Elastic：从 $\{1,2,\ldots,5\}$ 均匀抽样。

这是本研究的实验设定，不是 ScientificData2025 或 QoS 标准中的数据。它与当前 `generate_user_data.py` 的实际行为不同：现有代码仍对 Hard 使用 $[6,10]$、对 Elastic 使用 $[1,5]$。CSV 中的速率统一使用 Mbps；C++ 读取器通过 `unit_para=1000` 转为内部 Kbps。`p_out` 是无量纲的目标中断概率。

### 随机种子与条件配对

ToN 生成器统一使用 NumPy 随机数生成器，不再混用 Python `random`、NumPy 全局随机状态和 pandas 的独立抽样：

```python
master_seed = 20260904
effective_seed = master_seed + replicate_id - 1  # replicate_id 从 1 开始
rng = np.random.default_rng(effective_seed)
```

同一重复实例内的用户位置/顺序、Table 6 大类、Table 3 类内应用、QQ/WeChat 分支和用户权重均由同一个 `rng` 产生。pandas 只按 `rng` 生成的索引取行，不再自行产生随机索引。记录一个重复实例时，至少保留 `generation_date`、`master_seed`、`replicate_id`、`effective_seed` 和 `rng=numpy.random.default_rng`。

不同实验条件采用配对且嵌套的实例关系：

1. 每个 `replicate_id` 先生成一个有稳定随机顺序的 5000 用户基础池；
2. EXP1 的 1000、2000、3000、4000 和 5000 用户条件依次取该池的前缀，从而使小规模用户集合成为大规模集合的子集；
3. EXP2 和 EXP4 复用同一 `replicate_id` 下的 3000 用户前缀，只改变各自的实验自变量；
4. EXP3 复用同一 `replicate_id` 下的 3000 个位置与固定随机排列，并令排列前 $\lfloor n\alpha_H\rfloor$ 个用户为 Hard、其余为 Elastic，使不同 $\alpha_H$ 条件的 Hard 集合嵌套；EXP3 不执行 Table 3/6 抽样，QoS 仍由 EXP3 的受控生成配置决定。

这个简化方案不再为各抽样阶段派生子种子。只要随机调用顺序或 NumPy 版本发生变化，即使 `effective_seed` 相同，输出也可能改变；因此日期批次仍应与实际生成代码和环境配套解释，不能把种子本身当成完整版本。

### 日期版本标识

生成器、输入数据、配置和输出实例不使用独立 SemVer，也不做文件哈希验证。一次正式生成批次统一放入实际生成日期命名的文件夹：

```text
ExperimentsData/data_ToN/YYYY-MM-DD/
```

该日期文件夹是本批次唯一的版本标识；实验说明或日志引用实例时应同时记录日期文件夹和 `replicate_id`。同一日期目录中的生成器、输入数据、配置和输出实例视为一组，不覆盖既有日期批次。日期只表示生成批次，不表示 ScientificData2025 的官方数据版本。

### 实现状态与实验边界

上述 Table 3/6 抽样、业务映射、QoS/权重、NumPy-only 种子、条件配对和日期目录已经在 `generate_user_data_ToN.py` 与 `instance_generator/generate_instances_ToN.py` 中实现。新模块在导入时没有生成副作用，同名日期目录存在时拒绝覆盖；一次成功正式生成才会在日期根目录写入 `generation_config_ToN.json`。

本次仅实现和轻量验证代码，**没有调用完整入口，也没有运行 UAV 批处理或 C++ 实验**。因此“代码已实现”不等于“正式实例已经生成”。EXP1、EXP2 和 EXP4 使用上述 ScientificData2025 规则；EXP3 始终排除 ScientificData2025，并继续把 $\alpha_H$ 作为受控自变量。

## 当前复现限制

以下限制针对保留的历史入口；新 `_ToN` 入口没有修改或掩盖这些旧问题：

- `generate_user_data.py` 和 `instance_generator/inst1.py` 中仍有 `E:\Research\My paper\...` 形式的旧绝对路径，与当前工作区的 `My_paper` 不一致。
- `generate_user_data.py` 的 `run_batch_simulation()` 当前签名是 `selected_files, output_dir`，但文件底部入口传入了不存在的 `input_dir` 关键字；直接运行会发生接口错误。
- `instance_generator/inst2.py` 和 `inst3.py` 使用 `/home/pc/qinghui/...` Linux 绝对路径，而其他脚本包含 Windows 或依赖当前工作目录的相对路径。
- `inst2.py` 按旧接口向 `UAVDeployment` 传入第三个 `uav_num` 参数，但当前构造函数只接受 `input_dir, output_dir`。
- `inst3.py` 中重新生成不同 Hard 比例用户数据的循环目前被注释；其剩余 UAV 部署流程使用类默认的 15 架 UAV，而正式 EXP3 条件是 10 架 UAV。
- `uav_deployment.py` 当前默认部署 15 架 UAV，与正式实验中常用的 10 架 UAV 不是同一个条件；调用方必须显式确认实验配置。
- 用户坐标抽样、用户类型打乱、应用/QoS 抽样和批量输入文件抽样分别使用 pandas、Python `random` 和 NumPy 随机源，但当前流程没有统一设置或记录随机种子。
- 仓库没有一套经验证、冻结的环境和命令把原始数据逐字节重建为现有正式 CSV；保留 JSON 条件并不等于现有实例可逐字节复现。

这些问题是历史状态说明。新的 `_ToN` 入口已改用相对脚本路径和统一的 NumPy RNG，但尚未经过完整 30 个重复实例和 UAV 部署的正式运行；不能据此声称已有结果已逐字节重现。

## 数据、算法与版本控制边界

- 本机原始/中间数据资产、目录约定和恢复要求见 [data/README.md](data/README.md)。
- EXP1–EXP4 正式 user/UAV 实例、条件配置和文件命名约定见 [../data/README.md](../data/README.md)。
- 新 ToN 日期批次的结构和同步边界见 [../data_ToN/README.md](../data_ToN/README.md)。
- 读取这些实例的 C++ 算法工程见 [../../Algorithms/UAVBandwidthAllocation/README.md](../../Algorithms/UAVBandwidthAllocation/README.md)。
- 当前 C++ EXP1–EXP4 仍硬编码读取 `ExperimentsData/data`，不会自动消费 `data_ToN/YYYY-MM-DD`；正式运行前需另行切换目标批次。
- `.gitignore` 忽略 `DatasetTest/data/**`，但显式保留 `DatasetTest/data/README.md`；`ExperimentsData/data` 和 `data_ToN` 下的派生 CSV 也不随 Git 同步。
- JSON 条件配置和说明文档用于保存生成意图与恢复边界，不能替代未提交的大型/派生数据资产。
- 本顶层 `README.md` 不位于上述忽略范围内，应作为项目文档正常纳入版本控制。

本说明记录历史入口与 ToN 新入口的实际边界；README 本身不会生成、修改或替代任何实验实例。

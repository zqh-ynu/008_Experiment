# 008_Experiment：UAV 混合 QoS 带宽分配实验工作区

> 本文档面向接手本项目的研究者：说明当前实验代码、数据处理逻辑、输入实例、结果、历史原型与 ToN 扩展规划分别位于何处，以及在运行或引用结果前必须核对的边界条件。

本目录是 MASS 会议论文中无人机（UAV）带宽分配问题的实验工作区，也是后续 IEEE/ACM Transactions on Networking（ToN）扩展实验的基础。问题包含两类用户：具有离散/硬 QoS 需求的 hard 用户，以及具有连续效用的 elastic 用户。本机工作区仍保存提出算法、若干基线、人口分布驱动的合成实例、历史结果和辅助作图资产；用于同步的 Git 快照则保留代码、数据处理逻辑、小型生成条件配置、结果和说明文档，并刻意排除大规模原始/中间数据和正式实例文件。

这不是一个开箱即用的完整复现包。README 同时描述本机研究工作区与同步快照的边界；它不能替代后续 P0 资产冻结，也不能证明仅凭远端 Git 提交就能重建当前全部六算法组合和历史输入实例。

## 阅读边界与状态标签

为避免把不同成熟度的材料混在一起，本 README 统一采用下面三类状态。

| 状态 | 含义 | 本 README 中的典型内容 |
| --- | --- | --- |
| **当前主实验链** | 可从当前本机工作树中的代码、数据和结果直接追踪的会议实验资产。同步快照可能按本 README 的数据边界省略大文件。 | 人口栅格驱动实例、<code>UAVBandwidthAllocation</code> 的六方法循环、EXP1–EXP3 输入与结果。 |
| **历史/探索性资产** | 旧版原型、解释图、候选作图资源，或生成版本尚未完全闭环的材料。 | <code>LP_for_SAP</code>、<code>testSearchMethod</code>、参数扫描图、历史结果分支。 |
| **ToN 扩展规划，尚未实施** | 将来要进行的数据冻结、统一评测、精确验证、鲁棒性实验和统计分析。 | P0–P12、E0–E13 中尚未执行的条目。 |

配套行动计划位于本机相邻论文目录（**不属于本 Git 仓库内容**）：

<code>E:\Research\My_paper\2_Papers\008\008_Manuscript\ToN\扩充行动计划列表.md</code>

在本机中可通过相对路径打开：[扩充行动计划列表.md](../008_Manuscript/ToN/扩充行动计划列表.md)。该文件自身标记为 <code>Origin Mode: plan</code> 与 <code>UNVERIFIED</code>；其中的 P0–P12、E0–E13 是待执行路线，不应被理解为本仓库已经实现的功能。

本 README 的两个直接用途是：

1. 作为目录、算法入口、数据和结果的导航；
2. 作为后续 **P0：冻结当前资产** 与 **E0：重现会议实验** 的核对入口。

## Git 与复现基线

本仓库的远端名称为 <code>008</code>（不是 <code>origin</code>），地址为 <code>git@github.com:zqh-ynu/008_Experiment.git</code>。历史 <code>008/master</code> 是此前的备份基线；当前用于整理研究快照的本地分支为 <code>codex/research-snapshot-20260817</code>。该快照分支在推送前会经过明确的内容核对，不会修改 <code>master</code>。

### 当前同步边界

为避免把“本机能够运行的研究工作区”误当作“克隆即可复现实验的公开数据包”，本快照采用以下边界：

- 保留算法源码、数据处理逻辑、12 个小型实例生成条件 JSON、完整的 <code>ExperimentsResults</code>、历史作图/解释资产和项目说明；
- 不提交 <code>DatasetTest/data</code> 中的原始人口栅格、裁剪结果、基础用户坐标库和示例数据；详见 [DatasetTest/data/README.md](ExperimentsData/DatasetTest/data/README.md)；
- 不提交 <code>ExperimentsData/data</code> 中的 user/UAV 实例 CSV、ZIP 归档和历史部署 PNG；只保留 JSON 条件和说明，详见 [data/README.md](ExperimentsData/data/README.md)；
- 不提交 <code>.vs</code>、<code>x64</code>、<code>Debug</code>、<code>Release</code>、DLL/EXE/OBJ/PDB/TLOG 等本机构建资产；
- 恢复 EXP1–EXP4 前必须先从本机冻结备份或独立私有数据快照恢复实例。仅克隆 Git 仓库不能直接运行这些实验。

远端历史或本地快照对齐只说明已提交内容的一致性，**不代表完整复现已冻结**。P0 仍应记录数据包版本、配置、环境、随机规则、运行命令和输出目录。本仓库当前未发现许可证文件；对外发布代码、人口数据、通信数据或第三方算法前，应另行核验许可与再分发条件。

## 端到端主实验链

~~~mermaid
flowchart LR
    A["全国 100 m 人口栅格"]
    B["县级中心附近 5 km × 5 km 栅格裁剪"]
    C["按人口栅格权重抽样基础用户坐标"]
    D["选取县区，并附加 hard/elastic 类型、权重和 QoS"]
    E["DatasetTest/uav_deployment.py<br/>100 m 网格贪心部署 UAV"]
    F["本机或独立数据快照<br/>ExperimentsData/data<br/>配对 user/UAV CSV 实例"]
    G["UAVBandwidthAllocation<br/>同一实例依次运行六种方法"]
    H["ExperimentsResults<br/>逐实例算法 CSV"]
    I["summary/<br/>跨实例均值 CSV"]
    J["FigsDrawerProj / Origin<br/>图表资产"]

    A --> B --> C --> D --> E --> F --> G --> H --> I --> J
~~~

上图描述的是当前可追踪的主数据链，关键解释如下。

- 基础用户坐标由人口栅格值归一化后加权抽样得到，并在 100 m 栅格内加入扰动。它们是**基于人口空间分布的合成用户坐标**，不是可识别的真实个人位置。
- 主实验现存的 <code>variable_user_num</code>、<code>variable_uav_num</code> 和 <code>variable_hard_user_ratio</code> UAV CSV 与 [DatasetTest/uav_deployment.py](ExperimentsData/DatasetTest/uav_deployment.py) 的 100 m 网格贪心部署逻辑一致：抽查五组既有 CSV 后，按历史条件重新计算所得坐标逐点一致，最大绝对差约为 $1.4\times10^{-14}$。
- 该部署器在 5,000 m × 5,000 m 归一化区域中，以 100 m 网格中心为候选点；每轮选择新增覆盖用户数最多的候选点，并对除第一架外的 UAV 施加 800 m 连通约束。当前源文件中的默认 UAV 数不必等于所有历史 CSV 的实际生成参数，应以已保存实例和 P0 记录为准。
- [generate_uav_loc](ExperimentsData/generate_uav_loc/) 下的 <code>XH_UAV_LOC</code> 与 <code>LSY_UAV_LOC</code> 是独立/历史性的 C++ 最大覆盖或连通部署试验线。它们的候选网格、固定 UAV 数和输出方式与现存 <code>variable_*</code> 主数据不相符，**不是**这些主实验 UAV 坐标的生成器。
- 本机 [ExperimentsData/data](ExperimentsData/data/) 保存可被算法读取的**输入实例库**，不是程序运行日志；同步快照中该目录只保留生成条件 JSON 和 [数据恢复说明](ExperimentsData/data/README.md)，实例 CSV/ZIP 需从独立数据快照恢复。[ExperimentsData/ExperimentsResults](ExperimentsData/ExperimentsResults/) 则完整保存逐实例结果、跨实例汇总和图表资产，不能笼统只称为“统计数据”。

## 目录导航

| 路径 | 状态 | 作用与使用边界 |
| --- | --- | --- |
| [Algorithms/UAVBandwidthAllocation](Algorithms/UAVBandwidthAllocation/) | 当前主实验链 | Visual Studio C++17 主工程。包含系统模型、两个当前提出算法、四个当前基线、实验驱动、CSV 汇总和手写测试。实验入口见 [main.cpp](Algorithms/UAVBandwidthAllocation/main.cpp)，六方法调度见 [experiments.h](Algorithms/UAVBandwidthAllocation/experiments.h)。 |
| [Algorithms/baselineAlgorithms](Algorithms/baselineAlgorithms/) | 历史/独立基线 | 包含独立的 Python DRL、Sequential DQN、GNN-BDQ 训练与推理原型。其中 <code>DRL_Algorithm_3</code> 当前没有被 C++ 六方法循环、Visual Studio 主工程或会议实验驱动调用，且含大量本机训练/checkpoint 资产，因此不随本快照同步。不要把该目录自动等同于结果 CSV 中的 <code>AlgDRL</code>。 |
| [Algorithms/LP_for_SAP](Algorithms/LP_for_SAP/) | 历史/探索性 | 依赖 CPLEX 的旧版 SAP/参数化 fractional knapsack 原型。其当前入口调用 alpha 参数扫描，不属于 <code>UAVBandwidthAllocation</code> 主实验链。 |
| [ExperimentsData/DatasetTest](ExperimentsData/DatasetTest/) | 当前主实验链 | 人口栅格裁剪、基础用户坐标、QoS/业务属性生成，以及主实验的 Python UAV 贪心部署代码。<code>DatasetTest/data</code> 的原始/中间数据本体不随 Git 同步，见 [其 README](ExperimentsData/DatasetTest/data/README.md)。顶层 [main.py](ExperimentsData/DatasetTest/main.py) 是另一份无人机救援数据可视化示例，**不是**本主链的统一生成入口。 |
| [ExperimentsData/generate_uav_loc](ExperimentsData/generate_uav_loc/) | 历史/候选实现 | 两套 C++ UAV 部署实现（<code>XH_UAV_LOC</code>、<code>LSY_UAV_LOC</code>）。可供研究/对照，但不应替代对既有 <code>variable_*</code> 实例来源的结论。 |
| [ExperimentsData/data](ExperimentsData/data/) | 当前主实验链 | 本机输入实例库：<code>County_loc_Type_req</code> 为早期/兼容测试数据；<code>variable_user_num</code>、<code>variable_uav_num</code>、<code>variable_hard_user_ratio</code> 为当前主实验条件。Git 快照只保留 12 个 JSON 条件和 [恢复说明](ExperimentsData/data/README.md)，不含实例 CSV/ZIP/历史部署 PNG。 |
| [ExperimentsData/ExperimentsResults](ExperimentsData/ExperimentsResults/) | 当前主实验链 + 历史快照 | 保存六算法逐实例 CSV、<code>summary</code> 均值表、历史结果分支和作图代码。应在引用任何数值前做逐实例—summary 一致性复核。 |
| [ExperimentsData/ExperimentsResults/FigsDrawerProj](ExperimentsData/ExperimentsResults/FigsDrawerProj/) | 当前结果作图资产 | EXP1/EXP2/EXP3 绘图脚本读取 <code>summary</code> CSV 并输出图；这些脚本不是统计器，也不重新计算原始算法结果。 |
| [ExperimentsData/testSearchMethod](ExperimentsData/testSearchMethod/) | 历史/解释性资产 | ApproFast 单 UAV hard/elastic 带宽划分搜索、hard 阶跃效用及线性松弛的历史测试数据、实验日志、Origin 工程和候选论文解释图。生成逻辑主要在 <code>UAVBandwidthAllocation/test.h</code> 与 <code>EntityDefinition.cpp</code>，它不是独立源码项目，也不能据此断言这些图已进入最终会议论文。 |
| [ExperimentsData/testAlphaParametrizedFractionalKP](ExperimentsData/testAlphaParametrizedFractionalKP/) | 历史/探索性 | <code>LP_for_SAP</code> 对 alpha-parametrized fractional knapsack 的参数扫描输出。这里的 <code>Alpha</code> 是参数化松弛问题的扫描参数，**不等同于** hard-user ratio $\alpha_H$，也不等同于多 UAV 理论中的单 UAV oracle 近似因子。 |
| [FigProject](FigProject/) | 历史作图资产 | 对应上述 alpha 扫描的 OriginLab 工程。现存 PNG 与 TXT 输出的数值范围不一致，应作为待重新核对数据版本的历史图，而非当前结果证据。 |

### 推荐的首次阅读顺序

1. 先阅读本 README 的“运行前必须核对”部分，不要直接构建或运行；
2. 阅读 [main.cpp](Algorithms/UAVBandwidthAllocation/main.cpp) 与 [experiments.h](Algorithms/UAVBandwidthAllocation/experiments.h)，确认当前启用实验和六方法入口；
3. 阅读 [ExperimentsData/data/README.md](ExperimentsData/data/README.md) 中的 schema；如需对照实际 user/UAV CSV，先从本机或独立数据快照恢复一对实例；
4. 查看对应的逐实例 CSV 与 <code>summary</code> CSV，先检查各算法行数和聚合一致性；
5. 需要重建空间实例时，再从 [DatasetTest](ExperimentsData/DatasetTest/) 的人口—用户—UAV 链条开始；
6. 需要推进 ToN 工作时，先完成 P0/E0，而不是直接把历史 summary 用作新实验结论。

## 主算法工程与六种结果标签

### 当前入口与实验选择

[Algorithms/UAVBandwidthAllocation/main.cpp](Algorithms/UAVBandwidthAllocation/main.cpp) 是主工程入口。当前工作树中唯一启用的调用为：

~~~cpp
exp2_different_uav_number();
~~~

EXP1、EXP3、EXP4 和手写测试仍需通过手工注释/取消注释切换；工程没有统一 CLI、参数解析器或安全的输出目录隔离机制。

当前正式实验循环调用的六种方法及其真实入口如下。结果 CSV 的显示名存在历史命名，阅读结果时应以函数映射为准。

| 结果标签 | 实际调用 | 当前代码中的作用 |
| --- | --- | --- |
| <code>ApproBetter</code> | <code>approposed_multiUAV_allocation_new(..., 2)</code> | 当前多 UAV 去重框架 + <code>FPTAS_singleUAV_new</code>。 |
| <code>ApproFast</code> | <code>approposed_multiUAV_allocation_new(..., 1)</code> | 当前多 UAV 去重框架 + KKT/水填充 + hard 用户贪心舍入。 |
| <code>AlgDRL</code> | <code>ConvexRelaxationAndRounding_multiUAV()</code> | 当前 C++ 循环中是确定性的 IPOPT 关联/带宽松弛与舍入实现，**不是**相邻 Python <code>DRL_Algorithm_3</code> 的神经网络推理。会议稿将 <code>AlgDRL</code> 描述为 DQN/DRL 基线，二者存在待 P0 核对的实现—稿件不一致。 |
| <code>AlgMatching</code> | <code>MatchingSQP_Allocation()</code> | 匹配—SQP/IPOPT 过程。 |
| <code>AlgHardFirst</code> | <code>HungarianMatchingAllocation()</code> | Hungarian/KM 子信道匹配实现；不能只凭显示名断言其严格执行“hard first”。 |
| <code>AlgSADA</code> | <code>SADA_Allocation()</code> | successive approximation / dual decomposition / IPOPT 实现。 |

补充边界：

- 当前正式实验调用的是 <code>approposed_multiUAV_allocation_new</code>。保留的旧版 <code>approposed_multiUAV_allocation</code> 使用不同的顺序贪心逻辑，不是当前六方法循环的提出算法入口。
- <code>MatchingGameAllocation</code> 虽参与当前工程编译，但不在上述六方法循环中；不要把它与 <code>AlgMatching</code> 混同。
- <code>ApproBetter</code> 的正式调用通过外层默认参数将 <code>0.083</code> 传给 FPTAS 分支；这与 <code>BAProblem::epsilon = 0.001</code>、以及单 UAV FPTAS 声明中的默认 <code>0.02</code> 是不同位置的值。MASS 会议实验实际采用的 $\epsilon_{\mathrm{MASS}}$ 仍需按照 ToN 计划 P0.4 用配置、运行记录或冻结版本核实，不能仅因当前默认值存在就宣布参数已经确认。
- 现有基线来源线索主要写在源码注释中。本 README 只记录可观察到的实现映射，不补造 DOI、文献等价性或已经验证的文献结论。
- 会议稿中 <code>AlgDRL</code> 的 DQN/DRL 描述与当前 C++ 实验入口不一致；被忽略的 <code>DRL_Algorithm_3</code> 是本机独立原型，而不是已验证的会议实验调用源。后续 P0/E0 应核对会议稿、历史运行记录与实际实现之间的对应关系。

## 输入实例、单位与数据生成

> **同步边界：** 本节的文件名、字段和条件矩阵来自本机冻结实例的已核实结构。Git 快照不包含这些 user/UAV CSV；克隆后请先阅读 [ExperimentsData/data/README.md](ExperimentsData/data/README.md)，并恢复独立数据快照后再运行主实验。

### CSV 命名与字段

主实验通常将同一实例 ID 的一对文件配对读取：

~~~text
用户：<instance>_<N>users_data_<adcode>.csv
UAV： <instance>_<K>uavs_loc_<adcode>.csv
~~~

用户 CSV 字段为：

~~~text
user_id,longitude,latitude,user_type,user_weight,
user_requirement_1,user_requirement_2,app_category
~~~

UAV CSV 字段为：

~~~text
uav_id,longitude,latitude,bandwidth
~~~

当前解析的几个容易遗漏的规则：

- 只有 <code>user_type == "hard"</code> 会被识别为 hard 用户；其他值均进入 elastic 分支；
- 输入中的用户 ID 与 UAV ID 不会作为内部 ID 保留，程序会重新编号；
- 经纬度在读取后映射为小区域近似下的局部米制坐标；
- 工程使用 <code>unit_para = 1000</code> 进行内部单位缩放；
- EXP1–EXP3 会用实验条件覆盖 UAV CSV 中的带宽容量。因此这些实验中 CSV 的 <code>bandwidth</code> 列不是最终算法实际使用的容量；应以相应实验目录实际加载的 <code>def_config.json</code> 和驱动参数为准。

### 现有输入矩阵

| 实验 | 自变量与条件 | 固定条件 | 主要输入与状态 |
| --- | --- | --- | --- |
| EXP1 | 用户数 <code>1000 / 2000 / 3000 / 4000 / 5000</code> | 10 UAV | 本机/独立数据快照的 <code>variable_user_num</code>；每条件多数保存 30 对实例。 |
| EXP2 | UAV 数 <code>5 / 10 / 15 / 20</code> | 3000 用户 | 本机/独立数据快照的 <code>variable_uav_num</code>；用户文件复用 3000-user 条件。 |
| EXP3 | hard ratio <code>0 / 0.2 / 0.4 / 0.6 / 0.8 / 1.0</code> | 3000 用户、10 UAV | 本机/独立数据快照的 <code>variable_hard_user_ratio</code>；目录 <code>0/2/4/6/8/10</code> 对应上述比例。 |
| EXP4 | 每 UAV 带宽 <code>10 / 20 / 30 / 40 / 50</code> | 3000 用户、10 UAV | 当前仅见历史结果线索；标准结果路径所需配置缺失，不能按 EXP1–EXP3 的完备程度视为可直接重跑。 |

当前 C++ 驱动每个条件只运行按实例 ID 排序后的前 **10** 对 input CSV，即使多数输入目录保存约 30 对实例。<code>variable_uav_num/25</code> 只保留 6 个不按标准 <code>uav_data</code> 目录组织的 UAV 文件，不属于完整标准条件。

### 人口到实例的主生成链

[DatasetTest](ExperimentsData/DatasetTest/) 中可追踪的主要步骤是：

1. [cut_map.py](ExperimentsData/DatasetTest/cut_map.py) 以县级中心为基准裁剪约 5 km × 5 km 的人口栅格；
2. [generate_user_coordinates_simple.py](ExperimentsData/DatasetTest/user_location/generate_user_coordinates_simple.py) 按像元人口权重抽取基础用户坐标；
3. [generate_user_data.py](ExperimentsData/DatasetTest/generate_user_data.py) 选取用户、分配 hard/elastic 类型、权重、QoS 与应用类别；
4. [uav_deployment.py](ExperimentsData/DatasetTest/uav_deployment.py) 用主实验已验证的 Python 网格贪心逻辑部署 UAV；
5. 将配对 user/UAV CSV 恢复或生成到 [ExperimentsData/data](ExperimentsData/data/) 的相应条件目录，供主工程读取；这些实例本体不随 Git 快照同步。

现有 Python 脚本没有统一固定并记录人口抽样、县区选择、用户抽样、QoS 抽样和 shuffle 的随机种子。因此历史 CSV 可以作为已保存输入使用，但不能承诺从当前脚本逐字节重建。

## 结果文件、汇总与图表

### 逐实例结果

每个实验条件下，六种算法分别写入 CSV；每行代表一个已运行的输入实例。字段为：

~~~text
duration,
total_num,hard_num,elastic_num,
total_utility,hard_utility,elastic_utility,
hard_bandwidth,elastic_bandwidth,
hard_throughput,elastic_throughput,total_throughput
~~~

因此当前结果记录的是运行时间、服务人数、hard/elastic utility、带宽与吞吐量等聚合指标。它**没有**保存完整逐用户关联/带宽分配、随机种子、代码/配置版本、错误状态或峰值内存；这些内容是 ToN P5 中统一 run record 的待建设项。

### Summary 与图表

- 条件完成后，驱动会从各算法的逐实例 CSV 求均值，输出 <code>summary/</code> 下 12 类指标表；
- [FigsDrawerProj](ExperimentsData/ExperimentsResults/FigsDrawerProj/) 脚本只读取这些 summary CSV 来绘图，**不**承担统计汇总计算；
- 历史结果树 [EXP_hard_weight6-10](ExperimentsData/ExperimentsResults/EXP_hard_weight6-10/) 的方法命名、行数和当前六方法格式不完全一致，应作为历史/扩展快照，待 P0 核对生成版本与输入闭环；
- 已核实基础 EXP1 的 [Total_Utility.csv](ExperimentsData/ExperimentsResults/EXP1_user_num/summary/Total_Utility.csv) 中 <code>ApproBetter</code> 与 <code>ApproFast</code> 的数值列互换。引用现有 summary 前必须与逐实例 CSV 重算或复核；不能把全部历史 summary 直接当作已可靠复现的统计结论。

### 断点续跑限制

当前 checkpoint 仅通过 <code>ApproBetter.csv</code> 的行数判断已完成实例。一个实例的六种方法却是依次写入的：若程序在同一实例的六方法之间中断，重启时可能跳过该实例，造成各算法 CSV 行数不一致。因此它不是事务式恢复机制。每次重跑或引用结果前，至少应核验六个算法的实例 ID/行数一致，并保留失败记录。

## 构建与运行环境

### 主工程的实际构建入口

<code>Algorithms/UAVBandwidthAllocation</code> 是 Windows C++ 控制台工程，当前项目文件表明的主要环境为：

| 项目项 | 当前要求/事实 |
| --- | --- |
| IDE / 工具集 | Visual Studio 2022，MSVC <code>v143</code> |
| 语言标准 | C++17 |
| 常用配置 | <code>Release&#124;x64</code> |
| 依赖 | Boost、IPOPT、nlohmann/json；项目文件当前仍链接 CPLEX，因此在工程配置清理前应将 CPLEX 视为构建要求。 |
| 运行时文件 | IPOPT DLL 与各算法使用的 <code>.opt</code> 文件必须能被运行目录找到。 |
| 推荐工作目录 | <code>Algorithms/UAVBandwidthAllocation</code>，使相对 <code>.opt</code> 文件可见。 |

该目录下的 <code>.vscode/tasks.json</code> 仅尝试编译单个当前活动文件，不能代替 Visual Studio 对多源文件、Boost、IPOPT、JSON 和现有 CPLEX 链接依赖的完整工程构建。

Python 数据链按脚本不同依赖 <code>pandas</code>、<code>numpy</code>、<code>rasterio</code>；栅格裁剪还需要 <code>geopandas</code>、<code>shapely</code>，行政区查询需要 <code>requests</code>，可视化还可能需要 Matplotlib、Plotly 等。请针对将要运行的脚本建立隔离环境，不要将当前本机路径视为可移植配置。

### 为什么 README 不提供“一键运行”

主工程会直接向现有结果目录追加或生成 CSV，构建也会在本机生成 <code>.exe</code>、<code>.pdb</code>、<code>.obj</code> 和 <code>.tlog</code> 等资产。它们现已被 Git 忽略，不应重新加入快照；在 P0/E0 前，请先：

1. 复制或冻结输入、实验 JSON、源码版本、IPOPT 选项和依赖版本；
2. 校正本机路径并明确新的输出目录；
3. 在小规模、隔离的结果根目录上验证一次六算法行数、约束和写入行为；
4. 再决定是否运行完整条件矩阵。

## 运行前必须核对的已知问题

以下项目不是一般性提示，而是已经会影响当前复现或结果解释的边界。

1. **硬编码路径不匹配。** 多处主工程代码仍使用 <code>E:\Research\My paper\2_Papers\008\008_Experiment\...</code>，而本机真实目录为 <code>E:\Research\My_paper\2_Papers\008\008_Experiment\...</code>。按现状直接启动 EXP2 预计会找不到数据或配置。
2. **同步快照并非完整运行包。** 当前快照保留代码、处理逻辑、JSON 条件和结果，但刻意排除了原始人口数据、基础坐标、中间产物与正式实例 CSV；仅凭远端提交不能直接重建或运行完整主实验。
3. **实例生成脚本存在接口漂移。** <code>generate_user_data.py</code> 的当前主调用参数与方法签名不匹配；若干实例生成脚本与当前 <code>UAVDeployment</code> 构造方式也已失配。
4. **随机性没有统一审计。** 人口抽样、县区选择、用户与 QoS 抽样及 shuffle 未统一固定种子，历史输入不能由当前源码承诺逐字节再生。
5. **人口栅格中间产物不随 Git 同步。** <code>cut_maps_tif</code> 的本机历史资产可能仅有 ZIP，而脚本所期待的是解压目录；从独立数据快照恢复后仍需核对实际目录布局。
6. **配置存在多版本说明。** <code>config/readme.md</code>、通用 <code>config/def_config.json</code> 与各 EXP 目录的 <code>def_config.json</code> 参数不完全一致。正式实验应以驱动实际加载的实验目录 JSON 为准。
7. **EXP4 配置路径缺失。** 标准结果路径没有所需 EXP4 配置，历史带宽结果不可自动等同于当前可重跑实验。
8. **checkpoint 不是事务式恢复。** 如前文所述，只依赖 <code>ApproBetter</code> 行数，可能留下跨算法不对齐的逐实例结果。
9. **历史 summary 已发现一致性问题。** 基础 EXP1 <code>Total_Utility.csv</code> 中的 <code>ApproBetter</code>/<code>ApproFast</code> 列已确认互换；其他引用也应做逐实例核对，且人数均值的实现使用整数除法，可能截断非整数均值。
10. **凭据安全边界。** [get_county_center.py](ExperimentsData/DatasetTest/get_county_center.py) 当前从 <code>AMAP_API_KEY</code> 环境变量读取第三方 API 密钥；不要把密钥提交、粘贴到日志或写入 README。若旧密钥曾出现在其他历史副本中，应在独立安全维护任务中轮换并清理访问权限。
11. **编码与构建耦合。** 部分旧 C++ 中文注释存在编码混杂；本 README 使用 UTF-8。主工程和 <code>LP_for_SAP</code> 都含本机绝对依赖路径，历史 CPLEX 原型还依赖特定 CPLEX/VS 配置。

## 历史辅助资产的准确定位

### <code>testSearchMethod</code>

[ExperimentsData/testSearchMethod](ExperimentsData/testSearchMethod/) 主要用于观察单 UAV 混合 hard/elastic 带宽拆分下的目标/效用曲线，并比较原 hard 阶跃效用与 hard 线性松弛。它保存：

- ApproFast 单 UAV 子问题的历史搜索/松弛测试数据；
- 实验日志和 Origin <code>.opju</code> 工程；
- hard、elastic、total utility 候选解释图；
- 搜索思想的概念性图资产。

其测试入口与实现位于 <code>Algorithms/UAVBandwidthAllocation/test.h</code> 和 <code>EntityDefinition.cpp</code>，目前主 [main.cpp](Algorithms/UAVBandwidthAllocation/main.cpp) 未调用这些测试。现存 PDF/CSV/Origin 资产中有未受 Git 跟踪或版本未闭环的内容，且当前 TeX 中相关图引用块已注释；因此只能将其称为**历史性解释图/调试数据**，不能称为最终会议论文已经采用的正式图。

### <code>LP_for_SAP</code> 与 <code>testAlphaParametrizedFractionalKP</code>

[Algorithms/LP_for_SAP](Algorithms/LP_for_SAP/) 的旧 CPLEX 原型当前调用 alpha 参数扫描；[testAlphaParametrizedFractionalKP](ExperimentsData/testAlphaParametrizedFractionalKP/) 保存其输出。该测试在固定/随机构造的一个 bin、hard items 和 soft items 上扫描 <code>alpha = 0 ... 20</code>，考察松弛的 alpha-parametrized fractional knapsack objective 曲线。

- 这里的 <code>Alpha</code> 更接近参数化 LP 的边际值/乘子水平，不能混为 hard 用户比例或多 UAV 近似因子；
- 现存带“论文”前缀的 TXT 文件可能是文献或手工对照，但当前没有完整来源闭环，不能称为已验证复现；
- [FigProject](FigProject/) 中的 PNG 虽然文件名含 <code>3D</code>，实为二维 Alpha–Objective 图，且与当前 TXT 的数值范围不一致；它只是待版本核对的历史图；
- 该原型不属于会议主实验，也不是当前 ToN 扩展任务。

## ToN 扩展路线：尚未实施

本项目将作为 ToN 扩展的基础，但以下流程仍是计划，而非当前仓库已完成能力。应采用配套行动计划末尾给出的推荐严格顺序：

~~~text
P0 冻结当前代码、参数、六算法入口和环境
→ E0 重现会议实验
→ P1–P3 数据核验、冻结和预处理
→ P4 Population-driven 与 Activity-driven 两类空间实例生成
→ P5 统一 runner、evaluator、约束检查和完整 run schema
→ E1 小规模精确解/可靠上界验证
→ pilot
→ E2–E10 核心 ToN 实验
→ 统计分析与图表
→ P11/P12 证明、论文修订和完整性检查
~~~

特别是下面的事项仍是待建设内容：

- 通信活动数据与 Activity-driven 空间实例；
- 统一、不可变的输入实例格式和统一 evaluator；
- 小规模 OPT 或可信上界验证器；
- 完整的随机种子、代码/配置版本、失败状态、峰值内存与逐实例 run record；
- 以窗口为一级抽样单位的层次化 bootstrap 与配对比较；
- 位置误差、SNR 误差及联合误差实验；
- 新 ToN 图表、补充证明与最终完整性审计。

在 P0 之前，现有人口分布驱动坐标/QoS/信道研究应准确称为“data-driven semi-realistic”或“人口分布驱动的合成实例”评估，而不是已经完成的真实通信系统试验。人口空间分布与未来通信活动空间数据也应作为独立场景保存，不能简单相乘后冒充联合真值。

## 对本 README 的维护约定

- 新增或重命名算法、实例生成器、输入 schema、结果字段、实验入口时，应同步更新本 README 中的对应表，而不是只新增目录；
- P0 完成后，应在独立冻结记录中保存 commit/hash、未跟踪资产清单、配置、依赖、随机规则、运行命令和输出目录；不要用本 README 替代不可变 run manifest；
- E0 完成后，应新增逐实例核对结果和失败/不一致记录，再更新“当前主实验链”的复现状态；
- 文档维护不得借机执行 <code>git add -A</code>、重置、清理或批量删除当前工作树。任何 Git 提交都应仅暂存明确的目标文件。

---

### 维护范围

本 README 仅为项目导航与复现边界说明；它不修改算法 API、C++/Python 接口、CSV schema、配置、原始数据、实验结果或任何论文稿件。最后一次文档核对仅覆盖源码、配置、数据模式、现有产物和 Git 状态的只读交叉检查；没有在本次文档维护中构建主工程、运行六算法或批量生成数据。

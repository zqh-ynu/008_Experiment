# 修改日志

## 2026-09-08：EXP1–EXP3 提出方法安全定向重跑与十网络 EXP5 接续

本次在已有未提交的 EXP5 代码和入口修改上实施，保留原修改，不恢复此前移除的旧试运行代码。
正式工程、正式输入生成和 1080 次算法计算均未执行；下述是实现及隔离验证状态。

- 公共 `ApproBetter` 从 selector 2 切到 3，调用现有 `AlgBetter_singleUAV_ToN_faster`；
  `ApproFast` 保持 selector 1。算法内核、四基线、CSV 标签、种子规则、数值容差及物理参数不改。
- 组合版本设为 `ton-proposed-fast-faster-v1`，公共运行信息新增两个提出方法的 selector/入口记录；
  普通驱动仍严格拒绝跨版本续写。旧版本仅通过固定范围的临时迁移函数处理。
- 在 `main.cpp` 实现 `rerun_proposed_exp1_exp3(options)`：只接受固定 ToN 批次、
  `run_ton_01`、ID 1–10、种子 20260905、舍入 2、epsilon 0.1、空条件/复用设置、Release|x64。
  全部三个实验的输入/配置/旧记录/汇总预检通过，并完成全部原文件备份后，才执行方法索引 0、1。
- 每个实验的专用目录保存 `state.json`、一次性 `backup/`、原子更新的 `new_results/` 和完整 `candidate/`。
  输入及配置内容、基线和备份用 Windows SDK SHA-256 校验；无需新第三方依赖。
  逐实例保存、逐文件发布均使用同目录临时文件和检查后的重命名，不追加半行，不删除临时文件。
- 发布过渡保留旧版本且标记汇总过期/写入中，目标只允许旧内容或完整新候选；全部发布后最后切换正式版本。
  支持备份、逐实例、部分发布、正式元数据/恢复状态提交间隙的中断恢复。完成后重启只校验和跳过。
  预计由用户运行时替换 30 份提出方法 CSV、刷新 36 张汇总；60 份基线 CSV 原样保留。
- 主入口增加覆盖完整流程的 Windows 命名互斥对象，不创建锁文件；旧程序仍需由用户保证不并发写入。
  接续顺序为 300 次提出方法定向计算 → 780 次 EXP5；EXP4 调用暂停，旧 run_01/pilot_01 不动。
- 新增 `exp5_different_location_error(options)` 及纯参数映射；十网络由公共 options 显式传入。
  原三网络 `Options{}`/`run_pilot()` 兼容接口不改默认值；真值/估计分离、冻结评价、配对误差、NA 和两级平均保持。
- 扩展现有独立合成测试，使用宏排除正式 main()，只编译其辅助逻辑；不将测试加入正式工程编译清单。
  最终 MSVC 独立构建退出码 0，587 项检查通过，未加载正式用户 CSV 来运行算法。
  中间发现并修正深层测试临时路径超长以及 Windows 头文件/byte/INFINITE 冲突；失败产物全部保留。
  最终通过产物为 `Algorithms/UAVBandwidthAllocation/x64/RerunValidation/20260908_225441_4554e4c6/`。
- 前后 SHA-256 清单摘要核对：正式结果目录 711 个文件、固定输入目录 782 个文件，数量和全部文件内容均不变。
  `git diff --check` 和 UTF-8 校验通过；既有项目配置改动保留，本轮未修改项目配置或创建 Git 提交。

下面条目为较早的历史整理记录，其中旧入口描述不再代表本轮 main()。

## 2026-09-08：整理 ToN 实验框架、算法实现与数据生成流程

本条目以提交前的 `b3038e99d49f8a76a4382b81f115e5dc3eb52125` 为比较基准，
汇总当前工作树已有的全部未提交修改。日期为本次整理日期，不表示所有功能均在当天开发。
原有改动涉及 25 个文件：20 个修改、4 个新增、1 个既有删除；加上本日志，共纳入 26 个文件。

以下功能说明来自当前源码、配置和文档差异。本次仅整理日志、进行轻量静态检查并办理 Git 提交，
不重新实现算法、不启动完整数据生成或 C++ 实验，也不把既有开发记录当成本次复测结果。

### 1. Visual Studio 实验入口与独立续跑

- 将 `main.cpp` 从旧测试入口切换为显式参数配置的 EXP1–EXP4 顺序执行入口；拒绝旧 CLI 参数。
  当前选择 `data_ToN/2026-09-07`，各条件先计算 ID 1–10，结果使用独立的 `ToN_simple/run_ton_01`。
- 新增 `experiment_support.h`，集中六方法调度、统一指标统计、随机种子、CSV 序列化、
  每方法断点检查和十二项汇总输出。正式标签为 `ApproBetter`、`ApproFast`、`AlgRelaxRound`、
  `AlgSwapMatching`、`AlgHardFirst` 和 `AlgSA-DD`。
- 重构 `experiments.h`：先核对输入、目标目录、运行参数和已有记录，再计算缺失的“条件/实例/方法”。
  每个算法成功后立即写盘并关闭文件；只有 `SUCCESS` 与 `ZERO_ALLOCATION` 是有效完成记录。
  错误只报告并停止，不写失败结果行，不自动重试，不用零值补齐缺失样本。
- 同一结果目录支持在参数及输入前缀保持一致时增大 `instance_count`，例如由 10 扩展为 30，
  已有方法/ID 行独立跳过；重复 ID、损坏行、种子或参数冲突直接报错。
- 仅在所有选定实例和方法完成后更新十二项算术均值；计数指标使用浮点类型保存均值，避免整数截断。
  扩算期间通过运行记录标识旧汇总状态。
- 非历史 `data` 根目录必须具有 `generation_status="complete"` 且声明足够连续实例 ID 的生成配置。
  目标 ToN 输入不存在或不完整时不回退到历史数据；EXP4 覆盖 UAV 带宽后重新初始化信道。
- 保留受限的旧 EXP1 四方法结果导入功能，但当前入口不启用。保留手动启用的
  `rerun_hardfirst_exp1_run01()`：只重算旧 `run_01` 的五个 EXP1 条件、各 ID 1–10 的 HardFirst，
  每条件临时文件完整写入后重命名替换，其他五种方法只读；默认调用仍被注释。

### 2. 统一物理模型、分配校验与求解器诊断

- 新增 `allocation_contract.h`，统一绝对/相对容差、Hard QoS 判断、算法状态、失败异常、
  求解器诊断和确定性均匀随机数转换。
- `Channel::cal_average_SNR()` 根据每架 UAV 自身带宽积分噪声功率；全局 `noise_dbm` 保持
  dBm/Hz，不再因反复初始化累计修改。零带宽和非法物理参数显式处理。
- Hard 信道容量使用由中断概率反演得到的可靠 SNR，去掉额外的 `(1-pOut)` 乘子；
  反演或输入读取失败抛出错误，不伪装成正常零结果。
- `SystemMd::init_SystemModel()` 重建维度、用户类型计数、服务集合和信道矩阵，
  检查用户/UAV ID 与数组索引的一致性。
- 公共结果统计基于实际分配重新计算十二项指标，检查唯一关联、覆盖范围、正带宽、
  有限数值、Hard QoS 和每 UAV 总预算；不依赖可能过期的缓存效用。
  Hard 吞吐量统计保证的业务速率，Elastic 吞吐量统计实际分配速率。
- `predefine.h` 严格校验完整信道 JSON；输入配对按数值 ID 排序，并拒绝重复 ID、
  不可读文件及非法文件名前缀。
- 三个 IPOPT 配置禁止边界松弛，并将约束可行性容差收紧至 `1e-10`。

### 3. 基准算法实现与命名边界

- `AlgRelaxRound`：松弛和固定关联问题统一使用 `cap_list` 的容量口径；
  接入显式种子、舍入次数、候选预算/QoS 检查及分阶段求解器状态。
  未找到可行候选与求解器失败分别报告；新增小模型目标函数、Jacobian 和 Hessian 的有限差分核验入口。
- `AlgSwapMatching`：保留既有交换启发式和迭代配置，增加 IPOPT 初始化失败、
  数值带宽和回退诊断，统一 Hard QoS，排除零带宽“已服务”记录。
- `AlgSA-DD`：保留既有连续近似、对偶求解及可行性恢复流程，记录 IPOPT/对偶回退与
  零初始效用时的均匀 theta 分支；统一带宽与 Hard QoS 检查，明确未服务用户的结果字段。
- `AlgHardFirst`：在保留 `HungarianMatching.cpp` 文件名和旧接口兼容转发的同时，
  正式实现改为 Youssef-inspired priority-aware subchannel DA adaptation，已不是 Hungarian/KM。
  Hard 用户先逐槽申请，未达标者释放后改投，有限恢复后冻结达标者，再进行 Elastic 增量效用 DA。
- HardFirst 使用公共有效子信道宽度及含保护开销的槽预算，增加最少 Hard 槽数、
  整数槽、单 UAV 关联和预算验证，并记录真实申请、替换与恢复计数。
  不宣称该适配是原论文逐项复现、具有经典稳定性证明或全局最优保证。

### 4. ToN 提出方法的实现调整

- 多 UAV 结果完成唯一关联与重建后，残余带宽阶段统一调用 `AlgFast_singleUAV_ToN`；
  该策略调整与下述存储优化分开记录。
- `AlgBetter_singleUAV_ToN` 在单次求解内复用 SMAWK 顶层及各递归深度的缓冲区，
  不引入全局缓存或跨调用状态。
- 零缩放利润用户层跳过无效 DP 行复制；回溯表改为仅保存正利润用户的连续决策行，
  每行仍覆盖完整利润状态，保留原始用户索引和逆序回溯规则，并检查分配长度溢出。
- 相关优化按现有实现保留原始候选数、缩放规则、浮点表达式、容差、转移和平局顺序。
  小状态 SMAWK 穷举对照仅在显式定义 `TON_VERIFY_SMAWK` 时启用，正常实验默认关闭。
- README 保留了先前隔离验证和小样本性能测量的来源及范围说明；本次不重新运行这些检查，
  不把每规模单输入的探索性计时当作完整实验结果。

### 5. ToN 实例生成、部署修复与必要配置

- `generate_instances_ToN.py` 的手动入口固定批次标识 `2026-09-07`、种子 `20260904`
  和 30 个重复实例，与 C++ 输入根目录配套；已存在的批次不覆盖、不自动清理或追加。
- 新增目录级和批次级完整性检查，逐条件核对准确文件名集合、CSV 列顺序、
  用户/UAV 实际行数及连续实例编号，全部通过后才写 `generation_status="complete"`。
- `UAVDeployment.deploy_uavs()` 将初始最优新增覆盖值从 `0` 改为 `-1`，
  允许合法连通的零新增覆盖候选入选，避免误判为无法继续部署；正增益优先和平局遍历顺序保持不变。
- 新增 EXP4 的 `def_config.json`，保留运行所需的环境与 UAV 物理参数。
- 纳入 `data_ToN/2026-09-07/generation_config_ToN.json`，记录 30 个实例的种子、县区映射、
  生成环境、实验条件、服务类型来源及研究定义的 QoS 规则。
  该现有记录还注明了 ID 19 的 20-UAV 文件修复、原 15 行保留及其他 779 个 CSV 未改动；
  本次仅提交已有元数据，不重新执行修复或验证这些历史文件内容。
- 更新三份 README，区分输入生成完成与 C++ 实验完成，说明先算 10 个实例、后续扩算到 30 的操作，
  并保留 `real-data-informed synthetic instances` 的来源边界。

### 6. 工程配置与 Git 同步范围

- Visual Studio `Release|x64` 保留优化，同时设置与本机 IPOPT mdd 匹配的
  `_DEBUG`、`_ITERATOR_DEBUG_LEVEL=2` 和 `MultiThreadedDebugDLL`；登记公共头文件和相关工程条目。
- 随当前工作树记录旧 `test.h` 的删除，正常入口不再包含它；本次整理没有另外执行文件删除。
- `.gitignore` 新增全仓库 `*.csv`、`*.csv.*` 及实验结果目录的 `run_info.json` 忽略规则，
  并保留已有指定 Route A 批次目录排除。源码、工程文件、README 与必要生成/信道配置仍可跟踪。
- 忽略规则不删除本地文件，也不取消已有跟踪。本次保留原先已跟踪的 407 个 CSV，
  不改写历史提交；新增实验 CSV、备份、运行记录和构建产物不纳入本次提交。

### 7. 完整文件清单

下表的“删除”是整理开始前已存在的工作树状态；其余改动同样保留原始实现，只有本日志为本次新增。

| 状态 | 仓库相对路径 | 主要内容 |
| --- | --- | --- |
| 修改 | `.gitignore` | CSV、衍生备份、运行记录和指定历史批次忽略规则 |
| 修改 | `Algorithms/UAVBandwidthAllocation/Convexrelaxationandrounding.cpp` | 容量口径、种子化舍入、可行候选与导数核验 |
| 修改 | `Algorithms/UAVBandwidthAllocation/EntityDefinition.cpp` | 物理模型、统一 QoS、SMAWK/DP 存储优化及残余 AlgFast |
| 修改 | `Algorithms/UAVBandwidthAllocation/EntityDefinition.h` | 公共契约引用、算法诊断和 HardFirst/导数验证接口 |
| 修改 | `Algorithms/UAVBandwidthAllocation/HungarianMatching.cpp` | HardFirst 子信道 DA 实现与旧接口兼容 |
| 修改 | `Algorithms/UAVBandwidthAllocation/MatchingSQP.cpp` | 交换匹配诊断及公共可行性口径 |
| 修改 | `Algorithms/UAVBandwidthAllocation/Matching_SQP_ipopt.opt` | 非负边界与约束可行性容差 |
| 修改 | `Algorithms/UAVBandwidthAllocation/README.md` | 六方法语义、运行/续跑、来源和验证边界说明 |
| 修改 | `Algorithms/UAVBandwidthAllocation/SADA_algorithm.cpp` | SA-DD 诊断、回退及统一 QoS |
| 修改 | `Algorithms/UAVBandwidthAllocation/SADA_ipopt.opt` | 非负边界与约束可行性容差 |
| 修改 | `Algorithms/UAVBandwidthAllocation/UAVBandwidthAllocation.vcxproj` | Release/IPOPT ABI 与工程文件登记 |
| 修改 | `Algorithms/UAVBandwidthAllocation/UAVBandwidthAllocation.vcxproj.filters` | 工程项目分类同步 |
| 修改 | `Algorithms/UAVBandwidthAllocation/experiments.h` | EXP1–EXP4、输入门控、独立续跑与汇总 |
| 修改 | `Algorithms/UAVBandwidthAllocation/main.cpp` | 固定 ToN 批次入口及默认禁用的定向重跑函数 |
| 修改 | `Algorithms/UAVBandwidthAllocation/predefine.h` | 严格物理配置与数值 ID 文件配对 |
| 修改 | `Algorithms/UAVBandwidthAllocation/relax_rounding_ipopt.opt` | 非负边界与约束可行性容差 |
| 删除 | `Algorithms/UAVBandwidthAllocation/test.h` | 已移除的旧测试集合 |
| 修改 | `ExperimentsData/DatasetTest/README.md` | 固定批次、完整性检查与修复说明 |
| 修改 | `ExperimentsData/DatasetTest/instance_generator/generate_instances_ToN.py` | 固定手动入口及完成标记前的完整性校验 |
| 修改 | `ExperimentsData/DatasetTest/uav_deployment.py` | 接纳零新增覆盖的合法连通候选 |
| 修改 | `ExperimentsData/data_ToN/README.md` | 输入批次记录、使用方式及版本边界 |
| 新增 | `Algorithms/UAVBandwidthAllocation/allocation_contract.h` | 公共数值容差、状态与诊断契约 |
| 新增 | `Algorithms/UAVBandwidthAllocation/experiment_support.h` | 六方法调度、指标、CSV 与断点支持 |
| 新增 | `ExperimentsData/ExperimentsResults/EXP4_bandwidth/def_config.json` | EXP4 物理配置 |
| 新增 | `ExperimentsData/data_ToN/2026-09-07/generation_config_ToN.json` | 已有批次生成与定向修复元数据 |
| 新增 | `CHANGELOG.md` | 本次汇总修改日志 |

### 8. 本次验证与已知遗留项

- 原有 24 个仍存在的变更文件通过严格 UTF-8 解码；被删除的 `test.h` 不参与文件内容校验。
- 两个修改的 Python 文件通过 Python 3.11 AST 语法检查；未导入生成模块、未创建字节码缓存，
  未调用批次生成、UAV 部署或任何正式实验。
- 两个新增 JSON 文件解析通过；工程 `.vcxproj` 和 `.vcxproj.filters` 的 XML 解析通过。
- 提交前工作树差异的 `git diff --check` 通过。Git 暂存内容与远端推送状态由提交后的独立核对确认，
  本日志不预先声明推送成功。
- 工程文件仍列有 9 个本地不存在的旧条目：`test.h`、`test_route_a.h`、`route_a_batch.h`、
  `config/route_a_run_options.json`、`scripts/validate_route_a.ps1`、`scripts/run_route_a_batch.ps1`、
  `scripts/route_a_batch.py`、`scripts/plot_route_a.py`、`scripts/test_route_a_plot.py`。
  它们当前位于头文件/非编译项目条目中；本次保留现状，不据此推断 C++ 构建一定失败或成功。
- `MatchingSQP.cpp` 中部分中文注释仍有可见乱码，本次未批量转码或更改算法代码。
- 本次没有执行 C++ 编译、链接、数值回归或正式性能实验，因此不提供新的编译通过、
  数值等价性或性能结论。必要本机数据与第三方依赖仍需按 README 准备。

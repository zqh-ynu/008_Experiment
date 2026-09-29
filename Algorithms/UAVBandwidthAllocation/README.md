# UAVBandwidthAllocation

面向混合 QoS 用户的多 UAV 用户关联与带宽分配研究代码，包含两种 ToN 提出方法、四种基准、EXP1–EXP5 驱动及逐实例记录/汇总逻辑。

当前 `main.cpp` 顺序启用 **EXP4、EXP5，HardFirst 均固定请求 8 个候选**；EXP3 与其他入口保持注释。普通EXP3采用动态候选预算；`run_02/EXP3_hard_ratio` 的旧HardFirst定向覆盖单独放在 `main.cpp` 临时函数 `resume_exp3_with_dynamic_hardfirst(options)`，不放入 `experiments.h` 的普通批处理。规则和覆盖边界见1.4、4.4。本轮仅修改源码、测试源码和说明并做静态检查，未编译、运行测试或实验，也未改写已有结果。[EXP5 定位误差实验](LOCALIZATION_EXPERIMENT.md) 的旧流程仍保留。

> **历史入口记录（2026-09-08），并非当前运行指令：**
>
> 当时计划定向重算 EXP1–EXP3 的两个提出方法，再执行十网络 EXP5；旧 `run_01`、EXP5 `pilot_01` 和其他基线记录保持原状。该段仅说明历史批次来源，不能作为当前结果已完成的证据。
>
> **以下为此前四实验输入准备与运行方式的历史记录，不是本轮执行指令：**
>
> - 用户先手动生成固定批次 `data_ToN/2026-09-07` 的 30 个输入，再在 Visual Studio 中编译、运行 `main.cpp`。四个实验顺序执行，各条件先算 ID 1–10，默认输出 `run_ton_01`。
> - 正常运行逐算法、逐实例立即保存，只生成逐实例 CSV、12 个 summary CSV 和每实验一份 `run_info.json`。
> - `reuse_exp1_root=""`，新 ToN 批次重新测量当前六种方法；不导入旧数据实验的指标或耗时。旧导入代码保留但默认关闭。
> - 保留现有 `ApproBetter` 残余阶段 AlgFast 修改及新 `AlgHardFirst`；旧结果不改名、不覆盖、不删除。
> - 前期 DA 构建/合成测试与 ApproBetter 离线对照见 1.1，不代表当前批次已经运行。当时只修改输入衔接、生成完成检查和入口并做静态核对，没有编译、生成正式数据或运行算法。
> - 后续数据准备更新（2026-09-08）：固定 ToN 批次已生成，经 ID 19 的零新增覆盖候选修复后通过全部输入结构检查，完成配置已写入。其余输入未重算；现在可由用户进入 Visual Studio 编译/运行步骤，不要重新生成该批次。

仓库数据链与同步边界见[仓库级 README](../../README.md)；输入资产恢复规则见[数据 README](../../ExperimentsData/data/README.md)。正式实例 CSV 不随当前 Git 快照同步，第三方依赖和部分路径仍绑定本机。

## 1. 正式六方法与实现边界

顺序和文件标签由 [experiment_support.h](experiment_support.h) 的 `method_name_list` 固定；`run_algorithm()` 是唯一正式六方法调度包装层。四个基线仅按 hard 最低等级做决策；其中 AlgRelaxRound、AlgSwapMatching 和 AlgSA-DD 将已服务 hard 的报告带宽收至该等级阈值，AlgHardFirst 则为每个获接纳 hard 用户预留一个完整物理槽、报告最低等级有效门槛，槽内余量不可复用。两个提出方法仍使用完整多级模型。既有实验结果尚未按当前源码重算。

| 新标签 | 实际调用 | 实现文件 | 历史标签 |
| --- | --- | --- | --- |
| `ApproBetter` | `Appro_multiUAV_ToN(..., 3, 0.1)` → `AlgBetter_singleUAV_ToN_faster` | [EntityDefinition.cpp](EntityDefinition.cpp) | 同名 |
| `ApproFast` | `Appro_multiUAV_ToN(..., 1, 0.1)` | [EntityDefinition.cpp](EntityDefinition.cpp) | 同名 |
| `AlgRelaxRound` | `ConvexRelaxationAndRounding_multiUAV()` | [Convexrelaxationandrounding.cpp](Convexrelaxationandrounding.cpp) | `AlgDRL` |
| `AlgSwapMatching` | `MatchingSQP_Allocation()` | [MatchingSQP.cpp](MatchingSQP.cpp) | `AlgMatching` |
| `AlgHardFirst` | `HardFirstPriorityMatchingAllocation()` | [HungarianMatching.cpp](HungarianMatching.cpp) | hard-only 八候选矩形 Hungarian 单槽匹配；历史联合 Hungarian/KM 保留为独立入口 |
| `AlgSA-DD` | `SADA_Allocation()` | [SADA_algorithm.cpp](SADA_algorithm.cpp) | `AlgSADA` / 稿件中的 `AlgSADD` |

既有结果的来源信息保留。下文列出的四种 EXP1 方法曾复用旧批次记录，该说明用于历史追溯，不是后续运行的导入指引。

### 1.1 两种 ToN 提出方法

`Appro_multiUAV_ToN()` 先按同一冻结状态评估未选 UAV，按真实边际效用选择 UAV，再更新状态。唯一关联阶段保留每个用户绝对效用最大的分配，并完整重建列表、映射和聚合量。Fast在此结束。Better随后按UAV ID升序固定每架UAV的hard分配，以“原已匹配elastic带宽+剩余带宽”为预算，把该UAV已匹配及范围内全局未匹配的elastic用户一同重优化；得到零带宽的旧elastic用户解除关联，可由后续UAV考虑。旧`reallocate_residual`参数仅为接口兼容，不再切换这两个策略。

当前单 UAV 接口返回本次完整预算下的真实绝对带宽与效用；多 UAV 接口返回唯一关联及Better重优化后的最终绝对结果，再交给公共校验器。纯elastic候选下，`AlgBetter_singleUAV_ToN_faster` 直接返回 `AlgFast_singleUAV_ToN` 的连续最优解。`AlgFast_singleUAV_ToN` 的 LCM 松弛/二候选舍入及 `AlgBetter_singleUAV_ToN_faster` 的利润 DP/SMAWK 主体保留。小状态 SMAWK 对照枚举仅在显式定义 `TON_VERIFY_SMAWK` 时启用，正常实验默认关闭，即使工程定义了 `_DEBUG` 也不启用。

**ApproBetter 实现优化（2026-09-07）**：单次 AlgBetter 调用内复用 SMAWK 顶层及逐递归深度缓冲区；零缩放利润层不再复制/交换 DP 行；回溯表改为只包含正利润用户的连续 `int` 决策行，每行仍覆盖完整 `0..P`。保留原始候选数 `n`、`epsilon/c/delta/P`、浮点表达式、容差、转移与平局顺序；不改 AlgFast、多 UAV 贪心、残余分配、公共接口、单线程执行或现有 Release/IPOPT ABI 设置。工作区没有全局或跨调用缓存。

独立验证基线来自修改前的当前工作树。启用 `TON_VERIFY_SMAWK` 的合成对照通过 236 个案例、3256 条内部记录、123 次 UAV 选择记录；输出浮点字段逐位一致。工作区在 10 种规模各重复 12 次，初始化后地址和容量不变。实际生产源码及独立测试/性能版本均在原 Release|x64 配置下构建成功（0 错误，有既有代码/依赖警告）；生产入口没有执行。

只读使用 EXP1 的 1000/3000/5000 用户各实例 1，保持 10 架 UAV、40 MHz、`epsilon=0.1`。每个版本先预热一次，再串行交替测三次；每次独立进程运行，加载/模型复制/验证均不计时。优化前后所有输出及另行采集的选择顺序一致。下表是**每个规模一个输入、三次重复**，不是全量实验均值，也未续写任何原 CSV。

| 用户数 | 基线均值 / 中位数（秒） | 优化后均值 / 中位数（秒） | 中位数提速比（基线/优化后） |
| --- | --- | --- | --- |
| 1000 | 0.557420 / 0.556097 | 0.523228 / 0.531617 | 1.046× |
| 3000 | 4.345278 / 4.360617 | 4.180363 / 4.160677 | 1.048× |
| 5000 | 11.999577 / 12.033864 | 11.262405 / 11.360608 | 1.059× |

三组各 55 次 DP 调用的决策元素存储量合计分别减少 6.28%、4.86%、5.06%；最大单次决策表大小没有下降。这里统计的是逻辑决策元素字节数，不是进程峰值内存，也不包含工作区、行映射及分配器开销。逐次计时、源基线、测试驱动和核验报告保存在本地忽略目录 `x64/ApprBetterValidation/20260907_222034/`，不属于正常实验输出或新的自动测试框架。后续正式测量应选择新 `output_name`，不要把优化前后的耗时续写进同一批结果。

### 1.1.1 当前 Better 接口与旧优化记录

`AlgBetter_singleUAV_ToN_faster` 已通过 selector `3` 接入正式调度。Better在唯一关联后再次用它求纯elastic子问题；纯elastic时该函数返回Fast的精确连续解。当前调用方式为：

```cpp
// uav的预算为本次完整elastic预算，返回本次绝对带宽与效用。
auto result = problem.AlgBetter_singleUAV_ToN_faster(uav, elastic_candidates, epsilon);
```

当前接口按内部用户 ID 确定处理顺序，不修改模型或输入，返回本次预算下的真实绝对带宽与效用。以下原有的1–5项及旧验证记录描述此前的边际效用优化版本，仅供历史追溯；不能作为当前纯elastic后处理的执行说明。

1. 按真实平移函数划分凹用户与带激活门槛的非凹用户，不以 `m_i==0` 或带容差的 `tau==0` 代替分类。凹池保留允许状态中的零新增带宽常数收益。
2. 调用一次 Fast 得到可行值 `L`，独立计算弱对偶上界 `U(lambda)`。只有保守验证 `L >= (1-epsilon)*U` 才早退；最多 80 次价格二分不收敛时仍可进入 DP，未认证的松弛可行值不能充当上界。
3. 凹池对当前对数效用使用基准带宽偏移、注水断点和前缀统计，每个剩余预算的值查询为 `O(log n)`，只为获选状态重建带宽。
4. 非凹集合大小为 `d`。按 hard 阈值、elastic break-even 必要带宽求安全激活人数上界 `s<=d`，不使用 LCM 接触点 `tau`。取 `Delta=epsilon*L/s`、`P=ceil(2s/epsilon)`，只计算活动状态范围，复用原 SMAWK 递归的矩形适配器。
5. 最大化 `p*Delta+F_C(B-A(p))`，回溯后比较真实收益与 Fast，返回较优的合法分配，同分优先 Fast。

理论依据：最多 `s` 个非凹用户发生向下取整，总损失不超过 `s*Delta=epsilon*L<=epsilon*OPT`；凹部分精确求解。因此在当前模型及论文实数算术约定下保持 `(1-epsilon)` 近似保证，时间为 `O(n log n+(d*s+s log n)/epsilon) ⊆ O(n²/epsilon)`。浮点认证向保守方向保护，裁剪不确定时保留更多状态；非法输入、非有限数值、容量溢出及实质性回溯错误明确报错，不静默伪造成功。

最终版本的 MSVC 独立编译通过（退出码 0），20 个确定性合成小例、374 项断言通过，包括极小激活集合连续参考解、认证通过/不通过路径、矩形 SMAWK 枚举核对，以及稀疏全局用户 ID 和非零 UAV ID 的映射。未运行正式数据、未写入现有结果，不提供性能提速结论。验证产物置于本地忽略目录 `x64/AlgBetterFasterSmoke/20260908_153414/`；不增加长期测试框架，也不修改定位误差实验测试。上述是历史独立接口验证，不是本轮复测。本轮已接入调度，采用独立新版检查点和受控替换，不向旧正式 CSV 追加新版行；两个提出方法的全部指标和耗时均重新计算，不要求新旧分配逐位相同。

旧 MASS 函数（包括 `WaterFillingAlgorithm_singleUAV_new`、`FPTAS_singleUAV_new` 和 `approposed_multiUAV_allocation_new`）仍在源文件中，但不是上述两个正式标签的入口。

### 1.2 四种基准的准确含义

- **AlgRelaxRound**：IPOPT 连续松弛、随机舍入、固定关联后的再优化，在有限次尝试中选择实际效用最大的可行候选。没有 TD3、actor/critic、replay buffer 或 eMBB/URLLC 动态抢占，不再归属为 Tian 的 DRL 实现。`epsilon_tol` 参数为源代码兼容保留；最终可行性由公共容差决定。新增显式 seed、尝试次数和可选诊断参数。
- **AlgSwapMatching**：借鉴 Han–Wang [12] 的 beneficial-swap 思想。最强容量关联初始化后，默认 `max_outer_iter=1`，先进行每 UAV 连续带宽优化，再进行跨 UAV 成对交换搜索；交换后不会再运行一轮带宽优化。内部 hard sigmoid 幅度为 `w_i`，elastic log 按最大带宽效用归一化，因此 surrogate 与最终实际效用不同。某 UAV 的 hard 总需求不可行时会关闭其 hard 约束，随后后处理删除不满足 QoS 的 hard 用户；剩余带宽的 elastic 再优化代码仍未启用。本次不改变 surrogate 权重、迭代次数或这些启发式步骤。它不是 DEI、forbidden-pair/externality 机制的完整复现，也不是“求最大权匹配”。
- **AlgHardFirst**：只让可由一个物理槽达到最低等级的 hard 用户参与矩形 Hungarian 最大权匹配，默认生成8个排列候选（仅EXP3按1.4缩减前缀）并按hard最优值相同、总效用最大择优。匹配后保留有界 hard 专用局部搜索，但其可选边同样限于单槽；最后在剩余槽上按真实边际效用匹配 elastic。该算法借鉴 Youssef 等人的 hard 优先匹配思想，不包含其截止时间指标、NOMA 功率配对、稳定性证明或全局近似保证。
- **AlgSA-DD**：由 Li [16] 的 successive approximation / dual decomposition 思想适配。保留外层 theta 更新、每 UAV 的对偶价格/KKT 二分求解，以及本项目新增的 IPOPT warm start/refinement 和原有可行性恢复。默认最强容量初始化只在一条关联上给带宽；初始效用为正时，其他链路 theta 为零，通常不能在后续激活。现有初始化总 surrogate 效用不超过 `1e-12` 的兜底分支使用均匀 theta，不能把限制绝对化为“所有情形固定关联”。本次保留并记录这一分支，不扩展为一般联合关联算法。原论文显式更新用户价格 lambda 和节点价格 mu，当前内层并非逐项复现。

求解器创建/初始化失败会终止该方法；Matching 或 SA-DD 已存在的可行回退可以保留，但诊断会记录阶段、求解器返回码及是否使用回退，最终仍必须通过公共校验。

[MatchingGameAllocation.cpp](MatchingGameAllocation.cpp) 参与编译但未进入正式六方法循环。`Convexrelaxationandrounding_new.cpp`、`EntityDefinition_short.cpp`、`Matchinggameallocation_old.cpp` 和 `setSysCode/` 中的相似文件也不能凭文件名认定为当前实现；以 [vcxproj](UAVBandwidthAllocation.vcxproj) 编译清单为准。`../baselineAlgorithms/DRL_Algorithm_3/` 是未接入的本地 Python 原型，不作为新六方法中的 DRL 证据。

### 1.3 当前 AlgHardFirst 的八候选矩形匹配与单槽规则

每架 UAV 的槽数为 `floor(B_UAV/(BSub*10/9))`，其中 `BSub` 是有效槽宽，`10/9` 包括保护开销。hard 用户与某 UAV 必须可覆盖且一个槽即可达到最低等级，才在该 UAV 的每个物理槽上获得其最低等级 hard 效用边权；其余边权为零。设 hard 用户数为 `H`、物理槽总数为 `S`，矩形 Hungarian 仅建立 `H × max(H,S)` 的代价矩阵并增广真实用户行；`H>S` 时补充 `H-S` 个零权虚拟列。仅提取有效正权真实槽匹配，零权占位不产生预留，因此每名获接纳 hard 用户恰占一个槽。候选0保持原有用户行及UAV/本地槽升序。候选1–7分别用 `mt19937(20260927u+candidate_id)`，先打乱hard行，再打乱UAV槽组；每架UAV内部槽序及虚拟列后置规则保持固定，映射回结果时恢复原始ID。相同输入和构建下候选顺序可复现。

默认入口的正常实例固定求解8个独立候选，每个候选均完成矩形Hungarian、hard局部搜索和elastic分配并校验合法性。各候选hard效用必须与候选0的最优值在现有容差内一致，再按总效用择优，容差内平局保留较小编号。完整hard物理槽归属相同的候选计为重复，不追加重试；无hard、无物理槽或无正效用hard边时仅执行候选0。任一候选错误均使调用失败并报告候选编号。EXP3仅改变候选数量，不改变这些候选内部规则。

每个矩形求解的最坏时间复杂度为 `O(H²·max(H,S))`，矩阵空间为 `O(H·max(H,S))`；默认正常实例执行8次求解。例如200名hard用户与2000槽对应每候选 `200×2000`，累计1600次增广。候选0始终参与择优，最终效用在容差内不低于同输入的单候选方案；不保证严格提高或超过历史联合Hungarian。1000规模约10秒仅为此前观察目标，不是性能保证；不设最低耗时。

单槽匹配后，最多执行 5000 轮 hard 接纳、替换、迁移、交换的最佳改进搜索；搜索同样不允许多槽 hard 边，无改进即停止。随后 elastic 仅争取空闲槽，按实际对数效用增量作决定。hard 用户的输出带宽仅为最低等级门槛 `rMin/cap`，但物理预算仍扣除整个单槽，槽内余量不可再分配。专用校验检查 hard 恰占一个含保护开销的物理槽，并以原始完整模型检查所有已服务 hard 用户恰为第一级。

历史 `HungarianMatchingAllocation()` 保留供独立复核，不再进入正式方法 4。它来自 `34d27a72`，让 hard 与 elastic 同时参与最大权单槽匹配，并使用其原有槽计数；当前 HardFirst 只匹配 hard 且使用含保护开销的物理槽预算，两者结果不能混用。

### 1.4 仅EXP3的动态候选预算（exp3-h2maxhs-budget-v1）

令实际hard用户数为 $H$、总用户数为 $N$、含保护开销的物理槽总数为 $S$，参考点为 $H_0=\lfloor N/5\rfloor$。EXP3采用：

$$
L(H,S)=\begin{cases}
1,&H=0\text{ 或 }S=0,\\
\max\!\left(1,\min\!\left(8,\left\lceil\dfrac{8H_0^2\max(H_0,S)}{H^2\max(H,S)}\right\rceil\right)\right),&\text{其他情况}.
\end{cases}
$$

| α | 实际hard数（N=3000） | 候选数（S=2000） | 已有固定8候选HardFirst |
| --- | ---: | ---: | --- |
| 0 | 0 | 1 | 复用：旧记录请求8、实际仅完成候选0 |
| 0.2 | 600 | 8 | 复用 |
| 0.4 | 1200 | 2 | 重算并按实例覆盖 |
| 0.6 | 1800 | 1 | 重算并按实例覆盖 |
| 0.8 | 2400 | 1 | 已有旧记录需重算；缺失则按新预算计算 |
| 1.0 | 3000 | 1 | 已有旧记录需重算；缺失则按新预算计算 |

候选数按整数计算（使用已有Boost的128位整数避免乘积溢出和向上取整误差），只运行原序列前 $L$ 个候选。候选0、后续编号/固定种子、Hungarian、hard局部搜索、elastic分配、物理检查和择优逻辑均不变；无有效hard边仍只执行候选0。物理槽数与求解器共用槽宽、保护开销和取整函数；逐实例核对实际hard数与条件比例，异常输入不静默更换预算。

这是**搜索预算改变的实验版本，不是结果完全不变的加速**；较少候选可能降低总效用。各候选hard最优值一致的检查只针对现有最低等级、单槽匹配模型。公式只是确定性的计算量参考，不保证等耗时；至少运行一个候选，高比例组仍可能更慢，不用墙钟时间或机器负载决定候选数。`elapsed_ms`/`duration`保存实际完成调用的总耗时，不能用旧耗时除以候选数。

`HardFirstPriorityMatchingAllocation(diagnostics, candidate_count)`接受1–8，旧无参数/诊断参数入口及六方法调度器默认仍为8。只有EXP3执行器计算并传入动态值，EXP1、EXP2、EXP4、EXP5和其他既有调用不会随hard数自动缩减。

## 2. 统一物理模型与分配结果约定

对 hard 用户采用论文的可靠速率定义：

$$
C_{kj}^{H}=\log_2(1+\Omega_{kj}^{s}),\qquad
B_{kj}^{th}=\frac{r_j^{min}}{C_{kj}^{H}},\qquad
R_{kj}^{H}=b_{kj}C_{kj}^{H}.
$$

公共容量不再额外乘以 `(1-pOut)`；所有正式方法读取同一个 `cap_list`。松弛问题、固定关联问题及其梯度/Jacobian/Hessian 使用同一系数，hard 输出效用按实际带宽判断。

`noise_dbm` 始终表示 dBm/Hz 的噪声功率谱密度。每架 UAV 根据自己的总带宽计算噪声功率，不在初始化中累加全局噪声。`init_SystemModel()` 会清空可服务用户映射、重置统计量并重建矩阵，重复执行具有幂等性。实验条件覆盖 UAV 带宽后，重新初始化一次模型，然后为六种方法各自建立求解对象。上述准备不计入算法时间。

算法的原有 pair 返回接口保留。实验框架只从实际分配计算一次指标，必要检查为：

- 每架 UAV 恰有一个带合法 ID 的 `KnapsackResult`，包括未分配资源的 UAV。
- 每用户最多关联一次；被服务用户有严格正、有限带宽，所在链路可服务。
- 列表与实际带宽映射对应；不使用缓存效用和 UAV 缓存总量计算指标，也不反复对照这些缓存。
- 每 UAV 带宽不超预算；hard 用户实际可靠速率满足最低要求。
- `UserResult` 仍由算法返回，但不参与统计，不再在实验框架重复构造另一份结果与其对照。

[allocation_contract.h](allocation_contract.h) 定义公共比较规则：
`|a-b| <= 1e-9 + 1e-8 * max(|a|,|b|)`。hard 服务判定和效用计算共用 `hard_qos_satisfied()`。公共校验器不会通过删除用户或改配带宽来掩盖违规；超出容差的结果在控制台报错并停止，不写入 CSV。各基准原有、已披露的候选筛选及后处理仍属于其算法调用。

校验通过后重新计算人数、带宽、真实效用和吞吐量：

- hard 只有通过实际 QoS 检查才计服务；`hard_throughput=sum(rMin)`，含义是**已保证的业务速率**，不是多余带宽产生的物理速率。
- elastic 吞吐量按 `bandwidth*cap` 计算。
- `EXPResult` 人数字段为 `double`，原始人数的值仍是整数，均值允许小数，例如 2190、2191 的均值为 2190.5。

## 3. 在 Visual Studio 中设置与运行

当前沿用已完成的 `data_ToN_multiHard/run_01` 输入（50个实例中选ID 1–10），**不要重新运行生成器**。旧 `data_ToN/2026-09-07`、30实例及 [generate_instances_ToN.py](../../ExperimentsData/DatasetTest/instance_generator/generate_instances_ToN.py) 的记录仅用于历史追溯，不是本轮输入路径。

然后打开本项目解决方案，选择 **Release / x64**，在 [main.cpp](main.cpp) 的参数区查看设置，由用户编译并按 F5 或 Ctrl+F5 执行。项目调试命令参数留空；任何旧 CLI 参数均被拒绝。**数据生成、编译及正式实验均由用户显式启动，C++ 不自动调用 Python。**

| 参数 | 当前main值 | 用法 |
| --- | --- | --- |
| `instance_count` | 10 | 每条件ID 1–10；EXP3从固定8切换时必须保持原实例数 |
| `master_seed` | 20260905 | 保持同一实例/方法的随机种子稳定 |
| `rounding_trials` | 2 | AlgRelaxRound 的既有舍入次数 |
| `ton_epsilon` | 0.083 | 续跑保持原值，不与历史0.1批次混用 |
| `input_root` | `ExperimentsData/data_ToN_multiHard/run_01` 的绝对路径 | 读取已完成multi-hard批次，不重新生成输入 |
| `output_name` | `run_02` | 对应 `ExperimentsResults/ToN_multiHard/run_02/` |
| `conditions` | 空列表 | 各实验使用完整标准条件；当前EXP5是真实请求UAV数量实验 |
| `reuse_exp1_root` | 空字符串 | 后续运行保持为空，不再指向下文的历史来源路径 |

`main()` 当前按顺序调用 `exp4_real_requests_user_number(options)`、`exp5_real_requests_uav_number(options)`，两者仍为固定8候选。待这两组实验结束，由用户注释这两行、**只取消临时函数 `resume_exp3_with_dynamic_hardfirst(options)` 的注释**，再重新编译运行。保持上述输入、参数和 `run_02`，无需清理目录。临时函数完成4.4的旧HardFirst定向覆盖后，会自动调用普通EXP3补齐缺失项；不要同时启用 `exp3_different_hard_user_ratio(options)`。本次源码修改不自动启动任何入口。

新输出目录或已经完成候选预算切换后的正常续跑，可单独使用普通 `exp3_different_hard_user_ratio(options)`。普通批处理不接受固定8旧版本过渡，也不替换已存在的HardFirst记录；切换未完成时仍使用上述临时函数恢复。

旧EXP1比较/迁移入口仍保留但未启用；其 `_hardfirst_rectangular_hungarian_multistart8_candidate_v1` 检查点及旧版本门禁不属于EXP3方案，不要用它替代普通EXP3入口，也不要与普通EXP1同时启用。此前“1080次定向重跑”和定位误差EXP5均是历史安排，不表示本轮剩余工作量。

`config/route_a_run_options.json` 已不参与运行。当前五实验读取输入根目录的 `physical_config.json`、`application_profiles.json` 和 `generation_config_ToN.json`；旧四实验的 `def_config.json` 路径不适用于这五个入口。

| 驱动 | 标准条件 | 输入子目录（相对 `input_root`） | 每 UAV 带宽 |
| --- | --- | --- | --- |
| EXP1 用户数 | 1000/2000/3000/4000/5000，10 UAV | `EXP1_user_num/<N>/` | 40 MHz |
| EXP2 UAV 数 | 5/10/15/20，3000 用户 | `EXP2_uav_num/<K>/` | 40 MHz |
| EXP3 hard 比例 | 0/2/4/6/8/10，代表 0/0.2/…/1.0 | `EXP3_hard_ratio/<ratio>/` | 40 MHz |
| EXP4 真实请求用户数 | 1000/2000/3000/4000/5000，10 UAV | `EXP4_real_user_num/<N>/` | 40 MHz |
| EXP5 真实请求UAV数 | 5/10/15/20，3000 用户 | `EXP5_real_uav_num/<K>/` | 40 MHz |

历史四实验全方法矩阵共 20 个条件、200 个“条件—实例”组合、1200 次算法调用（EXP1/2/3/4 分别为 300/240/360/300），不是 200 个独立网络样本。

除旧 `data` 兼容目录外，输入根目录必须有 `generation_config_ToN.json`，且 `generation_status="complete"`、声明的连续 ID 和实例总数足以覆盖当前目标。目录或完成配置缺失均报错，绝不自动退回旧输入。实际文件必须成对包含 ID 1–N，不能跳过缺失 ID 后用更高编号补数；重复 ID、不可读或配对不足也会停止。用户/UAV CSV 保留既定表头：

```csv
user_id,longitude,latitude,user_type,user_weight,user_requirement_1,user_requirement_2,app_category
uav_id,longitude,latitude,bandwidth
```

原有输入构造器先按 hard/elastic 排序并重新编号，UAV 按输入行编号；`app_category` 不参与分配。当前 `unit_para=1000`，内部带宽/速率为 kHz/kbit/s，`BSub=0.18` 表示 MHz。人口数据处理、用户抽样和 UAV 部署算法不变；新数据属于 real-data-informed synthetic instances，EXP1/2/4 使用既定 Table 3/6 业务组成规则，EXP3 独立控制 Hard 比例。

### 3.1 编译环境保持不变

使用 Visual Studio 2022、MSVC v143、C++17 和原有 Release|x64 工程设置。IPOPT 为 `C:\\CodeEnv\\Ipopt-3.14.19-win64-msvs2022-mdd`；Boost、CPLEX 和 NuGet 依赖沿用现有路径。

已安装 IPOPT 的 C++ ABI 要求 `/MDd`、`_DEBUG` 和 `_ITERATOR_DEBUG_LEVEL=2`；工程保留 Release 优化并匹配该运行库，不是纯 Release CRT。正常构建的优化/ABI、求解器库和三份正式 `.opt` 文件不变。复用旧计时要求选择原有 **Release|x64**；其他配置会拒绝此复用，避免混合计时。

本机已有的 `.vcxproj.user` 为 Release|x64 配置了 IPOPT DLL 路径，Release 输出目录也已有依赖 DLL。若换机后出现缺失 DLL，应恢复该依赖环境，不必恢复旧批次脚本。

## 4. 写盘、续跑与最小输出

### 4.0 历史提出方法定向替换记录（不是本轮EXP3流程）

本节至4.3中的 `run_ton_01`、备份目录、临时入口和失败即停规则描述旧 `ToN_simple` 流程。当前multi-hard输出及本轮EXP3的无备份原位覆盖以4.4为准；不为EXP3建立本节清单或事务目录。

每个已有 `run_ton_01` 下新增 `rerun_proposed/ton-proposed-fast-faster-v1/`：

- `state.json`：运行身份、源版本、输入/配置 SHA-256、构建设置、基线校验值及恢复阶段。
- `backup/`：原两个提出方法 CSV、12 张汇总和运行信息；一次性备份，不覆盖、不删除。
- `new_results/`：逐实例原子保存的新版提出方法 CSV，不向正式旧文件追加。
- `candidate/`：完整待发布文件，发布中断后保留并校验。

先只读检查三个实验及全部输入，再完成全部备份，最后计算。恢复阶段为 `backing_up → computing → publishing → complete`。过渡期正式汇总标为 `stale`/`writing` 且保留旧版本；一个实验全部文件和汇总发布、基线校验通过后，才切换为规范的新版本元数据。发布目标只能是旧内容或精确的新候选，第三种内容报错停止。

主入口用 Windows 命名互斥对象限制并发，退出自动释放，不创建锁文件。旧程序不认识此锁，仍须手动确保无其他写入者。原子性是逐文件重命名，不承诺整目录事务或断电耐久性；临时文件保留，不计入完成记录。

**下面保留旧普通全方法驱动的历史说明；只有上述历史替换流程会生成4.0的检查点和备份，本轮EXP3不会。**

执行顺序为 **实验 → 条件 → 实例 → 六种算法**。每个算法运行完即计算指标、追加一行、刷新并关闭 CSV，然后运行下一种算法。控制台显示 `RUN`、`SAVED`、`SKIP` 和耗时；不生成监测日志。

新 ToN 批次手动运行时的 EXP1 输出位置（代码实现阶段不创建结果目录）：

```text
ExperimentsData/ExperimentsResults/EXP1_user_num/ToN_simple/run_ton_01/
├── run_info.json
├── 1000/                    # 2000、3000、4000、5000 同样组织
│   ├── ApproBetter.csv
│   ├── ApproFast.csv
│   ├── AlgRelaxRound.csv
│   ├── AlgSwapMatching.csv
│   ├── AlgHardFirst.csv
│   └── AlgSA-DD.csv
└── summary/                 # 完整完成后生成原有 12 张指标表
```

EXP1 完成后共 **30 个原始 CSV + 12 个 summary CSV + 1 个参数记录 = 43 个文件**。其余实验在各自实验根目录下独立使用 `ToN_simple/run_ton_01`；每个实验完成全部条件后发布自己的 12 张汇总，无需等待四个实验全部结束。不生成逐条件 manifest、输入/源码/程序快照、文件哈希清单、后台监控、验证报告或图片。

### 4.1 原始结果与错误处理

保留当前 CSV 列格式以兼容既有读取与汇总；本轮不导入旧数据结果：

```csv
instance_id,seed,status,elapsed_ms,diagnostics,duration,total_num,hard_num,elastic_num,total_utility,hard_utility,elastic_utility,hard_bandwidth,elastic_bandwidth,hard_throughput,elastic_throughput,total_throughput
```

正常结果为 `SUCCESS` 或合法的 `ZERO_ALLOCATION`；后者的人数/效用/资源/速率为零，但保留实际耗时。`diagnostics` 是 CSV 转义后的 JSON，保留既有回退信息；HardFirst 在events中追加一条汇总JSON：矩形行列是每候选规模，`hard_hungarian_augmentations`是累计增广次数；`candidate_count_requested`、`candidate_count_completed`、`distinct_hard_candidates`、`selected_candidate`分别记录配置/完成/不同方案数及获选编号。`candidates`数组记录各候选的种子、重复方案首编号、效用、工作量、搜索状态和阶段耗时。`candidate0_total_utility`与`gain_over_candidate0`记录原方案效用及择优增量。兼容列 `elapsed_ms` 和指标 `duration` 值相同。

计时边界沿用原实现：`steady_clock` 浮点毫秒，记录求解调用及其已有内部步骤；模型准备、公共指标计算、CSV 写入不计时。HardFirst顶层的 `hard_matching_ms`（含矩阵构造）、`hard_local_search_ms` 和 `elastic_ms` 累计所有实际完成候选的真实子区间；候选数组保留各次原始计时。总 `duration_ms` 包含实际候选调用、初始化、内部校验、结果构造和择优，不除以候选数。外层公共指标和最终校验仍在求解计时外。EXP1专用比较入口可打印分阶段统计；multi-hard入口保存完整诊断，并在EXP3覆盖时打印候选数和实际毫秒。全部计时字段均排除于确定性断言之外。

算法失败、无可行舍入候选、非法分配或输入/写盘错误时，**控制台报原因并停止，不写失败结果行、不补零、不自动重试**。你再次手动启动时，从没有保存的项继续。算法内部已经存在的可行回退不等同于框架重试，本轮不改变它。

### 4.2 独立断点与扩大样本数

- 每个算法以自己 CSV 中的实例 ID 判断完成；允许六个 CSV 长度不同，不要求共同前缀。
- 当前实例某算法尚未返回时中断，不会保存该次调用；重启后只重算未保存项，不恢复求解器内部状态。
- 续跑检查 CSV 表头、格式、唯一 ID、选定输入和种子。重复 ID、失败行、损坏/未完整写入的尾行会停止，**不自动截断、删除或修复**。
- 种子继续按 `[master_seed, experiment, condition, instance_id, method]` 的 FNV-1a/32 推导；不含输出名称、目标样本数或执行次序。
- 将 `instance_count` 从 10 增加到 30 时，继续使用固定输入 `data_ToN/2026-09-07` 和输出名 `run_ton_01`，保持算法版本、其他参数、条件次序及已选输入不变，只补算 ID 11–30。不重新运行生成器，不重算已保存的前 10 个实例。减少数量或修改其他参数需换 `output_name`。
- 从头运行或使用不同 seed/epsilon 时，换新 `output_name`；若参数不再匹配旧批次，还应清空 `reuse_exp1_root`。
- 一次只启动一个写入同一输出目录的进程。本框架不提供多进程锁或断电级原子写入；正常追加会刷新并关闭文件，损坏会在下次读取时明确报错。

### 4.3 参数记录与汇总

`run_info.json` 记录实际参数、物理配置数值、条件/输入路径及ID、算法版本、构建配置、复用来源和汇总状态。旧记录模式为 `ton-simple-v2`，当前五实验为 `ton-multihard-run-v1`。默认固定8候选的算法组合版本仍为 `ton-multihard-lowest-baselines-hardfirst-hard-only-rectangular-hungarian-single-slot-multistart8-v1-better-elastic-reopt-v3`，`hard_first_policy()`及其他实验的元数据构造不变。EXP3在该版本后附加 `-exp3-h2maxhs-budget-v1`，并单独记录候选预算。`proposed_algorithms`、最低等级/槽预留和固定种子规则不变。普通版本检查仍拒绝不匹配批次；仅EXP3允许4.4所述的已知固定8版本过渡。它不保存输入/源码副本，也不核对内容哈希；**同路径输入内容、算法代码或依赖发生其他变化时，应另选输出名称，不能依赖本框架识别这些变化。**

`summary.instance_count` 表示已有汇总对应的每条件样本数；`summary.status` 为 `not_generated`、`stale`、`writing` 或 `complete`。只有 `complete` 且样本数等于当前目标时，才是当前完整汇总。

扩大样本时保留旧 summary，控制台明确提示其旧样本数；新计算未全部完成前不发布新的部分均值。所有条件、方法和目标实例齐备后才更新 12 张表，写表前标记 `writing`，全部成功后标记 `complete`。写入途中中断时不信任这些表，重启后可用已保存原始行重新生成。

保留 `Run_time_ms.csv`、`Total_Num.csv` 等原文件名，首列仍为 `User_Scale`，含义取决于实验条件。人数均值使用浮点数，不截断，也不以零或成功子集替代缺失样本。

### 4.4 当前multi-hard与EXP3原位续跑

当前输出为 `ExperimentsResults/ToN_multiHard/run_02/<experiment>/`，每条件保存六个方法CSV，成功实例另存 `users/<method>/<id>.csv`。完整汇总包含12张指标表和 `Success_Rate.csv`。普通缺失调用保留既有“失败留行、性能单元格留空、继续其他调用、不自动重试”的规则；汇总均值取成功记录，并同时公开成功数/尝试数，绝不将失败当零值。

后续单独启用 `main.cpp` 的临时函数 `resume_exp3_with_dynamic_hardfirst(options)` 时，按以下顺序执行：

1. **先只读预检。** 核对原元数据、CSV、实例ID、种子及逐用户结构。只接受当前已知固定8版本切入预算版；输入路径、生成配置、物理/业务参数、ε、种子、实例数量和其他设置必须一致，不放宽共用版本检查。预检通过后才将EXP3的 `run_info.json` 更新为预算版并标记 `summary.status=stale`。
2. **临时函数只重跑旧HardFirst。** α=0、0.2的兼容原行保留；其余条件仅重算旧8候选记录，已完成新预算的记录跳过。其他五方法的已有记录不改；缺失的六方法调用留到定向覆盖全部成功后，由普通EXP3批处理补齐。新版失败尝试携带预算标记并视为已尝试，重启不自动重试。
3. **逐文件临时写入后替换。** 使用目标旁的 `.exp3-budget.pending` 精确临时文件名，不建立备份、迁移清单或事务目录。覆盖成功实例的逐用户文件后，按ID替换 `AlgHardFirst.csv` 中的一行，方法CSV最后发布作为完成依据，不能追加重复ID。中断在两文件之间时，CSV仍是旧预算记录，重启仍判为待重算；精确临时文件可被下次同条写入覆盖，其他未知条目或正式文件格式损坏明确报错。
4. **替换失败即停止。** 重算已有HardFirst时若求解失败，不改该旧行或其逐用户文件，不自动循环重试，不发布完整汇总。写盘失败也停止；若明细已更新而方法CSV尚未提交，保留这个未完成状态，重启后重新计算该条。普通新实例和其他方法的失败规则不变。
5. **转入普通批处理，再发布汇总。** 临时函数完成旧HardFirst替换后调用 `exp3_different_hard_user_ratio(options)`。`experiments.h`只负责动态预算下的正常六方法批处理：严格检查版本与候选策略、补缺失项，并在所有条件/方法/目标ID均已尝试后进入 `writing`，重建12张指标表及成功率表，最后标记 `complete`。α=0、0.2的兼容旧行可进入汇总，α≥0.4的待替换旧8候选行不可计入新版完整汇总；直接走普通入口不会触发定向覆盖。

**覆盖旧HardFirst结果没有自动备份，旧结果一经覆盖不由程序恢复。** 不清空实验目录，不批量删除文件，不修改EXP4、EXP5输出。同一输出只能由一个进程写入；逐文件替换不是整目录事务，也不承诺断电级耐久性。新实例普通追加过程中产生的孤立明细或损坏尾行仍按旧规则报错，不自动修复。

`run_info.json` 的 `hard_first_candidate_budget` 记录公式、参考比例0.2、参考数8、上下限、逐条件H/N/S/L及替换失败策略；`hard_first_policy`说明候选数按条件决定。逐条诊断以真实请求数生成机制名称，例如 `multistart2`、`multistart1`，α=0复用旧记录保留其真实的 `multistart8/requested=8/completed=1` 来源，不伪改旧诊断或耗时。

## 5. 既有 EXP1 结果的历史来源

既有 `run_01` 中部分结果曾来自以下批次，路径仅保留用于历史追溯：

`ExperimentsData/ExperimentsResults/ToN_routeA_v1_eps0p1/batch_20260905_n10_01`

该路径不再作为可用的运行输入。后续运行保持 `reuse_exp1_root=""`，不要按本节路径启用旧结果导入。已经导入的原始指标、耗时及诊断保存在 `run_01` 自己的 CSV 中，读取这些结果不依赖旧批次目录。`run_info.json` 中的 `reuse_source` 和 `reuse_methods` 应继续保留，它们说明数据的实际来源，不表示程序每次运行都要访问该路径，也不应清空以暗示所有方法均为本次重新计算。

| 方法（历史初始安排） | 1000–4000 用户，各 ID 1–10 | 5000 用户，ID 1–4 | 5000 用户，ID 5–10 | 当时待计算 |
| --- | --- | --- | --- | ---: |
| ApproBetter | 重算 | 重算 | 新算 | 50 |
| AlgHardFirst | 重算 | 重算 | 新算 | 50 |
| ApproFast | 复用 | 复用 | 补算 | 6 |
| AlgRelaxRound | 复用 | 复用 | 补算 | 6 |
| AlgSwapMatching | 复用 | 复用 | 补算 | 6 |
| AlgSA-DD | 复用 | 复用 | 补算 | 6 |
| **合计** | | | | **124** |

上述数量是先前 `run_01` 初始方案审计时的历史快照：当时四种可复用方法共有 **176 条 SUCCESS**，目标合计 300 条，其余 124 次需计算。它不表示现在仍有 124 次待运行，也不代表当前 HardFirst 的运行方式；当前完成情况以 `run_01` 中实际 CSV 和 `run_info.json` 为准。

当时未导入旧 `ApproBetter` 和 `AlgHungarian`，也未将旧 Hungarian 行改名为新 `AlgHardFirst`。后续 HardFirst 定向重跑的版本与完成状态由 `run_info.json` 的 `hardfirst_rerun` 单独记录，其他方法保留原有来源信息。

当时的导入检查包括源版本、所选方法、物理/运行参数、输入路径/ID、表头、成功状态及种子，不代表未来换模型、输入内容或环境后仍可直接沿用。保留结果中的历史耗时仍是旧测量，不能描述为新程序重新测得。

EXP2–EXP4 的旧批次没有结果，日后由你选相应入口完整运行。

### 5.1 验证说明

前期DA开发期间曾进行独立构建和合成验证，这是历史记录。当前独立合成测试源码为 `tests/localization_tests.cpp`，原有八候选、单槽匹配、确定性和EXP1门禁用例保留。新增EXP3用例覆盖六比例候选数、零hard/零槽、非法参数与整数边界、显式8与默认入口一致、1/2候选原序列前缀、hard最优值/单槽约束、诊断与计时累计、其他实验元数据不变、唯一版本过渡、混合断点与重复启动、覆盖失败与中断恢复、未知/损坏文件、重复ID/种子不匹配和完整汇总门禁。

**本轮只交付测试源码，未编译，也未执行上述合成测试或正式实验。** 源码差异、调用链、元数据构造及UTF-8/BOM检查不等于编译/测试通过。上述新增用例均待EXP4、EXP5结束后，在独立测试输出目录由用户另行编译执行；算法真实耗时及效用变化仍需正式EXP3结果验证。

## 6. 论文描述建议（仅供复制，未修改 TeX）

沿用用户提供的引用编号：Han–Wang (2024) [12]、Li et al. (2024) [16]、Tian et al. (2024) [27]/[28]、Youssef et al. (2020) [34]。这些编号依赖当前稿件的参考文献顺序，粘贴到其他版本前应转换为相应真实 cite key，不要机械沿用数字。

可复制的默认固定8候选基准描述（不是当前实验已完成的声明；EXP3须另注明1.4的动态预算，不能将下段的eight套用于所有EXP3比例）：

```text
Because the cited works address resource-allocation problems that differ
from ours, we distinguish adapted heuristics from independently constructed
baselines under our user-association and bandwidth-allocation model.
AlgRelaxRound solves a continuous relaxation, performs randomized rounding,
and reoptimizes bandwidth under each sampled association. AlgSwapMatching
is a one-pass beneficial-swap association heuristic with per-UAV continuous
bandwidth optimization, inspired by Han and Wang [12]. It uses surrogate
utilities, followed by hard-QoS recovery; bandwidth is not reoptimized after
the swap-search stage. AlgHardFirst generates eight deterministic ordering candidates,
each using rectangular Hungarian maximum-weight matching for hard users whose
lowest service level fits in one physical slot;
multi-slot hard links are ineligible. A bounded, single-slot hard-only local
search follows. Elastic users can then use only the remaining slots. Reported
hard bandwidth equals the lowest-level threshold, while each reserved full
slot remains unavailable to others. Candidates must attain the same optimal hard
utility within numerical tolerance; the largest total-utility result is retained,
with lower candidate ID breaking ties. Candidate zero retains the original
ordering. Degenerate cases with no positive feasible hard edge use one candidate.
This is a Youssef-inspired hard-first adaptation, without deadline scheduling,
NOMA pairing, or a transferred stability guarantee. AlgSA-DD is a
successive-approximation and dual-decomposition heuristic adapted from
Li et al. [16], with strongest-link association initialization and an
additional IPOPT refinement step. Its default positive-utility initialization
generally leaves initially inactive links inactive, whereas the retained
zero-utility initialization fallback assigns uniform approximation weights.
```

实现/证据边界建议：

- 删除“AlgDRL 实现 Tian 的 DRL 抢占方法”的对应关系。[27]/[28] 可作为相关工作保留，但不是当前 AlgRelaxRound 的算法来源。
- 当前 `AlgHardFirst` 使用 hard-only 八候选矩形 Hungarian 单槽匹配、有界单槽 hard 搜索和剩余槽 elastic 匹配；历史 hard 与 elastic 联合 Hungarian 仍保留独立入口。Youssef 等人 [34] 是 hard 优先匹配思想的来源，不能把其时延指标、NOMA 功率/SIC 或稳定性结论直接归于此适配实现。
- 不将 AlgSwapMatching 说成最大权匹配或交替优化到收敛；不将 AlgSA-DD 说成原文完整复现或一般联合关联求解器。
- 不保留缺乏共同依据的“所有基准复杂度不低于 `O(K n^3)`”断言。历史 Hungarian 的方阵维度为全部用户数与其总子信道数的较大者；当前 HardFirst 的矩阵为 `H×max(H,S)`（`H` 为 hard 用户数，`S` 为含保护开销的物理槽总数），其 Hungarian 求解、有界局部搜索和 elastic 阶段分别记录真实耗时，不能借用历史结果或 Youssef 等人的复杂度。
- 默认 N=10 是运行配置，不是完成实验的证据。未经正式重跑，不能声称“50 different networks”，也不能把旧均值改标签后当作修复版结果。
- 运行服务器配置和时间比较应以实际新构建/运行环境为准，不能沿用未核实的硬件或将合成测试耗时作为正式 benchmark 时间。

## 7. 维护与边界

[main.cpp](main.cpp) 是参数和实验选择入口，并集中保存EXP1/EXP3的临时重跑函数；[experiments.h](experiments.h) 负责EXP1–EXP5正常批处理、EXP3动态预算、参数记录及缺失项续跑，不负责EXP3旧HardFirst原位替换；[experiment_support.h](experiment_support.h) 负责六方法调度、轻量物理检查、CSV和诊断校验；[allocation_contract.h](allocation_contract.h) 保留共同容差和算法状态。

此前曾将第五个基准实现为逐槽 DA，并完成 ApproBetter 优化；这些是历史开发记录。当前正式方法 4 已改为 hard-only 八候选矩形 Hungarian 单槽匹配，历史联合 Hungarian 仍是独立函数。源码中的中文按 UTF-8 保存，带 BOM 的原代码文件继续保留 BOM。

一般修改算法、数据内容或依赖后，应更新适用版本并使用新输出名称；本轮已授权EXP3按4.4原位切换是明确限定的例外。本框架不做源码/输入全量快照或自动版本漂移监控。本次不修改绘图读取器，其历史版本兼容问题另行处理，不能通过取消版本检查解决；不生成图片或修改论文。

构建/合成测试不等同于正式实验复现。本轮只做源码和差异静态检查，不编译、不执行合成测试或正式实验，不导入或改写已有结果。当前仓库未发现许可证文件，对外分发前仍应确认代码、依赖和数据授权。

# UAVBandwidthAllocation

面向混合 QoS 用户的多 UAV 用户关联与带宽分配研究代码，包含两种 ToN 提出方法、四种基准、EXP1–EXP4 驱动及逐实例记录/比较均值逻辑。

当前入口已接入 [EXP5 定位误差实验](LOCALIZATION_EXPERIMENT.md)：十网络、六方法、
7 个水平 RMSE 档位，共 780 次调用，在 EXP1–EXP3 定向重跑成功后执行。
旧三网络默认接口保留；独立合成验证不代表正式实验已执行。

> **当前入口：EXP1–EXP3 定向重跑 → 十网络 EXP5（2026-09-08）**
>
> 固定批次已备好，不重跑生成器。本轮只重算 EXP1–EXP3 的两个提出方法（300 次），
> 然后执行十网络 EXP5（780 次）。先备份，再替换 30 份提出方法 CSV、刷新 36 张汇总；
> 60 份基线 CSV 保持字节不变。EXP4、旧 `run_01`、EXP5 `pilot_01` 不变。
> 新版独立断点与原正式结果隔离，全部阶段完成前不宣称新实验完成。
>
> **以下为此前四实验输入准备与运行方式的历史记录，不是本轮执行指令：**
>
> - 用户先手动生成固定批次 `data_ToN/2026-09-07` 的 30 个输入，再在 Visual Studio 中编译、运行 `main.cpp`。四个实验顺序执行，各条件先算 ID 1–10，默认输出 `run_ton_01`。
> - 正常运行逐算法、逐实例立即保存，只生成逐实例 CSV、12 个 summary CSV 和每实验一份 `run_info.json`。
> - `reuse_exp1_root=""`，新 ToN 批次重新测量当前六种方法；不导入旧数据实验的指标或耗时。旧导入代码保留但默认关闭。
> - 保留现有 `ApproBetter` 残余阶段 AlgFast 修改及新 `AlgHardFirst`；旧结果不改名、不覆盖、不删除。
> - 前期 DA 构建/合成测试与 ApproBetter 离线对照见 1.1，不代表本批次已经运行。本次仅修改输入衔接、生成完成检查和入口并做静态核对，没有编译、生成正式数据或运行算法。
> - 后续数据准备更新（2026-09-08）：固定 ToN 批次已生成，经 ID 19 的零新增覆盖候选修复后通过全部输入结构检查，完成配置已写入。其余输入未重算；现在可由用户进入 Visual Studio 编译/运行步骤，不要重新生成该批次。

仓库数据链与同步边界见[仓库级 README](../../README.md)；输入资产恢复规则见[数据 README](../../ExperimentsData/data/README.md)。正式实例 CSV 不随当前 Git 快照同步，第三方依赖和部分路径仍绑定本机。

## 1. 正式六方法与实现边界

顺序和文件标签由 [experiment_support.h](experiment_support.h) 的 `method_name_list` 固定；`run_algorithm()` 是唯一正式六方法调度包装层。返回的 pair 类型及实现文件名保持不变；第五项使用新入口，旧 `HungarianMatchingAllocation()` 仅作兼容转发，不再执行 Hungarian。

| 新标签 | 实际调用 | 实现文件 | 历史标签 |
| --- | --- | --- | --- |
| `ApproBetter` | `Appro_multiUAV_ToN(..., 3, 0.1)` → `AlgBetter_singleUAV_ToN_faster` | [EntityDefinition.cpp](EntityDefinition.cpp) | 同名 |
| `ApproFast` | `Appro_multiUAV_ToN(..., 1, 0.1)` | [EntityDefinition.cpp](EntityDefinition.cpp) | 同名 |
| `AlgRelaxRound` | `ConvexRelaxationAndRounding_multiUAV()` | [Convexrelaxationandrounding.cpp](Convexrelaxationandrounding.cpp) | `AlgDRL` |
| `AlgSwapMatching` | `MatchingSQP_Allocation()` | [MatchingSQP.cpp](MatchingSQP.cpp) | `AlgMatching` |
| `AlgHardFirst` | `HardFirstPriorityMatchingAllocation()` | [HungarianMatching.cpp](HungarianMatching.cpp) | v1 的 `AlgHungarian` / 更早误命名的 `AlgHardFirst` |
| `AlgSA-DD` | `SADA_Allocation()` | [SADA_algorithm.cpp](SADA_algorithm.cpp) | `AlgSADA` / 稿件中的 `AlgSADD` |

既有结果的来源信息保留。下文列出的四种 EXP1 方法曾复用旧批次记录，该说明用于历史追溯，不是后续运行的导入指引。

### 1.1 两种 ToN 提出方法

`Appro_multiUAV_ToN()` 先按同一冻结状态评估未选 UAV，按真实边际效用选择 UAV，再更新状态。唯一关联阶段保留每个用户绝对效用最大的分配，并完整重建列表、映射和聚合量。最后按贪心顺序统一使用 AlgFast 分配残余带宽，排除已由其他 UAV 服务的用户。这是已经完成的算法修改，本轮保留；ApproFast 在旧批次中本就采用 AlgFast，分配逻辑未因此改变。

单 UAV 接口返回新增带宽和边际效用；多 UAV 接口返回最终总带宽和绝对效用，再交给公共校验器。`AlgFast_singleUAV_ToN` 的 LCM 松弛/二候选舍入、`AlgBetter_singleUAV_ToN` 的利润 DP/SMAWK、两者主体与默认 `epsilon=0.1` 均保留。小状态 SMAWK 对照枚举仅在显式定义 `TON_VERIFY_SMAWK` 时启用，正常实验默认关闭，即使工程定义了 `_DEBUG` 也不启用。先前测试文件已由其他修改移除；本轮不恢复旧测试框架。

**ApproBetter 实现优化（2026-09-07）**：单次 AlgBetter 调用内复用 SMAWK 顶层及逐递归深度缓冲区；零缩放利润层不再复制/交换 DP 行；回溯表改为只包含正利润用户的连续 `int` 决策行，每行仍覆盖完整 `0..P`。保留原始候选数 `n`、`epsilon/c/delta/P`、浮点表达式、容差、转移与平局顺序；不改 AlgFast、多 UAV 贪心、残余分配、公共接口、单线程执行或现有 Release/IPOPT ABI 设置。工作区没有全局或跨调用缓存。

独立验证基线来自修改前的当前工作树。启用 `TON_VERIFY_SMAWK` 的合成对照通过 236 个案例、3256 条内部记录、123 次 UAV 选择记录；输出浮点字段逐位一致。工作区在 10 种规模各重复 12 次，初始化后地址和容量不变。实际生产源码及独立测试/性能版本均在原 Release|x64 配置下构建成功（0 错误，有既有代码/依赖警告）；生产入口没有执行。

只读使用 EXP1 的 1000/3000/5000 用户各实例 1，保持 10 架 UAV、40 MHz、`epsilon=0.1`。每个版本先预热一次，再串行交替测三次；每次独立进程运行，加载/模型复制/验证均不计时。优化前后所有输出及另行采集的选择顺序一致。下表是**每个规模一个输入、三次重复**，不是全量实验均值，也未续写任何原 CSV。

| 用户数 | 基线均值 / 中位数（秒） | 优化后均值 / 中位数（秒） | 中位数提速比（基线/优化后） |
| --- | --- | --- | --- |
| 1000 | 0.557420 / 0.556097 | 0.523228 / 0.531617 | 1.046× |
| 3000 | 4.345278 / 4.360617 | 4.180363 / 4.160677 | 1.048× |
| 5000 | 11.999577 / 12.033864 | 11.262405 / 11.360608 | 1.059× |

三组各 55 次 DP 调用的决策元素存储量合计分别减少 6.28%、4.86%、5.06%；最大单次决策表大小没有下降。这里统计的是逻辑决策元素字节数，不是进程峰值内存，也不包含工作区、行映射及分配器开销。逐次计时、源基线、测试驱动和核验报告保存在本地忽略目录 `x64/ApprBetterValidation/20260907_222034/`，不属于正常实验输出或新的自动测试框架。后续正式测量应选择新 `output_name`，不要把优化前后的耗时续写进同一批结果。

### 1.1.1 独立的新接口：AlgBetter_singleUAV_ToN_faster

新函数已通过 selector `3` **接入公共正式调度**；旧单 UAV 接口保留。AlgFast、多 UAV 贪心和残余带宽路径不变。也可显式调用：

```cpp
// 参数和返回值沿用旧单 UAV 接口；公共 ApproBetter 已使用对应的多 UAV selector 3。
auto result = problem.AlgBetter_singleUAV_ToN_faster(
    uav, candidate_users, current_utilities, base_bandwidths, 0.1);
```

新接口按用户 ID 确定处理顺序，不修改模型或输入。内部通过局部编号复用原校验和 AlgFast，避免按全局用户数分配临时数组；返回时恢复原用户与 UAV ID。它返回新增带宽与重新计算的真实边际效用，不要求与旧版分配逐位相同。

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
- **AlgHardFirst**：Youssef-inspired subchannel DA adaptation。hard/elastic 均逐槽申请，由槽侧比较新申请者和原暂存用户并同步更新；hard 未达标时释放后改投，有限恢复后冻结达标者，再执行 elastic DA。需求归一化偏好、单 UAV 约束、恢复及 elastic 增量是本项目适配，不宣称原文完整复现、经典稳定性或全局最优。
- **AlgSA-DD**：由 Li [16] 的 successive approximation / dual decomposition 思想适配。保留外层 theta 更新、每 UAV 的对偶价格/KKT 二分求解，以及本项目新增的 IPOPT warm start/refinement 和原有可行性恢复。默认最强容量初始化只在一条关联上给带宽；初始效用为正时，其他链路 theta 为零，通常不能在后续激活。现有初始化总 surrogate 效用不超过 `1e-12` 的兜底分支使用均匀 theta，不能把限制绝对化为“所有情形固定关联”。本次保留并记录这一分支，不扩展为一般联合关联算法。原论文显式更新用户价格 lambda 和节点价格 mu，当前内层并非逐项复现。

求解器创建/初始化失败会终止该方法；Matching 或 SA-DD 已存在的可行回退可以保留，但诊断会记录阶段、求解器返回码及是否使用回退，最终仍必须通过公共校验。

[MatchingGameAllocation.cpp](MatchingGameAllocation.cpp) 参与编译但未进入正式六方法循环。`Convexrelaxationandrounding_new.cpp`、`EntityDefinition_short.cpp`、`Matchinggameallocation_old.cpp` 和 `setSysCode/` 中的相似文件也不能凭文件名认定为当前实现；以 [vcxproj](UAVBandwidthAllocation.vcxproj) 编译清单为准。`../baselineAlgorithms/DRL_Algorithm_3/` 是未接入的本地 Python 原型，不作为新六方法中的 DRL 证据。

### 1.3 AlgHardFirst 的资源与确定性规则

默认有效子信道宽度为所有用户共有的 `BSub=0.18 MHz`，槽宽为有效宽度的 `10/9`，即 `0.20 MHz = 0.18 MHz 传输 + 0.02 MHz 间隔`。槽数按总带宽除以槽宽向下取整；整数附近只吸收机器舍入误差，不使用 QoS 容差扩大槽预算。

hard 在链路上的需求是通过公共 `hard_qos_satisfied()` 的最小正整数槽数，通过有界二分确定；用完该 UAV 的槽仍不达标则排除。零最低速率仍占一个正槽；零权重 hard 不因效用为零而被人数目标排除。

- 用户按单槽可达速率降序选择 UAV，平局按 UAV ID；同 UAV 内按本地槽 ID。采用游标访问实际槽，不复制同质容量矩阵。
- hard 槽侧偏好：零最低需求优先，然后单槽可靠速率/最低需求降序、达标效用降序、用户 ID 升序。最小达标槽数只筛选可行链路和检查 QoS，不一次性预留资源。
- 每轮每个未达标 hard 至多申请一个槽。收齐申请后比较新申请者和原暂存者，再统一提交；当轮达标者停止申请，但在当前 pass 内仍可被替换、丢槽后重新激活。
- 当前 UAV 的申请列表耗尽仍未达标时，释放全部部分分配，下一轮改投下一架 UAV；任意时刻不在多个 UAV 持有资源。一次 pass 中每个用户—槽对至多申请一次。
- 主过程结束后冻结达标 hard，在空闲槽上为未服务 hard 重建 DA 申请列表。恢复过程有新增达标用户才继续，无新增则停止；不再调用成组成批补充接纳。
- elastic 只访问 hard 之外的槽，也逐槽申请。槽侧使用轮初槽数计算真实对数增量，原占有者先排除争议槽；所有比较完成后统一更新。已有正分配时不跨 UAV 迁移，全部丢失且当前列表耗尽才可改投；没有全局最大堆。
- 输出只记有效传输带宽。保留整数槽、含间隔预算、hard 达标及无多余整槽校验，不通过删用户或改带宽掩盖失败。
- diagnostics 的单个 JSON 事件记录 hard/elastic proposals、replacements、rounds、hard reactivations/switches/released_slots、recovery_rounds/admissions 以及候选申请上界。不生成每槽日志，不填充空转轮次，不设运行时间目标。

**比较口径：只有 AlgHardFirst 承担该离散槽和 10% 间隔开销，其他五种方法及公共信道模型保持原样。性能差异同时包含调度策略、离散化和资源开销差异，不能全部归因于 hard-first 策略。历史同名 AlgHardFirst 实际为 Hungarian，不能与此版本混用。**

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

固定批次当前已完成，**不要重新运行生成器**。以下设置说明仅用于追溯 [generate_instances_ToN.py](../../ExperimentsData/DatasetTest/instance_generator/generate_instances_ToN.py)。它的 `main()` 固定 `batch_date="2026-09-07"`、生成种子 `20260904`、`replicate_count=30`，检查所有输入文件后才写 `generation_status="complete"` 的配置。生成完成后，不要为 C++ 续跑或扩样重新运行生成器。

然后打开本项目解决方案，选择 **Release / x64**，在 [main.cpp](main.cpp) 的参数区查看设置，由用户编译并按 F5 或 Ctrl+F5 执行。项目调试命令参数留空；任何旧 CLI 参数均被拒绝。**数据生成、编译及正式实验均由用户显式启动，C++ 不自动调用 Python。**

| 参数 | 第一轮默认值 | 用法 |
| --- | --- | --- |
| `instance_count` | 10 | 本轮固定 ID 1–10，EXP5 使用十网络；临时入口拒绝改为 30 |
| `master_seed` | 20260905 | 保持同一实例/方法的随机种子稳定 |
| `rounding_trials` | 2 | AlgRelaxRound 的既有舍入次数 |
| `ton_epsilon` | 0.1 | 算法范围不变，但本轮临时入口固定为 0.1 |
| `input_root` | `ExperimentsData/data_ToN/2026-09-07` 的绝对路径 | 只控制输入；固定批次不随续跑日期变化；结构体默认值仍是旧 `data` 以兼容已有调用 |
| `output_name` | `run_ton_01` | 仅字母、数字、下划线或连字符；不是完整路径 |
| `conditions` | 空列表 | 三个实验各自使用完整标准条件；EXP5 使用独立 RMSE 矩阵 |
| `reuse_exp1_root` | 空字符串 | 后续运行保持为空，不再指向下文的历史来源路径 |

`main()` 当前只调用 `rerun_proposed_exp1_exp3(options)`，成功后再调用 `exp5_different_location_error(options)`；EXP4 调用保留为注释。任一阶段失败即停止，不重试、不进入后续阶段。同一入口重启只恢复新版检查点；不把旧 CSV 视作新计算。旧临时 HardFirst/算法试运行代码不恢复。

本轮定向重跑为 EXP1/2/3 的 100/80/120 次，加上 EXP5 的 780 次，共 1080 次。输入根目录、输出名、ID 1–10、种子、舍入次数和 epsilon 均由临时入口严格固定。以后扩样须另行显式调整入口，不能只将本轮 `instance_count` 改为 30。

`config/route_a_run_options.json` 已不参与运行。各实验的物理参数仍读取对应的 `ExperimentsResults/EXP*/def_config.json`，没有修改原有信道配置。

| 驱动 | 标准条件 | 输入子目录（相对 `input_root`） | 每 UAV 带宽 |
| --- | --- | --- | --- |
| EXP1 用户数 | 1000/2000/3000/4000/5000，10 UAV | `variable_user_num/<N>u_num/` | 40 MHz |
| EXP2 UAV 数 | 5/10/15/20，3000 用户 | 用户来自 `variable_user_num/3000u_num/`；UAV 来自 `variable_uav_num/<K>/` | 40 MHz |
| EXP3 hard 比例 | 0/2/4/6/8/10，代表 0/0.2/…/1.0 | `variable_hard_user_ratio/<ratio>/` | 40 MHz |
| EXP4 带宽 | 10/20/30/40/50 | `variable_user_num/3000u_num/`，10 UAV | 条件值，MHz |

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

### 4.0 本轮定向替换的专用记录

每个已有 `run_ton_01` 下新增 `rerun_proposed/ton-proposed-fast-faster-v1/`：

- `state.json`：运行身份、源版本、输入/配置 SHA-256、构建设置、基线校验值及恢复阶段。
- `backup/`：原两个提出方法 CSV、12 张汇总和运行信息；一次性备份，不覆盖、不删除。
- `new_results/`：逐实例原子保存的新版提出方法 CSV，不向正式旧文件追加。
- `candidate/`：完整待发布文件，发布中断后保留并校验。

先只读检查三个实验及全部输入，再完成全部备份，最后计算。恢复阶段为 `backing_up → computing → publishing → complete`。过渡期正式汇总标为 `stale`/`writing` 且保留旧版本；一个实验全部文件和汇总发布、基线校验通过后，才切换为规范的新版本元数据。发布目标只能是旧内容或精确的新候选，第三种内容报错停止。

主入口用 Windows 命名互斥对象限制并发，退出自动释放，不创建锁文件。旧程序不认识此锁，仍须手动确保无其他写入者。原子性是逐文件重命名，不承诺整目录事务或断电耐久性；临时文件保留，不计入完成记录。

**下面的普通全方法驱动说明不适用于上述临时替换流程；本轮会额外生成 4.0 的检查点和备份。**

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

正常结果为 `SUCCESS` 或合法的 `ZERO_ALLOCATION`；后者的人数/效用/资源/速率为零，但保留实际耗时。`diagnostics` 是 CSV 转义后的 JSON，保留既有回退信息；DA 另在 events 中追加一条实际状态转移计数 JSON。兼容列 `elapsed_ms` 和指标 `duration` 值相同。

计时边界沿用原实现：`steady_clock` 浮点毫秒，记录求解调用及其已有内部步骤；模型准备、公共指标计算、CSV 写入不计时。HardFirst 原有包装层中的槽合法性检查位置不变。

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

`run_info.json` 记录实际参数、物理配置数值、条件/输入路径及 ID、算法版本、IDE 构建配置、复用来源和汇总状态。当前算法组合版本为 `ton-proposed-fast-faster-v1`，记录 `proposed_algorithms` 的实际 selector/入口，以及 `hard_first_da_policy`（偏好、单 UAV 改投、有限恢复和有效/占用宽度口径），与其他设置一起严格比对。旧成组版本和缺少/改变策略的记录不能续写；`run_01` 保持原样。它不保存输入或源码副本，也不核对文件内容哈希；**同路径输入内容、算法代码或依赖发生变化时，应由维护者换新输出名称，不能依赖本框架自动识别这些变化。**

`summary.instance_count` 表示已有汇总对应的每条件样本数；`summary.status` 为 `not_generated`、`stale`、`writing` 或 `complete`。只有 `complete` 且样本数等于当前目标时，才是当前完整汇总。

扩大样本时保留旧 summary，控制台明确提示其旧样本数；新计算未全部完成前不发布新的部分均值。所有条件、方法和目标实例齐备后才更新 12 张表，写表前标记 `writing`，全部成功后标记 `complete`。写入途中中断时不信任这些表，重启后可用已保存原始行重新生成。

保留 `Run_time_ms.csv`、`Total_Num.csv` 等原文件名，首列仍为 `User_Scale`，含义取决于实验条件。人数均值使用浮点数，不截断，也不以零或成功子集替代缺失样本。

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

上述数量是先前 `run_01` 初始方案审计时的历史快照：当时四种可复用方法共有 **176 条 SUCCESS**，目标合计 300 条，其余 124 次需计算。它不表示现在仍有 124 次待运行，也不代表当前默认 DA 的运行方式；当前完成情况以 `run_01` 中实际 CSV 和 `run_info.json` 为准。

当时未导入旧 `ApproBetter` 和 `AlgHungarian`，也未将旧 Hungarian 行改名为新 `AlgHardFirst`。后续 HardFirst 定向重跑的版本与完成状态由 `run_info.json` 的 `hardfirst_rerun` 单独记录，其他方法保留原有来源信息。

当时的导入检查包括源版本、所选方法、物理/运行参数、输入路径/ID、表头、成功状态及种子，不代表未来换模型、输入内容或环境后仍可直接沿用。保留结果中的历史耗时仍是旧测量，不能描述为新程序重新测得。

EXP2–EXP4 的旧批次没有结果，日后由你选相应入口完整运行。

### 5.1 验证说明

开发期间已进行独立构建和合成验证，日志保留在隔离验证目录。按精简要求，工程不再保留独立测试源码或测试构建分支，仅使用正式 `main.cpp` 入口；算法内必要的槽预算、唯一关联和 hard QoS 检查保留。开发验证不等同于正式实验结果，也不要求 DA 比 ApproFast 慢或效用必然更高。

## 6. 论文描述建议（仅供复制，未修改 TeX）

沿用用户提供的引用编号：Han–Wang (2024) [12]、Li et al. (2024) [16]、Tian et al. (2024) [27]/[28]、Youssef et al. (2020) [34]。这些编号依赖当前稿件的参考文献顺序，粘贴到其他版本前应转换为相应真实 cite key，不要机械沿用数字。

可复制的基准描述（不是当前实验已完成的声明）：

```text
Because the cited works address resource-allocation problems that differ
from ours, we distinguish adapted heuristics from independently constructed
baselines under our user-association and bandwidth-allocation model.
AlgRelaxRound solves a continuous relaxation, performs randomized rounding,
and reoptimizes bandwidth under each sampled association. AlgSwapMatching
is a one-pass beneficial-swap association heuristic with per-UAV continuous
bandwidth optimization, inspired by Han and Wang [12]. It uses surrogate
utilities, followed by hard-QoS recovery; bandwidth is not reoptimized after
the swap-search stage. AlgHardFirst is inspired by the priority-aware
matching framework of Youssef et al. [34]. It provisionally accepts or rejects
individual subchannel proposals from hard users, releases incomplete
allocations before trying another UAV, and performs finite recovery on free
subchannels while freezing satisfied users. Elastic users subsequently apply
to the remaining subchannels, whose preferences use round-start marginal
utilities with the contested subchannel excluded for its current owner.
Each user is associated with one UAV and may receive multiple subchannels.
Only this baseline uses 0.20-MHz resource slots with 0.18-MHz effective
bandwidth; the other five methods and common channel model are unchanged.
It is an adapted heuristic, not a full DA/NOMA reproduction or a globally
optimal hard-admission algorithm. AlgSA-DD is a
successive-approximation and dual-decomposition heuristic adapted from
Li et al. [16], with strongest-link association initialization and an
additional IPOPT refinement step. Its default positive-utility initialization
generally leaves initially inactive links inactive, whereas the retained
zero-utility initialization fallback assigns uniform approximation weights.
```

实现/证据边界建议：

- 删除“AlgDRL 实现 Tian 的 DRL 抢占方法”的对应关系。[27]/[28] 可作为相关工作保留，但不是当前 AlgRelaxRound 的算法来源。
- 成组 v2 和新 DA 的 AlgHardFirst 都先 hard 后 elastic；更早同名算法及 v1 AlgHungarian 没有这一机制。引用 [34] 时只归属其优先级匹配思想，不借用原文的动态时延指标、NOMA 功率/SIC 机制或稳定性结论。
- 不将 AlgSwapMatching 说成最大权匹配或交替优化到收敛；不将 AlgSA-DD 说成原文完整复现或一般联合关联求解器。
- 不保留缺乏共同依据的“所有基准复杂度不低于 `O(K n^3)`”断言。新 AlgHardFirst 不构造用户—子信道方阵。复杂度应分别统计每个 pass 的用户—槽申请、同步轮次、实际替换及有限恢复，不再包含全局 elastic 堆；不能继续引用旧 Hungarian 的三次复杂度，也不能借用原文复杂度作为适配实现保证。
- 默认 N=10 是运行配置，不是完成实验的证据。未经正式重跑，不能声称“50 different networks”，也不能把旧均值改标签后当作修复版结果。
- 运行服务器配置和时间比较应以实际新构建/运行环境为准，不能沿用未核实的硬件或将合成测试耗时作为正式 benchmark 时间。

## 7. 维护与边界

[main.cpp](main.cpp) 是参数和实验选择入口；[experiments.h](experiments.h) 负责条件、参数记录、旧行导入及独立续跑；[experiment_support.h](experiment_support.h) 负责六方法调度、轻量物理检查、CSV 和汇总；[allocation_contract.h](allocation_contract.h) 保留共同容差和算法状态。

此前已将第五个基准替换为逐槽 DA，并完成 ApproBetter 优化。本次只增加固定 ToN 输入衔接、生成完成检查和四实验直接调用，不修改六种算法、物理模型、求解器或工程编译配置，也不触碰历史结果。独立测试框架不恢复；源码中的中文按 UTF-8 保存，带 BOM 的原代码文件继续保留 BOM。

修改算法、数据内容或依赖后，维护者应更新适用的版本标识并使用新输出名称；本框架故意不做源码/输入的全量快照或自动版本漂移监控。绘图独立使用已完整汇总的数据，本轮不自动生成图片。

构建/合成测试不等同于正式实验复现。本轮不运行正式实验、不导入记录、不生成正式数据、不暂存、提交或推送；实际构建及测试结果以独立验证目录中的日志为准。当前仓库未发现许可证文件，对外分发前仍应确认代码、依赖和数据授权。

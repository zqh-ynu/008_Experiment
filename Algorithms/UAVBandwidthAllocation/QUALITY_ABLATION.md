# Better / Fast 同实例质量消融

本入口用于判断当前效用重合来自证书早退、离散精度，还是当前实例上没有找到改进。
不降低 Fast 的能力，不预设 Better 必须取胜，不把重复条件当成独立网络。

## 当前交付与执行边界

- 已实现独立驱动、显式设置、诊断、历史复现检查和纯合成测试代码。
- 本次交付仅作静态检查，**未进行 Visual Studio 编译、未执行合成测试、未运行 50 次真实网络消融**。
- 默认不定义新宏，原正式入口与六方法调度不变。原输入、`run_ton_01`、四个基准、EXP4 和 EXP5 均不修改。
- 两个新入口不会续接正式重跑或 EXP5。不得同时定义两个入口宏。

## 在 Visual Studio 中手动操作

使用现有 `UAVBandwidthAllocation.sln`，选择 **Release | x64**，保留现有 IPOPT/CRT 配置。
在项目属性 → C/C++ → 预处理器 → 预处理器定义中添加以下宏；保留既有定义和继承项。

1. **先验证纯合成测试**：仅添加 `TON_QUALITY_ABLATION_TESTS`，重新生成并运行。
   成功时控制台出现 `[QUALITY SYNTHETIC PASS] assertions=...`。测试只在内存中构造小模型，
   不读正式 CSV，不创建结果目录；失败返回 1。通过这一关不等于正式项目默认行为或真实实验已验证。
2. **再运行真实消融**：移除测试宏，添加 `TON_QUALITY_ABLATION`，关闭 `TON_VERIFY_SMAWK`，
   重新生成并运行。控制台先打印 `[QUALITY OUTPUT]` 和新目录，再逐项打印 `[QUALITY x/50]`。
3. **查看是否完成**：要求 `completion.json` 为 `complete`、`completed_network_calls=50`，
   且 50 个 case 都有 `complete.json`。出现 `failure.json` 或缺少完成标记均表示未完成。

不要仅用“程序退出”判断成功。未定义任何新宏时执行程序会回到原来的正式入口，
因此每次运行前应确认预处理器定义和已重新生成的可执行文件。
合成测试模式与真实消融模式都不需要命令行参数。

## 固定实验矩阵

使用现有 `data_ToN/2026-09-07/variable_user_num/3000u_num` 的 ID 1–10，
3000 用户、10 UAV、每架 40 MHz；配置来自 EXP1 的 `def_config.json`。
输入加载、公共校验与 seed 规则沿用当前实验：`master_seed=20260905`；
Fast 使用 `ApproFast` 的逐实例 seed，所有 Better 设置使用 `ApproBetter` 的对应 seed。

| 设置标签 | selector | ε | 证书早退 |
| --- | --- | --- | --- |
| Fast | 1 | 不适用 | 不适用 |
| Better_current | 3 | 0.1 | 开启 |
| Better_no_early_e010 | 3 | 0.1 | 关闭 |
| Better_no_early_e005 | 3 | 0.05 | 关闭 |
| Better_no_early_e001 | 3 | 0.01 | 关闭 |

每个输入加载一次，依次运行五种设置，总计 50 次网络求解，不另外重算四个基准。
关闭早退仍执行相同证书搜索并记录是否满足，区别只是通过证书后不立即返回。
纯凹子问题继续精确注水；非凹 DP 之后继续保留 Fast 可行解作为 incumbent。
单 UAV 保优不等于多 UAV 逐实例占优，负差值必须保留。

## 输出与完成语义

每次手动运行都原子预留一个新目录：

`ExperimentsData/ExperimentsResults/EXP1_user_num/ToN_quality_ablation/exp1_3000_ids1_10/run_<时间标识>_<序号>/`

不恢复旧目录，不删除文件，不覆盖任何现有结果。失败后再次运行会创建另一目录；
中断实例不能用空值或零值补齐，之前完成的 case 与临时文件保留供检查。

| 文件 | 内容 |
| --- | --- |
| `run_info.json` | 不变的运行身份、五设置、输入/配置/历史 CSV 的 SHA-256、编译信息、诊断开关和计时口径 |
| `cases/id_<ID>/<设置>/result.csv` | 一次求解的状态、seed、耗时及全部原始物理指标；求解失败的指标留空 |
| `cases/id_<ID>/<设置>/trace.json` | 完整冻结状态、候选 ID、调用轨迹、选中次序及残余前后总效用 |
| `cases/id_<ID>/<设置>/oracle_calls.csv` | 不含大型状态向量的逐次调用指标；`state_id` 指向同一 case 的 `trace.json` |
| `cases/id_<ID>/<设置>/validation.json` | 公共校验状态及 Fast/当前 Better 相对历史记录的精确相等、容差相等、差异明细 |
| `cases/id_<ID>/<设置>/complete.json` | 该 case 的求解与验证完成标记；失败或中断没有此标记 |
| `paired_comparisons.csv` | 50 行同实例配对：含 10 行 Fast 自比较与 40 行 Better 对比；保留绝对/相对/Hard/Elastic 差值及胜平负 |
| `summary.csv` | 五设置各 10 网络的全部指标均值、提升的均值/中位数/最小/最大、胜平负及相对提升有效样本数 |
| `diagnostic_summary.csv` | 各实例各设置的候选/残余/unsafe/证书/DP 计数，另列实际选中贪心调用计数 |
| `completion.json` | 仅全部 50 次完成、输入完整性复核和汇总写出后产生 |
| `failure.json` | 异常时尽力保存当前实例/设置、已完成数和错误；不会自动重试 |

`run_info.json` 本身不声明运行完成；只有最后的完成标记才声明完成。
其中 `runtime_disk_sources_not_build_snapshot` 是运行时磁盘源文件指纹，不能当作编译源码快照。
实际运行的可执行文件 SHA-256、编译日期、时间和 MSVC/STL 信息另行记录，
不以文件名或旧 README 推断二进制身份。旧对照在读取后和最终发布前再次做指纹核对。

## 诊断字段及解释

- `certificate_attempted`：这次是否进入原有证书搜索；未尝试时 `certificate_satisfied` 为 null。
- `best_upper`：原搜索实际计算出的最小有限上界；不是全网最优值，也不是保证求到了最紧对偶界。
  未计算时为 null/空单元格，不能当作零。上界仍受现有浮点实现容差约束。
- `certificate_fast`：证书通过且允许早退，直接返回 Fast。
- `concave_candidate` / `concave_fast_incumbent`：纯凹分支分别选择精确注水候选或保留 Fast；未进入 DP。
- `dp_candidate` / `dp_fast_incumbent`：进入 DP 后分别选择新候选或保留 Fast。
- `unsafe_count`、`unsafe_user_id`、`unsafe_bandwidth`、`unsafe_tau`：Fast 原始 LCM 舍入信息；
  Faster 内部局部 ID 已还原为原用户 ID。未调用 Fast 的空分支须结合 `fast_value=null` 识别。
- `gain_vs_internal_fast`：本次相同状态下最终 oracle 值减去其内部 Fast 值。
- `same_state_as_fast` / `delta_vs_fast_path`：与相同实例 Fast 路径的同轮同 UAV 调用比较；
  必须候选顺序、预算、完整状态向量逐值相同才产生差值，否则留空。
  `state_id` 仅在本 case 内有效，不能按不同文件中相同编号判定状态相同。
- `certificate_satisfied_calls / certificate_attempts` 才是实际搜索的证书命中比例。
  `selected_early_returns` 与 `selected_dp_calls` 只统计实际选中的贪心调用，避免候选重复计数混淆。

先比较 `Better_current` 与 `Better_no_early_e010`，再比较无早退的 ε 梯度。
若局部有改进而网络无改进，结合选中次序、状态分叉和残余效用分析。
所有设置均相同只意味着本次算法/精度未找到改进，不能仅凭重合认定最优。

## 验证与计时口径

- Fast 与当前 Better 的历史对照要求完整 ID 1–10，运行身份和 seed 匹配。
  非耗时指标采用现有 `1e-9 + 1e-8 * max(|a|,|b|)` 容差；人数要求精确相同，另报所有数值是否精确相同。
  不匹配时先保存现场再停止，不改旧记录。
- 公共物理校验重算预算、覆盖、唯一关联和 Hard QoS；消融额外核对缓存效用与逐用户投影。
- 合成测试穷举三 Hard 背包 OPT=19，检查 Fast=12、Better=19；另测可认证早退、
  强制继续、纯凹、多 UAV、零预算、非法 ε、状态比较和零分母。
  每设置有诊断开启/关闭结果精确一致检查；这不是未修改二进制与新二进制的逐位证明。
- 五设置在每个输入上各运行一次，没有重复计时或预热；诊断收集计入 `elapsed_ms`，
  输入加载、模型复制、校验与文件序列化在计时外。因此本轮耗时是探索性开销，不是正式性能数据。
- 另行测量诊断关闭开销时，可以显式将消融入口改为 `ton_quality::run(false)`，重新生成后运行，
  同样会创建新目录并进行 50 次求解。不应将它与原诊断结果混写或直接作为正式计时结论。

## Material Passport

- Origin Skill: academic-research-suite / experiment-agent
- Origin Date: 2026-09-09
- Version Label: ton-quality-ablation-v1
- Verification Status: UNVERIFIED（交付时仅静态检查；编译、合成测试、真实网络执行由用户手动完成）

实验版本与记录可参照既有《10_Codex工作流使用教程》；本实现不改写 Obsidian 笔记。

# EXP5 定位误差敏感性实验与兼容预实验

本模块测试**固定且准确的 UAV 部署下，用户水平定位误差对六种分配方法的影响**。
它不修改算法，不模拟移动/视觉检测/逐包衰落，不代表定位系统实测或完整跨地区结论。

## 手动入口

当前 `main.cpp` 在前三个实验的提出方法定向重跑和结果发布全部成功后，调用
`exp5_different_location_error(options)`。由用户在 Visual Studio 以现有 Release|x64 配置手动编译、运行。
旧的 `run_localization_pilot` 开关分支已撤掉；EXP4 本轮暂停。

统一包装函数通过 `localization::from_experiment_options()` 传递实例数、输入根目录、算法种子、
舍入次数、epsilon 和输出名，不接受条件过滤或旧结果导入。本轮设置如下：

- ID **1–10**，3000 用户、10 UAV、每 UAV 40 MHz，保留输入中的业务、权重和 QoS。
- 输入固定 `data_ToN/2026-09-07`；算法主种子 `20260905u`，`epsilon=0.1`，舍入 2 次。
- 水平 RMSE：0、1、2、5、10、20、50 m，非零档位各 2 次；误差主种子 `20260908u`。
- 全部六方法，每网络/方法零误差只算一次，合计 **`10*(1+6*2)*6=780`** 次。
- 新输出 `ExperimentsData/ExperimentsResults/EXP5_location_error/ToN_simple/run_ton_01`。
- `ApproBetter` 经公共调度使用 selector `3` / `AlgBetter_singleUAV_ToN_faster`；`ApproFast` 为 selector `1`。
- 零误差参照在 EXP5 内新算，不从 EXP1 的指标 CSV 导入。

`localization::Options{}` 与 `localization::run_pilot()` **仍保留以下三网络兼容默认值**，主入口不使用这些默认值：

- 输入：`ExperimentsData/data_ToN/2026-09-07/variable_user_num/3000u_num`；
- ID 1--3，3000 用户、10 UAV、每 UAV 40 MHz，保留输入中的业务、权重和 QoS；
- 六方法使用现有顺序及参数，`epsilon=0.1`、随机舍入 2 次；
- 水平 RMSE：0、1、2、5、10、20、50 m；非零档位各 2 次独立误差重复；
- 每网络/方法只计算一次零误差基线，合计 `3*(1+6*2)*6=234` 次；
- 物理参数只读取 `ExperimentsData/ExperimentsResults/EXP1_user_num/def_config.json`，
  不是工程 `config/def_config.json`。覆盖沿用 600 m **三维距离**；
- 新输出：`ExperimentsData/ExperimentsResults/EXP5_location_error/ToN_simple/pilot_01`。

旧 `pilot_01` 原样保留，不迁移、不覆盖、不续用其旧版本记录；当前十网络输出与之隔离。
目录仅在明确运行入口时创建。改变参数、输入标识、算法版本或记录的构建设置需换 `output_name`，
不会自动扩样。主入口的 Windows 命名互斥对象覆盖整个串行流程；旧程序和直接调用者仍须确保没有其他写入者。

## 误差和评价规则

原始文件只读。完成一次坐标转换后保留真实模型，每个误差条件复制模型，只修改用户 X/Y，
再调用原有 `init_SystemModel()` 重建全部派生信道状态。对于水平 RMSE $d$：

$$
\widehat x_i=x_i+\frac{d}{\sqrt2}z_{i,x},\qquad
\widehat y_i=y_i+\frac{d}{\sqrt2}z_{i,y},\qquad z\sim\mathcal N(0,1).
$$

同一网络/重复编号的标准高斯样本跨档位缩放；所有方法共享估计模型。误差种子由
`error_master_seed=20260908`、网络 ID、重复编号派生，使用独立的 `std::mt19937` 和
`std::normal_distribution<double>`。不裁剪越界坐标、不重采样、不强行归一化经验 RMSE。
方法种子沿用 `derive_algorithm_seed(20260905,"EXP1_user_num","3000",id,method)`，
不随 RMSE/误差重复改变。标准库的 normal_distribution 不承诺跨实现逐位一致；
运行信息记录 RNG 规则和 MSVC/STL 设置，换工具链应使用新的输出名。

算法只看到估计模型。其输出必须先通过现有严格校验；随后在真实模型上评价冻结的关联和带宽。
真实覆盖外效用为零；覆盖内 Hard 仍使用原可靠速率门槛和数值容差；Elastic 使用原对数效用。
失效用户占用的带宽仍计入消耗，不回收、不补带宽、不重新分配。

## 结果和断点

每个完整单元保存两份 CSV：

```text
run_ton_01/                 # 旧三网络默认接口对应 pilot_01
  run_info.json
  cases/id_1/rmse_0/rep_0/ApproBetter/
    hard_users.csv
    result.csv
  ...
  raw_results.csv
  network_means.csv
  overall_means.csv
```

`rep_0` 仅用于零误差。非零档位使用 `rep_1`、`rep_2`。
明细先用临时文件写完、关闭、同目录重命名；汇总最后发布。恢复时两份文件必须完整且互相一致
才跳过。缺少任何一份是未完成单元，会重新计算；已有完整单元拒绝覆盖。
存在但格式损坏/内容矛盾的记录会报错停止，不自动覆盖。中断留下的 `.tmp_*` 不算结果，
程序不删除它们；需要清理时由用户确认后处理。
只有汇总、缺少明细时，先将孤立汇总保留为 `result.csv.incomplete_*`，再发布新计算的完整文件对，
避免再次中断时将旧汇总与新明细混合。

`run_info.json` 的 `summary.status=complete` 仅在全部预期单元完整、三份派生汇总均发布后写入：本轮为 **780** 个，旧三网络默认接口为 234 个。
未完成运行中的旧派生汇总不能当作当前完整结果。原始记录不会导入旧实验 CSV。
输入身份使用绝对路径、文件大小、修改时间和现有批次标识；不是内容哈希或源码快照验证。
运行信息还记录实际 Matching/SA-DD 默认参数及三个 IPOPT 选项文件的内容；它们变化时也拒绝续写旧目录。

`result.csv` 保留估计/真实条件下的 11 项非耗时原指标及一次算法计时，还包括：

- 实际水平 RMSE；零误差基线效用；`utility_loss=1-realized_total_utility/baseline_utility`；
- `hard_service_ratio`：真实达标人数 / 输入全部 Hard 人数；
- `hard_admission_violation_ratio`：真实未达标的已接纳 Hard 人数 / 已接纳 Hard 人数；
- `outside_coverage` 与 `hard_outside_coverage`：已分配用户中真实关联链路覆盖外的人数；
- `hard_mean_shortfall`：已接纳、最低速率为正的 Hard 用户的平均相对可靠速率缺口，
  缺口为 $\max(0,1-R_i^{true}/r_i^{min})$，覆盖外 $R_i^{true}=0$。

`estimated_*` 是严格估计条件下的指标；`realized_*` 是真实条件下的指标。
两类 bandwidth 字段均为实际占用的分配带宽，包含真实服务失败者。
Hard throughput 仅累加达标用户的最低业务速率，而不是超过门槛的物理速率。
带宽单位 kHz，速率 Kbps，坐标 m，耗时 ms；耗时不包含扰动、模型重建、真值评价和写盘。

`hard_users.csv` 保留全部 Hard 用户，包括未接纳者。`user_id` 是 C++ 读取后先 Hard 后 Elastic
排列的内部 ID，不是源 CSV 的 user_id。未接纳用户为 `admitted=false`、`uav_id=-1`、
带宽 0，关联相关字段为空，原因 `NOT_ADMITTED`；不混记为接纳后违约。
已接纳用户的原因分为 `MET`、`OUTSIDE_COVERAGE`、`NO_RELIABLE_CAPACITY`、`RATE_SHORTFALL`。
明细同时保留估计/真实可靠频谱效率，便于独立核对带宽、可靠速率、达标状态与违约原因的一致性。
严格达标容差内仍可能存在很小的正缺口，这是两个指标定义不同，不扩大容差或截断缺口。

空单元格表示不可用（NA），不是零。零基线效用时相对损失为 NA；负损失表示该次启发式结果
实际效用高于零误差参照，应保留，不把基线冒充最优上界。

求解器失败、非法估计分配、写入失败：停止并保留之前结果，不写失败性能行、不自动重试。
真实覆盖或 Hard-QoS 违约：是有效观测，照常保存，不改变成功求解状态。

## 统计和验证边界

`raw_results.csv` 在全部单元完成后汇集 **780 行**（旧三网络默认接口为 234 行）。`network_means.csv` 先在网络内对误差重复
平均，报告 `valid_error_repeats` / `expected_error_repeats`；`overall_means.csv` 对网络均值
等权平均并报告 `valid_networks` / `expected_networks`。NA 不填零，零误差不复制重复。
两种规模均保持现有描述性统计，不自动添加显著性或置信区间结论。独立网络样本量本轮为 10，不是 780；各指标仍报告实际有效网络/误差重复数。

`tests/localization_tests.cpp` 是独立的小型确定性验证入口，**不加入正式工程的 ClCompile**。
它通过 `TON_RERUN_TESTING` 编译 `main.cpp` 内的临时辅助函数并排除正式 `main()`，不调用
`run_pilot()`、统一正式包装入口或正式定向重跑函数。所有求解器只接收小型合成模型；文件恢复测试仅在隔离目录使用合成记录。
它不读取正式用户 CSV、不创建正式 EXP5 或重跑结果；物理配置和求解器选项只读复用。
生产工程编译和正式实验运行仍由用户完成。

独立测试脚本为 `tests/run_localization_tests.ps1`；它创建新的
`x64/RerunValidation/<timestamp>_<id>`，保留测试构建产物和合成检查点，不删除已有目录。
使用 `-BuildOnly` 时只构建测试程序。脚本只构建带测试宏的辅助代码，不构建正式项目或执行正式 `main()`，不运行 300 次定向计算或 780 次 EXP5。
测试输出使用短子目录 `s/`，避免嵌套备份/候选文件及其临时名超过 Windows 路径限制。

2026-09-08 本轮最终隔离构建退出码 0，测试 **587 项检查通过**，包含原定位评价回归、selector 3/1 调度、
包装参数映射、SHA-256 已知向量、备份/逐实例/发布/最终元数据中断恢复、完成后重启、损坏和漂移拒绝、
基线/备份保护及失败不进入下一阶段。最终产物位于 `x64/RerunValidation/20260908_225441_4554e4c6/`。
此前路径长度测试故障和 Windows 头文件顺序编译故障均已修正；失败构建目录保留，不能作为通过证据。
这些结果不是正式工程构建或正式 1080 次运行的验证。

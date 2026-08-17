# `ExperimentsData/data`：主实验实例与配置说明

本目录在本机工作区中保存 MASS 主实验和历史兼容测试使用的 user/UAV 配对实例。本项目的 Git 快照**不提交**实例 CSV、ZIP 归档或历史部署 PNG；这些资产应继续保存在本机冻结备份或独立私有数据快照中。仓库仅保留本说明文件以及 12 个小型 JSON 条件配置，以同步生成意图和实验矩阵。

因此，克隆本仓库后不能直接运行 EXP1–EXP4。运行前必须按原相对路径恢复所需的实例 CSV，再从 `Algorithms/UAVBandwidthAllocation` 运行主工程。

## 同步边界

| 资产类型 | Git 快照策略 | 原因 |
| --- | --- | --- |
| user/UAV 实例 `*.csv` | 不同步 | 数量多、体积大，且是冻结输入而非处理逻辑。 |
| `*.zip` 历史实例归档 | 不同步 | 属于本机/独立数据备份，不应作为普通 Git blob。 |
| `County_loc_Type_req/simulated_data/*.png` | 不同步 | 历史部署可视化输出，不是生成逻辑或正式输入。 |
| `*.json` 条件配置 | 保留 | 记录用户数、hard ratio、实例数和 QoS 等生成条件。 |
| 本说明文件 | 保留 | 说明恢复位置、实例 schema、实验矩阵与复现边界。 |

不要在本说明中推断公开下载地址、校验和或数据许可。若需要恢复数据，应使用维护者管理的本机冻结副本或独立私有数据快照。这里保留的 JSON 是历史实例生成条件，不是 C++ 主实验运行时直接读取的通信配置；C++ 驱动实际加载各实验结果目录中的 `def_config.json` 并读取恢复后的 CSV。

## 条件目录与实验关系

| 本机目录 | 本机保存的主要内容 | 主工程中的用途 | Git 中保留的内容 |
| --- | --- | --- | --- |
| `County_loc_Type_req/` | 早期/兼容测试的 user/UAV 实例；`simulated_data/` 中还有历史部署图 | 历史手写测试和兼容性资产，不是当前 EXP1–EXP3 的核心输入 | `config.json` 与本 README |
| `variable_user_num/` | 用户数为 1000、2000、3000、4000、5000 的配对实例 | EXP1；其中 3000-user 条件也为 EXP2/EXP4 提供用户端输入 | 五个 `<N>u_data_config.json` |
| `variable_uav_num/` | 在固定 3000 用户下变动 UAV 数的 UAV CSV | EXP2；应与 `variable_user_num/3000u_num/` 的用户 CSV 配对 | 无实例文件；恢复时需重新建立该目录 |
| `variable_hard_user_ratio/` | hard ratio 为 0、0.2、0.4、0.6、0.8、1.0 的配对实例 | EXP3 | 六个 `u_data_config.json` |
| 根目录 ZIP | `variable_user_num.zip`、`data_hardweight_6-10.zip` 等历史归档 | 仅作本机数据备份/历史材料 | 不保留 |

现有条件矩阵为：

| 实验 | 自变量 | 固定条件 | 恢复输入时需要的目录 |
| --- | --- | --- | --- |
| EXP1 | 用户数 `1000 / 2000 / 3000 / 4000 / 5000` | 10 UAV | `variable_user_num/` |
| EXP2 | UAV 数 `5 / 10 / 15 / 20` | 3000 用户 | `variable_user_num/3000u_num/` 加 `variable_uav_num/` |
| EXP3 | hard ratio `0 / 0.2 / 0.4 / 0.6 / 0.8 / 1.0` | 3000 用户、10 UAV | `variable_hard_user_ratio/` |
| EXP4 | 每 UAV 带宽 `10 / 20 / 30 / 40 / 50` | 3000 用户、10 UAV | 历史上复用 `variable_user_num/3000u_num/`；当前标准配置链仍不完整 |

当前 C++ 驱动按实例 ID 排序后通常只执行每个条件的前 10 对输入；多数本机条件目录保存约 30 对实例。`variable_uav_num/25` 的少量历史 UAV 文件不遵循完整标准目录布局，不能据此视为另一项已闭环实验条件。

## 文件命名与 CSV schema

主实验通常按相同实例 ID 配对读取：

```text
用户：<instance>_<N>users_data_<adcode>.csv
UAV： <instance>_<K>uavs_loc_<adcode>.csv
```

用户 CSV 的字段为：

```text
user_id,longitude,latitude,user_type,user_weight,
user_requirement_1,user_requirement_2,app_category
```

UAV CSV 的字段为：

```text
uav_id,longitude,latitude,bandwidth
```

当前加载器以文件名前缀的数字 `<instance>` 配对并排序，通常应同时保持 user/UAV 的 source-id 一致，但加载器不会自动校验该 source-id。解析逻辑只将 `user_type == "hard"` 的用户识别为 hard 用户；工程读入后会重新编号用户与 UAV。EXP1–EXP3 还会用实验条件覆盖 UAV CSV 内的带宽容量，所以 CSV 的 `bandwidth` 字段不必等于一次实际运行的最终带宽参数。

这些 schema 和 JSON 条件配置保留在仓库中，用于理解或维护数据处理逻辑；它们不替代恢复实际历史实例。

## 与生成处理链的关系

最终实例的上游逻辑位于 [../DatasetTest/](../DatasetTest/)：人口栅格经裁剪后按权重抽样为基础坐标，随后赋予 hard/elastic 类型、权重、QoS 与业务类别，再由 Python 网格贪心部署器生成 UAV 坐标。完整说明见 [../DatasetTest/data/README.md](../DatasetTest/data/README.md)。

历史生成脚本仍存在旧绝对路径、接口漂移和未统一固定随机种子的情况。因此，即使保留 JSON 条件，也不能保证从当前源码不依赖原始/中间数据地逐字节重建既有 CSV。若需要可审计重现，应同时冻结原始输入、中间数据、随机规则、环境、运行配置和最终实例。

## 恢复实例后的运行前检查

1. 从独立数据快照恢复所需 CSV 和 ZIP（若使用历史归档），并保持本说明表中的目录结构；
2. 核对用户/UAV 文件是否按同一实例 ID 成对存在；
3. 核对 C++ 代码中的旧路径 `My paper` 与当前工作区 `My_paper` 的差异；
4. 以各实验目录实际加载的 `def_config.json` 为准，而不是仅依赖通用配置说明；
5. 为结果选择新的隔离输出目录，避免覆盖现有 `ExperimentsResults` 中的历史产物；
6. 在运行或引用结果前，复核六种算法逐实例 CSV 的行数和实例 ID 一致性。

本目录 README 只定义同步与恢复边界；它不生成、修改或替代任何本机实验实例。

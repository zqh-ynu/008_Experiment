// Visual Studio 直接运行入口：集中设置实验参数，按条件/实例/算法保存结果并支持断点续跑。
// 不调用旧批次脚本、自动测试、监测或绘图；原有物理配置仍由各实验加载。
// 常规入口读取固定 ToN 批次，顺序执行 EXP1--EXP4；当前默认仅试运行优化后的 ApprFast。
#include "experiments.h"

/**
 * @brief 试运行优化后的完整多 UAV ApprFast，并与同一实例的历史 CSV 指标比较。
 *
 * 读取固定 run_01 的 4000 用户条件和 ID 1--3；每实例只运行一次当前 selector 1，
 * 不重新运行优化前算法。计时沿用 execute_allocation，历史值来自 ApproFast.csv。
 * 结果仅输出控制台，不写入任何 CSV/JSON；跨时段比较不等同于同场配对性能测试。
 * @return 无返回值；历史记录、输入或本次分配无效时抛出带上下文的异常。
 */
void compare_apprfast_optimized_exp1_run01() {
    const fs::path root =
        "E:/Research/My_paper/2_Papers/008/008_Experiment/ExperimentsData/ExperimentsResults/"
        "EXP1_user_num/ToN_simple/run_01";
    const string condition_key = "4000";
    const vector<int> instance_ids = {1, 2, 3};
    const fs::path historical_csv = root / condition_key / "ApproFast.csv";
    constexpr int fast_selector = 1;

    // 仅保存本次需要的输入身份，避免依赖 run_info 中其他条件的排列顺序。
    struct BenchmarkInput {
        int id = 0;
        string user_path;
        string uav_path;
    };
    // 本次调用核验后只保留标量，使大型分配对象在下一实例开始前即可释放。
    struct BenchmarkSample {
        double duration_ms = 0.0;
        double total_utility = 0.0;
    };
    // 汇总同一实例的历史指标和本次指标，不保留算法分配明细。
    struct BenchmarkRow {
        int id = 0;
        double historical_ms = 0.0;
        double current_ms = 0.0;
        double historical_utility = 0.0;
        double current_utility = 0.0;
    };

    json info = read_experiment_json(root / "run_info.json");
    if (!info.is_object() || info.at("schema") != SIMPLE_RUN_SCHEMA ||
        info.at("experiment") != "EXP1_user_num" || info.at("methods") != method_name_list ||
        !info.at("conditions").is_array() || !info.at("instance_count").is_number_integer() ||
        info.at("instance_count").get<int>() < static_cast<int>(instance_ids.size()))
        throw runtime_error("ApprFast 试运行：run_info 不是预期的 EXP1 run_01 记录");

    if (!info.at("master_seed").is_number_integer())
        throw runtime_error("ApprFast 试运行：master_seed 必须是非负 32 位整数");
    const int64_t recorded_seed = info.at("master_seed").get<int64_t>();
    if (recorded_seed < 0 || recorded_seed > UINT32_MAX)
        throw runtime_error("ApprFast 试运行：master_seed 超出 32 位无符号整数范围");
    const uint32_t master_seed = static_cast<uint32_t>(recorded_seed);

    if (!info.at("ton_epsilon").is_number())
        throw runtime_error("ApprFast 试运行：ton_epsilon 必须是数值");
    const double epsilon = info.at("ton_epsilon").get<double>();
    if (!std::isfinite(epsilon) || epsilon <= 0.0 || epsilon >= 0.5)
        throw runtime_error("ApprFast 试运行：ton_epsilon 必须满足 0 < epsilon < 0.5");

    const string current_build = experiment_build_profile();
    if (!info.at("build_profile").is_string() ||
        info.at("build_profile").get<string>() != current_build || current_build != "Release|x64")
        throw runtime_error("ApprFast 试运行：请使用与 run_info 一致的 Release|x64 配置运行");
    if (info.at("unit_para") != unit_para || info.at("abs_tolerance") != ALLOCATION_ABS_TOL ||
        info.at("rel_tolerance") != ALLOCATION_REL_TOL)
        throw runtime_error("ApprFast 试运行：单位或公共数值容差与 run_info 不一致");

    if (!info.at("config_path").is_string() || !info.at("channel_config").is_object())
        throw runtime_error("ApprFast 试运行：信道配置记录无效");
    const string config = info.at("config_path").get<string>();
    if (!ifstream(config, ios::binary) || read_experiment_json(config) != info.at("channel_config"))
        throw runtime_error("ApprFast 试运行：信道配置文件不可读或内容已经改变");

    // 按 key 查找目标条件，不依赖其在 conditions 数组中的固定位置。
    const json* recorded_condition = nullptr;
    for (const auto& condition : info.at("conditions")) {
        if (!condition.is_object() || !condition.at("key").is_string())
            throw runtime_error("ApprFast 试运行：run_info 中存在无效条件记录");
        if (condition.at("key").get<string>() != condition_key)
            continue;
        if (recorded_condition != nullptr)
            throw runtime_error("ApprFast 试运行：run_info 中存在重复的 4000 用户条件");
        recorded_condition = &condition;
    }
    if (recorded_condition == nullptr)
        throw runtime_error("ApprFast 试运行：run_info 中缺少 4000 用户条件");
    if (!recorded_condition->at("bandwidth_mhz").is_number() ||
        recorded_condition->at("bandwidth_mhz").get<double>() != 40.0 ||
        !recorded_condition->at("inputs").is_array() ||
        recorded_condition->at("inputs").size() < instance_ids.size())
        throw runtime_error("ApprFast 试运行：4000 用户条件的带宽或输入数量不符合预期");
    const double bandwidth_mhz = recorded_condition->at("bandwidth_mhz").get<double>();

    vector<BenchmarkInput> inputs(instance_ids.size());
    vector<char> found_inputs(instance_ids.size(), 0);
    for (const auto& input : recorded_condition->at("inputs")) {
        if (!input.is_object() || !input.at("id").is_number_integer())
            throw runtime_error("ApprFast 试运行：4000 用户条件中存在无效输入记录");
        const int id = input.at("id").get<int>();
        size_t selected_index = instance_ids.size();
        for (size_t index = 0; index < instance_ids.size(); ++index) {
            if (instance_ids[index] == id) {
                selected_index = index;
                break;
            }
        }
        if (selected_index == instance_ids.size())
            continue;
        if (found_inputs[selected_index])
            throw runtime_error("ApprFast 试运行：目标实例 ID 重复：" + to_string(id));
        if (!input.at("user_path").is_string() || !input.at("uav_path").is_string())
            throw runtime_error("ApprFast 试运行：目标实例路径字段不是字符串：ID " + to_string(id));
        const string user_path = input.at("user_path").get<string>();
        const string uav_path = input.at("uav_path").get<string>();
        if (allocation_instance_id(user_path) != id || allocation_instance_id(uav_path) != id ||
            !ifstream(user_path, ios::binary) || !ifstream(uav_path, ios::binary))
            throw runtime_error("ApprFast 试运行：目标实例路径不可读或 ID 不一致：ID " + to_string(id));
        inputs[selected_index] = BenchmarkInput{id, user_path, uav_path};
        found_inputs[selected_index] = 1;
    }
    for (size_t index = 0; index < found_inputs.size(); ++index) {
        if (!found_inputs[index])
            throw runtime_error("ApprFast 试运行：缺少目标实例 ID " + to_string(instance_ids[index]));
    }

    // 历史 CSV 只读；按实例 ID 匹配，不能用全条件均值代替所选三例的基线。
    const auto historical_records = read_result_csv(historical_csv, true);
    for (int id : instance_ids) {
        const auto found = historical_records.find(id);
        if (found == historical_records.end())
            throw runtime_error("ApprFast 试运行：历史 ApproFast.csv 缺少 ID " + to_string(id));
        const auto& recorded = found->second;
        const uint32_t expected_seed = derive_algorithm_seed(
            master_seed, "EXP1_user_num", condition_key, id, "ApproFast");
        if (!algorithm_status_valid(recorded.status) || recorded.seed != expected_seed ||
            !std::isfinite(recorded.duration_ms) || recorded.duration_ms <= 0.0 ||
            !std::isfinite(recorded.metrics.total_utility) || recorded.metrics.total_utility <= 0.0)
            throw runtime_error("ApprFast 试运行：历史记录状态、种子、耗时或效用无效：ID " +
                to_string(id));
    }

    load_global_channel_config(config);
    cout << "============================================================\n"
        << "ApprFast 优化后完整多 UAV 试运行\n"
        << "输入记录: " << (root / "run_info.json").string() << '\n'
        << "历史指标: " << historical_csv.string() << '\n'
        << "条件: 4000 用户, 10 UAV, 每 UAV " << bandwidth_mhz << " MHz\n"
        << "实例: ID 1, 2, 3; epsilon=" << epsilon << "; 构建=" << current_build << '\n'
        << "计时范围: 仅 Appro_multiUAV_ToN；不含输入、模型初始化、指标核验和输出。\n"
        << "每实例仅运行一次当前优化版；优化前版本不重跑；结果仅输出控制台。\n"
        << "历史与本次测量不在同一时段，倍率仅供参考，不代表同场配对测试。\n"
        << "============================================================" << std::endl;

    // 只运行当前优化版完整 ApproFast；参数为已核验模型及实例 ID，返回本次耗时和效用。
    const auto run_current = [&](const SystemMd& model, int instance_id) -> BenchmarkSample {
        const uint32_t seed = derive_algorithm_seed(
            master_seed, "EXP1_user_num", condition_key, instance_id, "ApproFast");
        AlgorithmRunResult run = execute_allocation(model, seed,
            [&](BAProblem& problem, AllocationDiagnostics&) {
                return problem.Appro_multiUAV_ToN(
                    model.uavs, model.users, fast_selector, epsilon);
            });
        run.instance_id = instance_id;
        if (!algorithm_status_valid(run.status)) {
            const string reason = run.diagnostics.events.empty()
                ? "无诊断信息"
                : run.diagnostics.events.back();
            throw runtime_error("ApprFast 试运行 / ID " + to_string(instance_id) +
                " / 当前优化版 / " + algorithm_status_name(run.status) + ": " + reason);
        }
        if (!std::isfinite(run.duration_ms) || run.duration_ms <= 0.0)
            throw runtime_error("ApprFast 试运行 / ID " + to_string(instance_id) +
                " / 当前优化版：计时结果必须为有限正数");
        if (!std::isfinite(run.metrics.total_utility) || run.metrics.total_utility <= 0.0)
            throw runtime_error("ApprFast 试运行 / ID " + to_string(instance_id) +
                " / 当前优化版：4000 用户基准实例未产生有限正效用");
        return BenchmarkSample{run.duration_ms, run.metrics.total_utility};
    };

    vector<BenchmarkRow> rows;
    rows.reserve(inputs.size());
    for (const BenchmarkInput& input : inputs) {
        SystemMd model(input.user_path, input.uav_path, config);
        for (auto& uav : model.uavs)
            uav.total_bandwidth = bandwidth_mhz * unit_para;
        model.init_SystemModel();
        validate_allocation_model(model);
        if (model.users.size() != 4000 || model.uavs.size() != 10)
            throw runtime_error("ApprFast 试运行：模型规模与 4000 用户、10 UAV 不一致：ID " +
                to_string(input.id));

        cout << "[RUN] ID " << input.id << " / 当前优化版 ApproFast（selector 1）" << std::endl;
        const BenchmarkSample current_sample = run_current(model, input.id);
        const auto& historical = historical_records.at(input.id);
        const BenchmarkRow row{input.id, historical.duration_ms, current_sample.duration_ms,
            historical.metrics.total_utility, current_sample.total_utility};
        rows.push_back(row);
        const double saved_ms = row.historical_ms - row.current_ms;
        const double reduction_percent = 100.0 * (1.0 - row.current_ms / row.historical_ms);
        const double speedup = row.historical_ms / row.current_ms;
        const double utility_ratio = row.current_utility / row.historical_utility;
        cout << std::fixed << std::setprecision(6)
            << "[RESULT] ID " << row.id
            << " | historical_ms=" << row.historical_ms
            << " | current_ms=" << row.current_ms
            << " | saved_ms=" << saved_ms
            << " | reduction=" << reduction_percent << "%"
            << " | historical/current=" << speedup << "x\n"
            << "         historical_utility=" << row.historical_utility
            << " | current_utility=" << row.current_utility
            << " | utility_ratio=" << utility_ratio << std::endl;
    }

    double historical_time_sum = 0.0, current_time_sum = 0.0;
    double historical_utility_sum = 0.0, current_utility_sum = 0.0;
    vector<double> historical_times, current_times;
    historical_times.reserve(rows.size());
    current_times.reserve(rows.size());
    for (const BenchmarkRow& row : rows) {
        historical_time_sum += row.historical_ms;
        current_time_sum += row.current_ms;
        historical_utility_sum += row.historical_utility;
        current_utility_sum += row.current_utility;
        historical_times.push_back(row.historical_ms);
        current_times.push_back(row.current_ms);
    }
    std::sort(historical_times.begin(), historical_times.end());
    std::sort(current_times.begin(), current_times.end());
    const size_t middle = rows.size() / 2;
    const double historical_median = rows.size() % 2 != 0
        ? historical_times[middle]
        : (historical_times[middle - 1] + historical_times[middle]) / 2.0;
    const double current_median = rows.size() % 2 != 0
        ? current_times[middle]
        : (current_times[middle - 1] + current_times[middle]) / 2.0;
    const double count = static_cast<double>(rows.size());
    const double historical_mean = historical_time_sum / count;
    const double current_mean = current_time_sum / count;
    const double historical_utility_mean = historical_utility_sum / count;
    const double current_utility_mean = current_utility_sum / count;

    cout << "============================================================\n"
        << std::fixed << std::setprecision(6)
        << "汇总（相同 ID 1--3 的 3 条历史记录；本次每实例运行 1 次）\n"
        << "历史平均耗时: " << historical_mean << " ms\n"
        << "本次平均耗时: " << current_mean << " ms\n"
        << "平均节省时间: " << historical_mean - current_mean << " ms\n"
        << "平均耗时降低: " << 100.0 * (1.0 - current_mean / historical_mean) << "%\n"
        << "历史/本次平均耗时比（参考）: " << historical_mean / current_mean << "x\n"
        << "历史中位耗时: " << historical_median << " ms\n"
        << "本次中位耗时: " << current_median << " ms\n"
        << "历史/本次中位耗时比（参考）: " << historical_median / current_median << "x\n"
        << "历史平均总效用: " << historical_utility_mean << '\n'
        << "本次平均总效用: " << current_utility_mean << '\n'
        << "本次/历史平均总效用比: " << current_utility_mean / historical_utility_mean << '\n'
        << "说明：这是 3 个实例的历史记录与本次单次测量比较，不是完整 EXP1 均值或同场配对测试。\n"
        << "历史环境与本次环境可能不同，倍率不能单独证明代码优化幅度；本次不验证分配逐位一致。\n"
        << "============================================================" << std::endl;
}

/// 使用下方固定批次和参数顺序执行 EXP1--EXP4；argc 必须为 1，不接收旧 CLI 参数。
/// 成功返回 0；遇到错误保留已写结果并返回 1，后续实验不再自动启动。
int main(int argc, char*[]) {
    try {
        if (argc != 1)
            throw invalid_argument("Legacy batch/test CLI is disabled. Select experiments and parameters in main.cpp.");

        // 临时仅试运行优化后的完整多 UAV ApprFast，与历史 CSV 比较；结束后退出，不写结果。
        // 测量完成后改为 false，即可恢复下面的常规 EXP1--EXP4 入口。
        constexpr bool run_apprfast_comparison = true;
        if (run_apprfast_comparison) {
            compare_apprfast_optimized_exp1_run01();
            return 0;
        }

        // ===== 日常只需修改这一参数区，再在 Visual Studio 中编译、运行 =====
        ExperimentRunOptions options;
        options.instance_count = 10;         // 每条件先算 ID 1--10；以后改为 30，只补算尚未保存的 ID 11--30。
        options.master_seed = 20260905u;     // 保持不变，使新算、续跑和旧记录使用一致的逐实例种子。
        options.rounding_trials = 2;         // AlgRelaxRound 的既有随机舍入次数。
        options.ton_epsilon = 0.1;           // ToN 近似参数，要求 0 < epsilon < 0.5。
        options.input_root = (fs::path(experimentDataPath) / "data_ToN" / "2026-09-07").string();
        options.output_name = "run_ton_01";  // 新 ToN 输入独立输出；保留旧 run_01，不导入旧指标或耗时。
        options.conditions.clear();         // 空列表使四个实验各自选择完整的标准条件，不能共用 EXP1 的条件键。
        options.reuse_exp1_root = "";

        // 先手动运行 generate_instances_ToN.py 备好固定 30 个输入，再由 Visual Studio 启动这里。
        // EXP1：1000/2000/3000/4000/5000 用户，10 UAV，每 UAV 40 MHz。
        exp1_different_user_number(options);

        // EXP2：5/10/15/20 UAV，3000 用户，每 UAV 40 MHz。
        exp2_different_uav_number(options);

        // EXP3：Hard 比例 0/0.2/0.4/0.6/0.8/1.0，3000 用户、10 UAV，每 UAV 40 MHz。
        exp3_different_hard_user_ratio(options);

        // EXP4：每 UAV 10/20/30/40/50 MHz，3000 用户、10 UAV。
        exp4_different_total_bandwidth(options);
        return 0;
    } catch (const exception& error) {
        cerr << "实验停止，已写入的结果保持不变。\n原因: " << error.what() << std::endl;
        return 1;
    }
}

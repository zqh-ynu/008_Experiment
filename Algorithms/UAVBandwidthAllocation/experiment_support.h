#pragma once
// 六方法实验支持：统一调度、物理指标、CSV 检查点和算术汇总；方法4默认八候选，允许显式预算。
// 正常执行不自动保存输入快照、计算文件哈希或运行回归测试。
#include "EntityDefinition.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <climits>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

inline const vector<string> method_name_list = {
    "ApproBetter", "ApproFast", "AlgRelaxRound", "AlgSwapMatching", "AlgHardFirst", "AlgSA-DD"
};
inline const string TON_RESULT_VERSION_DIR = "ToN_simple/";
inline const string SIMPLE_RUN_SCHEMA = "ton-simple-v2";
inline const string SIMPLE_ALGORITHM_VERSION = "ton-multihard-lowest-baselines-hardfirst-hard-only-rectangular-hungarian-single-slot-multistart8-v1-better-elastic-reopt-v3";

/// 记录两种提出方法的单UAV入口与唯一关联后的固定策略；返回值用于结果版本比较。
inline json proposed_algorithm_policy() {
    return {{"ApproBetter", {{"selector", 3}, {"entry", "AlgBetter_singleUAV_ToN_faster"},
        {"post_association", "uav_id_order_matched_or_unassigned_elastic_reoptimization"}}},
        {"ApproFast", {{"selector", 1}, {"entry", "AlgFast_singleUAV_ToN"},
        {"post_association", "stop_after_unique_association"}}}};
}

/// 返回正式 AlgHardFirst 的八候选单槽矩形 Hungarian、槽占用与最低等级报告策略。
/// 无参数；返回值参与实验版本比较，避免把历史 Hungarian 结果续写为新算法。
inline json hard_first_policy() {
    return {{"mechanism", HARD_FIRST_MECHANISM},
        {"hard_level", "lowest_positive_only"},
        {"matching", "hard_only_rectangular_hungarian_maximum_weight"},
        {"matching_shape", "H_by_max_H_S_real_hard_rows_only"},
        {"candidate_count", HARD_FIRST_CANDIDATE_COUNT},
        {"candidate_base_seed", HARD_FIRST_CANDIDATE_BASE_SEED},
        {"candidate_order", "identity_then_mt19937_shuffle_hard_rows_then_uav_blocks"},
        {"candidate_selection", "equal_optimal_hard_then_total_utility_then_lowest_id"},
        {"degenerate_candidates", "one_when_no_positive_hard_edge"},
        {"hard_edge", "covered_and_one_slot_meets_lowest_level"},
        {"hard_slots_per_user", 1},
        {"local_search", "hard_only_single_slot_best_improvement_max_5000"},
        {"slot_width", "effective_BSub_physical_BSub_times_10_over_9"},
        {"hard_output", "reserve_one_integer_slot_report_lowest_threshold"},
        {"elastic", "remaining_slot_marginal_utility_matching"}};
}

/// 区分完整模型、传统最低等级收口和 HardFirst 单个整槽预留三种求解及校验路径。
enum class HardBaselinePolicy { FullModel, ClampLowestThreshold, ReserveIntegerSlots };

/// Settings supplied explicitly from main.cpp; input_root selects inputs, not physical configs or result paths.
struct ExperimentRunOptions {
    int instance_count = 10; // 每个实验条件选取的实例数，按成对的用户与无人机输入ID取样。
    uint32_t master_seed = 20260905u; // 派生各条件、实例和算法实际随机种子的主种子。
    int rounding_trials = 2; // AlgRelaxRound 每个实例执行的随机舍入尝试次数。
    double ton_epsilon = 0.083; // ToN算法的近似精度参数，要求有限且满足 0 < epsilon < 0.5。
    bool reallocate_residual = false; // 为兼容旧接口保留；当前Fast/Better的唯一关联后策略由选择器固定，此值不切换策略。
    vector<string> conditions; // 实验条件键的筛选列表；为空时运行该实验的全部标准条件。
    string output_name = "run_da_01"; // 结果版本的子目录名，不是路径；仅允许字母、数字、下划线和连字符。
    string reuse_exp1_root; // 历史EXP1结果复用目录预留项；当前非空会被校验拒绝，必须留空。
    string input_root = (fs::path(experimentDataPath) / "data").string(); // 输入数据根目录，默认旧版data；只选实例输入，不决定配置文件或结果目录。
};

/// Validate supplied counts, epsilon and path options; invalid values throw before any output write.
inline void validate_run_options(const ExperimentRunOptions& options) {
    // 数量必须为正，ToN近似精度必须是有限数且落在开区间(0, 0.5)内。
    if (options.instance_count <= 0 || options.rounding_trials <= 0 ||
        !std::isfinite(options.ton_epsilon) || options.ton_epsilon <= 0 || options.ton_epsilon >= 0.5)
        throw invalid_argument("Positive counts and 0 < epsilon < 0.5 are required");

    // output_name会成为结果路径中的单个目录名，因此拒绝空值和路径分隔符等字符。
    if (options.output_name.empty() ||
        options.output_name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != string::npos)
        throw invalid_argument("output_name must contain only letters, digits, underscores or hyphens");

    // 输入根目录必须显式给出，避免运行时从未指定的位置隐式寻找数据。
    if (options.input_root.empty())
        throw invalid_argument("input_root must explicitly select an existing input directory");
    // 旧复用来源包含旧ApproFast行，不能仅凭相同参数导入新算法批次。
    if (!options.reuse_exp1_root.empty())
        throw invalid_argument("Historical EXP1 result reuse is disabled for the new absolute-gain algorithms");
}

/// Return an absolute normalized path for input identities; this does not hash or copy file contents.
inline string experiment_absolute_path(const fs::path& path) {
    return fs::absolute(path).lexically_normal().generic_string();
}

/// Identify the selected IDE configuration, not measured hardware or a full compiler/environment fingerprint.
inline string experiment_build_profile() {
#if defined(NDEBUG)
    // NDEBUG由Release构建定义；未定义时将本次配置记为Debug。
    string profile = "Release";
#else
    string profile = "Debug";
#endif
#if defined(_WIN64)
    // 平台后缀来自当前编译目标，不代表对运行机器硬件的检测。
    return profile + "|x64";
#else
    return profile + "|Win32";
#endif
}

/// Raw counts are integral in value; doubles preserve fractional means in the same interface.
struct EXPResult {
    double duration = 0;
    double total_num = 0, hard_num = 0, elastic_num = 0;
    double total_utility = 0, hard_utility = 0, elastic_utility = 0;
    double hard_bandwidth = 0, elastic_bandwidth = 0;
    double hard_throughput = 0, elastic_throughput = 0, total_throughput = 0;
};

inline const std::array<double EXPResult::*, 12> EXP_METRICS = {
    &EXPResult::duration, &EXPResult::total_num, &EXPResult::hard_num, &EXPResult::elastic_num,
    &EXPResult::total_utility, &EXPResult::hard_utility, &EXPResult::elastic_utility,
    &EXPResult::hard_bandwidth, &EXPResult::elastic_bandwidth,
    &EXPResult::hard_throughput, &EXPResult::elastic_throughput, &EXPResult::total_throughput
};
inline const string METRIC_CSV_HEADER =
    "duration,total_num,hard_num,elastic_num,total_utility,hard_utility,elastic_utility,"
    "hard_bandwidth,elastic_bandwidth,hard_throughput,elastic_throughput,total_throughput";
inline const string CSV_HEADER =
    "instance_id,seed,status,elapsed_ms,diagnostics," + METRIC_CSV_HEADER;

using AllocationPair = std::pair<vector<KnapsackResult>, map<int, UserResult>>;

/// One attempted algorithm call, including failures whose performance metrics are unavailable.
struct AlgorithmRunResult {
    AllocationPair allocation;
    AlgorithmRunStatus status = AlgorithmRunStatus::Exception;
    AllocationDiagnostics diagnostics;
    EXPResult metrics;
    double duration_ms = 0;
    uint32_t seed = 0;
    int instance_id = 0;
};

/// Reject malformed model dimensions/IDs before indexing any matrix in an allocator.
inline void validate_allocation_model(const SystemMd& model) {
    const size_t m = model.uavs.size(), n = model.users.size();

    // 先验证模型计数和各UAV信道矩阵的外层维度，避免后续按ID访问时越界。
    if (model.m != static_cast<int>(m) || model.n1 < 0 || model.n2 < 0 ||
        model.n1 + model.n2 != static_cast<int>(n) ||
        model.cap_list.size() != m || model.dis_list.size() != m ||
        model.SNRave_list.size() != m || model.Bth_list.size() != m)
        throw invalid_argument("Model dimension mismatch");

    // 用户向量必须按连续ID排列，且Hard/Elastic分界要与n1、n2一致；同时检查效用参数有限且非负。
    for (size_t i = 0; i < n; ++i) {
        const auto& u = model.users[i];
        if (u.ID != static_cast<int>(i) ||
            u.uType != (i < static_cast<size_t>(model.n1) ? HARD_UTILITY : ELASTIC_UTILITY) ||
            !std::isfinite(u.weight) || u.weight < 0 || !std::isfinite(u.rMin) || u.rMin < 0)
            throw invalid_argument("Invalid user ordering or parameters");
    }

    // 逐架UAV检查ID、预算和信道行长度，再检查每条链路的容量、距离及Hard阈值。
    for (size_t k = 0; k < m; ++k) {
        if (model.uavs[k].ID != static_cast<int>(k) ||
            !std::isfinite(model.uavs[k].total_bandwidth) || model.uavs[k].total_bandwidth < 0 ||
            model.cap_list[k].size() != n || model.dis_list[k].size() != n ||
            model.SNRave_list[k].size() != n || model.Bth_list[k].size() != static_cast<size_t>(model.n1))
            throw invalid_argument("Invalid UAV or channel matrix");
        for (size_t i = 0; i < n; ++i) {
            const double cap = model.cap_list[k][i];
            if (!std::isfinite(cap) || cap < 0 || !std::isfinite(model.dis_list[k][i]) ||
                model.dis_list[k][i] < 0)
                throw invalid_argument("Invalid capacity or link distance");
            if (i < static_cast<size_t>(model.n1)) {
                // 有正容量时最低速率阈值应为rMin/cap；无容量链路必须表示为无穷阈值。
                const double threshold = model.Bth_list[k][i];
                if (cap > 0 ? !allocation_near(threshold, model.users[i].rMin / cap) :
                    threshold != std::numeric_limits<double>::infinity())
                    throw invalid_argument("Hard threshold and capacity are inconsistent");
            }
        }
    }

    // 服务关系表中的UAV和用户ID必须有效；同一架UAV下不能重复登记同一用户。
    for (const auto& entry : model.uav_serviceable_users_map) {
        if (entry.first < 0 || entry.first >= model.m)
            throw invalid_argument("Invalid serviceable UAV ID");
        set<int> seen;
        for (const auto& user : entry.second)
            if (user.ID < 0 || user.ID >= static_cast<int>(n) || !seen.insert(user.ID).second)
                throw invalid_argument("Invalid or duplicate serviceable user ID");
    }
}

/// Raise an actionable invalid-allocation outcome; never repair an infeasible output here.
inline void require_allocation(bool condition, const string& message) {
    if (!condition) throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, message);
}

/// 将基线已服务hard用户的带宽收至最低等级阈值；实质性不足直接报告无效分配。
/// model是原始完整实例，results为待原位更新的分配；返回void，释放带宽不再分配。
inline void clamp_baseline_hard_to_lowest_level(const SystemMd& model,
    vector<KnapsackResult>& results) {
    // 输出必须恰好有每架UAV一项，后续按uav_id回到原始模型查阈值。
    require_allocation(results.size() == model.uavs.size(), "Baseline result dimension mismatch");
    for (auto& allocation : results) {
        const int k = allocation.uav_id;
        require_allocation(k >= 0 && k < model.m, "Invalid baseline UAV ID");

        // 只调整已服务的Hard用户；Elastic分配和未服务用户保持原样。
        for (int id : allocation.allocatedList) {
            require_allocation(id >= 0 && id < static_cast<int>(model.users.size()),
                "Invalid baseline user ID");
            const User& user = model.users[id];
            if (user.uType != HARD_UTILITY) continue;
            auto found = allocation.allocatedBandwidth.find(id);
            require_allocation(found != allocation.allocatedBandwidth.end(),
                "Missing baseline hard bandwidth");
            const double bandwidth = found->second;
            const double capacity = model.cap_list.at(k).at(id);
            const double minimum_rate = user.hard_rate_levels.empty()
                ? user.rMin : user.hard_rate_levels.front();

            // 带宽、链路容量和最低速率必须可用于计算有效的最低等级阈值。
            require_allocation(std::isfinite(bandwidth) && bandwidth > 0 &&
                std::isfinite(capacity) && capacity > 0 &&
                std::isfinite(minimum_rate) && minimum_rate >= 0,
                "Invalid baseline hard bandwidth or threshold");
            // 零阈值单级用户仍需保留正带宽；多级hard的等级阈值由模型验证为正。
            if (minimum_rate == 0) continue;
            const double threshold = minimum_rate / capacity;

            // 先确认原分配已经满足最低Hard QoS，再将多出的Hard带宽收回至阈值。
            require_allocation(std::isfinite(threshold) && threshold > 0 &&
                bandwidth + allocation_tolerance(bandwidth, threshold) >= threshold &&
                hard_qos_satisfied(bandwidth, capacity, minimum_rate),
                "Baseline minimum hard level not met");
            found->second = threshold;
        }
    }
}

/// 按原始完整实例计算真实指标，不使用算法内部缓存的效用；duration_ms仅为算法计时。
/// baseline_minimum_level_only=true要求基线已服务hard用户处于第一级；正阈值带宽须在阈值附近。
/// 六方法统一使用原始完整模型评价；基线视图只参与求解，不能替代此处的model。
inline EXPResult compute_single_EXPResult(const SystemMd& model,
    const vector<KnapsackResult>& results, double duration_ms, bool baseline_minimum_level_only = false) {
    // 计时值由调用方提供；结果向量必须与模型中的UAV逐一对应。
    require_allocation(std::isfinite(duration_ms) && duration_ms >= 0, "Invalid duration");
    require_allocation(results.size() == model.uavs.size(), "One result entry per UAV is required");
    EXPResult totals;
    totals.duration = duration_ms;

    // seen数组用于全局防止重复UAV结果和跨UAV重复分配同一用户。
    vector<bool> seen_user(model.users.size(), false), seen_uav(model.uavs.size(), false);
    for (const auto& result : results) {
        const int k = result.uav_id;

        // 校验UAV索引唯一，且分配列表与带宽映射描述的是同一组用户。
        require_allocation(k >= 0 && k < model.m, "Invalid result UAV ID");
        require_allocation(!seen_uav[k], "Duplicate result UAV ID");
        seen_uav[k] = true;
        require_allocation(result.allocatedList.size() == result.allocatedBandwidth.size(),
            "Allocation list and bandwidth map disagree");
        double used_bandwidth = 0;
        EXPResult local; // Preserve the original per-UAV grouping of floating-point utility/bandwidth sums.
        for (int i : result.allocatedList) {
            // 每个用户只能服务一次，且必须有对应的正带宽记录。
            require_allocation(i >= 0 && i < static_cast<int>(model.users.size()), "Invalid result user ID");
            require_allocation(!seen_user[i], "User assigned more than once: " + to_string(i));
            seen_user[i] = true;
            require_allocation(result.allocatedBandwidth.count(i) != 0, "Missing bandwidth entry");
            const double bw = result.allocatedBandwidth.at(i), cap = model.cap_list[k][i];
            const auto& user = model.users[i];
            require_allocation(std::isfinite(bw) && bw > 0, "Served bandwidth must be finite and positive");

            // 只对覆盖范围内且具有正容量的链路计分，并由带宽和容量计算实际传输速率。
            // init_SystemModel defines serviceable links by this same distance bound.
            require_allocation(model.dis_list[k][i] <= max_coverage_distance && cap > 0,
                "Assigned link is not serviceable");
            const double rate = bw * cap;
            require_allocation(std::isfinite(rate), "Non-finite allocated rate");
            if (user.uType == HARD_UTILITY) {
                // 先按可靠速率找原始等级；基线视图的唯一等级编号不进入最终统计。
                const int level = user.achieved_hard_level(bw, cap);
                require_allocation(level > 0,
                    "Hard QoS not met: UAV " + to_string(k) + ", user " + to_string(i));
                if (baseline_minimum_level_only) {
                    require_allocation(level == 1,
                        "Baseline hard user reached a level other than the minimum: user " + to_string(i));
                    const double minimum_rate = user.hard_rate_levels.empty()
                        ? user.rMin : user.hard_rate_levels.front();
                    if (minimum_rate > 0)
                        require_allocation(allocation_near(bw, minimum_rate / cap),
                            "Baseline hard bandwidth differs from minimum threshold: user " + to_string(i));
                }
                ++totals.hard_num;
                local.hard_utility += user.hard_level_utility(level);
                local.hard_bandwidth += bw;
                // 保证业务吞吐量按实际等级阈值累计，而不是物理超额速率或固定最高阈值。
                totals.hard_throughput += user.hard_rate_levels.empty() ? user.rMin :
                    user.hard_rate_levels[level - 1];
            } else {
                // Elastic用户按原模型效用函数计分，吞吐量使用实际速率。
                ++totals.elastic_num;
                local.elastic_utility += user.elastic_utility(bw, cap);
                local.elastic_bandwidth += bw;
                totals.elastic_throughput += rate;
            }
            used_bandwidth += bw;
        }

        // 将本架UAV的局部使用量与原始总带宽比较，再把局部效用和带宽计入全局总量。
        const double budget = model.uavs[k].total_bandwidth;
        require_allocation(std::isfinite(used_bandwidth) &&
            used_bandwidth <= budget + allocation_tolerance(used_bandwidth, budget),
            "UAV bandwidth budget exceeded");
        totals.hard_utility += local.hard_utility;
        totals.elastic_utility += local.elastic_utility;
        totals.hard_bandwidth += local.hard_bandwidth;
        totals.elastic_bandwidth += local.elastic_bandwidth;
    }

    // 由Hard/Elastic分项得到总量，并拒绝任何非有限或负的汇总指标。
    totals.total_num = totals.hard_num + totals.elastic_num;
    totals.total_utility = totals.hard_utility + totals.elastic_utility;
    totals.total_throughput = totals.hard_throughput + totals.elastic_throughput;
    for (auto member : EXP_METRICS)
        require_allocation(std::isfinite(totals.*member) && totals.*member >= 0, "Invalid aggregate metric");
    return totals;
}

/// 在策略指定的私有求解视图上运行算法，始终以原模型计分且不写文件。
/// 传统基线在计时内收回 hard 超额带宽；HardFirst 预留单个整槽并另行校验物理占用。
inline AlgorithmRunResult execute_allocation(const SystemMd& model, uint32_t seed,
    const std::function<AllocationPair(BAProblem&, AllocationDiagnostics&)>& solver,
    HardBaselinePolicy policy = HardBaselinePolicy::FullModel) {
    // 即使算法失败也保留本次派生种子，便于原样定位或复现调用。
    AlgorithmRunResult run;
    run.seed = seed;
    try {
        const bool lowest_level = policy != HardBaselinePolicy::FullModel;
        // 构造与求解模型均在计时外；最低等级视图不修改原始完整实例。
        const SystemMd solve_model = lowest_level ? model.lowest_level_view() : model;
        BAProblem problem(solve_model);

        // 计时从求解入口前开始；传统基线的Hard带宽收口也包含在算法耗时内。
        const auto start = std::chrono::steady_clock::now();
        try {
            run.allocation = solver(problem, run.diagnostics);
            if (policy == HardBaselinePolicy::ClampLowestThreshold)
                clamp_baseline_hard_to_lowest_level(model, run.allocation.first);
        }
        catch (...) {
            // 求解抛错时仍记录已经消耗的算法时间，然后让外层状态处理捕获该异常。
            run.duration_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            throw;
        }
        run.duration_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        // 先检查结构、覆盖、预算与真实最低等级，避免无效分配进入后续专用检查。
        run.metrics = compute_single_EXPResult(model, run.allocation.first, run.duration_ms, lowest_level);

        // HardFirst需要额外验证整数槽预留和物理带宽占用；该检查不改变求解结果。
        if (policy == HardBaselinePolicy::ReserveIntegerSlots)
            validate_hard_first_allocation(solve_model, run.allocation.first);
        if (policy == HardBaselinePolicy::ClampLowestThreshold) {
            // 带宽已在计时内收至最低阈值；此处仅重建真实效用和冗余汇总。
            for (auto& allocation : run.allocation.first) {
                allocation.allocatedValue.clear();
                allocation.totalWeight = allocation.totalValue = 0;
                allocation.hardWeight = allocation.hardValue = 0;
                allocation.elasticWeight = allocation.elasticValue = 0;
                for (int id : allocation.allocatedList) {
                    const User& user = model.users[id];
                    const double bw = allocation.allocatedBandwidth.at(id);
                    const double cap = model.cap_list[allocation.uav_id][id];
                    const double value = user.uType == HARD_UTILITY
                        ? user.hard_level_utility(user.achieved_hard_level(bw, cap))
                        : user.elastic_utility(bw, cap);
                    allocation.allocatedValue.emplace(id, value);
                    allocation.totalWeight += bw; allocation.totalValue += value;
                    if (user.uType == HARD_UTILITY) {
                        allocation.hardWeight += bw; allocation.hardValue += value;
                    } else {
                        allocation.elasticWeight += bw; allocation.elasticValue += value;
                    }
                }
            }
            // 求解视图与原模型的内部ID完全相同；该投影函数只读取ID及整理后的分配。
            run.allocation.second = problem.construct_user_results(run.allocation.first);
        }

        // 合法空分配单独标记；其余有效分配标记成功。
        run.status = run.metrics.total_num == 0 ? AlgorithmRunStatus::ZeroAllocation : AlgorithmRunStatus::Success;
    } catch (const AllocationFailure& failure) {
        // 分配约束失败保留其专用状态，其它异常统一作为Exception记录。
        run.status = failure.status;
        run.diagnostics.events.push_back(failure.what());
    } catch (const std::exception& failure) {
        run.status = AlgorithmRunStatus::Exception;
        run.diagnostics.events.push_back(failure.what());
    } catch (...) {
        run.status = AlgorithmRunStatus::Exception;
        run.diagnostics.events.push_back("Unknown non-standard exception");
    }
    return run;
}

/// 用完整实例、选项和种子调度六方法；hardfirst_candidate_count仅作用于方法4，默认8保持原行为。
/// 返回分配、真实指标和状态；非法方法或方法4的非法候选数在求解前抛错，空资源返回合法零分配。
inline AlgorithmRunResult run_algorithm(const SystemMd& model, size_t method,
    const ExperimentRunOptions& options, uint32_t seed,
    int hardfirst_candidate_count = HARD_FIRST_CANDIDATE_COUNT) {
    // 先拒绝非法方法索引；HardFirst方法还要验证候选数是否符合其机制约束。
    if (method >= method_name_list.size()) throw invalid_argument("Unknown method index");
    if (method == 4) hard_first_mechanism(hardfirst_candidate_count);

    // 前两个提出方法使用完整模型；其余为最低等级基线，方法4另走整数槽校验路径。
    const HardBaselinePolicy policy = method < 2 ? HardBaselinePolicy::FullModel :
        (method == 4 ? HardBaselinePolicy::ReserveIntegerSlots : HardBaselinePolicy::ClampLowestThreshold);
    return execute_allocation(model, seed, [&](BAProblem& problem, AllocationDiagnostics& diagnostics) {
        bool any_budget = false;
        for (const auto& uav : model.uavs) any_budget = any_budget || uav.total_bandwidth > 0;
        // 保留默认8候选的空资源路径；显式缩减预算仍进入HardFirst，留下可核对的退化诊断。
        if ((model.users.empty() || model.uavs.empty() || !any_budget) &&
            (method != 4 || hardfirst_candidate_count == HARD_FIRST_CANDIDATE_COUNT)) {
            // 空实例构造每架UAV的空结果和默认用户投影，不进入各分配器。
            vector<KnapsackResult> empty(model.uavs.size());
            for (size_t k = 0; k < empty.size(); ++k) empty[k].uav_id = static_cast<int>(k);
            return AllocationPair{empty, problem.construct_user_results(empty)};
        }

        // 方法索引固定对应六个分配入口；仅前两项传入Fast/Better选择器，方法4接收候选数。
        switch (method) {
        case 0: return problem.Appro_multiUAV_ToN(model.uavs, model.users, 3, options.ton_epsilon, options.reallocate_residual);
        case 1: return problem.Appro_multiUAV_ToN(model.uavs, model.users, 1, options.ton_epsilon, options.reallocate_residual);
        case 2: return problem.ConvexRelaxationAndRounding_multiUAV(1e-4, seed, options.rounding_trials, &diagnostics);
        case 3: return problem.MatchingSQP_Allocation(MatchingSQPConfig(), &diagnostics);
        case 4: return problem.HardFirstPriorityMatchingAllocation(&diagnostics, hardfirst_candidate_count);
        default: return problem.SADA_Allocation(SADAConfig(), &diagnostics);
        }
    }, policy); // 四个基线只看最低等级；HardFirst 独立校验单槽及其保护开销。
}

/// 按请求候选数核对机制、各候选规模/种子/计时、累计值及择优结果；返回已校验JSON，不修改记录。
inline json checked_hardfirst_diagnostics(const AlgorithmRunResult& run,
    int candidate_count = HARD_FIRST_CANDIDATE_COUNT) {
    // 先按调用请求的候选数确定期望机制，再要求运行器恰好提供一条诊断JSON。
    const string mechanism = hard_first_mechanism(candidate_count);
    if (run.diagnostics.events.size() != 1)
        throw runtime_error("Expected one HardFirst diagnostic event");
    const json diag = json::parse(run.diagnostics.events.front());
    if (diag.at("algorithm") != mechanism)
        throw runtime_error("HardFirst diagnostic algorithm version mismatch");
    // JSON中的计数必须是非负整数；拒绝负数转换到uint64_t后被误认为合法计数。
    const auto count = [](const json& object, const char* key) -> uint64_t {
        if (!object.at(key).is_number_unsigned())
            throw runtime_error("Invalid HardFirst counter: " + string(key));
        return object.at(key).get<uint64_t>();
    };
    const uint64_t rows = count(diag, "hard_hungarian_rows"), columns = count(diag, "hard_hungarian_columns");
    const uint64_t completed = count(diag, "candidate_count_completed");
    const auto& candidates = diag.at("candidates");

    // Hungarian行列数、请求/完成候选数以及候选数组长度必须彼此吻合。
    if (rows > columns || count(diag, "candidate_count_requested") != candidate_count ||
        completed < 1 || completed > candidate_count ||
        !candidates.is_array() || candidates.size() != completed)
        throw runtime_error("Invalid HardFirst candidate dimensions or count");
    uint64_t augmentations = 0, distinct = 0, selected = 0;
    double matching = 0, search = 0, elastic = 0, best_total = 0;
    const double reference_hard = candidates.front().at("hard_utility").get<double>();

    // 逐候选核对编号、矩阵规模、Hungarian工作量及局部搜索轮数，并累计诊断总计。
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates.at(i);
        const uint64_t aug = count(candidate, "hard_hungarian_augmentations");
        const uint64_t duplicate = count(candidate, "duplicate_of");
        if (count(candidate, "candidate_id") != i || count(candidate, "hard_hungarian_rows") != rows ||
            count(candidate, "hard_hungarian_columns") != columns || aug > rows || duplicate > i ||
            count(candidates.at(static_cast<size_t>(duplicate)), "duplicate_of") != duplicate ||
            count(candidate, "local_search_max_rounds") != HARD_FIRST_LOCAL_SEARCH_MAX_ROUNDS ||
            count(candidate, "local_search_rounds") > HARD_FIRST_LOCAL_SEARCH_MAX_ROUNDS ||
            count(candidate, "local_search_improvements") > count(candidate, "local_search_rounds"))
            throw runtime_error("Invalid HardFirst per-candidate work counters");
        const uint64_t edges = count(candidate, "hard_eligible_edges");

        // 无可行Hard边时只允许退化到候选0；正常情况下每个候选都应完成一次匹配。
        if (aug != (edges == 0 ? 0 : rows) ||
            (i == 0 && completed != (edges == 0 ? 1 : candidate_count)))
            throw runtime_error("Invalid HardFirst augmentation or degenerate candidate count");

        // 候选0是原始顺序；后续候选的随机排列种子必须按固定基种子递增。
        if ((i == 0 && !candidate.at("permutation_seed").is_null()) ||
            (i > 0 && count(candidate, "permutation_seed") != HARD_FIRST_CANDIDATE_BASE_SEED + i))
            throw runtime_error("Invalid HardFirst permutation seed");

        // 候选耗时与效用必须是有限非负数，避免无效数值污染择优和汇总。
        for (const char* key : {"hard_matching_ms", "hard_local_search_ms", "elastic_ms", "hard_utility", "total_utility"}) {
            const double value = candidate.at(key).get<double>();
            if (!std::isfinite(value) || value < 0)
                throw runtime_error("Invalid HardFirst candidate value: " + string(key));
        }
            if (!allocation_near(candidate.at("hard_utility").get<double>(), reference_hard))
                throw runtime_error("HardFirst candidates disagree on optimal hard utility");

        // 候选Hard效用应相同；总效用更高者胜出，容差范围内保持先出现的候选。
        const double total = candidate.at("total_utility").get<double>();
        if (i == 0 || total > best_total + allocation_tolerance(total, best_total)) {
            selected = i;
            best_total = total;
        }
        if (duplicate == i) ++distinct;
        augmentations += aug;
        matching += candidate.at("hard_matching_ms").get<double>();
        search += candidate.at("hard_local_search_ms").get<double>();
        elastic += candidate.at("elastic_ms").get<double>();
    }

    // 汇总诊断中的数值也必须有限且非负，之后与逐候选累加值及最终运行指标交叉核对。
    const double reference_total = candidates.front().at("total_utility").get<double>();
    for (const char* key : {"hard_matching_ms", "hard_local_search_ms", "elastic_ms", "candidate0_total_utility",
        "gain_over_candidate0", "hard_utility", "total_utility"}) {
        const double value = diag.at(key).get<double>();
        if (!std::isfinite(value) || value < 0)
            throw runtime_error("Invalid HardFirst aggregate value: " + string(key));
    }

    // 对照候选计数、分阶段耗时、择优结果及run.metrics，发现诊断与实际结果不一致时拒绝记录。
    if (count(diag, "hard_hungarian_augmentations") != augmentations ||
        count(diag, "distinct_hard_candidates") != distinct || count(diag, "selected_candidate") != selected ||
        !allocation_near(diag.at("hard_matching_ms").get<double>(), matching) ||
        !allocation_near(diag.at("hard_local_search_ms").get<double>(), search) ||
        !allocation_near(diag.at("elastic_ms").get<double>(), elastic) ||
        !allocation_near(diag.at("candidate0_total_utility").get<double>(), reference_total) ||
        !allocation_near(diag.at("gain_over_candidate0").get<double>(), best_total - reference_total) ||
        !allocation_near(diag.at("total_utility").get<double>(), best_total) ||
        !allocation_near(diag.at("hard_utility").get<double>(),
            candidates.at(static_cast<size_t>(selected)).at("hard_utility").get<double>()) ||
        !allocation_near(best_total, run.metrics.total_utility) ||
        !allocation_near(diag.at("hard_utility").get<double>(), run.metrics.hard_utility))
        throw runtime_error("HardFirst aggregate timing, selection or utility mismatch");
    return diag;
}

/// Derive an instance-local seed using specified FNV-1a/32 over UTF-8, independent of invocation order.
inline uint32_t derive_algorithm_seed(uint32_t master_seed, const string& experiment,
    const string& condition, int instance_id, const string& method) {
    // 用JSON数组明确字段边界和顺序，形成与执行先后无关的调用身份字符串。
    const string identity = json::array({master_seed, experiment, condition, instance_id, method}).dump();

    // 对身份字符串的UTF-8字节执行32位FNV-1a，得到该实例、条件、方法专属种子。
    uint32_t hash = 2166136261u;
    for (unsigned char byte : identity) { hash ^= byte; hash *= 16777619u; }
    return hash;
}

/// Encode a CSV field; diagnostics JSON escapes embedded newlines before reaching this function.
inline string csv_quote(const string& value) {
    // 不含CSV特殊字符时原样返回；否则包双引号，并把字段内部的双引号重复转义。
    if (value.find_first_of(",\"\r\n") == string::npos) return value;
    string encoded = "\"";
    for (char c : value) { if (c == '"') encoded += '"'; encoded += c; }
    return encoded + "\"";
}

/// Parse one physical CSV line strictly, including quoted commas and doubled quotes.
inline vector<string> csv_fields(const string& raw_line) {
    // 去除Windows行尾的CR，再逐列解析；逗号只在引号外作为字段分隔符。
    string line = raw_line;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    vector<string> fields;
    size_t pos = 0;
    while (true) {
        string value;
        if (pos < line.size() && line[pos] == '"') {
            // 引号字段内的两个连续双引号表示一个字面双引号。
            ++pos;
            bool closed = false;
            while (pos < line.size()) {
                char c = line[pos++];
                if (c != '"') { value += c; continue; }
                if (pos < line.size() && line[pos] == '"') { value += '"'; ++pos; }
                else { closed = true; break; }
            }
            // 引号闭合后只能接字段分隔符或行尾，拒绝格式不完整的字段。
            if (!closed || (pos < line.size() && line[pos] != ','))
                throw invalid_argument("Malformed quoted CSV field");
        } else {
            // 非引号字段读到逗号为止；字段中出现双引号属于非法CSV。
            while (pos < line.size() && line[pos] != ',') {
                if (line[pos] == '"') throw invalid_argument("Quote in unquoted CSV field");
                value += line[pos++];
            }
        }
        fields.push_back(value);
        if (pos == line.size()) break;
        ++pos;
        // 末尾逗号代表最后一个空字段，需显式补入。
        if (pos == line.size()) { fields.emplace_back(); break; }
    }
    return fields;
}

/// Parse a complete finite number, refusing trailing text and unavailable metric cells.
inline double parse_finite_number(const string& token) {
    // stod返回已消费字符数；必须完整解析整个字段，并拒绝非有限或负数。
    size_t consumed = 0;
    double value = stod(token, &consumed);
    if (consumed != token.size() || !std::isfinite(value) || value < 0)
        throw invalid_argument("Invalid numeric CSV field");
    return value;
}

/// Serialize common metrics with round-trip floating-point precision.
inline string expResultToCSVLine(const EXPResult& result) {
    ostringstream out;
    // 使用double可往返精度，按EXP_METRICS定义的固定顺序写出指标。
    out << setprecision(std::numeric_limits<double>::max_digits10);
    bool first = true;
    for (auto member : EXP_METRICS) {
        if (!first) out << ',';
        out << result.*member;
        first = false;
    }
    return out.str();
}

/// Serialize every attempt; failed performance cells are blank, not zero-valued samples.
inline string run_result_to_csv(const AlgorithmRunResult& run) {
    ostringstream out;
    out << setprecision(std::numeric_limits<double>::max_digits10);

    // 诊断标志和事件数组作为JSON整体序列化，再经CSV转义写入单一字段。
    const json diagnostics = {{"used_fallback", run.diagnostics.used_fallback}, {"events", run.diagnostics.events}};
    out << run.instance_id << ',' << run.seed << ',' << algorithm_status_name(run.status)
        << ',' << run.duration_ms << ',' << csv_quote(diagnostics.dump()) << ',';

    // 有效状态写全套指标；失败尝试保留同样列数，但所有性能单元格留空。
    if (algorithm_status_valid(run.status)) out << expResultToCSVLine(run.metrics);
    else out << string(EXP_METRICS.size() - 1, ',');
    return out.str();
}

/// Parse a typed attempt and reject contradictions between status, metrics and metadata.
inline AlgorithmRunResult parse_run_result(const string& line) {
    // 先按CSV规则拆列并检查字段数量，再逐字段恢复一次算法尝试。
    const auto fields = csv_fields(line);
    if (fields.size() != 5 + EXP_METRICS.size()) throw invalid_argument("Wrong result column count");
    AlgorithmRunResult run;

    // ID和种子以数字解析后再检查整数范围，防止截断或越界转换。
    const double id = parse_finite_number(fields[0]), seed = parse_finite_number(fields[1]);
    if (id < 1 || id > INT_MAX || id != floor(id) || seed > UINT32_MAX || seed != floor(seed))
        throw invalid_argument("Invalid result ID/seed");
    run.instance_id = static_cast<int>(id);
    run.seed = static_cast<uint32_t>(seed);
    run.status = parse_algorithm_status(fields[2]);
    run.duration_ms = parse_finite_number(fields[3]);

    // 诊断字段必须是含回退标志和事件数组的合法JSON。
    const auto diagnostics = json::parse(fields[4]);
    run.diagnostics.used_fallback = diagnostics.at("used_fallback").get<bool>();
    run.diagnostics.events = diagnostics.at("events").get<vector<string>>();

    // 成功/零分配状态读取所有指标；失败状态则要求性能指标列全部为空。
    for (size_t i = 0; i < EXP_METRICS.size(); ++i) {
        if (algorithm_status_valid(run.status)) run.metrics.*EXP_METRICS[i] = parse_finite_number(fields[5 + i]);
        else if (!fields[5 + i].empty()) throw invalid_argument("Failed result must have blank performance cells");
    }
    if (algorithm_status_valid(run.status)) {
        // 检查总量等于分项之和、计数为整数，且ZeroAllocation状态与服务人数一致。
        const auto& r = run.metrics;
        if (!allocation_near(r.duration, run.duration_ms) ||
            !allocation_near(r.total_num, r.hard_num + r.elastic_num) ||
            !allocation_near(r.total_utility, r.hard_utility + r.elastic_utility) ||
            !allocation_near(r.total_throughput, r.hard_throughput + r.elastic_throughput) ||
            r.total_num != floor(r.total_num) || r.hard_num != floor(r.hard_num) ||
            r.elastic_num != floor(r.elastic_num))
            throw invalid_argument("Inconsistent raw-run metrics");
        if ((run.status == AlgorithmRunStatus::ZeroAllocation) != (r.total_num == 0))
            throw invalid_argument("Zero-allocation status/count mismatch");
        if (run.status == AlgorithmRunStatus::ZeroAllocation)
            for (size_t i = 1; i < EXP_METRICS.size(); ++i)
                if (r.*EXP_METRICS[i] != 0.0) throw invalid_argument("Nonzero metric in zero allocation");
    }
    return run;
}

/// Completed rows for one method, indexed by stable input ID; row order and other methods' lengths are irrelevant.
using MethodRecords = map<int, AlgorithmRunResult>;
using ConditionRecords = vector<MethodRecords>;

/// 读取逐实例记录；旧调用默认拒绝失败，新五实验显式allow_failed=true，将失败视为已尝试。
/// The required flag is used for explicitly requested legacy imports, never to repair an incomplete file.
inline MethodRecords read_result_csv(const fs::path& path, bool required = false, bool allow_failed = false) {
    // 可选结果文件不存在时返回空记录；required用于明确要求导入该文件的调用方。
    if (!fs::exists(path)) {
        if (required) throw runtime_error("Missing requested result CSV: " + path.string());
        return {};
    }

    // 以二进制方式读取并核对固定表头，避免换行转换或旧版列结构被静默接受。
    ifstream input(path, ios::binary);
    string line;
    if (!input || !getline(input, line)) throw runtime_error("Cannot read CSV header: " + path.string());
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != CSV_HEADER) throw runtime_error("CSV header mismatch: " + path.string());
    MethodRecords rows;
    size_t line_number = 1;

    // 每行解析为一次尝试；按选项决定失败行能否作为已尝试记录，并拒绝重复实例ID。
    while (getline(input, line)) {
        ++line_number;
        try {
            if (input.eof()) throw runtime_error("Unterminated final record");
            auto row = parse_run_result(line);
            if (!allow_failed && !algorithm_status_valid(row.status)) throw runtime_error("Recorded failure is not a completed result");
            const int id = row.instance_id;
            if (!rows.emplace(id, std::move(row)).second) throw runtime_error("Duplicate instance ID");
        } catch (const std::exception& error) {
            // 为解析错误补上文件路径和行号，便于定位损坏的检查点。
            throw runtime_error(path.string() + ":" + to_string(line_number) + ": " + error.what());
        }
    }

    // 区分正常到达文件末尾与底层读取错误，成功后返回按实例ID索引的记录。
    if (input.bad()) throw runtime_error("CSV read error: " + path.string());
    return rows;
}

/// Read each method independently; rows must belong to selected inputs and retain the original deterministic seeds.
inline ConditionRecords read_condition_records(const fs::path& directory, const string& experiment,
    const string& condition, const json& inputs, const ExperimentRunOptions& options) {
    // 从本次选中的输入元数据收集合法实例ID，后续检查点不能包含其他批次的行。
    set<int> selected;
    for (const auto& input : inputs) selected.insert(input.at("id").get<int>());

    // 每种方法分别读取自己的CSV，再核验每行ID属于所选输入且种子可由当前配置重现。
    ConditionRecords result(method_name_list.size());
    for (size_t m = 0; m < method_name_list.size(); ++m) {
        result[m] = read_result_csv(directory / (method_name_list[m] + ".csv"));
        for (const auto& entry : result[m])
            if (!selected.count(entry.first) || entry.second.seed !=
                derive_algorithm_seed(options.master_seed, experiment, condition, entry.first, method_name_list[m]))
                throw runtime_error("Checkpoint ID/seed mismatch: " + directory.string() + "/" + method_name_list[m]);
    }

    // 保持方法索引与method_name_list一致，供调度器按位置续跑。
    return result;
}

/// 追加一条记录并关闭文件；新五实验显式allow_failed=true时允许写失败状态，性能指标留空。
/// The caller has read existing headers/IDs and created the condition directory; no serialize/parse round trip is done.
inline void appendResult(const string& directory, size_t method, const AlgorithmRunResult& run, bool allow_failed = false) {
    // 除非调用方显式允许失败尝试，否则失败结果不得写入普通成功结果文件。
    if (!allow_failed && !algorithm_status_valid(run.status)) throw runtime_error("Refusing to write a failed algorithm result");

    // 文件名由方法索引映射；仅在文件尚不存在时写入列标题。
    const fs::path path = fs::path(directory) / (method_name_list.at(method) + ".csv");
    const bool needs_header = !fs::exists(path);
    ofstream output(path, ios::app | ios::binary);
    if (!output) throw runtime_error("Cannot append result: " + path.string());

    // 追加一条序列化尝试并显式刷新、关闭，随后检查磁盘写入是否成功。
    if (needs_header) output << CSV_HEADER << '\n';
    output << run_result_to_csv(run) << '\n';
    output.flush();
    output.close();
    if (!output) throw runtime_error("Result write/close failed: " + path.string());
}

/// Average all selected completed rows for a method; missing/failed rows never become zeros or a successful-only mean.
inline EXPResult average_valid_attempts(const MethodRecords& rows, size_t expected_count) {
    // 汇总要求所选实例齐全；缺行或多行都说明当前条件不能生成均值。
    if (expected_count == 0 || rows.size() != expected_count)
        throw runtime_error("Summary refused: incomplete selected instance set");
    EXPResult mean;

    // 失败尝试不作为零值样本；通过状态检查后，将每条指标按总样本数累加成算术均值。
    for (const auto& entry : rows) {
        const auto& row = entry.second;
        if (!algorithm_status_valid(row.status)) throw runtime_error("Summary refused: failed result");
        for (auto member : EXP_METRICS)
            mean.*member += row.metrics.*member / static_cast<double>(expected_count);
    }
    return mean;
}

/// Write the existing twelve summary tables from a complete condition-by-method mean matrix.
/// Only derived summaries in the current output directory are replaced; raw CSVs are append-only.
inline void exportAllSummaryCSV(const string& directory, const vector<string>& conditions,
    const vector<vector<EXPResult>>& means) {
    // 条件和均值矩阵必须逐行对应，且每个条件都必须包含全部方法的结果。
    if (conditions.empty() || conditions.size() != means.size())
        throw runtime_error("Summary condition count mismatch");
    for (const auto& row : means)
        if (row.size() != method_name_list.size()) throw runtime_error("Summary method count mismatch");

    // 文件顺序与EXP_METRICS的成员指针顺序对应，每个文件输出一个指标的条件×方法宽表。
    const std::array<string, 12> files = {"Run_time_ms", "Total_Num", "Hard_Num", "Elastic_Num",
        "Total_Utility", "Hard_Utility", "Elastic_Utility", "Hard_Bandwidth", "Elastic_Bandwidth",
        "Hard_Throughput", "Elastic_Throughput", "Total_Throughput"};
    fs::create_directories(directory);
    for (size_t metric = 0; metric < files.size(); ++metric) {
        // 每个指标单独写CSV：首行是方法名，之后每行对应一个实验条件。
        const fs::path path = fs::path(directory) / (files[metric] + ".csv");
        ofstream output(path, ios::binary);
        output << setprecision(std::numeric_limits<double>::max_digits10) << "User_Scale";
        for (const auto& method : method_name_list) output << ',' << method;
        output << '\n';
        for (size_t i = 0; i < conditions.size(); ++i) {
            output << conditions[i];
            for (const auto& mean : means[i]) output << ',' << mean.*EXP_METRICS[metric];
            output << '\n';
        }

        // 每张汇总表关闭后检查流状态，避免把未完整写出的文件报告为成功。
        output.flush();
        output.close();
        if (!output) throw runtime_error("Summary write/close failed: " + path.string());
    }
}

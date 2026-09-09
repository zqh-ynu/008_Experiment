#pragma once
// 本文件实现固定 EXP1/3000/ID1--10 的五设置质量消融。由 main.cpp 在 proposed_rerun
// 命名空间之后包含，只复用其只读字节指纹工具；从不调用正式重跑、EXP5 或旧结果写入器。
#include "experiments.h"
#include <numeric>

namespace ton_quality {
inline const string VERSION = "ton-quality-ablation-v1";

/// 一种固定对照设置；Fast 的 epsilon 仅满足外层接口约束，导出时标记为不适用。
struct Variant { string name; int selector; double epsilon; bool early_return; };

/// 返回获批的五种设置，顺序固定为 Fast、当前 Better、同精度无早退及两档细精度。
inline vector<Variant> variants() {
    return {{"Fast", 1, 0.1, true}, {"Better_current", 3, 0.1, true},
        {"Better_no_early_e010", 3, 0.1, false}, {"Better_no_early_e005", 3, 0.05, false},
        {"Better_no_early_e001", 3, 0.01, false}};
}

/// 将有限观测量序列化为数字，未计算值为 null，绝不伪装成零或无穷上界。
inline json number(double value) { return std::isfinite(value) ? json(value) : json(nullptr); }

/// 返回单次运行的全部物理指标；字段名和数值精度沿用原始实验口径。
inline json metrics(const EXPResult& value) {
    const auto names = csv_fields(METRIC_CSV_HEADER);
    json out = json::object();
    for (size_t i = 0; i < EXP_METRICS.size(); ++i) out[names[i]] = value.*EXP_METRICS[i];
    return out;
}

/// 比较所有非耗时指标，报告精确相等与公共容差内相等；无任何修正或回填。
inline json metric_comparison(const EXPResult& actual, const EXPResult& reference) {
    bool exact = true, near = true;
    json differences = json::object();
    const auto names = csv_fields(METRIC_CSV_HEADER);
    for (size_t i = 1; i < EXP_METRICS.size(); ++i) {
        const double a = actual.*EXP_METRICS[i], b = reference.*EXP_METRICS[i];
        exact = exact && a == b;
        const bool equal = i <= 3 ? a == b : allocation_near(a, b);
        near = near && equal;
        if (a != b) differences[names[i]] = {{"actual", a}, {"reference", b}, {"delta", a - b}};
    }
    return {{"exact", exact}, {"within_tolerance", near}, {"differences", differences}};
}

/// 将一个观测对象转换为可追溯字段；证书未尝试时满足性为 null，非 false。
inline json oracle_json(const TonSingleUavDiagnostics& d) {
    return {{"candidate_count", d.candidate_count}, {"retained_count", d.retained_count},
        {"concave_count", d.concave_count}, {"nonconcave_count", d.nonconcave_count},
        {"epsilon", number(d.epsilon)}, {"certificate_early_return", d.certificate_early_return},
        {"certificate_attempted", d.certificate_attempted},
        {"certificate_satisfied", d.certificate_attempted ? json(d.certificate_satisfied) : json(nullptr)},
        {"dual_evaluations", d.dual_evaluations}, {"best_upper", number(d.best_upper)},
        {"dp_entered", d.dp_entered}, {"active_bound", d.active_bound},
        {"delta", number(d.delta)}, {"profit_limit", d.profit_limit},
        {"fast_value", number(d.fast_value)}, {"candidate_value", number(d.candidate_value)},
        {"final_value", number(d.final_value)}, {"return_reason", d.return_reason},
        {"unsafe_count", d.fast.unsafe_count}, {"unsafe_user_id", d.fast.unsafe_user_id},
        {"unsafe_bandwidth", d.fast.unsafe_bandwidth}, {"unsafe_tau", d.fast.unsafe_tau},
        {"fast_return_reason", d.fast.return_reason},
        {"gain_vs_internal_fast", number(d.final_value - d.fast_value)}};
}

/// 仅在同一模型内对完整输入逐值相等的调用进行局部对比；不使用近似状态匹配。
inline bool same_state(const TonOracleTrace& a, const TonOracleTrace& b) {
    return a.uav_id == b.uav_id && a.budget == b.budget && a.candidate_ids == b.candidate_ids &&
        a.current_utilities == b.current_utilities && a.base_bandwidths == b.base_bandwidths;
}

/// 序列化网络轨迹并去重完整状态向量；state_id 只在本文件内有效，跨设置必须比较状态内容。
/// fast_reference 来自同一输入的 Fast 运行；路径分叉后不匹配的局部差值保留 null。
inline json trace_json(const TonNetworkDiagnostics& d, const TonNetworkDiagnostics* fast_reference = nullptr) {
    json out = {{"selection_order", d.selection_order}, {"pre_residual_utility", number(d.pre_residual_utility)},
        {"post_residual_utility", number(d.post_residual_utility)}, {"states", json::array()}, {"calls", json::array()}};
    vector<size_t> state_representatives;
    for (size_t i = 0; i < d.calls.size(); ++i) {
        const auto& call = d.calls[i];
        size_t state_id = 0;
        for (; state_id < state_representatives.size(); ++state_id) {
            const auto& prior = d.calls[state_representatives[state_id]];
            if (call.current_utilities == prior.current_utilities && call.base_bandwidths == prior.base_bandwidths) break;
        }
        if (state_id == state_representatives.size()) {
            state_representatives.push_back(i);
            out["states"].push_back({{"state_id", state_id}, {"current_utilities", call.current_utilities},
                {"base_bandwidths", call.base_bandwidths}});
        }
        json row = oracle_json(call.oracle);
        row["call_index"] = i; row["phase"] = call.phase; row["round"] = call.round;
        row["uav_id"] = call.uav_id; row["selector"] = call.selector; row["budget"] = call.budget;
        if (call.selector == 1) row["certificate_early_return"] = nullptr;
        row["selected"] = call.selected; row["candidate_ids"] = call.candidate_ids; row["state_id"] = state_id;
        row["same_state_as_fast"] = nullptr; row["delta_vs_fast_path"] = nullptr;
        if (fast_reference) {
            row["same_state_as_fast"] = false;
            for (const auto& ref : fast_reference->calls)
                if (call.phase == ref.phase && call.round == ref.round && same_state(call, ref)) {
                    row["same_state_as_fast"] = true;
                    row["delta_vs_fast_path"] = number(call.oracle.final_value - ref.oracle.final_value);
                    break;
                }
        }
        out["calls"].push_back(std::move(row));
    }
    return out;
}

/// 只创建新文件，使用同目录临时文件与重命名提交；目标存在即拒绝，保留中断临时文件。
inline void write_new(const fs::path& path, const string& contents) {
    if (fs::exists(path)) throw runtime_error("消融拒绝覆盖文件: " + path.string());
    localization::atomic_write(path, contents);
}

/// 通过原子创建目录预留本次唯一运行位置；不恢复、不清理任何既有运行。
inline fs::path reserve_run(const fs::path& parent) {
    fs::create_directories(parent);
    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    for (unsigned suffix = 0; suffix < 1000; ++suffix) {
        const auto path = parent / ("run_" + to_string(stamp) + "_" + to_string(suffix));
        if (fs::create_directory(path)) return path;
    }
    throw runtime_error("无法预留唯一消融目录");
}

/// 返回当前文件字节的 SHA-256 与大小；只读复用现有 Windows SDK 指纹实现。
inline json file_identity(const fs::path& path) {
    auto result = proposed_rerun::fingerprint(proposed_rerun::bytes(path));
    result["path"] = experiment_absolute_path(path);
    return result;
}

/// 记录实际运行的 Windows 可执行文件字节身份，不把磁盘源码指纹冒充构建快照。
inline json executable_identity() {
    vector<wchar_t> path(32768, L'\0');
    const DWORD length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (length == 0 || length >= path.size()) throw runtime_error("无法读取消融可执行文件路径");
    return file_identity(fs::path(std::wstring(path.data(), length)));
}

/// 执行一种设置并复用公共结果校验；模型复制在计时外，诊断开销在计时内且明确标记。
inline AlgorithmRunResult solve(const SystemMd& model, const Variant& variant, int id,
    TonNetworkDiagnostics* diagnostics) {
    const auto seed = derive_algorithm_seed(20260905u, "EXP1_user_num", "3000", id,
        variant.selector == 1 ? "ApproFast" : "ApproBetter");
    TonFasterOptions options; options.certificate_early_return = variant.early_return;
    auto result = execute_allocation(model, seed,
        [&](BAProblem& problem, AllocationDiagnostics&) {
            return problem.Appro_multiUAV_ToN(model.uavs, model.users, variant.selector,
                variant.epsilon, options, diagnostics);
        });
    result.instance_id = id;
    return result;
}

/// 严格核对缓存结果与逐用户投影；物理约束仍由 compute_single_EXPResult 进行独立重算。
inline void validate_cached_results(const SystemMd& model, const AlgorithmRunResult& run) {
    const auto& allocations = run.allocation.first;
    double total = 0;
    size_t served = 0;
    for (const auto& a : allocations) {
        double value = 0, bandwidth = 0;
        for (int id : a.allocatedList) {
            const double bw = a.allocatedBandwidth.at(id);
            const auto& user = model.users.at(id);
            const double expected = user.uType == HARD_UTILITY ? user.weight * std::log2(1 + user.rMin)
                : user.weight * std::log2(1 + bw * model.cap_list.at(a.uav_id).at(id));
            require_allocation(allocation_near(a.allocatedValue.at(id), expected), "消融: 缓存用户效用不一致");
            const auto& projected = run.allocation.second.at(id);
            require_allocation(projected.uav_id == a.uav_id && allocation_near(projected.allocated_bandwidth, bw) &&
                allocation_near(projected.utility, expected), "消融: 逐用户投影不一致");
            value += expected; bandwidth += bw; ++served;
        }
        require_allocation(allocation_near(value, a.totalValue) && allocation_near(bandwidth, a.totalWeight),
            "消融: UAV 缓存总量不一致");
        total += value;
    }
    require_allocation(allocation_near(total, run.metrics.total_utility) && served == run.metrics.total_num,
        "消融: 网络效用或人数不一致");
}

/// 将具有共同字段的记录编码为 UTF-8 CSV；数值不预先舍入，缺失量留空。
inline string table_csv(const vector<string>& columns, const vector<json>& rows) {
    ostringstream out;
    for (size_t i = 0; i < columns.size(); ++i) out << (i ? "," : "") << csv_quote(columns[i]);
    out << '\n';
    for (const auto& row : rows) {
        for (size_t i = 0; i < columns.size(); ++i) {
            out << (i ? "," : "");
            const auto& value = row.at(columns[i]);
            if (!value.is_null()) out << csv_quote(value.is_string() ? value.get<string>() : value.dump());
        }
        out << '\n';
    }
    return out.str();
}

/// 汇总单个网络设置的实际调用计数；证书命中分母是 certificate_attempts，不是网络数量。
inline json diagnostic_counts(int id, const string& variant, const json& trace) {
    size_t greedy = 0, residual = 0, attempted = 0, satisfied = 0, early = 0, dp = 0, unsafe = 0, improved = 0;
    size_t selected = 0, selected_early = 0, selected_dp = 0;
    for (const auto& call : trace.at("calls")) {
        if (call.at("phase") == "greedy") ++greedy; else ++residual;
        if (call.at("certificate_attempted").get<bool>()) ++attempted;
        if (!call.at("certificate_satisfied").is_null() && call.at("certificate_satisfied").get<bool>()) ++satisfied;
        const bool is_early = call.at("return_reason") == "certificate_fast";
        const bool is_dp = call.at("dp_entered").get<bool>();
        if (is_early) ++early;
        if (is_dp) ++dp;
        if (call.at("unsafe_count").get<size_t>() > 0) ++unsafe;
        if (!call.at("gain_vs_internal_fast").is_null() && call.at("gain_vs_internal_fast").get<double>() > 0) ++improved;
        if (call.at("phase") == "greedy" && call.at("selected").get<bool>()) {
            ++selected; if (is_early) ++selected_early; if (is_dp) ++selected_dp;
        }
    }
    return {{"instance_id", id}, {"variant", variant}, {"greedy_calls", greedy}, {"residual_calls", residual},
        {"certificate_attempts", attempted}, {"certificate_satisfied_calls", satisfied}, {"early_returns", early},
        {"dp_calls", dp}, {"unsafe_calls", unsafe}, {"improved_vs_internal_fast_calls", improved},
        {"selected_greedy_calls", selected}, {"selected_early_returns", selected_early}, {"selected_dp_calls", selected_dp},
        {"pre_residual_utility", trace.at("pre_residual_utility")}, {"post_residual_utility", trace.at("post_residual_utility")}};
}

/// 返回已计算样本的描述统计；只用于汇总，不进行显著性检验或最优性判定。
inline json describe(vector<double> values) {
    if (values.empty()) return {{"n", 0}, {"mean", nullptr}, {"median", nullptr}, {"min", nullptr}, {"max", nullptr}};
    const double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    std::sort(values.begin(), values.end());
    const size_t n = values.size();
    return {{"n", n}, {"mean", mean}, {"median", n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2},
        {"min", values.front()}, {"max", values.back()}};
}

/// 比较同一实例的最终网络指标，保留负收益与零收益；Fast 为零时相对提升不适用。
inline json paired_row(int id, const Variant& v, const EXPResult& value, const EXPResult& fast) {
    const double delta = value.total_utility - fast.total_utility;
    const bool near = allocation_near(value.total_utility, fast.total_utility);
    return {{"instance_id", id}, {"variant", v.name}, {"fast_total_utility", fast.total_utility},
        {"total_utility", value.total_utility}, {"absolute_gain", delta},
        {"relative_gain_pct", fast.total_utility > 0 ? number(100 * delta / fast.total_utility) : json(nullptr)},
        {"hard_utility_gain", value.hard_utility - fast.hard_utility},
        {"elastic_utility_gain", value.elastic_utility - fast.elastic_utility},
        {"outcome", near ? "tie" : (delta > 0 ? "win" : "loss")},
        {"exact_total_equal", value.total_utility == fast.total_utility}, {"within_tolerance", near}};
}

/// 仅在五种设置均完成十个网络后产生总汇；不足时拒绝输出成功样本均值。
inline void write_summary(const fs::path& root, const vector<Variant>& settings,
    const std::array<vector<EXPResult>, 5>& collected, const vector<json>& pairs) {
    vector<json> rows;
    auto columns = vector<string>{"variant", "n"};
    const auto metric_names = csv_fields(METRIC_CSV_HEADER);
    columns.insert(columns.end(), metric_names.begin(), metric_names.end());
    const vector<string> extra = {"absolute_gain_mean", "absolute_gain_median", "absolute_gain_min", "absolute_gain_max",
        "relative_gain_pct_mean", "relative_gain_pct_median", "relative_gain_pct_min", "relative_gain_pct_max",
        "relative_gain_valid_n", "wins", "ties", "losses", "timing_scope"};
    columns.insert(columns.end(), extra.begin(), extra.end());
    for (size_t v = 0; v < settings.size(); ++v) {
        if (collected[v].size() != 10) throw runtime_error("消融汇总要求每设置完整十个网络");
        EXPResult average;
        for (const auto& result : collected[v])
            for (auto member : EXP_METRICS) average.*member += result.*member / 10.0;
        json row = metrics(average);
        row["variant"] = settings[v].name; row["n"] = 10;
        vector<double> absolute, relative;
        size_t wins = 0, ties = 0, losses = 0;
        for (const auto& pair : pairs) if (pair.at("variant") == settings[v].name) {
            absolute.push_back(pair.at("absolute_gain").get<double>());
            if (!pair.at("relative_gain_pct").is_null()) relative.push_back(pair.at("relative_gain_pct").get<double>());
            if (pair.at("outcome") == "win") ++wins;
            else if (pair.at("outcome") == "loss") ++losses;
            else ++ties;
        }
        const auto a = describe(absolute), r = describe(relative);
        for (const string name : {"mean", "median", "min", "max"}) {
            row["absolute_gain_" + name] = a.at(name);
            row["relative_gain_pct_" + name] = r.at(name);
        }
        row["relative_gain_valid_n"] = r.at("n"); row["wins"] = wins; row["ties"] = ties; row["losses"] = losses;
        row["timing_scope"] = "exploratory_single_run_not_formal_timing";
        rows.push_back(std::move(row));
    }
    write_new(root / "summary.csv", table_csv(columns, rows));
    write_new(root / "paired_comparisons.csv", table_csv({"instance_id", "variant", "fast_total_utility",
        "total_utility", "absolute_gain", "relative_gain_pct", "hard_utility_gain", "elastic_utility_gain",
        "outcome", "exact_total_equal", "within_tolerance"}, pairs));
}

/// 固定运行获批的五设置、十网络；每个 case 先保存原始输出和诊断，再写完成标记。
/// collect_diagnostics=false 供用户另行运行诊断关闭对照；默认 true，不额外重复网络求解。
/// 返回 0 表示 50 次求解及复现检查均完成；失败抛异常，保留目录与现场，不自动重试。
inline int run(bool collect_diagnostics = true) {
    if (experiment_build_profile() != "Release|x64") throw runtime_error("质量消融要求 Release|x64");
#if defined(TON_VERIFY_SMAWK)
    throw runtime_error("质量消融请关闭 TON_VERIFY_SMAWK；机制验证使用独立测试入口");
#endif
    ExperimentRunOptions options;
    options.instance_count = 10; options.master_seed = 20260905u; options.rounding_trials = 2;
    options.ton_epsilon = 0.1; options.output_name = "run_ton_01";
    options.input_root = (fs::path(experimentDataPath) / "data_ToN" / "2026-09-07").string();
    const fs::path input_root = resolve_experiment_input_root(options);
    const fs::path exp_root = fs::path(experimentDataPath) / "ExperimentsResults" / "EXP1_user_num";
    const fs::path reference_root = exp_root / "ToN_simple" / "run_ton_01";
    const fs::path config = exp_root / "def_config.json";
    const auto directory = (input_root / "variable_user_num" / "3000u_num").string();
    const auto condition = select_experiment_condition("3000", directory, directory,
        "3000users_data", "10uavs_loc", 40, options);
    const auto expected = make_experiment_run_info("EXP1_user_num", {condition}, config.string(), options);
    const auto reference_info_identity = file_identity(reference_root / "run_info.json");
    const auto reference_fast_identity = file_identity(reference_root / "3000" / "ApproFast.csv");
    const auto reference_better_identity = file_identity(reference_root / "3000" / "ApproBetter.csv");
    const auto reference = read_experiment_json(reference_root / "run_info.json");
    for (const string key : {"algorithm_version", "proposed_algorithms", "master_seed", "ton_epsilon",
        "instance_count", "channel_config", "unit_para", "abs_tolerance", "rel_tolerance", "build_profile"})
        if (reference.at(key) != expected.at(key)) throw runtime_error("历史对照身份不匹配: " + key);
    if (reference.at("summary").at("status") != "complete") throw runtime_error("历史对照尚未完成");
    bool matched_condition = false;
    for (const auto& item : reference.at("conditions"))
        if (item == expected.at("conditions").at(0)) matched_condition = true;
    if (!matched_condition) throw runtime_error("历史对照的 3000 条件输入与当前固定输入不一致");
    const auto historic_fast = read_result_csv(reference_root / "3000" / "ApproFast.csv", true);
    const auto historic_better = read_result_csv(reference_root / "3000" / "ApproBetter.csv", true);
    for (const auto* rows : {&historic_fast, &historic_better}) {
        if (rows->size() != 10) throw runtime_error("历史对照必须完整包含 ID 1--10");
        for (int id = 1; id <= 10; ++id) if (!rows->count(id)) throw runtime_error("历史对照缺少实例 ID");
    }
    /// 在读取后和最终发布前检查旧对照是否被其他进程更改，不修改这些对照文件。
    const auto verify_reference = [&]() {
        if (file_identity(reference_root / "run_info.json") != reference_info_identity ||
            file_identity(reference_root / "3000" / "ApproFast.csv") != reference_fast_identity ||
            file_identity(reference_root / "3000" / "ApproBetter.csv") != reference_better_identity)
            throw runtime_error("消融期间历史对照被外部更改");
    };
    verify_reference();
    vector<json> inputs;
    for (int i = 0; i < 10; ++i) inputs.push_back({{"instance_id", i + 1},
        {"users", file_identity(condition.users[i])}, {"uavs", file_identity(condition.uavs[i])}});
    const auto config_identity = file_identity(config);
    const auto settings = variants();
    json setting_rows = json::array();
    for (const auto& v : settings) setting_rows.push_back({{"name", v.name}, {"selector", v.selector},
        {"epsilon", v.selector == 1 ? json(nullptr) : json(v.epsilon)}, {"certificate_early_return", v.early_return}});
    json source_files = json::array();
    const auto source_dir = (fs::path(experimentDataPath) / ".." / "Algorithms" / "UAVBandwidthAllocation").lexically_normal();
    for (const string name : {"main.cpp", "EntityDefinition.cpp", "EntityDefinition.h", "ton_quality_diagnostics.h",
        "ton_quality_ablation.h", "experiment_support.h", "allocation_contract.h", "predefine.h"})
        source_files.push_back(file_identity(source_dir / name));
    const auto root = reserve_run(exp_root / "ToN_quality_ablation" / "exp1_3000_ids1_10");
    size_t completed = 0;
    int active_id = 0;
    string active_variant;
    cout << "[QUALITY OUTPUT] " << root << std::endl;
    try {
        json manifest = {{"schema", VERSION}, {"compile_date", __DATE__}, {"compile_time", __TIME__},
            {"msc_full_ver", _MSC_FULL_VER}, {"build_profile", experiment_build_profile()},
            {"iterator_debug_level", _ITERATOR_DEBUG_LEVEL}, {"msvc_stl_update", _MSVC_STL_UPDATE},
            {"diagnostics_enabled", collect_diagnostics}, {"expected_network_calls", 50},
            {"instance_count", 10}, {"master_seed", options.master_seed}, {"variants", setting_rows},
            {"inputs", inputs}, {"config", config_identity}, {"channel_config", expected.at("channel_config")},
            {"units", {{"bandwidth", "kHz"}, {"rate", "Kbps"}, {"duration", "ms"}}},
            {"abs_tolerance", ALLOCATION_ABS_TOL}, {"rel_tolerance", ALLOCATION_REL_TOL},
            {"historical_fast", reference_fast_identity}, {"historical_better", reference_better_identity},
            {"historical_run_info", reference_info_identity}, {"executable", executable_identity()},
            {"runtime_disk_sources_not_build_snapshot", source_files},
            {"timing_scope", "exploratory; no formal timing claim; diagnostics included when enabled"},
            {"completion_rule", "completion.json complete, 50 calls, all case complete.json present"}};
        write_new(root / "run_info.json", manifest.dump(2) + "\n");
        std::array<vector<EXPResult>, 5> collected;
        vector<json> pairs;
        vector<json> diagnostics_rows;
        for (int id = 1; id <= 10; ++id) {
            active_id = id; active_variant = "loading";
            if (file_identity(condition.users[id - 1]) != inputs[id - 1].at("users") ||
                file_identity(condition.uavs[id - 1]) != inputs[id - 1].at("uavs") || file_identity(config) != config_identity)
                throw runtime_error("消融运行期间输入或配置已改变");
            SystemMd model(condition.users[id - 1], condition.uavs[id - 1], config.string());
            if (model.users.size() != 3000 || model.uavs.size() != 10) throw runtime_error("消融网络规模不匹配");
            for (auto& uav : model.uavs) uav.total_bandwidth = 40 * unit_para;
            model.init_SystemModel(); validate_allocation_model(model);
            EXPResult fast_metrics;
            TonNetworkDiagnostics fast_trace;
            for (size_t v = 0; v < settings.size(); ++v) {
                active_variant = settings[v].name;
                const auto case_root = root / "cases" / ("id_" + to_string(id)) / active_variant;
                TonNetworkDiagnostics trace;
                cout << "[QUALITY " << completed + 1 << "/50] ID=" << id << " " << active_variant << std::endl;
                auto result = solve(model, settings[v], id, collect_diagnostics ? &trace : nullptr);
                // 失败也先保存带状态的原始记录；失败指标由原 CSV 规则留空。
                write_new(case_root / "result.csv", CSV_HEADER + "\n" + run_result_to_csv(result) + "\n");
                if (collect_diagnostics) {
                    const auto observed = trace_json(trace, v == 0 ? nullptr : &fast_trace);
                    write_new(case_root / "trace.json", observed.dump() + "\n");
                    write_new(case_root / "oracle_calls.csv", table_csv({"call_index", "phase", "round", "uav_id", "selector",
                        "budget", "selected", "state_id", "candidate_count", "retained_count", "concave_count", "nonconcave_count",
                        "epsilon", "unsafe_count", "unsafe_user_id", "unsafe_bandwidth", "unsafe_tau", "fast_return_reason",
                        "fast_value", "certificate_early_return", "certificate_attempted", "certificate_satisfied", "dual_evaluations",
                        "best_upper", "dp_entered", "active_bound", "delta", "profit_limit", "candidate_value", "final_value",
                        "return_reason", "gain_vs_internal_fast", "same_state_as_fast", "delta_vs_fast_path"},
                        observed.at("calls").get<vector<json>>()));
                    diagnostics_rows.push_back(diagnostic_counts(id, active_variant, observed));
                }
                json validation = {{"status", algorithm_status_name(result.status)}, {"history_comparison", nullptr}};
                if (algorithm_status_valid(result.status)) {
                    validate_cached_results(model, result);
                    if (v <= 1) {
                        const auto& old = (v == 0 ? historic_fast : historic_better).at(id);
                        if (old.seed != result.seed) throw runtime_error("历史对照种子不匹配");
                        validation["history_comparison"] = metric_comparison(result.metrics, old.metrics);
                    }
                }
                write_new(case_root / "validation.json", validation.dump(2) + "\n");
                if (!algorithm_status_valid(result.status)) throw runtime_error("消融求解失败，保留当前 case");
                if (v <= 1 && !validation.at("history_comparison").at("within_tolerance").get<bool>())
                    throw runtime_error("默认算法未重现历史非耗时指标，停止而不继续混入消融结果");
                if (v == 0) fast_metrics = result.metrics;
                collected[v].push_back(result.metrics);
                pairs.push_back(paired_row(id, settings[v], result.metrics, fast_metrics));
                write_new(case_root / "complete.json", json({{"instance_id", id}, {"variant", active_variant},
                    {"status", "complete"}, {"diagnostics_enabled", collect_diagnostics}}).dump(2) + "\n");
                if (v == 0 && collect_diagnostics) fast_trace = std::move(trace);
                ++completed;
            }
        }
        // 结束前再次核对输入，不把运行期间被外部修改的数据当成同一批次。
        for (int i = 0; i < 10; ++i)
            if (file_identity(condition.users[i]) != inputs[i].at("users") || file_identity(condition.uavs[i]) != inputs[i].at("uavs"))
                throw runtime_error("消融结束检查发现输入已改变");
        if (file_identity(config) != config_identity) throw runtime_error("消融结束检查发现配置已改变");
        verify_reference();
        write_summary(root, settings, collected, pairs);
        if (collect_diagnostics) write_new(root / "diagnostic_summary.csv", table_csv({"instance_id", "variant", "greedy_calls",
            "residual_calls", "certificate_attempts", "certificate_satisfied_calls", "early_returns", "dp_calls", "unsafe_calls",
            "improved_vs_internal_fast_calls", "selected_greedy_calls", "selected_early_returns", "selected_dp_calls",
            "pre_residual_utility", "post_residual_utility"}, diagnostics_rows));
        write_new(root / "completion.json", json({{"status", "complete"}, {"completed_network_calls", completed}}).dump(2) + "\n");
        cout << "[QUALITY COMPLETE] 50/50，结果: " << root << std::endl;
        return 0;
    } catch (const std::exception& error) {
        try { write_new(root / "failure.json", json({{"status", "incomplete"}, {"completed_network_calls", completed},
            {"instance_id", active_id}, {"variant", active_variant}, {"error", error.what()}}).dump(2) + "\n"); }
        catch (const std::exception& save_error) { cerr << "保存失败现场时再次出错: " << save_error.what() << std::endl; }
        throw;
    }
}

} // namespace ton_quality

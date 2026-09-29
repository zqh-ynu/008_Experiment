#pragma once
// 实验入口：保留旧带宽/定位误差所需流程，新五实验读取multi-hard输入并按已尝试记录续跑。
// 新实验保存run_info、逐方法汇总与逐用户CSV；仅EXP3按计算量缩减HardFirst候选，临时定向重跑放在main.cpp。
#include "experiment_support.h"
#include <memory> // 新循环在输入构造失败后用空指针记录六种方法的失败，不重复构造实例。
#include <boost/multiprecision/cpp_int.hpp> // 用已有Boost的128位整数精确计算EXP3预算，避免ceil边界误差。

/// Inputs for one condition; users/uavs contain exactly the selected numeric-ID prefix.
struct ExperimentCondition {
    string key;
    vector<string> users, uavs;
    double bandwidth_mhz = 40.0;
};

/// Return the requested condition order, or all defaults; reject unknown/duplicate keys before path construction.
inline vector<string> experiment_condition_keys(const ExperimentRunOptions& options,
    const vector<string>& defaults) {
    const auto& keys = options.conditions.empty() ? defaults : options.conditions;
    set<string> seen;
    for (const auto& key : keys)
        if (std::find(defaults.begin(), defaults.end(), key) == defaults.end() || !seen.insert(key).second)
            throw invalid_argument("Unknown or duplicate experiment condition: " + key);
    return keys;
}

/// Select readable user/UAV pairs for exactly IDs 1--N from the supplied directories and filename patterns.
/// Return one bandwidth condition; missing IDs throw instead of substituting a higher-numbered instance.
inline ExperimentCondition select_experiment_condition(const string& key, const string& user_directory,
    const string& uav_directory, const string& user_pattern, const string& uav_pattern,
    double bandwidth_mhz, const ExperimentRunOptions& options) {
    ExperimentCondition condition{key, {}, {}, bandwidth_mhz};
    getMatchedFilePairs(user_directory, uav_directory, user_pattern, uav_pattern, INT_MAX,
        condition.users, condition.uavs);
    if (condition.users.size() != condition.uavs.size() ||
        condition.users.size() < static_cast<size_t>(options.instance_count))
        throw runtime_error("Not enough paired instances for condition " + key);
    condition.users.resize(options.instance_count);
    condition.uavs.resize(options.instance_count);
    for (int i = 0; i < options.instance_count; ++i) {
        const int id = allocation_instance_id(condition.users[i]);
        if (id != i + 1 || id != allocation_instance_id(condition.uavs[i]))
            throw runtime_error("Condition " + key + " requires paired IDs 1--" +
                to_string(options.instance_count) + "; missing or mismatched ID " + to_string(i + 1));
    }
    return condition;
}

/// Read a requested JSON object; errors retain the filename and never create/repair a metadata file.
inline json read_experiment_json(const fs::path& path) {
    ifstream input(path, ios::binary);
    if (!input) throw runtime_error("Cannot read JSON: " + path.string());
    json value;
    try { input >> value; }
    catch (const std::exception& error) { throw runtime_error(path.string() + ": " + error.what()); }
    if (input.bad()) throw runtime_error("JSON read error: " + path.string());
    if (!value.is_object()) throw runtime_error("Expected JSON object: " + path.string());
    return value;
}

/// Resolve options.input_root without fallback; return its absolute path after checking input readiness.
/// The legacy data directory needs no generation marker. Other roots require a completed ToN batch
/// with a declared ID 1--replicate_count prefix large enough for the requested instance_count.
inline fs::path resolve_experiment_input_root(const ExperimentRunOptions& options) {
    const fs::path root = fs::absolute(options.input_root).lexically_normal();
    if (!fs::is_directory(root))
        throw runtime_error("Input directory does not exist; generate inputs first: " + root.string());
    const fs::path legacy = fs::path(experimentDataPath) / "data";
    if (fs::is_directory(legacy) && fs::equivalent(root, legacy)) return root;

    const fs::path marker = root / "generation_config_ToN.json";
    const json info = read_experiment_json(marker);
    if (info.value("generation_status", string{}) != "complete")
        throw runtime_error("ToN input generation is not complete: " + marker.string());
    if (!info.at("replicate_count").is_number_integer())
        throw runtime_error("Invalid ToN replicate_count: " + marker.string());
    const int64_t available = info.at("replicate_count").get<int64_t>();
    const auto& replicates = info.at("replicates");
    if (available < options.instance_count || available > INT_MAX || !replicates.is_array() ||
        replicates.size() != static_cast<size_t>(available))
        throw runtime_error("ToN batch has insufficient or inconsistent declared inputs: " + marker.string());
    for (int64_t i = 0; i < available; ++i)
        if (!replicates.at(static_cast<size_t>(i)).at("replicate_id").is_number_integer() ||
            replicates.at(static_cast<size_t>(i)).at("replicate_id").get<int64_t>() != i + 1)
            throw runtime_error("ToN batch must declare contiguous replicate IDs from 1: " + marker.string());
    return root;
}

/// 生成包含算法版本、hard 八候选匹配/种子/择优策略、参数和输入路径的元数据；返回值供续跑严格比较。
inline json make_experiment_run_info(const string& experiment, const vector<ExperimentCondition>& conditions,
    const string& config, const ExperimentRunOptions& options) {
    load_global_channel_config(config);
    json info = {
        {"schema", SIMPLE_RUN_SCHEMA}, {"algorithm_version", SIMPLE_ALGORITHM_VERSION},
        {"proposed_algorithms", proposed_algorithm_policy()},
        {"experiment", experiment}, {"methods", method_name_list},
        {"hard_first_policy", hard_first_policy()},
        {"instance_count", options.instance_count}, {"master_seed", options.master_seed},
        {"rounding_trials", options.rounding_trials}, {"ton_epsilon", options.ton_epsilon},
        {"reallocate_residual", options.reallocate_residual}, // 保留历史字段；算法忽略该值，新策略记录在proposed_algorithms中。
        {"baseline_hard_policy", "lowest_level_only"},
        {"baseline_hard_bandwidth_policy", {
            {"AlgRelaxRound", "clamp_to_lowest_threshold_no_reallocation"},
            {"AlgSwapMatching", "clamp_to_lowest_threshold_no_reallocation"},
            {"AlgHardFirst", "reserve_one_integer_slot_report_lowest_threshold"},
            {"AlgSA-DD", "clamp_to_lowest_threshold_no_reallocation"}}},
        {"utility_evaluation", "original_model"},
        {"config_path", experiment_absolute_path(config)}, {"channel_config", read_experiment_json(config)},
        {"unit_para", unit_para}, {"abs_tolerance", ALLOCATION_ABS_TOL}, {"rel_tolerance", ALLOCATION_REL_TOL},
        {"build_profile", experiment_build_profile()},
        {"reuse_source", experiment == "EXP1_user_num" && !options.reuse_exp1_root.empty()
            ? experiment_absolute_path(options.reuse_exp1_root) : ""},
        {"reuse_methods", json::array()},
        {"conditions", json::array()},
        {"summary", {{"status", "not_generated"}, {"instance_count", 0}}}
    };
#if defined(TON_VERIFY_SMAWK)
    info["smawk_verification"] = true;
#else
    info["smawk_verification"] = false;
#endif
    if (!info.at("reuse_source").get<string>().empty())
        info["reuse_methods"] = {"ApproFast", "AlgRelaxRound", "AlgSwapMatching", "AlgSA-DD"};
    for (const auto& condition : conditions) {
        json entry = {{"key", condition.key}, {"bandwidth_mhz", condition.bandwidth_mhz},
            {"inputs", json::array()}};
        for (int i = 0; i < options.instance_count; ++i)
            entry["inputs"].push_back({{"id", allocation_instance_id(condition.users[i])},
                {"user_path", experiment_absolute_path(condition.users[i])},
                {"uav_path", experiment_absolute_path(condition.uavs[i])}});
        info["conditions"].push_back(std::move(entry));
    }
    return info;
}

/// Inspect an existing run without writes; only a larger selected-input prefix is allowed in the same directory.
/// Return expected settings with the previous summary's sample count/status retained for stale-summary reporting.
inline json inspect_experiment_destination(const fs::path& root, const json& expected) {
    const fs::path metadata = root / "run_info.json";
    if (!fs::exists(metadata)) {
        if (fs::exists(root) && !fs::is_empty(root))
            throw runtime_error("Nonempty output has no run_info.json; choose a new output_name");
        return expected;
    }
    const json old = read_experiment_json(metadata);
    const int previous_count = old.at("instance_count").get<int>();
    if (previous_count <= 0 || expected.at("instance_count").get<int>() < previous_count)
        throw runtime_error("Cannot reduce instance_count in an existing run; choose a new output_name");
    json previous_settings = old, next_settings = expected;
    for (const string field : {"instance_count", "conditions", "summary"}) {
        previous_settings.erase(field);
        next_settings.erase(field);
    }
    if (previous_settings != next_settings)
        throw runtime_error("Run parameters/version changed; choose a new output_name");
    const auto& old_conditions = old.at("conditions");
    const auto& new_conditions = expected.at("conditions");
    if (old_conditions.size() != new_conditions.size())
        throw runtime_error("Condition selection changed; choose a new output_name");
    for (size_t c = 0; c < new_conditions.size(); ++c) {
        const auto& prior = old_conditions.at(c);
        const auto& next = new_conditions.at(c);
        if (prior.at("key") != next.at("key") || prior.at("bandwidth_mhz") != next.at("bandwidth_mhz") ||
            prior.at("inputs").size() != static_cast<size_t>(previous_count))
            throw runtime_error("Previous condition/input selection is inconsistent");
        for (int i = 0; i < previous_count; ++i)
            if (prior.at("inputs").at(i) != next.at("inputs").at(i))
                throw runtime_error("Previously selected input changed; choose a new output_name");
    }
    const int summary_count = old.at("summary").at("instance_count").get<int>();
    const string status = old.at("summary").at("status").get<string>();
    if (summary_count < 0 || summary_count > previous_count ||
        (status != "not_generated" && status != "complete" && status != "stale" && status != "writing"))
        throw runtime_error("Invalid summary metadata; existing files preserved");
    json result = expected;
    result["summary"] = old.at("summary");
    return result;
}

/// Persist the single mutable run-info file, reporting flush/close errors; raw results are never overwritten.
inline void write_experiment_run_info(const fs::path& root, const json& info) {
    ofstream output(root / "run_info.json", ios::binary);
    output << info.dump(2) << '\n';
    output.flush();
    output.close();
    if (!output) throw runtime_error("Cannot write run_info.json: " + root.string());
}

/// Read only the audited v1 EXP1 source and prepare missing rows for the four unchanged methods.
/// Existing equal rows are skipped; incompatible settings, seeds, selected paths or conflicting rows stop before writes.
inline vector<ConditionRecords> prepare_exp1_reuse(const json& info,
    const vector<ConditionRecords>& existing, size_t& reusable_rows) {
    reusable_rows = 0;
    vector<ConditionRecords> pending(info.at("conditions").size(), ConditionRecords(method_name_list.size()));
    const string source = info.at("reuse_source").get<string>();
    if (source.empty()) return pending;
    const string approved = experiment_absolute_path(fs::path(experimentDataPath) /
        "ExperimentsResults/ToN_routeA_v1_eps0p1/batch_20260905_n10_01");
    if (source != approved || info.at("experiment") != "EXP1_user_num")
        throw runtime_error("Only the explicitly audited v1 EXP1 reuse source is supported");
    if (info.at("build_profile") != "Release|x64")
        throw runtime_error("Legacy timing reuse requires the original Release|x64 configuration");
    const json legacy = read_experiment_json(fs::path(source) / "run_manifest.json");
    const auto& plan = legacy.at("plan");
    if (plan.at("result_version") != "ToN_routeA_v1_eps0p1/")
        throw runtime_error("Wrong legacy result version");
    const json* old_experiment = nullptr;
    for (const auto& exp : plan.at("experiments"))
        if (exp.at("name") == "EXP1_user_num") old_experiment = &exp;
    if (!old_experiment) throw runtime_error("Legacy source has no EXP1");
    const std::array<size_t, 4> reusable_methods = {1, 2, 3, 5}; // 不导入 ApproBetter 或已更换机制的 AlgHardFirst。
    for (size_t c = 0; c < pending.size(); ++c) {
        const auto& target = info.at("conditions").at(c);
        const string key = target.at("key").get<string>();
        const json* old_condition = nullptr;
        for (const auto& candidate : old_experiment->at("conditions"))
            if (candidate.at("condition") == key) old_condition = &candidate;
        if (!old_condition) throw runtime_error("Legacy source lacks condition " + key);
        for (const string field : {"master_seed", "rounding_trials", "ton_epsilon", "channel_config",
            "unit_para", "abs_tolerance", "rel_tolerance"})
            if (old_condition->at(field) != info.at(field))
                throw runtime_error("Legacy reuse parameter mismatch: " + field);
        if (old_condition->at("bandwidth_mhz") != target.at("bandwidth_mhz"))
            throw runtime_error("Legacy bandwidth differs for condition " + key);
        map<int, json> target_inputs, old_inputs;
        for (const auto& input : target.at("inputs"))
            target_inputs.emplace(input.at("id").get<int>(), input);
        for (const auto& input : old_condition->at("inputs"))
            if (!old_inputs.emplace(input.at("id").get<int>(), input).second)
                throw runtime_error("Duplicate legacy input ID");
        for (size_t m : reusable_methods) {
            if (old_condition->at("methods").at(m) != method_name_list[m])
                throw runtime_error("Legacy method identity mismatch");
            const auto records = read_result_csv(fs::path(source) / "EXP1_user_num" / key /
                (method_name_list[m] + ".csv"), true);
            for (const auto& entry : records) {
                const int id = entry.first;
                const auto selected = target_inputs.find(id);
                if (selected == target_inputs.end()) continue; // A smaller requested prefix imports only its own IDs.
                const auto original = old_inputs.find(id);
                if (original == old_inputs.end()) throw runtime_error("Legacy row has no declared input");
                for (const string field : {"user_path", "uav_path"})
                    if (experiment_absolute_path(original->second.at(field).get<string>()) !=
                        selected->second.at(field).get<string>())
                        throw runtime_error("Legacy selected input differs: " + key + "/" + to_string(id));
                const uint32_t seed = derive_algorithm_seed(info.at("master_seed").get<uint32_t>(),
                    "EXP1_user_num", key, id, method_name_list[m]);
                if (entry.second.seed != seed || original->second.at("seeds").at(m).get<uint32_t>() != seed)
                    throw runtime_error("Legacy seed mismatch: " + key + "/" + method_name_list[m]);
                ++reusable_rows;
                const auto prior = existing.at(c).at(m).find(id);
                if (prior != existing.at(c).at(m).end()) {
                    if (run_result_to_csv(prior->second) != run_result_to_csv(entry.second))
                        throw runtime_error("Reuse conflict: " + key + "/" + method_name_list[m] + "/" + to_string(id));
                } else {
                    pending[c][m].emplace(id, entry.second);
                }
            }
        }
    }
    return pending;
}

/// Run one condition in instance/method order, skipping independent checkpoints and saving every completed method.
/// On model/solver/output errors, throw a contextual message immediately; no failure row or automatic retry is produced.
inline void run_Instance_with_checkpoint(const fs::path& directory, const string& experiment,
    const ExperimentCondition& condition, const string& config, const ExperimentRunOptions& options,
    ConditionRecords& records) {
    fs::create_directories(directory);
    for (int i = 0; i < options.instance_count; ++i) {
        const int id = allocation_instance_id(condition.users[i]);
        const string context = experiment + " / " + condition.key + " / ID " + to_string(id);
        bool completed = true;
        for (const auto& method : records) completed = completed && method.count(id) != 0;
        if (completed) {
            cout << "[SKIP] " << context << " (all methods saved)" << std::endl;
            continue;
        }
        try {
            SystemMd model(condition.users[i], condition.uavs[i], config);
            for (auto& uav : model.uavs) uav.total_bandwidth = condition.bandwidth_mhz * unit_para;
            model.init_SystemModel(); // Rebuild channels/noise after the requested bandwidth override.
            validate_allocation_model(model); // Once per instance, outside all algorithm timers.
            for (size_t m = 0; m < method_name_list.size(); ++m) {
                const string call = context + " / " + method_name_list[m];
                if (records[m].count(id)) {
                    cout << "[SKIP] " << call << std::endl;
                    continue;
                }
                cout << "[RUN " << (i + 1) << "/" << options.instance_count << "] " << call << std::endl;
                const uint32_t seed = derive_algorithm_seed(options.master_seed, experiment,
                    condition.key, id, method_name_list[m]);
                auto result = run_algorithm(model, m, options, seed);
                result.instance_id = id;
                if (!algorithm_status_valid(result.status)) {
                    const string reason = result.diagnostics.events.empty() ? "No diagnostic" : result.diagnostics.events.back();
                    throw runtime_error(method_name_list[m] + " / " + algorithm_status_name(result.status) + ": " + reason);
                }
                try { appendResult(directory.string(), m, result); }
                catch (const std::exception& error) { throw runtime_error(method_name_list[m] + ": " + error.what()); }
                cout << "[SAVED] " << call << " / " << result.duration_ms << " ms / "
                    << algorithm_status_name(result.status) << std::endl;
                result.allocation = AllocationPair{}; // Keep small metrics/checkpoints, not all allocation maps.
                records[m].emplace(id, std::move(result));
            }
        } catch (const std::exception& error) {
            throw runtime_error(context + ": " + error.what());
        }
    }
}

/// Inspect all destinations/imports first, then reuse and compute missing rows before publishing complete summaries.
/// Only this explicit experiment call creates output; EXP2--EXP4 ignore the optional EXP1 reuse source.
inline void execute_experiment(const string& experiment, const vector<ExperimentCondition>& conditions,
    const ExperimentRunOptions& options) {
    if (conditions.empty()) throw invalid_argument("No experiment conditions selected");
    const fs::path base = fs::path(experimentDataPath) / "ExperimentsResults" / experiment;
    const string config = (base / "def_config.json").string();
    const fs::path root = base / TON_RESULT_VERSION_DIR / options.output_name;
    json info = inspect_experiment_destination(root, make_experiment_run_info(experiment, conditions, config, options));
    vector<ConditionRecords> checkpoints;
    for (const auto& condition : info.at("conditions")) {
        const string key = condition.at("key").get<string>();
        checkpoints.push_back(read_condition_records(root / key, experiment, key, condition.at("inputs"), options));
    }
    size_t reusable = 0, to_import = 0, saved = 0;
    const auto imports = prepare_exp1_reuse(info, checkpoints, reusable);
    for (size_t c = 0; c < conditions.size(); ++c)
        for (size_t m = 0; m < method_name_list.size(); ++m) {
            saved += checkpoints[c][m].size();
            to_import += imports[c][m].size();
        }
    const size_t total = conditions.size() * static_cast<size_t>(options.instance_count) * method_name_list.size();
    const size_t remaining = total - saved - to_import;
    cout << "\n" << experiment << " -> " << root.string() << "\n"
        << "Saved: " << saved << "; reusable source rows: " << reusable
        << "; importing now: " << to_import << "; algorithm calls remaining: " << remaining << std::endl;
    if (remaining && info.at("summary").at("instance_count").get<int>() > 0)
        info["summary"]["status"] = "stale";
    if (info.at("summary").at("instance_count").get<int>() > 0 &&
        (info.at("summary").at("instance_count") != options.instance_count ||
            info.at("summary").at("status") != "complete"))
        cout << "NOTE: existing summary represents N=" << info.at("summary").at("instance_count")
             << ", target N=" << options.instance_count << "; it is not the current complete summary." << std::endl;
    // No destination is touched until all selected checkpoints and reuse conflicts have been inspected.
    fs::create_directories(root);
    write_experiment_run_info(root, info);
    for (size_t c = 0; c < conditions.size(); ++c) {
        const fs::path directory = root / conditions[c].key;
        fs::create_directories(directory);
        for (size_t m = 0; m < method_name_list.size(); ++m)
            for (const auto& entry : imports[c][m]) {
                appendResult(directory.string(), m, entry.second);
                checkpoints[c][m].emplace(entry.first, entry.second);
            }
    }
    if (to_import) cout << "Imported " << to_import << " legacy rows; original timing/metrics retained." << std::endl;
    vector<string> keys;
    vector<vector<EXPResult>> means;
    for (size_t c = 0; c < conditions.size(); ++c) {
        run_Instance_with_checkpoint(root / conditions[c].key, experiment, conditions[c], config, options, checkpoints[c]);
        keys.push_back(conditions[c].key);
        vector<EXPResult> condition_means;
        for (const auto& records : checkpoints[c])
            condition_means.push_back(average_valid_attempts(records, options.instance_count));
        means.push_back(std::move(condition_means));
    }
    info["summary"]["status"] = "writing";
    write_experiment_run_info(root, info);
    exportAllSummaryCSV((root / "summary").string(), keys, means);
    info["summary"] = {{"status", "complete"}, {"instance_count", options.instance_count}};
    write_experiment_run_info(root, info);
    cout << experiment << " complete: " << total << " saved results; 12 summary CSVs, N="
         << options.instance_count << "." << std::endl;
}


// 新五实验复用旧文件名约定，但用户数/UAV数显式保存，用于实例及逐用户结果的基本检查。
struct MultiHardCondition {
    ExperimentCondition input;
    int user_count = 0, uav_count = 0;
    int hardfirst_candidates = HARD_FIRST_CANDIDATE_COUNT; // 其他实验不覆盖默认8。
    int budget_hard_count = -1, budget_physical_slots = -1; // 仅EXP3预检后填写，未配置时禁止原位替换。
};

inline const string EXP3_HARDFIRST_BUDGET_POLICY = "exp3-h2maxhs-budget-v1";
inline const string EXP3_BUDGET_ALGORITHM_VERSION = SIMPLE_ALGORITHM_VERSION + "-" + EXP3_HARDFIRST_BUDGET_POLICY;

/// 以H0=floor(N/5)、C=H^2*max(H,S)计算ceil(8*C0/C)，截到1--8；H或S为0返回1。
/// 参数为实际hard数、总用户数和物理槽数；负值/H>N抛错，128位乘积覆盖所有非负int参数。
inline int exp3_hardfirst_candidate_count(int hard_count, int user_count, int slots) {
    if (hard_count < 0 || user_count < 0 || slots < 0 || hard_count > user_count)
        throw invalid_argument("Invalid EXP3 HardFirst budget dimensions");
    if (hard_count == 0 || slots == 0) return 1;
    using boost::multiprecision::uint128_t;
    const int reference = user_count / 5;
    const uint128_t budget = uint128_t(HARD_FIRST_CANDIDATE_COUNT) * reference * reference * std::max(reference, slots);
    const uint128_t cost = uint128_t(hard_count) * hard_count * std::max(hard_count, slots);
    for (int count = 1; count < HARD_FIRST_CANDIDATE_COUNT; ++count)
        if (uint128_t(count) * cost >= budget) return count;
    return HARD_FIRST_CANDIDATE_COUNT;
}

/// 将EXP3整数条件键0/2/4/6/8/10转换成floor(N*alpha)；不接受自由浮点值或未知比例。
inline int exp3_expected_hard_count(const string& key, int user_count) {
    const vector<string> keys = {"0", "2", "4", "6", "8", "10"};
    if (user_count < 0 || std::find(keys.begin(), keys.end(), key) == keys.end())
        throw invalid_argument("Invalid EXP3 hard-ratio condition");
    return static_cast<int>(static_cast<int64_t>(user_count) * stoi(key) / 10);
}

/// 只读核对EXP3模型的用户/UAV数、hard比例、统一预算与有效槽数，返回应使用的候选数。
/// configured=true时还要求与预检记录一致，禁止某实例静默切换候选预算。
inline int checked_exp3_model_budget(const SystemMd& model, const MultiHardCondition& condition,
    bool configured = true) {
    if (model.users.size() != static_cast<size_t>(condition.user_count) ||
        model.uavs.size() != static_cast<size_t>(condition.uav_count) ||
        model.n1 != exp3_expected_hard_count(condition.input.key, condition.user_count) ||
        std::count_if(model.users.begin(), model.users.end(), [](const User& user) {
            return user.uType == HARD_UTILITY; // 核对缓存n1与实际类型，返回是否属于hard集合。
        }) != model.n1)
        throw runtime_error("EXP3 user/UAV count or hard ratio differs from its condition");
    for (const auto& uav : model.uavs)
        if (!allocation_near(uav.total_bandwidth, condition.input.bandwidth_mhz * unit_para))
            throw runtime_error("EXP3 UAV bandwidth differs from its condition");
    const int slots = hard_first_physical_slot_count(model);
    const int count = exp3_hardfirst_candidate_count(model.n1, condition.user_count, slots);
    if (configured && (condition.budget_hard_count != model.n1 ||
        condition.budget_physical_slots != slots || condition.hardfirst_candidates != count))
        throw runtime_error("EXP3 instance differs from the recorded candidate budget");
    return count;
}

/// 仅EXP3调用：读取每条件首实例确定物理槽预算；无文件写入，后续求解逐实例再次核对。
inline void configure_exp3_budgets(vector<MultiHardCondition>& conditions,
    const fs::path& physical, const fs::path& profiles) {
    for (auto& condition : conditions) {
        const SystemMd model(condition.input.users.at(0), condition.input.uavs.at(0),
            physical.string(), profiles.string());
        validate_allocation_model(model);
        condition.hardfirst_candidates = checked_exp3_model_budget(model, condition, false);
        condition.budget_hard_count = model.n1;
        condition.budget_physical_slots = hard_first_physical_slot_count(model);
    }
}

/// 返回EXP3预算元数据；各条件保存实际H/N/S和候选数，旧固定8策略不增加任何字段。
inline json exp3_candidate_budget_policy(const vector<MultiHardCondition>& conditions) {
    json policy = {{"policy", EXP3_HARDFIRST_BUDGET_POLICY}, {"reference_alpha", 0.2},
        {"reference_candidate_count", HARD_FIRST_CANDIDATE_COUNT}, {"minimum", 1}, {"maximum", 8},
        {"formula", "H0=floor(N/5); H=0 or S=0:1; otherwise clamp(ceil(8*H0^2*max(H0,S)/(H^2*max(H,S))),1,8)"},
        {"replacement_failure", "preserve_old_record_and_stop"}, {"conditions", json::object()}};
    for (const auto& condition : conditions) {
        if (condition.budget_hard_count != exp3_expected_hard_count(condition.input.key, condition.user_count) ||
            condition.hardfirst_candidates != exp3_hardfirst_candidate_count(
                condition.budget_hard_count, condition.user_count, condition.budget_physical_slots))
            throw runtime_error("EXP3 candidate budget was not configured");
        policy["conditions"][condition.input.key] = {{"user_count", condition.user_count},
            {"hard_count", condition.budget_hard_count}, {"physical_slots", condition.budget_physical_slots},
            {"candidate_count", condition.hardfirst_candidates}};
    }
    return policy;
}

/// 仅标注EXP3的新策略；其他实验直接返回，固定8版本和hard_first_policy保持逐字段兼容。
inline void apply_exp3_budget_metadata(json& info, const vector<MultiHardCondition>& conditions) {
    if (info.at("experiment") != "EXP3_hard_ratio") return;
    info["algorithm_version"] = EXP3_BUDGET_ALGORITHM_VERSION;
    info["hard_first_policy"]["mechanism"] = EXP3_HARDFIRST_BUDGET_POLICY;
    info["hard_first_policy"]["candidate_count"] = "per_condition";
    info["hard_first_candidate_budget"] = exp3_candidate_budget_policy(conditions);
}

/// 普通EXP3批处理只接受本条件预算的终止记录；alpha=0可复用旧请求8/实际完成1的退化记录。
/// 未带新标记的失败仅在预算未改变时兼容；旧预算行明确报错，重算由main.cpp临时入口负责。
inline void check_exp3_hardfirst_record(const AlgorithmRunResult& run, const MultiHardCondition& condition) {
    if (condition.budget_hard_count < 0 || condition.budget_physical_slots < 0)
        throw runtime_error("EXP3 candidate budget was not configured");
    const int expected = condition.hardfirst_candidates;
    if (!algorithm_status_valid(run.status)) {
        for (const auto& event : run.diagnostics.events) {
            const json tag = json::parse(event, nullptr, false);
            if (tag.is_object() && tag.value("kind", string{}) == "exp3_hardfirst_attempt") {
                if (tag.at("policy") != EXP3_HARDFIRST_BUDGET_POLICY ||
                    tag.at("candidate_count_requested") != expected)
                    throw runtime_error("EXP3 failed attempt has a different candidate budget");
                return;
            }
        }
        if (condition.budget_hard_count != 0 && expected != HARD_FIRST_CANDIDATE_COUNT)
            throw runtime_error("Legacy EXP3 failed attempt requires the temporary resume entry in main.cpp");
        return;
    }
    int requested = expected;
    if (condition.budget_hard_count == 0 && run.diagnostics.events.size() == 1 &&
        json::parse(run.diagnostics.events.front()).at("candidate_count_requested") == HARD_FIRST_CANDIDATE_COUNT)
        requested = HARD_FIRST_CANDIDATE_COUNT;
    const json diag = checked_hardfirst_diagnostics(run, requested);
    if (diag.at("hard_hungarian_rows") != condition.budget_hard_count ||
        diag.at("hard_hungarian_columns") != std::max(condition.budget_hard_count, condition.budget_physical_slots))
        throw runtime_error("EXP3 checkpoint matching dimensions differ from its condition");
}

/// 根据EXP1--EXP5的实验网格和生成元数据，构造条件列表及配对的用户、无人机输入路径。
/// experiment为五组实验之一的内部标识；input_root是数据根目录，generation提供各实例地区代码，bandwidth单位为MHz。
/// options控制实例数和条件筛选；返回筛选后的MultiHardCondition列表，路径按实例ID生成，不因缺失文件改换ID。
inline vector<MultiHardCondition> multi_hard_conditions(const string& experiment,
    const fs::path& input_root, const json& generation, double bandwidth,
    const ExperimentRunOptions& options) {
    // EXP1/EXP4改变用户数，EXP2/EXP5改变无人机数；EXP3改变Hard用户比例。
    const bool user_axis = experiment == "EXP1_user_num" || experiment == "EXP4_real_user_num";
    const bool uav_axis = experiment == "EXP2_uav_num" || experiment == "EXP5_real_uav_num";

    // 只接受五组批处理支持的实验键，其他名称无法确定对应的条件网格。
    if (!user_axis && !uav_axis && experiment != "EXP3_hard_ratio")
        throw invalid_argument("Unknown multi-hard experiment");

    // 为当前变化轴定义全部标准条件：用户数、无人机数或Hard比例网格。
    const vector<string> defaults = user_axis ? vector<string>{"1000","2000","3000","4000","5000"} :
        uav_axis ? vector<string>{"5","10","15","20"} : vector<string>{"0","2","4","6","8","10"};
    vector<MultiHardCondition> conditions;

    // 空筛选使用完整标准网格；非空筛选由experiment_condition_keys校验后仅构造指定条件。
    for (const string& key : experiment_condition_keys(options, defaults)) {
        MultiHardCondition item;

        // 用户数实验从key读取用户数；无人机数实验从key读取UAV数；其余维度固定为3000用户、10架UAV。
        item.user_count = user_axis ? stoi(key) : 3000;
        item.uav_count = uav_axis ? stoi(key) : 10;

        // 将条件键和每架UAV的统一带宽写入条件输入描述，供模型构造与结果元数据使用。
        item.input.key = key; item.input.bandwidth_mhz = bandwidth;

        // 按实例ID读取对应replicate的地区代码，并用相同ID/代码配对用户文件和无人机部署文件。
        for (int id = 1; id <= options.instance_count; ++id) {
            const string code = generation.at("replicates").at(id-1).at("adcode").get<string>();

            // 文件名中的地区代码必须为非空数字串，拒绝目录分隔符等非法字符。
            if (code.empty() || code.find_first_not_of("0123456789") != string::npos)
                throw invalid_argument("Invalid source county code");

            // 按固定目录结构生成输入路径；这里不检查文件是否存在，也不搜索其他实例替代缺失ID。
            const fs::path base = input_root / experiment / key;
            item.input.users.push_back((base / "user_data" /
                (to_string(id)+"_"+to_string(item.user_count)+"users_data_"+code+".csv")).string());
            item.input.uavs.push_back((base / "uav_data" /
                (to_string(id)+"_"+to_string(item.uav_count)+"uavs_loc_"+code+".csv")).string());
        }

        // 保存当前条件；用户和UAV路径在相同实例ID位置一一对应。
        conditions.push_back(std::move(item));
    }

    // 返回所有筛选条件及其用户/UAV文件路径，后续预检和批处理负责读取、验证文件。
    return conditions;
}

inline const string MULTI_USER_RESULT_HEADER = "instance_id,user_id,uav_id,bandwidth,level,utility";

/// 成功实例逐用户文件路径；算法/条件由目录表示，文件名稳定绑定实例编号。
inline fs::path multi_user_result_path(const fs::path& directory, size_t method, int id) {
    return directory / "users" / method_name_list.at(method) / (to_string(id)+".csv");
}

/// 检查逐用户文件结构和汇总配套；普通批处理总是比较指标，临时入口可显式跳过旧指标比较但不跳过结构。
inline void check_multi_user_result(const fs::path& path, const AlgorithmRunResult& run, int expected_users,
    int expected_uavs, bool allow_summary_mismatch = false) {
    ifstream input(path, ios::binary);
    string line;
    if (!input || !getline(input,line)) throw runtime_error("Missing user result: "+path.string());
    if (!line.empty() && line.back()=='\r') line.pop_back();
    if (line != MULTI_USER_RESULT_HEADER) throw runtime_error("Wrong user-result header: "+path.string());
    set<string> ids;
    double total = 0;
    int served = 0;
    while (getline(input,line)) {
        if (input.eof()) throw runtime_error("Truncated user result: "+path.string());
        const auto fields = csv_fields(line);
        if (fields.size()!=6 || parse_finite_number(fields[0])!=run.instance_id ||
            fields[1].empty() || !ids.insert(fields[1]).second)
            throw runtime_error("Invalid user-result identity: "+path.string());
        const double bw = parse_finite_number(fields[3]), value = parse_finite_number(fields[5]);
        if (fields[2]=="-1") {
            if (bw!=0 || value!=0 || (!fields[4].empty() && fields[4]!="0"))
                throw runtime_error("Nonzero unserved result: "+path.string());
        } else {
            const double k = parse_finite_number(fields[2]);
            if (k!=floor(k) || k>=expected_uavs || bw<=0 || value<=0) throw runtime_error("Invalid served user result");
            if (!fields[4].empty()) {
                const double level = parse_finite_number(fields[4]);
                if (level<1 || level!=floor(level)) throw runtime_error("Invalid hard level in user result");
            }
            ++served;
        }
        total += value;
    }
    if (input.bad() || ids.size()!=static_cast<size_t>(expected_users) ||
        (!allow_summary_mismatch && (served!=run.metrics.total_num || !allocation_near(total,run.metrics.total_utility))))
        throw runtime_error("User result and summary disagree: "+path.string());
}

/// 普通批处理按已尝试ID读取六方法记录，严格检查种子、逐用户文件及孤立文件；不迁移或替换旧行。
inline ConditionRecords read_multi_records(const fs::path& directory, const string& experiment,
    const MultiHardCondition& condition, const ExperimentRunOptions& options) {
    ConditionRecords records(method_name_list.size());
    for (size_t m=0; m<method_name_list.size(); ++m) {
        records[m] = read_result_csv(directory/(method_name_list[m]+".csv"),false,true);
        for (const auto& entry : records[m]) {
            const int id=entry.first;
            if (id<1 || id>options.instance_count || entry.second.seed !=
                derive_algorithm_seed(options.master_seed,experiment,condition.input.key,id,method_name_list[m]))
                throw runtime_error("Multi-hard checkpoint ID/seed mismatch");
            const auto detail=multi_user_result_path(directory,m,id);
            if (algorithm_status_valid(entry.second.status))
                check_multi_user_result(detail,entry.second,condition.user_count,condition.uav_count);
            else if (fs::exists(detail)) throw runtime_error("Failed attempt has user result: "+detail.string());
        }
        const fs::path user_dir=directory/"users"/method_name_list[m];
        if (fs::exists(user_dir))
            for (const auto& file:fs::directory_iterator(user_dir)) {
                if (!file.is_regular_file() || file.path().extension()!=".csv")
                    throw runtime_error("Unexpected user-result entry: "+file.path().string());
                const double raw=parse_finite_number(file.path().stem().string());
                if (raw<1 || raw>INT_MAX || raw!=floor(raw) ||
                    !records[m].count(static_cast<int>(raw)) ||
                    file.path().filename()!=fs::path(to_string(static_cast<int>(raw))+".csv"))
                    throw runtime_error("Orphan user result: "+file.path().string());
            }
    }
    return records;
}

/// 保存全部源用户结果；仅创建新文件，带宽为内部kHz，hard等级按原始完整实例判定。
inline void write_multi_user_result(const fs::path& path, const SystemMd& model, const AlgorithmRunResult& run) {
    if (fs::exists(path)) throw runtime_error("Refusing to overwrite user result: "+path.string());
    fs::create_directories(path.parent_path());
    ofstream output(path,ios::binary);
    output << setprecision(std::numeric_limits<double>::max_digits10) << MULTI_USER_RESULT_HEADER << '\n';
    for (const User& user:model.users) {
        const auto& result=run.allocation.second.at(user.ID);
        output << run.instance_id << ',' << csv_quote(user.source_user_id) << ',' << result.uav_id << ','
               << result.allocated_bandwidth << ',';
        if (user.uType==HARD_UTILITY)
            output << (result.uav_id<0 ? 0 : user.achieved_hard_level(result.allocated_bandwidth,model.cap_list[result.uav_id][user.ID]));
        output << ',' << result.utility << '\n';
    }
    output.flush(); output.close();
    if (!output) throw runtime_error("User result write failed: "+path.string());
}

/// 处理单个实验条件：每个实例只构造一次模型，只运行尚无记录的方法，并将尝试结果写入检查点。
/// directory是条件结果目录；experiment和condition标识实验及输入集合；physical/profiles提供物理与应用配置。
/// options提供实例规模、条件和算法参数；records按方法保存已有行并在本函数内更新。无返回值，输入构造失败会记录，其他校验/写入异常向上抛出。
inline void run_multi_condition(const fs::path& directory, const string& experiment,
    const MultiHardCondition& condition, const fs::path& physical, const fs::path& profiles,
    const ExperimentRunOptions& options, ConditionRecords& records) {
    // 确保当前条件的结果目录存在；已有检查点由调用方预检后传入records。
    fs::create_directories(directory);

    // 按实例ID从1到instance_count处理，数组下标index对应实例ID减1。
    for (int index=0; index<options.instance_count; ++index) {
        const int id=index+1;

        // 只有六种方法都已有该ID记录时才跳过整个实例，避免重复构造输入模型。
        bool complete=true;
        for (const auto& rows:records) complete=complete && rows.count(id)!=0;
        if (complete) continue;

        // 当前ID的所有缺失方法共用同一个模型；先构造并验证输入，再开始算法调用。
        std::unique_ptr<SystemMd> model;
        string input_error;
        try {
            model=std::make_unique<SystemMd>(condition.input.users[index],condition.input.uavs[index],
                physical.string(),profiles.string());

            // 核对实例规模及每架UAV的物理带宽与条件配置一致，并验证模型矩阵和用户/UAV ID。
            if (model->users.size()!=static_cast<size_t>(condition.user_count) ||
                model->uavs.size()!=static_cast<size_t>(condition.uav_count))
                throw runtime_error("Unexpected user/UAV count");
            for (const auto& uav:model->uavs)
                if (!allocation_near(uav.total_bandwidth,condition.input.bandwidth_mhz*unit_para))
                    throw runtime_error("UAV bandwidth differs from common configuration");
            validate_allocation_model(*model);
        } catch (const std::exception& error) {
            // 输入读取或模型校验失败时保存错误原因；该ID的缺失方法会登记失败状态，不调用分配算法。
            input_error=string("input_error: ")+error.what(); model.reset();
        }

        // EXP3使用已验证模型计算本条件的HardFirst候选数；其他实验使用统一默认数。
        const int candidates = experiment == "EXP3_hard_ratio" && model
            ? checked_exp3_model_budget(*model, condition) : HARD_FIRST_CANDIDATE_COUNT;

        // 逐方法补齐缺失记录；records中已有的成功或失败行都跳过，不自动重试。
        for (size_t m=0; m<method_name_list.size(); ++m) {
            if (records[m].count(id)) continue; // 成功和失败均不自动重试。

            // 种子由主种子、实验、条件、实例ID和方法名共同派生，保证每个调用可稳定复现。
            const uint32_t seed=derive_algorithm_seed(options.master_seed,experiment,condition.input.key,id,method_name_list[m]);
            AlgorithmRunResult run;

            // 模型可用时执行对应方法；否则将前面的输入错误转成失败记录，运行时间保持默认值。
            if (model) run=run_algorithm(*model,m,options,seed,candidates);
            else {
                run.status=AlgorithmRunStatus::Exception; run.seed=seed;
                run.diagnostics.events.push_back(input_error); // 没有调用算法，duration为默认0。
            }
            run.instance_id=id;

            // EXP3的HardFirst（方法索引4）额外核验动态候选预算；失败时也保留策略和请求预算诊断。
            if (experiment == "EXP3_hard_ratio" && m == 4) {
                if (algorithm_status_valid(run.status)) check_exp3_hardfirst_record(run, condition);
                else run.diagnostics.events.push_back(json{{"kind", "exp3_hardfirst_attempt"},
                    {"policy", EXP3_HARDFIRST_BUDGET_POLICY},
                    {"candidate_count_requested", condition.hardfirst_candidates}}.dump());
            }

            // 成功结果先写逐用户明细，再追加方法CSV；appendResult允许保存失败状态行。
            if (algorithm_status_valid(run.status))
                write_multi_user_result(multi_user_result_path(directory,m,id),*model,run);
            appendResult(directory.string(),m,run,true);

            // 输出本次状态并清空不再需要的分配内容；保留汇总所需字段，移入内存中的结果记录。
            cout << experiment << '/' << condition.input.key << "/ID" << id << '/'
                 << method_name_list[m] << ": " << algorithm_status_name(run.status) << std::endl;
            run.allocation=AllocationPair{};
            records[m].emplace(id,std::move(run));
        }
    }
}

/// 汇总前核对EXP3所有条件/方法/ID均已尝试且不存在待替换旧预算行；失败仍通过成功率单独报告。
inline void check_exp3_budget_completion(const vector<MultiHardCondition>& conditions,
    const vector<ConditionRecords>& records, int instance_count) {
    if (conditions.size() != records.size()) throw runtime_error("EXP3 summary condition count mismatch");
    for (size_t c = 0; c < conditions.size(); ++c) {
        if (records[c].size() != method_name_list.size()) throw runtime_error("EXP3 summary method count mismatch");
        for (const auto& method : records[c]) {
            if (method.size() != static_cast<size_t>(instance_count)) throw runtime_error("EXP3 summary is incomplete");
            for (int id = 1; id <= instance_count; ++id)
                if (!method.count(id)) throw runtime_error("EXP3 summary has a missing instance ID");
        }
        for (const auto& row : records[c][4]) check_exp3_hardfirst_record(row.second, conditions[c]);
    }
}

/// 输出原12张宽表和独立成功率表；无成功记录时写空均值，绝不把失败值当零样本。
inline void export_multi_summary(const fs::path& directory, const vector<MultiHardCondition>& conditions,
    const vector<ConditionRecords>& all_records) {
    const std::array<string,12> names={"Run_time_ms","Total_Num","Hard_Num","Elastic_Num","Total_Utility",
        "Hard_Utility","Elastic_Utility","Hard_Bandwidth","Elastic_Bandwidth","Hard_Throughput","Elastic_Throughput","Total_Throughput"};
    fs::create_directories(directory);
    ofstream rates(directory/"Success_Rate.csv",ios::binary);
    rates << "condition,method,success_count,attempt_count,success_rate\n" << setprecision(17);
    vector<vector<EXPResult>> means(conditions.size(),vector<EXPResult>(method_name_list.size()));
    vector<vector<size_t>> counts(conditions.size(),vector<size_t>(method_name_list.size(),0));
    for(size_t c=0;c<conditions.size();++c)
        for(size_t m=0;m<method_name_list.size();++m) {
            const auto& records=all_records[c][m];
            for(const auto& entry:records)
                if(algorithm_status_valid(entry.second.status)) {
                    ++counts[c][m];
                    for(auto member:EXP_METRICS) means[c][m].*member += entry.second.metrics.*member;
                }
            if(counts[c][m]) for(auto member:EXP_METRICS) means[c][m].*member /= static_cast<double>(counts[c][m]);
            rates << conditions[c].input.key << ',' << method_name_list[m] << ',' << counts[c][m] << ',' << records.size() << ',';
            if(!records.empty()) rates << static_cast<double>(counts[c][m])/records.size();
            rates << '\n';
        }
    rates.flush(); rates.close();
    if(!rates) throw runtime_error("Success-rate summary write failed");
    for(size_t metric=0;metric<names.size();++metric) {
        ofstream output(directory/(names[metric]+".csv"),ios::binary);
        output << setprecision(17) << "User_Scale";
        for(const auto& method:method_name_list) output << ',' << method;
        output << '\n';
        for(size_t c=0;c<conditions.size();++c) {
            output << conditions[c].input.key;
            for(size_t m=0;m<method_name_list.size();++m) {
                output << ',';
                if(counts[c][m]) output << means[c][m].*EXP_METRICS[metric];
            }
            output << '\n';
        }
        output.flush(); output.close();
        if(!output) throw runtime_error("Metric summary write failed");
    }
}

/// 构造五实验批处理元数据；普通入口与main.cpp临时预检共用，避免版本/输入字段发生偏差。
/// conditions已配置候选数；除EXP3预算字段外，保持固定8版本的原始字段和值。
inline json make_multi_hard_run_info(const string& experiment, const vector<MultiHardCondition>& conditions,
    const fs::path& physical, const fs::path& profiles, const json& generation, const ExperimentRunOptions& options) {
    vector<ExperimentCondition> base_conditions;
    for(const auto& condition:conditions) base_conditions.push_back(condition.input);
    json expected=make_experiment_run_info(experiment,base_conditions,physical.string(),options);
    expected["schema"]="ton-multihard-run-v1";
    expected["input_schema"]=generation.at("input_schema");
    expected["input_generation"]=generation;
    expected["application_profiles"]=read_experiment_json(profiles);
    expected["application_profiles_path"]=experiment_absolute_path(profiles);
    expected["user_result_bandwidth_unit"]="kHz";
    expected["failure_policy"]="record_and_continue_no_automatic_retry";
    apply_exp3_budget_metadata(expected, conditions);
    return expected;
}

/// 执行五组多等级hard实验的普通批处理：严格预检输入和结果版本，只补缺失调用并生成汇总。
/// experiment为EXP1--EXP5之一的内部标识；options提供输入目录、实例数、条件筛选、输出名称和随机种子等参数。
/// 返回void；配置、版本、校验或写入错误向上抛出，单实例算法失败按批处理规则记录状态并继续。
inline void execute_multi_hard_experiment(const string& experiment, const ExperimentRunOptions& options) {
    // 先校验运行参数并解析指定输入根目录；输入路径错误时立即停止，不回退到其他目录。
    validate_run_options(options);
    const fs::path input_root=resolve_experiment_input_root(options);

    // 读取生成器元数据并确认输入属于当前多等级hard格式，防止旧格式实例进入本批次。
    const json generation=read_experiment_json(input_root/"generation_config_ToN.json");
    if(generation.value("input_schema",string{})!="ton-multihard-input-v1")
        throw runtime_error("New experiments require multi-hard input schema");

    // 读取统一物理带宽配置；physical和profiles还用于模型构造及运行元数据记录。
    const fs::path physical=input_root/"physical_config.json", profiles=input_root/"application_profiles.json";
    const json physical_json=read_experiment_json(physical);
    const double bandwidth=physical_json.at("uav_bandwidth_mhz").get<double>();
    if(!std::isfinite(bandwidth) || bandwidth<=0) throw runtime_error("Invalid common UAV bandwidth");

    // 根据实验类型、生成配置和options筛出本次条件；EXP3额外为各条件配置HardFirst候选预算。
    auto conditions=multi_hard_conditions(experiment,input_root,generation,bandwidth,options);
    if (experiment == "EXP3_hard_ratio") configure_exp3_budgets(conditions, physical, profiles);

    // 用本次输入和算法策略构造预期run_info，并选定独立的实验结果目录。
    const json expected=make_multi_hard_run_info(experiment,conditions,physical,profiles,generation,options);
    const fs::path root=fs::path(experimentDataPath)/"ExperimentsResults"/"ToN_multiHard"/options.output_name/experiment;
    // 只读检查已有结果是否与预期版本兼容；无法安全续写时在写入前报错。
    json info = inspect_experiment_destination(root,expected);

    // 每项条件保留一份六方法检查点记录，用于续跑和最终汇总。
    vector<ConditionRecords> records;
    // 先检查所有条件的已有记录、种子及逐用户结果文件；缺少源实例时不改用其他编号。
    // EXP3还会确认已保存的HardFirst结果使用本条件要求的候选预算。全部检查通过前不写目标文件。
    for(const auto& condition:conditions) {
        records.push_back(read_multi_records(root/condition.input.key,experiment,condition,options));
        if (experiment == "EXP3_hard_ratio")
            for (const auto& row : records.back()[4]) check_exp3_hardfirst_record(row.second, condition);
    }

    // 全部预检通过后才创建结果根目录，并先将汇总状态标记为stale，表示本轮尚未完成。
    fs::create_directories(root);
    info["summary"]["status"]="stale";
    write_experiment_run_info(root,info);

    // 逐条件补跑尚无记录的实例；已有成功或失败记录均由run_multi_condition跳过、不自动重试。
    for(size_t c=0;c<conditions.size();++c)
        run_multi_condition(root/conditions[c].input.key,experiment,conditions[c],physical,profiles,options,records[c]);

    // EXP3写汇总前确认所有条件、方法和实例均已记录，且HardFirst动态预算核对通过。
    if (experiment == "EXP3_hard_ratio") check_exp3_budget_completion(conditions,records,options.instance_count);

    // 先标记汇总正在写入，再导出各指标CSV，避免中断时旧汇总仍被误认为当前完整结果。
    info["summary"]["status"]="writing";
    write_experiment_run_info(root,info);
    export_multi_summary(root/"summary",conditions,records);

    // attempt_count统计所有已记录的尝试；success_count只统计状态有效的记录，失败不会被当作成功。
    size_t attempts=0,successes=0;
    for(const auto& condition:records) for(const auto& method:condition) for(const auto& row:method) {
        ++attempts; if(algorithm_status_valid(row.second.status)) ++successes;
    }

    // 汇总CSV写完后才发布complete状态和实例、尝试、成功数量。
    info["summary"]={{"status","complete"},{"instance_count",options.instance_count},
        {"attempt_count",attempts},{"success_count",successes}};
    write_experiment_run_info(root,info);
}

/// 主体用户数实验：每个规模精确20% hard，K=10。
inline void exp1_different_user_number(const ExperimentRunOptions& options) {
    execute_multi_hard_experiment("EXP1_user_num",options);
}

/// 主体UAV数实验：复用3000用户、20% hard，只改变部署数量。
inline void exp2_different_uav_number(const ExperimentRunOptions& options) {
    execute_multi_hard_experiment("EXP2_uav_num",options);
}

/// 主体比例实验：固定3000用户/10 UAV、嵌套hard集合；动态预算批处理只补缺失项，不覆盖旧结果。
inline void exp3_different_hard_user_ratio(const ExperimentRunOptions& options) {
    execute_multi_hard_experiment("EXP3_hard_ratio",options);
}

/// 真实请求用户数实验：按会话统计生成的请求池取嵌套前缀，K=10。
inline void exp4_real_requests_user_number(const ExperimentRunOptions& options) {
    execute_multi_hard_experiment("EXP4_real_user_num",options);
}

/// 真实请求UAV数实验：固定EXP4的3000用户请求，只改变部署数量。
inline void exp5_real_requests_uav_number(const ExperimentRunOptions& options) {
    execute_multi_hard_experiment("EXP5_real_uav_num",options);
}

/// EXP4: reuse options.input_root's 3000-user/10-UAV inputs and save/resume bandwidth conditions in MHz.
/// The runner rebuilds physical channels after overriding bandwidth and before timing algorithms.
inline void exp4_different_total_bandwidth(const ExperimentRunOptions& options) {
    validate_run_options(options);
    const fs::path input_root = resolve_experiment_input_root(options);
    vector<ExperimentCondition> conditions;
    const string input = (input_root / "variable_user_num" / "3000u_num").string();
    for (const auto& bandwidth : experiment_condition_keys(options, {"10", "20", "30", "40", "50"}))
        conditions.push_back(select_experiment_condition(bandwidth, input, input,
            "3000users_data", "10uavs_loc", stod(bandwidth), options));
    execute_experiment("EXP4_bandwidth", conditions, options);
}

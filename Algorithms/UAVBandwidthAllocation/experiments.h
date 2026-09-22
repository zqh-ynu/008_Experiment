#pragma once
// 实验入口：保留旧带宽/定位误差所需流程，新五实验读取multi-hard输入并按已尝试记录续跑。
// 新实验保存run_info、逐方法汇总与精简逐用户CSV；旧批次不写入、不覆盖。
#include "experiment_support.h"
#include <memory> // 新循环在输入构造失败后用空指针记录六种方法的失败，不重复构造实例。

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

/// Record numerical settings and selected paths once per experiment; no input contents or file hashes are copied.
inline json make_experiment_run_info(const string& experiment, const vector<ExperimentCondition>& conditions,
    const string& config, const ExperimentRunOptions& options) {
    load_global_channel_config(config);
    json info = {
        {"schema", SIMPLE_RUN_SCHEMA}, {"algorithm_version", SIMPLE_ALGORITHM_VERSION},
        {"proposed_algorithms", proposed_algorithm_policy()},
        {"experiment", experiment}, {"methods", method_name_list},
        {"hard_first_da_policy", hard_first_da_policy()},
        {"instance_count", options.instance_count}, {"master_seed", options.master_seed},
        {"rounding_trials", options.rounding_trials}, {"ton_epsilon", options.ton_epsilon},
        {"reallocate_residual", options.reallocate_residual}, // 现有完整参数比较会拒绝不同开关续写同批次。
        {"baseline_hard_policy", "highest_level_only"}, {"utility_evaluation", "original_model"},
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
    const std::array<size_t, 4> reusable_methods = {1, 2, 3, 5}; // Never import ApproBetter or AlgHungarian.
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
};

/// 按已确定的五组网格构造完整文件路径；不以文件存在与否替换/跳过实例编号。
inline vector<MultiHardCondition> multi_hard_conditions(const string& experiment,
    const fs::path& input_root, const json& generation, double bandwidth,
    const ExperimentRunOptions& options) {
    const bool user_axis = experiment == "EXP1_user_num" || experiment == "EXP4_real_user_num";
    const bool uav_axis = experiment == "EXP2_uav_num" || experiment == "EXP5_real_uav_num";
    if (!user_axis && !uav_axis && experiment != "EXP3_hard_ratio")
        throw invalid_argument("Unknown multi-hard experiment");
    const vector<string> defaults = user_axis ? vector<string>{"1000","2000","3000","4000","5000"} :
        uav_axis ? vector<string>{"5","10","15","20"} : vector<string>{"0","2","4","6","8","10"};
    vector<MultiHardCondition> conditions;
    for (const string& key : experiment_condition_keys(options, defaults)) {
        MultiHardCondition item;
        item.user_count = user_axis ? stoi(key) : 3000;
        item.uav_count = uav_axis ? stoi(key) : 10;
        item.input.key = key; item.input.bandwidth_mhz = bandwidth;
        for (int id = 1; id <= options.instance_count; ++id) {
            const string code = generation.at("replicates").at(id-1).at("adcode").get<string>();
            if (code.empty() || code.find_first_not_of("0123456789") != string::npos)
                throw invalid_argument("Invalid source county code");
            const fs::path base = input_root / experiment / key;
            item.input.users.push_back((base / "user_data" /
                (to_string(id)+"_"+to_string(item.user_count)+"users_data_"+code+".csv")).string());
            item.input.uavs.push_back((base / "uav_data" /
                (to_string(id)+"_"+to_string(item.uav_count)+"uavs_loc_"+code+".csv")).string());
        }
        conditions.push_back(std::move(item));
    }
    return conditions;
}

inline const string MULTI_USER_RESULT_HEADER = "instance_id,user_id,uav_id,bandwidth,level,utility";

/// 成功实例逐用户文件路径；算法/条件由目录表示，文件名稳定绑定实例编号。
inline fs::path multi_user_result_path(const fs::path& directory, size_t method, int id) {
    return directory / "users" / method_name_list.at(method) / (to_string(id)+".csv");
}

/// 只检查小型结果文件的表头、行数、唯一源ID及与汇总行的效用/服务数配套，不读取或哈希原输入。
inline void check_multi_user_result(const fs::path& path, const AlgorithmRunResult& run, int expected_users, int expected_uavs) {
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
        served!=run.metrics.total_num || !allocation_near(total,run.metrics.total_utility))
        throw runtime_error("User result and summary disagree: "+path.string());
}

/// 读取本条件终止记录；失败也算已尝试。逐用户文件缺失、孤立或与汇总矛盾时停止，不覆盖。
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

/// 处理一个条件：实例只构造一次，失败记录后继续；写入失败在捕获输入异常的范围之外抛出。
inline void run_multi_condition(const fs::path& directory, const string& experiment,
    const MultiHardCondition& condition, const fs::path& physical, const fs::path& profiles,
    const ExperimentRunOptions& options, ConditionRecords& records) {
    fs::create_directories(directory);
    for (int index=0; index<options.instance_count; ++index) {
        const int id=index+1;
        bool complete=true;
        for (const auto& rows:records) complete=complete && rows.count(id)!=0;
        if (complete) continue;
        std::unique_ptr<SystemMd> model;
        string input_error;
        try {
            model=std::make_unique<SystemMd>(condition.input.users[index],condition.input.uavs[index],
                physical.string(),profiles.string());
            if (model->users.size()!=static_cast<size_t>(condition.user_count) ||
                model->uavs.size()!=static_cast<size_t>(condition.uav_count))
                throw runtime_error("Unexpected user/UAV count");
            for (const auto& uav:model->uavs)
                if (!allocation_near(uav.total_bandwidth,condition.input.bandwidth_mhz*unit_para))
                    throw runtime_error("UAV bandwidth differs from common configuration");
            validate_allocation_model(*model);
        } catch (const std::exception& error) {
            input_error=string("input_error: ")+error.what(); model.reset();
        }
        for (size_t m=0; m<method_name_list.size(); ++m) {
            if (records[m].count(id)) continue; // 成功和失败均不自动重试。
            const uint32_t seed=derive_algorithm_seed(options.master_seed,experiment,condition.input.key,id,method_name_list[m]);
            AlgorithmRunResult run;
            if (model) run=run_algorithm(*model,m,options,seed);
            else {
                run.status=AlgorithmRunStatus::Exception; run.seed=seed;
                run.diagnostics.events.push_back(input_error); // 没有调用算法，duration为默认0。
            }
            run.instance_id=id;
            if (algorithm_status_valid(run.status))
                write_multi_user_result(multi_user_result_path(directory,m,id),*model,run);
            appendResult(directory.string(),m,run,true);
            cout << experiment << '/' << condition.input.key << "/ID" << id << '/'
                 << method_name_list[m] << ": " << algorithm_status_name(run.status) << std::endl;
            run.allocation=AllocationPair{};
            records[m].emplace(id,std::move(run));
        }
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

/// 新五实验共用执行器：只读预检后在新命名空间写结果，旧execute_experiment保持原行为。
inline void execute_multi_hard_experiment(const string& experiment, const ExperimentRunOptions& options) {
    validate_run_options(options);
    const fs::path input_root=resolve_experiment_input_root(options);
    const json generation=read_experiment_json(input_root/"generation_config_ToN.json");
    if(generation.value("input_schema",string{})!="ton-multihard-input-v1")
        throw runtime_error("New experiments require multi-hard input schema");
    const fs::path physical=input_root/"physical_config.json", profiles=input_root/"application_profiles.json";
    const json physical_json=read_experiment_json(physical), profile_json=read_experiment_json(profiles);
    const double bandwidth=physical_json.at("uav_bandwidth_mhz").get<double>();
    if(!std::isfinite(bandwidth) || bandwidth<=0) throw runtime_error("Invalid common UAV bandwidth");
    const auto conditions=multi_hard_conditions(experiment,input_root,generation,bandwidth,options);
    vector<ExperimentCondition> base_conditions;
    for(const auto& condition:conditions) base_conditions.push_back(condition.input);
    json expected=make_experiment_run_info(experiment,base_conditions,physical.string(),options);
    expected["schema"]="ton-multihard-run-v1";
    expected["input_schema"]=generation.at("input_schema");
    expected["input_generation"]=generation;
    expected["application_profiles"]=profile_json;
    expected["application_profiles_path"]=experiment_absolute_path(profiles);
    expected["user_result_bandwidth_unit"]="kHz";
    expected["failure_policy"]="record_and_continue_no_automatic_retry";
    const fs::path root=fs::path(experimentDataPath)/"ExperimentsResults"/"ToN_multiHard"/options.output_name/experiment;
    json info=inspect_experiment_destination(root,expected);
    vector<ConditionRecords> records;
    // 全条件预检完成前不写任何目标文件，缺少某个源实例不在此阶段替换为其他编号。
    for(const auto& condition:conditions)
        records.push_back(read_multi_records(root/condition.input.key,experiment,condition,options));
    fs::create_directories(root);
    info["summary"]["status"]="stale";
    write_experiment_run_info(root,info);
    for(size_t c=0;c<conditions.size();++c)
        run_multi_condition(root/conditions[c].input.key,experiment,conditions[c],physical,profiles,options,records[c]);
    info["summary"]["status"]="writing";
    write_experiment_run_info(root,info);
    export_multi_summary(root/"summary",conditions,records);
    size_t attempts=0,successes=0;
    for(const auto& condition:records) for(const auto& method:condition) for(const auto& row:method) {
        ++attempts; if(algorithm_status_valid(row.second.status)) ++successes;
    }
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

/// 主体比例实验：固定3000用户空间位置和10-UAV部署，hard集合嵌套扩大。
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

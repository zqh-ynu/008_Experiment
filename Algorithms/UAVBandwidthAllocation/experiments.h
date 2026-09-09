#pragma once
// Visual Studio experiment driver: select legacy/complete ToN inputs, append per-method results and resume.
// Only one run_info.json plus raw/summary CSVs are written; historical batches are always read-only.
#include "experiment_support.h"

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

/// EXP1: use options.input_root to vary user count at 10 UAVs/40 MHz; save/resume selected conditions, no return value.
inline void exp1_different_user_number(const ExperimentRunOptions& options) {
    validate_run_options(options);
    const fs::path input_root = resolve_experiment_input_root(options);
    vector<ExperimentCondition> conditions;
    for (const auto& count : experiment_condition_keys(options, {"1000", "2000", "3000", "4000", "5000"})) {
        const string input = (input_root / "variable_user_num" / (count + "u_num")).string();
        conditions.push_back(select_experiment_condition(count, input, input,
            count + "users_data", "10uavs_loc", 40, options));
    }
    execute_experiment("EXP1_user_num", conditions, options);
}

/// EXP2: use options.input_root to vary UAV count at 3000 users/40 MHz; save/resume results and ignore EXP1 reuse.
inline void exp2_different_uav_number(const ExperimentRunOptions& options) {
    validate_run_options(options);
    const fs::path input_root = resolve_experiment_input_root(options);
    vector<ExperimentCondition> conditions;
    for (const auto& count : experiment_condition_keys(options, {"5", "10", "15", "20"}))
        conditions.push_back(select_experiment_condition(count,
            (input_root / "variable_user_num" / "3000u_num").string(),
            (input_root / "variable_uav_num" / count).string(),
            "3000users_data", count + "uavs_loc", 40, options));
    execute_experiment("EXP2_uav_num", conditions, options);
}

/// EXP3: use options.input_root and keys 0/2/4/6/8/10 at 3000 users/10 UAVs/40 MHz; save/resume results.
inline void exp3_different_hard_user_ratio(const ExperimentRunOptions& options) {
    validate_run_options(options);
    const fs::path input_root = resolve_experiment_input_root(options);
    vector<ExperimentCondition> conditions;
    for (const auto& ratio : experiment_condition_keys(options, {"0", "2", "4", "6", "8", "10"})) {
        const string input = (input_root / "variable_hard_user_ratio" / ratio).string();
        conditions.push_back(select_experiment_condition(ratio, input, input,
            "3000users_data", "10uavs_loc", 40, options));
    }
    execute_experiment("EXP3_hard_user_ratio", conditions, options);
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

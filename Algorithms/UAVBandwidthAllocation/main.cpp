// Visual Studio 直接运行入口：集中设置实验参数，按条件/实例/算法保存结果并支持断点续跑。
// 备份并定向重跑 EXP1--EXP3 两个提出方法，再接续十网络 EXP5；不删除、不自动重试、不运行 EXP4。
// 显式定义 TON_QUALITY_ABLATION 或 TON_QUALITY_ABLATION_TESTS 时改走互斥的独立入口。
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <bcrypt.h>
#undef INFINITE // predefine.h owns a numerical sentinel with this name; no Windows wait timeout is used here.
#include "localization_experiments.h"
#pragma comment(lib, "bcrypt.lib") // Windows SDK SHA-256; no new third-party dependency.


// Temporary rerun helpers live in this translation unit, not in the general experiment loop.
namespace proposed_rerun {
const string SOURCE_VERSION = "hardfirst-subchannel-da-v1-residual-fast-v1";
const std::array<string, 12> SUMMARY_NAMES = {"Run_time_ms", "Total_Num", "Hard_Num", "Elastic_Num",
    "Total_Utility", "Hard_Utility", "Elastic_Utility", "Hard_Bandwidth", "Elastic_Bandwidth",
    "Hard_Throughput", "Elastic_Throughput", "Total_Throughput"};

/// Read exact bytes for integrity checks/copies; missing or unreadable files throw.
string bytes(const fs::path& path) {
    ifstream input(path, ios::binary);
    if (!input) throw runtime_error("Cannot read: " + path.string());
    ostringstream data;
    data << input.rdbuf();
    if (input.bad() || data.bad()) throw runtime_error("Read failed: " + path.string());
    return data.str();
}

/// Return SHA-256 for a bounded in-memory file using Windows SDK, not a new external dependency.
string sha256(const string& data) {
    if (data.size() > std::numeric_limits<ULONG>::max()) throw runtime_error("Checksum input too large");
    BCRYPT_ALG_HANDLE algorithm = nullptr;
    if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) < 0)
        throw runtime_error("Cannot initialize SHA-256");
    unsigned char digest[32]{};
    const auto status = BCryptHash(algorithm, nullptr, 0,
        reinterpret_cast<PUCHAR>(const_cast<char*>(data.data())), static_cast<ULONG>(data.size()),
        digest, sizeof(digest));
    BCryptCloseAlgorithmProvider(algorithm, 0);
    if (status < 0) throw runtime_error("SHA-256 failed");
    ostringstream text;
    text << hex << setfill('0');
    for (unsigned char value : digest) text << setw(2) << static_cast<unsigned>(value);
    return text.str();
}

/// Describe exact file contents to detect accidental/external changes; this is not an authenticated signature.
json fingerprint(const string& data) {
    return {{"size", data.size()}, {"sha256", sha256(data)}};
}

/// Save through a unique same-directory temporary, checked close and rename; never remove leftovers.
void atomic_save(const fs::path& path, const string& data) { localization::atomic_write(path, data); }

/// Own a named Windows object for the whole manual sequence; process termination releases its handle.
/// Legacy binaries do not honor this guard and must not write the same outputs concurrently.
class SequenceLock {
    HANDLE handle_ = nullptr;
public:
    /// Acquire a nonpersistent output-specific guard, throwing if another invocation holds it.
    explicit SequenceLock(const fs::path& output_root) {
        string id = experiment_absolute_path(output_root);
        std::transform(id.begin(), id.end(), id.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        const string name = "Local\\UAVBandwidthAllocation_" + sha256(id);
        handle_ = CreateMutexW(nullptr, FALSE, std::wstring(name.begin(), name.end()).c_str());
        const DWORD error = GetLastError();
        if (!handle_ || error == ERROR_ALREADY_EXISTS) {
            if (handle_) CloseHandle(handle_);
            handle_ = nullptr;
            throw runtime_error("Another manual experiment sequence owns this output; no files changed");
        }
    }
    /// Release only the OS handle, not a file or a directory.
    ~SequenceLock() { if (handle_) CloseHandle(handle_); }
    SequenceLock(const SequenceLock&) = delete;
    SequenceLock& operator=(const SequenceLock&) = delete;
};

/// Inspected run: immutable old bytes/baselines and independent new proposed-method checkpoint maps.
struct Run {
    fs::path root, side;
    ExperimentRunOptions options;
    json expected, manifest;
    map<string, string> originals;
    vector<ConditionRecords> source, fresh;
};
using Hook = std::function<void(const string&)>; // Isolated fault injection, outside all algorithm timers.
using Evaluate = std::function<AlgorithmRunResult(size_t, int, size_t)>;

/// Invoke an optional interruption boundary; production passes no hook and never retries exceptions.
void boundary(const Hook& hook, const string& event) { if (hook) hook(event); }

/// Build a relative raw name from validated condition/method indices, never from a saved destination path.
string raw_name(const Run& run, size_t condition, size_t method) {
    return run.expected.at("conditions").at(condition).at("key").get<string>() + "/" +
        method_name_list.at(method) + ".csv";
}

/// List only the two proposed CSVs per condition, the twelve summaries, and source metadata.
vector<string> original_names(const Run& run) {
    vector<string> result{"run_info.json"};
    for (size_t c = 0; c < run.expected.at("conditions").size(); ++c)
        for (size_t m = 0; m < 2; ++m) result.push_back(raw_name(run, c, m));
    for (const auto& name : SUMMARY_NAMES) result.push_back("summary/" + name + ".csv");
    return result;
}

/// Capture live input contents/stat identities and actual compiler settings plus canonical run settings.
json identity(const Run& run) {
    json inputs = json::array();
    for (const auto& condition : run.expected.at("conditions"))
        for (const auto& input : condition.at("inputs"))
            for (const string field : {"user_path", "uav_path"}) {
                const fs::path path = input.at(field).get<string>();
                json item = fingerprint(bytes(path));
                item["path"] = experiment_absolute_path(path);
                item["last_write_ticks"] = fs::last_write_time(path).time_since_epoch().count();
                inputs.push_back(std::move(item));
            }
    return {{"target", experiment_absolute_path(run.root)}, {"settings", run.expected}, {"inputs", inputs},
        {"config_file", fingerprint(bytes(run.expected.at("config_path").get<string>()))},
        {"compiler", _MSC_FULL_VER}, {"iterator_debug_level", _ITERATOR_DEBUG_LEVEL},
        {"msvc_stl_update", _MSVC_STL_UPDATE}};
}

/// Distinguish newly computed proposed methods from the four byte-preserved baseline results.
json provenance() {
    return {{"source_algorithm_version", SOURCE_VERSION}, {"recomputed", {"ApproBetter", "ApproFast"}},
        {"preserved", {"AlgRelaxRound", "AlgSwapMatching", "AlgHardFirst", "AlgSA-DD"}}};
}

/// Serialize ID-sorted successful records using the existing public CSV schema.
string records_text(const MethodRecords& records) {
    string result = CSV_HEADER + "\n";
    for (const auto& row : records) result += run_result_to_csv(row.second) + "\n";
    return result;
}

/// Validate a selected-ID subset or full set and its unchanged deterministic seeds.
void validate_records(const Run& run, size_t condition, size_t method,
    const MethodRecords& records, bool complete) {
    if (complete && records.size() != static_cast<size_t>(run.options.instance_count))
        throw runtime_error("Missing selected records: " + raw_name(run, condition, method));
    for (const auto& row : records)
        if (row.first < 1 || row.first > run.options.instance_count ||
            row.second.seed != derive_algorithm_seed(run.options.master_seed,
                run.expected.at("experiment").get<string>(),
                run.expected.at("conditions").at(condition).at("key").get<string>(), row.first, method_name_list[method]))
            throw runtime_error("Rerun checkpoint ID/seed mismatch: " + raw_name(run, condition, method));
}

/// Produce the existing twelve summary formats in memory, requiring complete rows for all six methods.
map<string, string> summary_texts(const Run& run, const vector<ConditionRecords>& records) {
    vector<vector<EXPResult>> means;
    for (const auto& condition : records) {
        vector<EXPResult> row;
        for (const auto& method : condition) row.push_back(average_valid_attempts(method, run.options.instance_count));
        means.push_back(std::move(row));
    }
    map<string, string> result;
    for (size_t metric = 0; metric < SUMMARY_NAMES.size(); ++metric) {
        ostringstream output;
        output << setprecision(std::numeric_limits<double>::max_digits10) << "User_Scale";
        for (const auto& method : method_name_list) output << ',' << method;
        output << '\n';
        for (size_t c = 0; c < means.size(); ++c) {
            output << run.expected.at("conditions").at(c).at("key").get<string>();
            for (const auto& mean : means[c]) output << ',' << mean.*EXP_METRICS[metric];
            output << '\n';
        }
        result.emplace("summary/" + SUMMARY_NAMES[metric] + ".csv", output.str());
    }
    return result;
}

/// Check old summaries against old raw metrics with public tolerances, preserving the exact original bytes.
void validate_old_summaries(const Run& run) {
    for (const auto& table : summary_texts(run, run.source)) {
        const auto& actual = run.originals.at(table.first);
        if (actual.empty() || actual.back() != '\n') throw runtime_error("Truncated summary: " + table.first);
        istringstream old_stream(actual), expected_stream(table.second);
        string old_line, expected_line;
        size_t line = 0;
        while (getline(expected_stream, expected_line)) {
            if (!getline(old_stream, old_line)) throw runtime_error("Missing summary row: " + table.first);
            const auto a = csv_fields(old_line), b = csv_fields(expected_line);
            if (a.size() != b.size() || a[0] != b[0]) throw runtime_error("Summary shape/key mismatch");
            for (size_t i = 1; i < a.size(); ++i)
                if (line == 0 ? a[i] != b[i] :
                    !allocation_near(parse_finite_number(a[i]), parse_finite_number(b[i])))
                    throw runtime_error("Old summary does not match its raw records: " + table.first);
            ++line;
        }
        if (getline(old_stream, old_line)) throw runtime_error("Extra summary row: " + table.first);
    }
}

/// Verify every baseline's immutable digest without ever opening it for writing.
void verify_baselines(const Run& run) {
    const auto& descriptors = run.manifest.at("baseline_files");
    if (descriptors.size() != run.source.size() * 4) throw runtime_error("Baseline inventory mismatch");
    for (size_t c = 0; c < run.source.size(); ++c) for (size_t m = 2; m < 6; ++m) {
        const string name = raw_name(run, c, m);
        if (fingerprint(bytes(run.root / name)) != descriptors.at(name))
            throw runtime_error("Baseline changed; refusing to publish: " + name);
    }
}

/// Construct all complete new targets using new proposed rows and the unchanged baseline rows.
map<string, string> candidates(const Run& run) {
    auto combined = run.source;
    map<string, string> result;
    for (size_t c = 0; c < combined.size(); ++c) for (size_t m = 0; m < 2; ++m) {
        validate_records(run, c, m, run.fresh[c][m], true);
        combined[c][m] = run.fresh[c][m];
        result.emplace(raw_name(run, c, m), records_text(run.fresh[c][m]));
    }
    const auto summaries = summary_texts(run, combined);
    result.insert(summaries.begin(), summaries.end());
    result.emplace("run_info.json", run.expected.dump(2) + "\n");
    return result;
}

/// Return transitional metadata with the OLD suite version; new-version completion is committed last.
json transitional_info(const Run& run, const string& status) {
    json result = run.manifest.at("source_run_info");
    result["summary"]["status"] = status;
    return result;
}

/// Reject unexpected formal contents; publication accepts only the immutable original or exact new candidate.
void verify_official(const Run& run) {
    const string phase = run.manifest.at("phase");
    const bool publishing = phase == "publishing" || phase == "complete";
    const auto next = publishing ? candidates(run) : map<string, string>{};
    for (const auto& original : run.originals) {
        const string actual = bytes(run.root / original.first);
        if (original.first == "run_info.json") {
            const json value = json::parse(actual);
            bool valid = value == run.manifest.at("source_run_info");
            if (phase == "computing" || publishing) valid = valid || value == transitional_info(run, "stale");
            if (publishing) valid = valid || value == transitional_info(run, "writing") || value == run.expected;
            if (phase == "complete") valid = value == run.expected;
            if (!valid) throw runtime_error("Unexpected formal run_info.json; files retained");
        } else if ((phase == "complete" && actual != next.at(original.first)) ||
            (actual != original.second && (!publishing || actual != next.at(original.first)))) {
            throw runtime_error("Formal target is neither original nor staged new result: " + original.first);
        }
    }
    verify_baselines(run);
}

/// Inspect one exact run, backups and checkpoints without writes; recover interrupted backup/publication.
Run inspect(const fs::path& root, json expected, const ExperimentRunOptions& options) {
    expected["summary"] = {{"status", "complete"}, {"instance_count", options.instance_count}};
    Run run{root, root / "rerun_proposed" / SIMPLE_ALGORITHM_VERSION, options, expected};
    json old = expected;
    old["algorithm_version"] = SOURCE_VERSION;
    old.erase("proposed_algorithms");
    const fs::path state_path = run.side / "state.json";
    const bool resumed = fs::exists(state_path);
    if (resumed) {
        run.manifest = read_experiment_json(state_path);
        if (run.manifest.at("schema") != "proposed-rerun-v1" ||
            run.manifest.at("identity") != identity(run) || run.manifest.at("source_run_info") != old ||
            run.manifest.at("provenance") != provenance())
            throw runtime_error("Rerun input/parameter/version/build identity changed; no automatic fallback");
    } else {
        if (read_experiment_json(root / "run_info.json") != old)
            throw runtime_error("Expected complete original results, not an unrelated/new-version run");
        if (fs::exists(run.side)) for (const auto& entry : fs::directory_iterator(run.side))
            if (!entry.is_regular_file() || entry.path().filename().string().rfind("state.json.tmp_", 0) != 0)
                throw runtime_error("Nonempty rerun directory lacks state.json; retained without repair");
        run.manifest = {{"schema", "proposed-rerun-v1"}, {"identity", identity(run)}, {"source_run_info", old},
            {"phase", "backing_up"}, {"provenance", provenance()},
            {"original_files", json::object()}, {"baseline_files", json::object()}};
    }
    const string phase = run.manifest.at("phase");
    if (phase != "backing_up" && phase != "computing" && phase != "publishing" && phase != "complete")
        throw runtime_error("Invalid rerun phase");
    const auto names = original_names(run);
    if (resumed && run.manifest.at("original_files").size() != names.size())
        throw runtime_error("Original inventory mismatch");
    for (const auto& name : names) {
        const fs::path backup = run.side / "backup" / name;
        if (phase != "backing_up" && !fs::exists(backup)) throw runtime_error("Required backup missing: " + name);
        const string original = bytes(fs::exists(backup) ? backup : root / name);
        if (resumed && fingerprint(original) != run.manifest.at("original_files").at(name))
            throw runtime_error("Original/backup digest mismatch: " + name);
        run.originals.emplace(name, original);
        if (!resumed) run.manifest["original_files"][name] = fingerprint(original);
    }
    if (json::parse(run.originals.at("run_info.json")) != old) throw runtime_error("Wrong source metadata backup");
    for (size_t c = 0; c < expected.at("conditions").size(); ++c) {
        run.source.emplace_back(6);
        run.fresh.emplace_back(2);
        for (size_t m = 0; m < 6; ++m) {
            const string name = raw_name(run, c, m);
            const fs::path backup = run.side / "backup" / name;
            run.source[c][m] = read_result_csv(m < 2 && fs::exists(backup) ? backup : root / name, true);
            validate_records(run, c, m, run.source[c][m], true);
            if (m >= 2 && !resumed) run.manifest["baseline_files"][name] = fingerprint(bytes(root / name));
            if (m < 2) {
                run.fresh[c][m] = read_result_csv(run.side / "new_results" / name);
                validate_records(run, c, m, run.fresh[c][m], phase == "publishing" || phase == "complete");
                if (phase == "backing_up" && !run.fresh[c][m].empty())
                    throw runtime_error("New results exist before backups completed");
            }
        }
    }
    validate_old_summaries(run);
    verify_official(run);
    if (phase == "publishing" || phase == "complete") for (const auto& item : candidates(run))
        if (bytes(run.side / "candidate" / item.first) != item.second)
            throw runtime_error("Publication candidate missing/changed: " + item.first);
    return run;
}

/// Atomically save only dedicated recovery state, not the formal metadata.
void save_state(const Run& run) { atomic_save(run.side / "state.json", run.manifest.dump(2) + "\n"); }

/// Finish one-time byte-preserving backups; caller preflights all experiments before invoking this.
void backup(Run& run, const Hook& hook = {}) {
    if (run.manifest.at("phase") != "backing_up") return;
    verify_official(run);
    save_state(run);
    for (const auto& item : run.originals) {
        const auto path = run.side / "backup" / item.first;
        if (fs::exists(path)) {
            if (bytes(path) != item.second) throw runtime_error("Refusing to overwrite different backup");
        } else atomic_save(path, item.second);
        if (fingerprint(bytes(path)) != run.manifest.at("original_files").at(item.first))
            throw runtime_error("Backup verification failed");
        boundary(hook, "backup:" + item.first);
    }
    run.manifest["phase"] = "computing";
    save_state(run);
}

/// Evaluate only missing proposed units, checkpoint each atomically, then recoverably publish this run.
/// evaluate(c,id,m) is a real allocator in production and a tiny deterministic fixture in isolated tests.
void perform(Run& run, const Evaluate& evaluate, const Hook& hook = {}) {
    verify_official(run);
    if (run.manifest.at("phase") == "complete") {
        cout << "[RERUN SKIP] " << run.expected.at("experiment") << " (new version complete)\n";
        return;
    }
    if (run.manifest.at("phase") == "backing_up") throw runtime_error("All backups must finish before calculation");
    if (run.manifest.at("phase") == "computing") {
        atomic_save(run.root / "run_info.json", transitional_info(run, "stale").dump(2) + "\n");
        for (size_t c = 0; c < run.fresh.size(); ++c) for (int id = 1; id <= run.options.instance_count; ++id)
            for (size_t m = 0; m < 2; ++m) {
                const string context = run.expected.at("experiment").get<string>() + "/" + raw_name(run, c, m) +
                    "/ID " + to_string(id);
                if (run.fresh[c][m].count(id)) { cout << "[RERUN SKIP] " << context << std::endl; continue; }
                cout << "[RERUN RUN] " << context << std::endl;
                auto result = evaluate(c, id, m);
                result.instance_id = id;
                if (!algorithm_status_valid(result.status))
                    throw runtime_error(context + ": " + algorithm_status_name(result.status) + " / " +
                        (result.diagnostics.events.empty() ? "no diagnostic" : result.diagnostics.events.back()));
                result = parse_run_result(run_result_to_csv(result)); // Validate scalars and discard allocation maps.
                MethodRecords next = run.fresh[c][m];
                next.emplace(id, std::move(result));
                validate_records(run, c, m, next, false);
                atomic_save(run.side / "new_results" / raw_name(run, c, m), records_text(next));
                run.fresh[c][m] = std::move(next);
                cout << "[RERUN SAVED] " << context << std::endl;
                boundary(hook, "saved:" + context);
            }
        if (identity(run) != run.manifest.at("identity")) throw runtime_error("Inputs changed during calculation");
        verify_official(run);
        for (const auto& item : candidates(run)) {
            const auto path = run.side / "candidate" / item.first;
            if (fs::exists(path) && bytes(path) != item.second) throw runtime_error("Conflicting candidate: " + item.first);
            if (!fs::exists(path)) atomic_save(path, item.second);
            if (bytes(path) != item.second) throw runtime_error("Candidate verification failed");
        }
        run.manifest["phase"] = "publishing";
        save_state(run);
        boundary(hook, "publishing");
    }
    verify_official(run);
    const auto next = candidates(run);
    // A crash after the last metadata write must not regress the already-complete formal metadata.
    if (read_experiment_json(run.root / "run_info.json") != run.expected)
        atomic_save(run.root / "run_info.json", transitional_info(run, "writing").dump(2) + "\n");
    for (const auto& item : next) if (item.first != "run_info.json") {
        const string current = bytes(run.root / item.first);
        if (current != item.second) {
            if (current != run.originals.at(item.first)) throw runtime_error("Target changed during publication");
            atomic_save(run.root / item.first, item.second);
        }
        boundary(hook, "published:" + item.first);
    }
    verify_baselines(run);
    for (const auto& item : next) if (item.first != "run_info.json")
        if (bytes(run.root / item.first) != item.second) throw runtime_error("Published result verification failed");
    atomic_save(run.root / "run_info.json", next.at("run_info.json"));
    boundary(hook, "metadata_published");
    run.manifest["phase"] = "complete";
    save_state(run);
    cout << "[RERUN COMPLETE] " << run.expected.at("experiment") << "; N=" << run.options.instance_count << std::endl;
}

/// Load and validate a selected model outside the allocator timer, retaining the fixed physical dimensions.
SystemMd model_for(const Run& run, size_t condition, int id) {
    const auto& entry = run.expected.at("conditions").at(condition);
    const auto& input = entry.at("inputs").at(static_cast<size_t>(id - 1));
    SystemMd model(input.at("user_path").get<string>(), input.at("uav_path").get<string>(),
        run.expected.at("config_path").get<string>());
    const string experiment = run.expected.at("experiment"), key = entry.at("key");
    const size_t users = experiment == "EXP1_user_num" ? std::stoul(key) : 3000;
    const size_t uavs = experiment == "EXP2_uav_num" ? std::stoul(key) : 10;
    if (model.users.size() != users || model.uavs.size() != uavs)
        throw runtime_error("Unexpected model dimensions for " + experiment + "/" + key);
    for (auto& uav : model.uavs) uav.total_bandwidth = entry.at("bandwidth_mhz").get<double>() * unit_para;
    model.init_SystemModel();
    validate_allocation_model(model);
    return model;
}
} // namespace proposed_rerun

/// Temporarily replace only EXP1--EXP3 ApproBetter/ApproFast for the fixed 10-instance ToN batch.
/// Inspect all inputs/results and finish all backups before solving; failures throw and prevent EXP5.
void rerun_proposed_exp1_exp3(const ExperimentRunOptions& options) {
    using namespace proposed_rerun;
    validate_run_options(options);
    const auto input_root = resolve_experiment_input_root(options);
    const auto approved_input = fs::path(experimentDataPath) / "data_ToN" / "2026-09-07";
    if (options.instance_count != 10 || options.master_seed != 20260905u || options.rounding_trials != 2 ||
        options.ton_epsilon != 0.1 || options.output_name != "run_ton_01" ||
        !options.conditions.empty() || !options.reuse_exp1_root.empty() ||
        !fs::equivalent(input_root, approved_input) || experiment_build_profile() != "Release|x64")
        throw invalid_argument("Temporary rerun requires run_ton_01, IDs 1--10, approved parameters, Release|x64");
#if defined(TON_VERIFY_SMAWK)
    throw invalid_argument("Disable TON_VERIFY_SMAWK for formal timing");
#endif
    vector<Run> runs;
    for (const string experiment : {"EXP1_user_num", "EXP2_uav_num", "EXP3_hard_user_ratio"}) {
        const vector<string> keys = experiment == "EXP1_user_num" ? vector<string>{"1000", "2000", "3000", "4000", "5000"} :
            experiment == "EXP2_uav_num" ? vector<string>{"5", "10", "15", "20"} : vector<string>{"0", "2", "4", "6", "8", "10"};
        vector<ExperimentCondition> conditions;
        for (const auto& key : keys) {
            const string users = experiment == "EXP1_user_num" ? key : "3000";
            const fs::path user_dir = experiment == "EXP3_hard_user_ratio" ?
                input_root / "variable_hard_user_ratio" / key : input_root / "variable_user_num" / (users + "u_num");
            const fs::path uav_dir = experiment == "EXP2_uav_num" ? input_root / "variable_uav_num" / key : user_dir;
            conditions.push_back(select_experiment_condition(key, user_dir.string(), uav_dir.string(),
                users + "users_data", (experiment == "EXP2_uav_num" ? key : "10") + "uavs_loc", 40, options));
        }
        const fs::path base = fs::path(experimentDataPath) / "ExperimentsResults" / experiment;
        runs.push_back(inspect(base / TON_RESULT_VERSION_DIR / options.output_name,
            make_experiment_run_info(experiment, conditions, (base / "def_config.json").string(), options), options));
        for (size_t c = 0; c < conditions.size(); ++c) for (int id = 1; id <= options.instance_count; ++id)
            (void)model_for(runs.back(), c, id); // Read-only physical preflight, not an algorithm call.
    }
    for (auto& run : runs) backup(run);
    for (auto& run : runs) {
        std::optional<SystemMd> cached;
        size_t cached_condition = std::numeric_limits<size_t>::max();
        int cached_id = -1;
        // Load once per network for the two methods; all loading stays outside run_algorithm's timer.
        perform(run, [&](size_t c, int id, size_t method) {
            if (!cached || c != cached_condition || id != cached_id) {
                cached = model_for(run, c, id); cached_condition = c; cached_id = id;
            }
            const uint32_t seed = derive_algorithm_seed(options.master_seed, run.expected.at("experiment").get<string>(),
                run.expected.at("conditions").at(c).at("key").get<string>(), id, method_name_list.at(method));
            return run_algorithm(*cached, method, options, seed);
        });
    }
}

// Standalone tests compile the helpers but cannot enter this formal main().
#if defined(TON_QUALITY_ABLATION) && defined(TON_QUALITY_ABLATION_TESTS)
#error TON_QUALITY_ABLATION and TON_QUALITY_ABLATION_TESTS are mutually exclusive
#endif
#if defined(TON_QUALITY_ABLATION) || defined(TON_QUALITY_ABLATION_TESTS)
#include "ton_quality_ablation.h"
#endif
#if defined(TON_QUALITY_ABLATION_TESTS)
#include "tests/ton_quality_ablation_tests.h"
#endif
#ifndef TON_RERUN_TESTING
/// 使用固定批次先定向重跑 EXP1--EXP3，再接续 EXP5；argc 必须为 1，不接收旧 CLI 参数。
/// 定义质量消融/合成测试宏时，只运行对应独立分支并立即返回，不进入正式实验。
/// 成功返回 0；遇到错误保留已写结果并返回 1，后续实验不再自动启动。
int main(int argc, char*[]) {
    try {
        if (argc != 1)
            throw invalid_argument("Legacy batch/test CLI is disabled. Select experiments and parameters in main.cpp.");

#if defined(TON_QUALITY_ABLATION_TESTS)
        // 纯合成验证分支不读正式输入、不创建输出，结束后不进入任何正式实验。
        return ton_quality_tests::run();
#elif defined(TON_QUALITY_ABLATION)
        // 独立质量消融分支；每次新目录、固定 50 次，不进入下方重跑或 EXP5。
        return ton_quality::run(true); // true 开启诊断；另行比较开销时可显式改为 false。
#else
        // 本轮临时重跑严格固定这些设置，不能改为 30 或更换输出名后继续使用此入口。
        ExperimentRunOptions options;
        options.instance_count = 10;         // 本轮仅 ID 1--10；EXP5 也显式使用 10 个网络。
        options.master_seed = 20260905u;     // 保持不变，使新算、续跑和旧记录使用一致的逐实例种子。
        options.rounding_trials = 2;         // AlgRelaxRound 的既有随机舍入次数。
        options.ton_epsilon = 0.1;           // ToN 近似参数，要求 0 < epsilon < 0.5。
        options.input_root = (fs::path(experimentDataPath) / "data_ToN" / "2026-09-07").string();
        options.output_name = "run_ton_01";  // 新 ToN 输入独立输出；保留旧 run_01，不导入旧指标或耗时。
        options.conditions.clear();         // 各实验使用完整标准条件；EXP5 使用独立 RMSE 矩阵。
        options.reuse_exp1_root = "";

        // 输入已备好，不重跑生成器；互斥锁覆盖定向重跑和 EXP5 的整个手动流程。
        proposed_rerun::SequenceLock lock(fs::path(experimentDataPath) / "ExperimentsResults" / options.output_name);
        rerun_proposed_exp1_exp3(options);       // 300 次新计算；新版完整检查点跳过。
        exp5_different_location_error(options); // 780 次新计算；仅在前三个实验完成后进入。

        // EXP4 暂停；保留代码和已有结果。
        // exp4_different_total_bandwidth(options);
        return 0;
#endif // 独立消融/合成测试与原正式入口互斥。
    } catch (const exception& error) {
        cerr << "实验停止，已写入的结果保持不变。\n原因: " << error.what() << std::endl;
        return 1;
    }
}
#endif // TON_RERUN_TESTING

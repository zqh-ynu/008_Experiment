#pragma once
// Lightweight experiment support: six-method dispatch, physical metrics, CSV checkpoints and arithmetic summaries.
// Normal execution has no snapshots, file hashes, background monitoring or automatic regression tests.
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
inline const string SIMPLE_RUN_SCHEMA = "ton-simple-v1";
inline const string SIMPLE_ALGORITHM_VERSION = "hardfirst-subchannel-da-v1-residual-fast-v1";

/// Immutable DA adaptation identity, compared by the existing lightweight run-info gate.
inline json hard_first_da_policy() {
    return {{"mechanism", "priority-aware-subchannel-da-v1"},
        {"hard_preference", "zero-demand_then_slot-rate/min-rate_then_utility_then_ID"},
        {"user_preference", "slot-rate_desc_then_UAV-ID_then_slot-ID"},
        {"association", "single-UAV_release_failed_hard_then_next-UAV"},
        {"recovery", "freeze_completed_hard_repeat_only_after_new_admissions"},
        {"elastic_preference", "round-start_exact_log_increment_excluding_incumbent_slot"},
        {"effective_width", "common_User.BSub_MHz"}, {"slot_to_effective_ratio", 10.0 / 9.0},
        {"overhead_scope", "AlgHardFirst_only"}};
}

/// Settings supplied explicitly from main.cpp; input_root selects inputs, not physical configs or result paths.
struct ExperimentRunOptions {
    int instance_count = 10;
    uint32_t master_seed = 20260905u;
    int rounding_trials = 2;
    double ton_epsilon = 0.1;
    vector<string> conditions;       // Empty selects all standard conditions of this experiment.
    string output_name = "run_da_01"; // A single directory name, never an absolute/relative path.
    string reuse_exp1_root;          // Optional audited v1 source; ignored by EXP2--EXP4.
    string input_root = (fs::path(experimentDataPath) / "data").string(); // Legacy-compatible default; main selects ToN.
};

/// Validate supplied counts, epsilon and path options; invalid values throw before any output write.
inline void validate_run_options(const ExperimentRunOptions& options) {
    if (options.instance_count <= 0 || options.rounding_trials <= 0 ||
        !std::isfinite(options.ton_epsilon) || options.ton_epsilon <= 0 || options.ton_epsilon >= 0.5)
        throw invalid_argument("Positive counts and 0 < epsilon < 0.5 are required");
    if (options.output_name.empty() ||
        options.output_name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != string::npos)
        throw invalid_argument("output_name must contain only letters, digits, underscores or hyphens");
    if (options.input_root.empty())
        throw invalid_argument("input_root must explicitly select an existing input directory");
}

/// Return an absolute normalized path for input identities; this does not hash or copy file contents.
inline string experiment_absolute_path(const fs::path& path) {
    return fs::absolute(path).lexically_normal().generic_string();
}

/// Identify the selected IDE configuration, not measured hardware or a full compiler/environment fingerprint.
inline string experiment_build_profile() {
#if defined(NDEBUG)
    string profile = "Release";
#else
    string profile = "Debug";
#endif
#if defined(_WIN64)
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
    if (model.m != static_cast<int>(m) || model.n1 < 0 || model.n2 < 0 ||
        model.n1 + model.n2 != static_cast<int>(n) ||
        model.cap_list.size() != m || model.dis_list.size() != m ||
        model.SNRave_list.size() != m || model.Bth_list.size() != m)
        throw invalid_argument("Model dimension mismatch");
    for (size_t i = 0; i < n; ++i) {
        const auto& u = model.users[i];
        if (u.ID != static_cast<int>(i) ||
            u.uType != (i < static_cast<size_t>(model.n1) ? HARD_UTILITY : ELASTIC_UTILITY) ||
            !std::isfinite(u.weight) || u.weight < 0 || !std::isfinite(u.rMin) || u.rMin < 0)
            throw invalid_argument("Invalid user ordering or parameters");
    }
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
                const double threshold = model.Bth_list[k][i];
                if (cap > 0 ? !allocation_near(threshold, model.users[i].rMin / cap) :
                    threshold != std::numeric_limits<double>::infinity())
                    throw invalid_argument("Hard threshold and capacity are inconsistent");
            }
        }
    }
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

/// Check one physical allocation and compute the twelve metrics once; model was checked before the method loop.
/// Cached utility totals and the unused UserResult projection do not participate in these statistics.
inline EXPResult compute_single_EXPResult(const SystemMd& model,
    const vector<KnapsackResult>& results, double duration_ms) {
    require_allocation(std::isfinite(duration_ms) && duration_ms >= 0, "Invalid duration");
    require_allocation(results.size() == model.uavs.size(), "One result entry per UAV is required");
    EXPResult totals;
    totals.duration = duration_ms;
    vector<bool> seen_user(model.users.size(), false), seen_uav(model.uavs.size(), false);
    for (const auto& result : results) {
        const int k = result.uav_id;
        require_allocation(k >= 0 && k < model.m, "Invalid result UAV ID");
        require_allocation(!seen_uav[k], "Duplicate result UAV ID");
        seen_uav[k] = true;
        require_allocation(result.allocatedList.size() == result.allocatedBandwidth.size(),
            "Allocation list and bandwidth map disagree");
        double used_bandwidth = 0;
        EXPResult local; // Preserve the original per-UAV grouping of floating-point utility/bandwidth sums.
        for (int i : result.allocatedList) {
            require_allocation(i >= 0 && i < static_cast<int>(model.users.size()), "Invalid result user ID");
            require_allocation(!seen_user[i], "User assigned more than once: " + to_string(i));
            seen_user[i] = true;
            require_allocation(result.allocatedBandwidth.count(i) != 0, "Missing bandwidth entry");
            const double bw = result.allocatedBandwidth.at(i), cap = model.cap_list[k][i];
            const auto& user = model.users[i];
            require_allocation(std::isfinite(bw) && bw > 0, "Served bandwidth must be finite and positive");
            // init_SystemModel defines serviceable links by this same distance bound.
            require_allocation(model.dis_list[k][i] <= max_coverage_distance && cap > 0,
                "Assigned link is not serviceable");
            const double rate = bw * cap;
            require_allocation(std::isfinite(rate), "Non-finite allocated rate");
            if (user.uType == HARD_UTILITY) {
                require_allocation(hard_qos_satisfied(bw, cap, user.rMin),
                    "Hard QoS not met: UAV " + to_string(k) + ", user " + to_string(i));
                ++totals.hard_num;
                local.hard_utility += user.weight * log2(1.0 + user.rMin);
                local.hard_bandwidth += bw;
                totals.hard_throughput += user.rMin; // Guaranteed business rate, not excess physical rate.
            } else {
                ++totals.elastic_num;
                local.elastic_utility += user.elastic_utility(bw, cap);
                local.elastic_bandwidth += bw;
                totals.elastic_throughput += rate;
            }
            used_bandwidth += bw;
        }
        const double budget = model.uavs[k].total_bandwidth;
        require_allocation(std::isfinite(used_bandwidth) &&
            used_bandwidth <= budget + allocation_tolerance(used_bandwidth, budget),
            "UAV bandwidth budget exceeded");
        totals.hard_utility += local.hard_utility;
        totals.elastic_utility += local.elastic_utility;
        totals.hard_bandwidth += local.hard_bandwidth;
        totals.elastic_bandwidth += local.elastic_bandwidth;
    }
    totals.total_num = totals.hard_num + totals.elastic_num;
    totals.total_utility = totals.hard_utility + totals.elastic_utility;
    totals.total_throughput = totals.hard_throughput + totals.elastic_throughput;
    for (auto member : EXP_METRICS)
        require_allocation(std::isfinite(totals.*member) && totals.*member >= 0, "Invalid aggregate metric");
    return totals;
}

/// Run solver on a private copy of an already checked model; return timing, metrics or an unwritten failure.
inline AlgorithmRunResult execute_allocation(const SystemMd& model, uint32_t seed,
    const std::function<AllocationPair(BAProblem&, AllocationDiagnostics&)>& solver) {
    AlgorithmRunResult run;
    run.seed = seed;
    try {
        BAProblem problem(model);
        const auto start = std::chrono::steady_clock::now();
        try { run.allocation = solver(problem, run.diagnostics); }
        catch (...) {
            run.duration_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
            throw;
        }
        run.duration_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        run.metrics = compute_single_EXPResult(model, run.allocation.first, run.duration_ms);
        run.status = run.metrics.total_num == 0 ? AlgorithmRunStatus::ZeroAllocation : AlgorithmRunStatus::Success;
    } catch (const AllocationFailure& failure) {
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

/// Dispatch method index 0--5 with prevalidated options/model; empty resources are legitimate zero allocations.
inline AlgorithmRunResult run_algorithm(const SystemMd& model, size_t method,
    const ExperimentRunOptions& options, uint32_t seed) {
    if (method >= method_name_list.size()) throw invalid_argument("Unknown method index");
    return execute_allocation(model, seed, [&](BAProblem& problem, AllocationDiagnostics& diagnostics) {
        bool any_budget = false;
        for (const auto& uav : model.uavs) any_budget = any_budget || uav.total_bandwidth > 0;
        if (model.users.empty() || model.uavs.empty() || !any_budget) {
            vector<KnapsackResult> empty(model.uavs.size());
            for (size_t k = 0; k < empty.size(); ++k) empty[k].uav_id = static_cast<int>(k);
            return AllocationPair{empty, problem.construct_user_results(empty)};
        }
        switch (method) {
        case 0: return problem.Appro_multiUAV_ToN(model.uavs, model.users, 2, options.ton_epsilon);
        case 1: return problem.Appro_multiUAV_ToN(model.uavs, model.users, 1, options.ton_epsilon);
        case 2: return problem.ConvexRelaxationAndRounding_multiUAV(1e-4, seed, options.rounding_trials, &diagnostics);
        case 3: return problem.MatchingSQP_Allocation(MatchingSQPConfig(), &diagnostics);
        case 4: {
            auto allocation = problem.HardFirstPriorityMatchingAllocation(&diagnostics);
            validate_hard_first_allocation(model, allocation.first);
            return allocation;
        }
        default: return problem.SADA_Allocation(SADAConfig(), &diagnostics);
        }
    });
}

/// Derive an instance-local seed using specified FNV-1a/32 over UTF-8, independent of invocation order.
inline uint32_t derive_algorithm_seed(uint32_t master_seed, const string& experiment,
    const string& condition, int instance_id, const string& method) {
    const string identity = json::array({master_seed, experiment, condition, instance_id, method}).dump();
    uint32_t hash = 2166136261u;
    for (unsigned char byte : identity) { hash ^= byte; hash *= 16777619u; }
    return hash;
}

/// Encode a CSV field; diagnostics JSON escapes embedded newlines before reaching this function.
inline string csv_quote(const string& value) {
    if (value.find_first_of(",\"\r\n") == string::npos) return value;
    string encoded = "\"";
    for (char c : value) { if (c == '"') encoded += '"'; encoded += c; }
    return encoded + "\"";
}

/// Parse one physical CSV line strictly, including quoted commas and doubled quotes.
inline vector<string> csv_fields(const string& raw_line) {
    string line = raw_line;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    vector<string> fields;
    size_t pos = 0;
    while (true) {
        string value;
        if (pos < line.size() && line[pos] == '"') {
            ++pos;
            bool closed = false;
            while (pos < line.size()) {
                char c = line[pos++];
                if (c != '"') { value += c; continue; }
                if (pos < line.size() && line[pos] == '"') { value += '"'; ++pos; }
                else { closed = true; break; }
            }
            if (!closed || (pos < line.size() && line[pos] != ','))
                throw invalid_argument("Malformed quoted CSV field");
        } else {
            while (pos < line.size() && line[pos] != ',') {
                if (line[pos] == '"') throw invalid_argument("Quote in unquoted CSV field");
                value += line[pos++];
            }
        }
        fields.push_back(value);
        if (pos == line.size()) break;
        ++pos;
        if (pos == line.size()) { fields.emplace_back(); break; }
    }
    return fields;
}

/// Parse a complete finite number, refusing trailing text and unavailable metric cells.
inline double parse_finite_number(const string& token) {
    size_t consumed = 0;
    double value = stod(token, &consumed);
    if (consumed != token.size() || !std::isfinite(value) || value < 0)
        throw invalid_argument("Invalid numeric CSV field");
    return value;
}

/// Serialize common metrics with round-trip floating-point precision.
inline string expResultToCSVLine(const EXPResult& result) {
    ostringstream out;
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
    const json diagnostics = {{"used_fallback", run.diagnostics.used_fallback}, {"events", run.diagnostics.events}};
    out << run.instance_id << ',' << run.seed << ',' << algorithm_status_name(run.status)
        << ',' << run.duration_ms << ',' << csv_quote(diagnostics.dump()) << ',';
    if (algorithm_status_valid(run.status)) out << expResultToCSVLine(run.metrics);
    else out << string(EXP_METRICS.size() - 1, ',');
    return out.str();
}

/// Parse a typed attempt and reject contradictions between status, metrics and metadata.
inline AlgorithmRunResult parse_run_result(const string& line) {
    const auto fields = csv_fields(line);
    if (fields.size() != 5 + EXP_METRICS.size()) throw invalid_argument("Wrong result column count");
    AlgorithmRunResult run;
    const double id = parse_finite_number(fields[0]), seed = parse_finite_number(fields[1]);
    if (id < 1 || id > INT_MAX || id != floor(id) || seed > UINT32_MAX || seed != floor(seed))
        throw invalid_argument("Invalid result ID/seed");
    run.instance_id = static_cast<int>(id);
    run.seed = static_cast<uint32_t>(seed);
    run.status = parse_algorithm_status(fields[2]);
    run.duration_ms = parse_finite_number(fields[3]);
    const auto diagnostics = json::parse(fields[4]);
    run.diagnostics.used_fallback = diagnostics.at("used_fallback").get<bool>();
    run.diagnostics.events = diagnostics.at("events").get<vector<string>>();
    for (size_t i = 0; i < EXP_METRICS.size(); ++i) {
        if (algorithm_status_valid(run.status)) run.metrics.*EXP_METRICS[i] = parse_finite_number(fields[5 + i]);
        else if (!fields[5 + i].empty()) throw invalid_argument("Failed result must have blank performance cells");
    }
    if (algorithm_status_valid(run.status)) {
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

/// Read one CSV into independent checkpoints; missing optional files are empty, malformed/duplicate/failed rows throw.
/// The required flag is used for explicitly requested legacy imports, never to repair an incomplete file.
inline MethodRecords read_result_csv(const fs::path& path, bool required = false) {
    if (!fs::exists(path)) {
        if (required) throw runtime_error("Missing requested result CSV: " + path.string());
        return {};
    }
    ifstream input(path, ios::binary);
    string line;
    if (!input || !getline(input, line)) throw runtime_error("Cannot read CSV header: " + path.string());
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != CSV_HEADER) throw runtime_error("CSV header mismatch: " + path.string());
    MethodRecords rows;
    size_t line_number = 1;
    while (getline(input, line)) {
        ++line_number;
        try {
            if (input.eof()) throw runtime_error("Unterminated final record");
            auto row = parse_run_result(line);
            if (!algorithm_status_valid(row.status)) throw runtime_error("Recorded failure is not a completed result");
            const int id = row.instance_id;
            if (!rows.emplace(id, std::move(row)).second) throw runtime_error("Duplicate instance ID");
        } catch (const std::exception& error) {
            throw runtime_error(path.string() + ":" + to_string(line_number) + ": " + error.what());
        }
    }
    if (input.bad()) throw runtime_error("CSV read error: " + path.string());
    return rows;
}

/// Read each method independently; rows must belong to selected inputs and retain the original deterministic seeds.
inline ConditionRecords read_condition_records(const fs::path& directory, const string& experiment,
    const string& condition, const json& inputs, const ExperimentRunOptions& options) {
    set<int> selected;
    for (const auto& input : inputs) selected.insert(input.at("id").get<int>());
    ConditionRecords result(method_name_list.size());
    for (size_t m = 0; m < method_name_list.size(); ++m) {
        result[m] = read_result_csv(directory / (method_name_list[m] + ".csv"));
        for (const auto& entry : result[m])
            if (!selected.count(entry.first) || entry.second.seed !=
                derive_algorithm_seed(options.master_seed, experiment, condition, entry.first, method_name_list[m]))
                throw runtime_error("Checkpoint ID/seed mismatch: " + directory.string() + "/" + method_name_list[m]);
    }
    return result;
}

/// Append exactly one already checked successful result and close the stream; failures never get a numerical row.
/// The caller has read existing headers/IDs and created the condition directory; no serialize/parse round trip is done.
inline void appendResult(const string& directory, size_t method, const AlgorithmRunResult& run) {
    if (!algorithm_status_valid(run.status)) throw runtime_error("Refusing to write a failed algorithm result");
    const fs::path path = fs::path(directory) / (method_name_list.at(method) + ".csv");
    const bool needs_header = !fs::exists(path);
    ofstream output(path, ios::app | ios::binary);
    if (!output) throw runtime_error("Cannot append result: " + path.string());
    if (needs_header) output << CSV_HEADER << '\n';
    output << run_result_to_csv(run) << '\n';
    output.flush();
    output.close();
    if (!output) throw runtime_error("Result write/close failed: " + path.string());
}

/// Average all selected completed rows for a method; missing/failed rows never become zeros or a successful-only mean.
inline EXPResult average_valid_attempts(const MethodRecords& rows, size_t expected_count) {
    if (expected_count == 0 || rows.size() != expected_count)
        throw runtime_error("Summary refused: incomplete selected instance set");
    EXPResult mean;
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
    if (conditions.empty() || conditions.size() != means.size())
        throw runtime_error("Summary condition count mismatch");
    for (const auto& row : means)
        if (row.size() != method_name_list.size()) throw runtime_error("Summary method count mismatch");
    const std::array<string, 12> files = {"Run_time_ms", "Total_Num", "Hard_Num", "Elastic_Num",
        "Total_Utility", "Hard_Utility", "Elastic_Utility", "Hard_Bandwidth", "Elastic_Bandwidth",
        "Hard_Throughput", "Elastic_Throughput", "Total_Throughput"};
    fs::create_directories(directory);
    for (size_t metric = 0; metric < files.size(); ++metric) {
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
        output.flush();
        output.close();
        if (!output) throw runtime_error("Summary write/close failed: " + path.string());
    }
}

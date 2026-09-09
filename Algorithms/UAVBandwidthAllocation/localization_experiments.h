#pragma once
// EXP5: fixed-deployment localization sensitivity, paired Gaussian inputs, truth evaluation,
// and independent two-file checkpoints. No allocator, legacy CSV, or source input is modified.
#include "experiments.h"
#include <optional>
#include <locale>

namespace localization {

inline const string VERSION = "localization-pilot-v1";

/// Manual experiment settings. Defaults select only the approved 3-network, 234-call pilot.
struct Options {
    int instance_count = 3;
    int error_repeats = 2;
    vector<int> rmse_m = {0, 1, 2, 5, 10, 20, 50};
    uint32_t error_master_seed = 20260908u;
    uint32_t algorithm_master_seed = 20260905u;
    int rounding_trials = 2;
    double epsilon = 0.1;
    string output_name = "pilot_01";
    string input_root = (fs::path(experimentDataPath) / "data_ToN" / "2026-09-07").string();
};

/// Return the unchanged six-method options; EXP5 never imports historical results.
inline ExperimentRunOptions algorithm_options(const Options& options) {
    ExperimentRunOptions result;
    result.instance_count = options.instance_count;
    result.master_seed = options.algorithm_master_seed;
    result.rounding_trials = options.rounding_trials;
    result.ton_epsilon = options.epsilon;
    result.output_name = options.output_name;
    result.input_root = options.input_root;
    return result;
}

/// Map the common manual settings to EXP5 without changing the legacy three-network defaults.
/// Only shared numerical/input/output settings are accepted; EXP5 has its own fixed RMSE matrix.
inline Options from_experiment_options(const ExperimentRunOptions& options) {
    validate_run_options(options);
    if (!options.conditions.empty() || !options.reuse_exp1_root.empty())
        throw invalid_argument("EXP5 wrapper requires empty conditions and no historical reuse");
    Options result;
    result.instance_count = options.instance_count;
    result.algorithm_master_seed = options.master_seed;
    result.rounding_trials = options.rounding_trials;
    result.epsilon = options.ton_epsilon;
    result.input_root = options.input_root;
    result.output_name = options.output_name;
    return result;
}

/// Validate explicit settings before touching any output; zero must occur exactly once and first.
inline void validate_options(const Options& options) {
    validate_run_options(algorithm_options(options));
    if (options.error_repeats <= 0 || options.rmse_m.empty() || options.rmse_m.front() != 0)
        throw invalid_argument("EXP5 requires positive repeats and an initial zero-RMSE baseline");
    for (size_t i = 1; i < options.rmse_m.size(); ++i)
        if (options.rmse_m[i] <= options.rmse_m[i - 1])
            throw invalid_argument("EXP5 RMSE levels must be nonnegative and strictly increasing");
}

/// Return the complete unit count, counting the zero baseline only once per network/method.
inline size_t expected_calls(const Options& options) {
    validate_options(options);
    return static_cast<size_t>(options.instance_count) * method_name_list.size() *
        (1 + (options.rmse_m.size() - 1) * static_cast<size_t>(options.error_repeats));
}

/// Return a method seed paired across every error level/repetition, matching EXP1's 3000-user seed.
inline uint32_t method_seed(const Options& options, int instance, size_t method) {
    return derive_algorithm_seed(options.algorithm_master_seed, "EXP1_user_num", "3000",
        instance, method_name_list.at(method));
}

/// Return a domain-separated position seed; repetition zero is the deterministic no-error baseline.
inline uint32_t position_seed(const Options& options, int instance, int repetition) {
    return repetition == 0 ? 0u : derive_algorithm_seed(options.error_master_seed,
        "EXP5_location_error", "normal_rep_" + to_string(repetition), instance, "position");
}

using GaussianField = vector<std::array<double, 2>>;

/// Generate independent N(0,1) X/Y samples with a local RNG, never an allocator/global RNG.
inline GaussianField gaussian_field(size_t users, uint32_t seed) {
    std::mt19937 engine(seed);
    std::normal_distribution<double> normal(0.0, 1.0);
    GaussianField field(users);
    for (auto& xy : field) { xy[0] = normal(engine); xy[1] = normal(engine); }
    return field;
}

/// Estimated copy plus the empirical horizontal RMSE actually represented by its coordinates.
struct EstimatedModel {
    SystemMd model;
    double actual_rmse_m = 0.0;
};

/// Copy truth and perturb only horizontal user coordinates; rebuild all derived channel state.
/// The same standard field is rescaled at each d, without clipping, renormalizing, or changing IDs.
inline EstimatedModel make_estimate(const SystemMd& truth, const GaussianField& field, double d) {
    if (!std::isfinite(d) || d < 0 || field.size() != truth.users.size())
        throw invalid_argument("Invalid position field or horizontal RMSE");
    EstimatedModel estimated{truth, 0.0};
    const double scale = d / std::sqrt(2.0);
    double square_sum = 0.0;
    for (size_t i = 0; i < field.size(); ++i) {
        if (!std::isfinite(field[i][0]) || !std::isfinite(field[i][1]))
            throw invalid_argument("Non-finite standard position error");
        // Preserve the exact zero-error coordinates, including signed zeros.
        if (d != 0) {
            estimated.model.users[i].X += scale * field[i][0];
            estimated.model.users[i].Y += scale * field[i][1];
        }
        const double dx = estimated.model.users[i].X - truth.users[i].X;
        const double dy = estimated.model.users[i].Y - truth.users[i].Y;
        square_sum += dx * dx + dy * dy;
    }
    if (!std::isfinite(square_sum)) throw invalid_argument("Position error overflow");
    estimated.actual_rmse_m = field.empty() ? 0.0 : std::sqrt(square_sum / field.size());
    estimated.model.init_SystemModel();
    validate_allocation_model(estimated.model);
    return estimated;
}

/// A missing ratio is JSON null (an empty CSV cell), never an imputed zero.
inline json ratio(double numerator, double denominator) {
    if (!std::isfinite(numerator) || !std::isfinite(denominator) || denominator < 0)
        throw invalid_argument("Invalid ratio operands");
    if (denominator == 0) return nullptr;
    const double value = numerator / denominator;
    if (!std::isfinite(value)) throw invalid_argument("Ratio overflow");
    return value;
}

/// Original allocations evaluated in truth; bandwidth is consumed even when actual service fails.
struct TruthEvaluation {
    EXPResult estimated;
    EXPResult realized;
    int outside_coverage = 0;
    int hard_outside_coverage = 0;
    int shortfall_valid_n = 0;
    double shortfall_sum = 0.0;
    vector<json> hard_users;
};

/// Require a horizontal-only paired model, so truth evaluation cannot silently change QoS or UAVs.
inline void validate_pair(const SystemMd& truth, const SystemMd& estimated) {
    validate_allocation_model(truth);
    validate_allocation_model(estimated);
    require_allocation(truth.m == estimated.m && truth.n1 == estimated.n1 && truth.n2 == estimated.n2,
        "EXP5 truth/estimate dimensions differ");
    for (size_t i = 0; i < truth.users.size(); ++i) {
        const auto& a = truth.users[i]; const auto& b = estimated.users[i];
        require_allocation(a.ID == b.ID && a.uType == b.uType && a.weight == b.weight &&
            a.rMin == b.rMin && a.pOut == b.pOut && a.rData == b.rData && a.Z == b.Z &&
            a.B == b.B && a.BSub == b.BSub, "EXP5 changed non-position user parameters");
    }
    for (size_t k = 0; k < truth.uavs.size(); ++k) {
        const auto& a = truth.uavs[k]; const auto& b = estimated.uavs[k];
        require_allocation(a.ID == b.ID && a.X == b.X && a.Y == b.Y && a.Z == b.Z &&
            a.total_bandwidth == b.total_bandwidth && a.pTrans == b.pTrans,
            "EXP5 changed deployment or UAV parameters");
    }
}

/// Strictly validate the estimated allocation, then evaluate that frozen allocation on truth.
/// Truth-side coverage/rate violations are observations, not solver failures. No input is mutated.
inline TruthEvaluation evaluate_truth(const SystemMd& truth, const SystemMd& estimated,
    const AlgorithmRunResult& run) {
    if (!algorithm_status_valid(run.status))
        throw runtime_error("EXP5 refuses failed solver output: " + algorithm_status_name(run.status) +
            (run.diagnostics.events.empty() ? "" : " / " + run.diagnostics.events.back()));
    validate_pair(truth, estimated);
    TruthEvaluation output;
    output.estimated = compute_single_EXPResult(estimated, run.allocation.first, run.duration_ms);
    for (auto member : EXP_METRICS)
        require_allocation(allocation_near(output.estimated.*member, run.metrics.*member),
            "EXP5 estimated metrics disagree with strict evaluator");
    output.realized.duration = run.duration_ms;
    for (int i = 0; i < truth.n1; ++i) {
        const auto& user = truth.users[i];
        output.hard_users.push_back({{"user_id", i}, {"admitted", false}, {"uav_id", -1},
            {"bandwidth_khz", 0.0}, {"weight", user.weight}, {"minimum_rate_kbps", user.rMin},
            {"target_outage", user.pOut}, {"true_x_m", user.X}, {"true_y_m", user.Y},
            {"estimated_x_m", estimated.users[i].X}, {"estimated_y_m", estimated.users[i].Y},
            {"estimated_covered", nullptr}, {"true_covered", nullptr},
            {"estimated_capacity_bps_hz", nullptr}, {"true_capacity_bps_hz", nullptr},
            {"estimated_reliable_rate_kbps", nullptr}, {"true_reliable_rate_kbps", nullptr},
            {"true_qos_met", nullptr}, {"relative_shortfall", nullptr}, {"reason", "NOT_ADMITTED"}});
    }
    for (const auto& allocation : run.allocation.first) {
        const int k = allocation.uav_id;
        EXPResult local; // Retain the same per-UAV summation order as compute_single_EXPResult.
        for (int i : allocation.allocatedList) {
            const auto& user = truth.users[i];
            const double bandwidth = allocation.allocatedBandwidth.at(i), capacity = truth.cap_list[k][i];
            const bool covered = truth.dis_list[k][i] <= max_coverage_distance;
            const double rate = covered ? bandwidth * capacity : 0.0;
            require_allocation(std::isfinite(rate), "EXP5 non-finite realized rate");
            if (!covered) ++output.outside_coverage;
            if (user.uType == HARD_UTILITY) {
                const bool met = covered && hard_qos_satisfied(bandwidth, capacity, user.rMin);
                local.hard_bandwidth += bandwidth; // Never reclaim a violated user's resources.
                if (met) {
                    ++output.realized.hard_num;
                    local.hard_utility += user.weight * std::log2(1.0 + user.rMin);
                    output.realized.hard_throughput += user.rMin;
                }
                if (!covered) ++output.hard_outside_coverage;
                auto& detail = output.hard_users.at(i);
                detail["admitted"] = true; detail["uav_id"] = k; detail["bandwidth_khz"] = bandwidth;
                detail["estimated_covered"] = true; detail["true_covered"] = covered;
                detail["estimated_capacity_bps_hz"] = estimated.cap_list[k][i];
                detail["true_capacity_bps_hz"] = capacity;
                detail["estimated_reliable_rate_kbps"] = bandwidth * estimated.cap_list[k][i];
                detail["true_reliable_rate_kbps"] = rate; detail["true_qos_met"] = met;
                detail["reason"] = met ? "MET" : (!covered ? "OUTSIDE_COVERAGE" :
                    (capacity <= 0 ? "NO_RELIABLE_CAPACITY" : "RATE_SHORTFALL"));
                if (user.rMin > 0) {
                    const double shortage = std::max(0.0, 1.0 - rate / user.rMin);
                    detail["relative_shortfall"] = shortage;
                    output.shortfall_sum += shortage;
                    ++output.shortfall_valid_n;
                }
            } else {
                local.elastic_bandwidth += bandwidth;
                if (covered && capacity > 0) {
                    ++output.realized.elastic_num;
                    local.elastic_utility += user.elastic_utility(bandwidth, capacity);
                    output.realized.elastic_throughput += rate;
                }
            }
        }
        output.realized.hard_utility += local.hard_utility;
        output.realized.elastic_utility += local.elastic_utility;
        output.realized.hard_bandwidth += local.hard_bandwidth;
        output.realized.elastic_bandwidth += local.elastic_bandwidth;
    }
    auto& actual = output.realized;
    actual.total_num = actual.hard_num + actual.elastic_num;
    actual.total_utility = actual.hard_utility + actual.elastic_utility;
    actual.total_throughput = actual.hard_throughput + actual.elastic_throughput;
    for (auto member : EXP_METRICS)
        require_allocation(std::isfinite(actual.*member) && actual.*member >= 0, "Invalid truth metric");
    return output;
}

/// Immutable unit identity shared by both CSVs; rep=0 denotes the unique zero-error baseline.
inline json identity(int instance, int rmse, int repetition, size_t method) {
    return {{"instance_id", instance}, {"rmse_m", rmse}, {"error_rep", repetition},
        {"method", method_name_list.at(method)}};
}

/// Metric column names, excluding duration (elapsed_ms already records it once).
inline vector<string> metric_columns() {
    auto columns = csv_fields(METRIC_CSV_HEADER);
    columns.erase(columns.begin());
    return columns;
}

/// Stable summary schema; unlike legacy parsers, utility_loss accepts negative numbers and NA.
inline vector<string> result_columns() {
    vector<string> columns = {"instance_id", "rmse_m", "error_rep", "method", "seed", "error_seed",
        "status", "diagnostics", "actual_rmse_m", "elapsed_ms", "baseline_utility", "utility_loss"};
    for (const string prefix : {"estimated_", "realized_"})
        for (const auto& name : metric_columns()) columns.push_back(prefix + name);
    const vector<string> extra = {"hard_total", "hard_admitted", "hard_violated", "hard_service_ratio",
        "hard_admission_violation_ratio", "outside_coverage", "hard_outside_coverage",
        "shortfall_valid_n", "hard_mean_shortfall"};
    columns.insert(columns.end(), extra.begin(), extra.end());
    return columns;
}

/// Stable all-Hard detail schema; association-dependent cells are empty for unadmitted users.
inline vector<string> hard_columns() {
    return {"instance_id", "rmse_m", "error_rep", "method", "user_id", "admitted", "uav_id",
        "bandwidth_khz", "weight", "minimum_rate_kbps", "target_outage", "true_x_m", "true_y_m",
        "estimated_x_m", "estimated_y_m", "estimated_covered", "true_covered",
        "estimated_capacity_bps_hz", "true_capacity_bps_hz",
        "estimated_reliable_rate_kbps", "true_reliable_rate_kbps", "true_qos_met", "relative_shortfall", "reason"};
}

/// Flatten a successful call and its truth evaluation into one complete summary, retaining negative loss.
inline json make_result(const json& key, const Options& options, size_t method,
    const EstimatedModel& estimate, const AlgorithmRunResult& run, const TruthEvaluation& truth, double baseline) {
    json result = key;
    result["seed"] = method_seed(options, key.at("instance_id"), method);
    result["error_seed"] = position_seed(options, key.at("instance_id"), key.at("error_rep"));
    result["status"] = algorithm_status_name(run.status);
    result["diagnostics"] = json({{"used_fallback", run.diagnostics.used_fallback},
        {"events", run.diagnostics.events}}).dump();
    result["actual_rmse_m"] = estimate.actual_rmse_m;
    result["elapsed_ms"] = run.duration_ms;
    result["baseline_utility"] = baseline;
    result["utility_loss"] = baseline == 0 ? json(nullptr) : json(1.0 - truth.realized.total_utility / baseline);
    const auto names = metric_columns();
    for (size_t i = 0; i < names.size(); ++i) {
        result["estimated_" + names[i]] = truth.estimated.*EXP_METRICS[i + 1];
        result["realized_" + names[i]] = truth.realized.*EXP_METRICS[i + 1];
    }
    const double violated = truth.estimated.hard_num - truth.realized.hard_num;
    result["hard_total"] = truth.hard_users.size(); result["hard_admitted"] = truth.estimated.hard_num;
    result["hard_violated"] = violated;
    result["hard_service_ratio"] = ratio(truth.realized.hard_num, static_cast<double>(truth.hard_users.size()));
    result["hard_admission_violation_ratio"] = ratio(violated, truth.estimated.hard_num);
    result["outside_coverage"] = truth.outside_coverage;
    result["hard_outside_coverage"] = truth.hard_outside_coverage;
    result["shortfall_valid_n"] = truth.shortfall_valid_n;
    result["hard_mean_shortfall"] = ratio(truth.shortfall_sum, truth.shortfall_valid_n);
    return result;
}

/// Add the immutable case identity to every Hard row, including unadmitted users.
inline vector<json> identified_hard_rows(const json& key, const TruthEvaluation& evaluation) {
    auto rows = evaluation.hard_users;
    for (auto& row : rows) row.update(key);
    return rows;
}

/// Return a flat CSV cell; strings are quoted and null is an empty cell, not textual zero.
inline string cell(const json& value) {
    if (value.is_null()) return "";
    if (value.is_string()) return csv_quote(value.get<string>());
    if (!(value.is_boolean() || value.is_number()) ||
        (value.is_number() && !std::isfinite(value.get<double>())))
        throw invalid_argument("EXP5 CSV requires finite flat scalar values");
    return value.dump();
}

/// Serialize complete rows with explicit columns, round-trip precision and a final newline.
inline string csv_text(const vector<string>& columns, const vector<json>& rows) {
    ostringstream output;
    for (size_t i = 0; i < columns.size(); ++i) output << (i ? "," : "") << columns[i];
    output << '\n';
    for (const auto& row : rows) {
        for (size_t i = 0; i < columns.size(); ++i) output << (i ? "," : "") << cell(row.at(columns[i]));
        output << '\n';
    }
    return output.str();
}

/// Strictly parse this module's CSV schema; incomplete lines, malformed values and headers are errors.
inline vector<json> read_csv(const fs::path& path, const vector<string>& columns) {
    ifstream input(path, ios::binary);
    string line;
    if (!input || !getline(input, line) || input.eof() || csv_fields(line) != columns)
        throw runtime_error("EXP5 invalid/missing CSV header: " + path.string());
    const set<string> text_columns = {"method", "status", "diagnostics", "reason", "metric"};
    vector<json> rows;
    while (getline(input, line)) {
        if (input.eof()) throw runtime_error("EXP5 unterminated CSV row: " + path.string());
        const auto fields = csv_fields(line);
        if (fields.size() != columns.size()) throw runtime_error("EXP5 CSV column count: " + path.string());
        json row = json::object();
        for (size_t i = 0; i < columns.size(); ++i) {
            const auto& name = columns[i];
            if (text_columns.count(name)) row[name] = fields[i];
            else row[name] = fields[i].empty() ? json(nullptr) : json::parse(fields[i]);
            (void)cell(row[name]); // Reject arrays, objects, and numeric overflow.
        }
        rows.push_back(std::move(row));
    }
    if (input.bad()) throw runtime_error("EXP5 CSV read failed: " + path.string());
    return rows;
}

/// Write one explicit file through a same-directory temporary and rename after close; never delete.
/// Interrupted temporary files remain recoverable and never count as completed results.
inline void atomic_write(const fs::path& target, const string& contents) {
    static uint64_t sequence = 0; // Driver is deliberately single-process and serial.
    fs::create_directories(target.parent_path());
    fs::path temporary;
    do {
        temporary = fs::path(target.string() + ".tmp_" +
            to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" + to_string(++sequence));
    } while (fs::exists(temporary));
    ofstream output(temporary, ios::binary);
    if (!output) throw runtime_error("EXP5 cannot open temporary: " + temporary.string());
    output << contents;
    output.flush(); output.close();
    if (!output) throw runtime_error("EXP5 write/close failed; temporary retained: " + temporary.string());
    fs::rename(temporary, target);
}

/// Return the isolated directory for one network/RMSE/repetition/method tuple, never a legacy path.
inline fs::path case_directory(const fs::path& root, const json& key) {
    return root / "cases" / ("id_" + to_string(key.at("instance_id").get<int>())) /
        ("rmse_" + to_string(key.at("rmse_m").get<int>())) /
        ("rep_" + to_string(key.at("error_rep").get<int>())) / key.at("method").get<string>();
}

/// Require every persisted identity field to match the case being read or written.
inline void validate_identity(const json& row, const json& key) {
    for (auto entry = key.begin(); entry != key.end(); ++entry)
        if (!row.contains(entry.key()) || row.at(entry.key()) != entry.value())
            throw runtime_error("EXP5 checkpoint identity mismatch: " + entry.key());
}

/// Validate counts, rates, NA handling and all-Hard detail before accepting a completion checkpoint.
inline void validate_bundle(const json& result, const vector<json>& hard, const json& key, int hard_count) {
    validate_identity(result, key);
    (void)csv_text(result_columns(), {result});
    if (!algorithm_status_valid(parse_algorithm_status(result.at("status").get<string>())) ||
        hard.size() != static_cast<size_t>(hard_count) || result.at("hard_total") != hard_count)
        throw runtime_error("EXP5 incomplete/failed checkpoint");
    const auto diagnostics = json::parse(result.at("diagnostics").get<string>());
    (void)diagnostics.at("used_fallback").get<bool>();
    (void)diagnostics.at("events").get<vector<string>>();
    int admitted = 0, met = 0, outside = 0, short_n = 0;
    double shortage = 0, bandwidth = 0, utility = 0, throughput = 0;
    for (int i = 0; i < hard_count; ++i) {
        const auto& row = hard.at(i);
        validate_identity(row, key);
        if (row.at("user_id") != i) throw runtime_error("EXP5 missing/duplicate Hard ID");
        (void)csv_text(hard_columns(), {row});
        const double bw = row.at("bandwidth_khz").get<double>(), rmin = row.at("minimum_rate_kbps").get<double>();
        if (rmin < 0 || row.at("weight").get<double>() < 0 || row.at("target_outage").get<double>() <= 0 ||
            row.at("target_outage").get<double>() >= 1)
            throw runtime_error("EXP5 invalid Hard user parameters");
        if (!row.at("admitted").get<bool>()) {
            if (row.at("uav_id") != -1 || bw != 0 || row.at("reason") != "NOT_ADMITTED")
                throw runtime_error("EXP5 inconsistent unadmitted Hard row");
            for (const string name : {"estimated_covered", "true_covered", "estimated_reliable_rate_kbps",
                "true_reliable_rate_kbps", "true_qos_met", "relative_shortfall",
                "estimated_capacity_bps_hz", "true_capacity_bps_hz"})
                if (!row.at(name).is_null()) throw runtime_error("EXP5 unadmitted link metric must be NA");
            continue;
        }
        ++admitted; bandwidth += bw;
        if (bw <= 0 || row.at("uav_id").get<int>() < 0 || !row.at("estimated_covered").get<bool>())
            throw runtime_error("EXP5 invalid admitted Hard link");
        const bool covered = row.at("true_covered").get<bool>(), satisfied = row.at("true_qos_met").get<bool>();
        const double rate = row.at("true_reliable_rate_kbps").get<double>();
        const double true_cap = row.at("true_capacity_bps_hz").get<double>();
        const double estimated_cap = row.at("estimated_capacity_bps_hz").get<double>();
        const string reason = satisfied ? "MET" : (!covered ? "OUTSIDE_COVERAGE" :
            (true_cap <= 0 ? "NO_RELIABLE_CAPACITY" : "RATE_SHORTFALL"));
        if (true_cap < 0 || !hard_qos_satisfied(bw, estimated_cap, rmin) ||
            !allocation_near(row.at("estimated_reliable_rate_kbps"), bw * estimated_cap) ||
            !allocation_near(rate, covered ? bw * true_cap : 0.0) ||
            satisfied != (covered && hard_qos_satisfied(bw, true_cap, rmin)) || row.at("reason") != reason)
            throw runtime_error("EXP5 inconsistent Hard capacity/rate/QoS/reason");
        if (rate < 0 || (!covered && rate != 0) || (satisfied && !covered))
            throw runtime_error("EXP5 inconsistent truth coverage/rate");
        if (!covered) ++outside;
        if (satisfied) { ++met; utility += row.at("weight").get<double>() * std::log2(1.0 + rmin); throughput += rmin; }
        if (rmin > 0) {
            const double gap = std::max(0.0, 1.0 - rate / rmin);
            if (!row.at("relative_shortfall").is_number() || !allocation_near(row.at("relative_shortfall"), gap))
                throw runtime_error("EXP5 inconsistent Hard shortfall");
            shortage += gap; ++short_n;
        } else if (!row.at("relative_shortfall").is_null()) throw runtime_error("EXP5 zero-demand shortfall must be NA");
    }
    const json expected = {{"hard_admitted", admitted}, {"estimated_hard_num", admitted},
        {"hard_violated", admitted - met}, {"realized_hard_num", met}, {"hard_outside_coverage", outside},
        {"shortfall_valid_n", short_n}, {"estimated_hard_bandwidth", bandwidth},
        {"realized_hard_bandwidth", bandwidth}, {"realized_hard_utility", utility},
        {"realized_hard_throughput", throughput}, {"hard_service_ratio", ratio(met, hard_count)},
        {"hard_admission_violation_ratio", ratio(admitted - met, admitted)}, {"hard_mean_shortfall", ratio(shortage, short_n)}};
    for (auto it = expected.begin(); it != expected.end(); ++it) {
        const auto& actual = result.at(it.key());
        if (it.value().is_null() ? !actual.is_null() :
            (!actual.is_number() || !allocation_near(actual.get<double>(), it.value().get<double>())))
            throw runtime_error("EXP5 summary/Hard detail mismatch: " + it.key());
    }
    for (const string prefix : {"estimated_", "realized_"}) {
        for (const auto& name : metric_columns())
            if (!result.at(prefix + name).is_number() || result.at(prefix + name).get<double>() < 0)
                throw runtime_error("EXP5 invalid nonnegative metric: " + prefix + name);
        for (const string suffix : {"num", "utility", "throughput"})
            if (!allocation_near(result.at(prefix + "total_" + suffix).get<double>(),
                result.at(prefix + "hard_" + suffix).get<double>() + result.at(prefix + "elastic_" + suffix).get<double>()))
                throw runtime_error("EXP5 inconsistent total metric");
    }
    const double baseline = result.at("baseline_utility").get<double>();
    if (result.at("elapsed_ms").get<double>() < 0 || result.at("actual_rmse_m").get<double>() < 0 ||
        result.at("outside_coverage").get<double>() < outside ||
        result.at("outside_coverage").get<double>() > result.at("estimated_total_num").get<double>() ||
        !allocation_near(result.at("estimated_elastic_bandwidth"), result.at("realized_elastic_bandwidth")))
        throw runtime_error("EXP5 invalid elapsed/error/coverage/resource metrics");
    const json expected_loss = baseline == 0 ? json(nullptr) :
        json(1.0 - result.at("realized_total_utility").get<double>() / baseline);
    if (baseline < 0 || (expected_loss.is_null() ? !result.at("utility_loss").is_null() :
        (!result.at("utility_loss").is_number() || !allocation_near(result.at("utility_loss"), expected_loss))))
        throw runtime_error("EXP5 invalid baseline/loss");
    if (key.at("rmse_m") == 0) {
        if (!allocation_near(baseline, result.at("realized_total_utility")))
            throw runtime_error("EXP5 zero-error reference is not this method's baseline");
        if (admitted != met || result.at("outside_coverage") != 0 || result.at("actual_rmse_m") != 0)
            throw runtime_error("EXP5 zero-error baseline has a truth violation");
        for (const auto& name : metric_columns())
            if (!allocation_near(result.at("estimated_" + name), result.at("realized_" + name)))
                throw runtime_error("EXP5 zero-error metrics differ: " + name);
    }
}

/// Read a complete two-file unit; absent partners are unfinished, malformed committed data stop the run.
inline std::optional<json> read_case(const fs::path& directory, const json& key, int hard_count) {
    if (!fs::exists(directory / "result.csv") || !fs::exists(directory / "hard_users.csv")) return std::nullopt;
    const auto rows = read_csv(directory / "result.csv", result_columns());
    if (rows.size() != 1) throw runtime_error("EXP5 unit requires one result row: " + directory.string());
    validate_bundle(rows.front(), read_csv(directory / "hard_users.csv", hard_columns()), key, hard_count);
    return rows.front();
}

/// Publish a checked pair: detail first, summary last. Only incomplete units may be replaced on resume.
inline void write_case(const fs::path& directory, const json& key, const json& result,
    const vector<json>& hard, int hard_count) {
    validate_bundle(result, hard, key, hard_count);
    if (read_case(directory, key, hard_count)) throw runtime_error("EXP5 refuses to overwrite a complete unit");
    // A summary with a missing partner is not committed. Preserve it under a unique name before
    // publishing new detail, so a second interruption cannot pair an old summary with new detail.
    if (fs::exists(directory / "result.csv")) {
        fs::path retained;
        do {
            retained = directory / ("result.csv.incomplete_" +
                to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        } while (fs::exists(retained));
        fs::rename(directory / "result.csv", retained);
    }
    atomic_write(directory / "hard_users.csv", csv_text(hard_columns(), hard));
    atomic_write(directory / "result.csv", csv_text(result_columns(), {result}));
}

/// Record cheap input identity without copying data or claiming a content-hash verification.
inline json input_identity(const string& path) {
    if (!fs::is_regular_file(path)) throw runtime_error("EXP5 input missing: " + path);
    return {{"path", experiment_absolute_path(path)}, {"size", fs::file_size(path)},
        {"last_write_ticks", fs::last_write_time(path).time_since_epoch().count()}};
}

/// Build immutable run settings using the actual EXP1 physical configuration and selected input metadata.
inline json run_settings(const Options& options, const ExperimentCondition& condition, const string& config) {
    const auto run_options = algorithm_options(options);
    json settings = make_experiment_run_info("EXP5_location_error", {condition}, config, run_options);
    settings.erase("summary"); settings["schema"] = VERSION;
    settings["rmse_m"] = options.rmse_m; settings["error_repeats"] = options.error_repeats;
    settings["error_master_seed"] = options.error_master_seed;
    settings["expected_calls"] = expected_calls(options);
    settings["position_model"] = "XY+=d/sqrt(2)*N(0,1); no clipping or renormalization; fixed UAVs; 3D coverage";
    settings["position_rng"] = "std::mt19937 + std::normal_distribution<double>; interleaved user X,Y";
    settings["position_seed"] = "derive_algorithm_seed(error_master_seed,EXP5_location_error,normal_rep_<rep>,id,position); rep0=0";
    settings["algorithm_seed"] = "derive_algorithm_seed(master_seed,EXP1_user_num,3000,id,method); invariant across errors";
    settings["units"] = {{"coordinates", "m"}, {"bandwidth", "kHz"}, {"rate", "Kbps"}, {"duration", "ms"}};
    settings["aggregation"] = "error-repeat mean within network, then equal-weight network mean; NA excluded with counts";
    const MatchingSQPConfig matching;
    settings["matching_defaults"] = {{"nu_init", matching.nu_init}, {"nu_step", matching.nu_step},
        {"nu_max", matching.nu_max}, {"conv_eps", matching.conv_eps}, {"max_outer_iter", matching.max_outer_iter},
        {"max_ipopt_iter", matching.max_ipopt_iter}, {"ipopt_tol", matching.ipopt_tol}};
    const SADAConfig sada;
    settings["sada_defaults"] = {{"m_init", sada.m_init}, {"m_scale", sada.m_scale}, {"m_max", sada.m_max},
        {"max_outer_iter", sada.max_outer_iter}, {"theta_tol", sada.theta_tol}, {"max_inner_iter", sada.max_inner_iter},
        {"alpha_init", sada.alpha_init}, {"mu_init", sada.mu_init}, {"dual_tol", sada.dual_tol},
        {"bisect_max_iter", sada.bisect_max_iter}, {"bisect_tol", sada.bisect_tol}, {"verbose", sada.verbose}};
    settings["solver_option_files"] = json::array();
    for (const string name : {"relax_rounding_ipopt.opt", "Matching_SQP_ipopt.opt", "SADA_ipopt.opt"}) {
        const fs::path path = fs::path(algProjPath) / name;
        ifstream input(path, ios::binary);
        if (!input) throw runtime_error("EXP5 cannot read solver options: " + path.string());
        ostringstream contents; contents << input.rdbuf();
        if (input.bad()) throw runtime_error("EXP5 solver options read failed: " + path.string());
        settings["solver_option_files"].push_back({{"path", experiment_absolute_path(path)}, {"contents", contents.str()}});
    }
    settings["input_files"] = json::array();
    for (size_t i = 0; i < condition.users.size(); ++i)
        settings["input_files"].push_back({{"id", i + 1}, {"users", input_identity(condition.users[i])},
            {"uavs", input_identity(condition.uavs[i])}});
#ifdef _MSC_FULL_VER
    settings["msc_full_ver"] = _MSC_FULL_VER;
#endif
#ifdef _ITERATOR_DEBUG_LEVEL
    settings["iterator_debug_level"] = _ITERATOR_DEBUG_LEVEL;
#endif
#ifdef _MSVC_STL_UPDATE
    settings["msvc_stl_update"] = _MSVC_STL_UPDATE;
#endif
    return settings;
}

/// Inspect settings without mutation; changed parameters/inputs/versions require a new output name.
inline void inspect_destination(const fs::path& root, const json& settings) {
    if (fs::exists(root / "run_info.json")) {
        if (read_experiment_json(root / "run_info.json").at("settings") != settings)
            throw runtime_error("EXP5 settings/input/version changed; choose a new output_name");
    } else if (fs::exists(root) && !fs::is_empty(root)) {
        // A first metadata write interrupted before rename leaves only unpublished temporary metadata.
        for (const auto& entry : fs::directory_iterator(root))
            if (!entry.is_regular_file() || entry.path().filename().string().rfind("run_info.json.tmp_", 0) != 0)
                throw runtime_error("EXP5 nonempty output lacks metadata; choose a new output_name");
    }
}

/// Average available numeric observations only; expose counts separately to avoid silently imputing NA.
inline json numeric_mean(const vector<json>& values) {
    double sum = 0; size_t count = 0;
    for (const auto& value : values) if (!value.is_null()) { sum += value.get<double>(); ++count; }
    return count ? json(sum / count) : json(nullptr);
}

/// Produce per-network and equal-network summaries only for an entirely complete experiment.
inline void export_summaries(const fs::path& root, const Options& options, const vector<json>& results) {
    if (results.size() != expected_calls(options)) throw runtime_error("EXP5 refuses incomplete summary");
    vector<string> metrics;
    for (const auto& name : result_columns())
        if (name != "instance_id" && name != "rmse_m" && name != "error_rep" && name != "method" &&
            name != "seed" && name != "error_seed" && name != "status" && name != "diagnostics") metrics.push_back(name);
    vector<json> network_rows, overall_rows;
    for (int rmse : options.rmse_m) for (const auto& method : method_name_list) for (const auto& metric : metrics) {
        vector<json> network_means;
        for (int id = 1; id <= options.instance_count; ++id) {
            vector<json> values; set<int> repeats;
            for (const auto& result : results)
                if (result.at("instance_id") == id && result.at("rmse_m") == rmse && result.at("method") == method) {
                    if (!repeats.insert(result.at("error_rep").get<int>()).second)
                        throw runtime_error("EXP5 duplicate summary unit");
                    values.push_back(result.at(metric));
                }
            const size_t expected = rmse == 0 ? 1 : options.error_repeats;
            if (values.size() != expected || (rmse == 0 ? !repeats.count(0) :
                (*repeats.begin() != 1 || *repeats.rbegin() != options.error_repeats)))
                throw runtime_error("EXP5 missing/mismatched error repetitions");
            size_t valid = 0; for (const auto& value : values) if (!value.is_null()) ++valid;
            const json mean = numeric_mean(values);
            network_rows.push_back({{"instance_id", id}, {"rmse_m", rmse}, {"method", method},
                {"metric", metric}, {"mean", mean}, {"valid_error_repeats", valid}, {"expected_error_repeats", expected}});
            network_means.push_back(mean);
        }
        size_t valid = 0; for (const auto& value : network_means) if (!value.is_null()) ++valid;
        overall_rows.push_back({{"rmse_m", rmse}, {"method", method}, {"metric", metric},
            {"mean", numeric_mean(network_means)}, {"valid_networks", valid}, {"expected_networks", options.instance_count}});
    }
    atomic_write(root / "raw_results.csv", csv_text(result_columns(), results));
    atomic_write(root / "network_means.csv", csv_text({"instance_id", "rmse_m", "method", "metric", "mean",
        "valid_error_repeats", "expected_error_repeats"}, network_rows));
    atomic_write(root / "overall_means.csv", csv_text({"rmse_m", "method", "metric", "mean",
        "valid_networks", "expected_networks"}, overall_rows));
}

/// Execute/resume only this explicit pilot call; all six solvers receive the same estimate and frozen truth.
/// Throws with a unit context on failure, retaining previous complete pairs; never starts EXP1--EXP4.
inline void run_pilot(const Options& options = Options{}) {
    validate_options(options);
    const auto run_options = algorithm_options(options);
    const fs::path inputs = resolve_experiment_input_root(run_options);
    const string directory = (inputs / "variable_user_num" / "3000u_num").string();
    const auto condition = select_experiment_condition("3000", directory, directory,
        "3000users_data", "10uavs_loc", 40.0, run_options);
    const string config = (fs::path(experimentDataPath) / "ExperimentsResults" / "EXP1_user_num" / "def_config.json").string();
    const fs::path root = fs::path(experimentDataPath) / "ExperimentsResults" / "EXP5_location_error" /
        TON_RESULT_VERSION_DIR / options.output_name;
    const json settings = run_settings(options, condition, config);
    inspect_destination(root, settings);
    vector<SystemMd> truths;
    // Preflight every selected input before writing metadata or starting an allocator.
    for (int id = 1; id <= options.instance_count; ++id) {
        SystemMd truth(condition.users[id - 1], condition.uavs[id - 1], config);
        if (truth.users.size() != 3000 || truth.uavs.size() != 10 || max_coverage_distance != 600)
            throw runtime_error("EXP5 requires 3000 users, 10 UAVs and the unchanged 600-m 3D coverage model");
        for (auto& uav : truth.uavs) uav.total_bandwidth = 40.0 * unit_para;
        truth.init_SystemModel(); validate_allocation_model(truth);
        truths.push_back(std::move(truth));
    }
    json metadata = {{"settings", settings}, {"summary", {{"status", "incomplete"}, {"completed_calls", 0}}}};
    atomic_write(root / "run_info.json", metadata.dump(2) + "\n");
    vector<json> results;
    for (int id = 1; id <= options.instance_count; ++id) {
        const auto& truth = truths[id - 1];
        vector<GaussianField> fields(1, GaussianField(truth.users.size(), {0.0, 0.0}));
        for (int rep = 1; rep <= options.error_repeats; ++rep)
            fields.push_back(gaussian_field(truth.users.size(), position_seed(options, id, rep)));
        vector<double> baselines(method_name_list.size(), 0.0);
        for (int rmse : options.rmse_m) {
            const int first = rmse == 0 ? 0 : 1, last = rmse == 0 ? 0 : options.error_repeats;
            for (int rep = first; rep <= last; ++rep) {
                const auto estimate = make_estimate(truth, fields[rep], rmse);
                for (size_t method = 0; method < method_name_list.size(); ++method) {
                    const auto key = identity(id, rmse, rep, method);
                    const auto destination = case_directory(root, key);
                    const string context = key.dump();
                    try {
                        auto saved = read_case(destination, key, truth.n1);
                        if (saved) {
                            if (saved->at("seed") != method_seed(options, id, method) ||
                                saved->at("error_seed") != position_seed(options, id, rep) ||
                                !allocation_near(saved->at("actual_rmse_m"), estimate.actual_rmse_m) ||
                                (rmse != 0 && !allocation_near(saved->at("baseline_utility"), baselines[method])))
                                throw runtime_error("EXP5 checkpoint seed/RMSE/baseline mismatch");
                            cout << "[EXP5 SKIP] " << context << std::endl;
                        } else {
                            cout << "[EXP5 RUN] " << context << std::endl;
                            const auto run = run_algorithm(estimate.model, method, run_options, method_seed(options, id, method));
                            const auto evaluation = evaluate_truth(truth, estimate.model, run);
                            const double baseline = rmse == 0 ? evaluation.realized.total_utility : baselines[method];
                            const auto result = make_result(key, options, method, estimate, run, evaluation, baseline);
                            write_case(destination, key, result, identified_hard_rows(key, evaluation), truth.n1);
                            saved = result;
                            cout << "[EXP5 SAVED] " << context << " / " << run.duration_ms << " ms / hard violations "
                                << result.at("hard_violated") << std::endl;
                        }
                        if (rmse == 0) baselines[method] = saved->at("realized_total_utility").get<double>();
                        results.push_back(*saved);
                    } catch (const std::exception& error) {
                        throw runtime_error("EXP5 " + context + ": " + error.what() + "; complete units retained; no failed row written");
                    }
                }
            }
        }
    }
    metadata["summary"] = {{"status", "writing"}, {"completed_calls", results.size()}};
    atomic_write(root / "run_info.json", metadata.dump(2) + "\n");
    export_summaries(root, options, results);
    metadata["summary"]["status"] = "complete";
    atomic_write(root / "run_info.json", metadata.dump(2) + "\n");
    cout << "EXP5 pilot complete: " << results.size() << " calls; descriptive results from "
        << options.instance_count << " networks. No automatic expansion." << std::endl;
}

} // namespace localization

/// Run/resume EXP5 with the common main.cpp settings; returns only after all units/summaries finish.
/// Failure throws to the caller; this wrapper never imports EXP1 metrics or edits pilot_01.
inline void exp5_different_location_error(const ExperimentRunOptions& options) {
    localization::run_pilot(localization::from_experiment_options(options));
}

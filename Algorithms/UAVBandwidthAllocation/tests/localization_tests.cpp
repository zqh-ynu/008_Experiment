// Standalone deterministic EXP5 validation. Uses tiny synthetic models and a caller-supplied
// output directory only; never calls run_pilot()/the formal rerun, or reads formal user CSVs.
// Include the temporary rerun helpers while excluding the production main() entry.
#define TON_RERUN_TESTING
#include "../main.cpp"

namespace {
int checks = 0;

/// Require a test condition without relying on assert (production uses NDEBUG); count successes.
void check(bool condition, const string& description) {
    if (!condition) throw runtime_error("TEST FAILED: " + description);
    ++checks;
}

/// Verify a scoped invalid operation throws; the supplied operation must target synthetic test data.
template<class Function> void expect_error(Function operation, const string& description) {
    bool threw = false;
    try { operation(); } catch (const std::exception&) { threw = true; }
    check(threw, description);
}

/// Return a small physical mixed-QoS instance; the third Hard user is outside the one-UAV coverage.
SystemMd fixture() {
    vector<User> users = {
        User(0, HARD_UTILITY, 3, 100, 0, 0, 32, 0.01),
        User(1, HARD_UTILITY, 4, 250, 50, 0, 80, 0.001),
        User(2, HARD_UTILITY, 1, 550, 0, 0, 32, 0.01),
        User(3, ELASTIC_UTILITY, 2, 150, 30, 0, 0, 0)
    };
    Uav uav(0, 0, 0, uav_alt, 40 * unit_para);
    uav.pTrans = uav_trans_power;
    return SystemMd(users, {uav});
}

/// Serialize all relevant model state in memory to check evaluation and copy isolation.
json model_state(const SystemMd& model) {
    json state = {{"n1", model.n1}, {"n2", model.n2}, {"m", model.m}, {"max", model.max_user_utility},
        {"distances", model.dis_list}, {"caps", model.cap_list}, {"snr_avg", model.SNRave_list},
        {"snr_th", model.SNRth_list}, {"nakagami", model.M_list}, {"thresholds", model.Bth_list},
        {"users", json::array()}, {"uavs", json::array()}, {"coverage", json::array()}};
    for (const auto& u : model.users)
        state["users"].push_back({u.ID, u.X, u.Y, u.Z, u.uType, u.weight, u.rMin, u.pOut, u.rData, u.B, u.BSub});
    for (const auto& u : model.uavs) state["uavs"].push_back({u.ID, u.X, u.Y, u.Z, u.total_bandwidth, u.pTrans});
    for (const auto& entry : model.uav_serviceable_users_map) {
        vector<int> ids; for (const auto& u : entry.second) ids.push_back(u.ID);
        state["coverage"].push_back({entry.first, ids});
    }
    return state;
}

/// Serialize allocation fields, including cached totals, to detect any mutation or hidden repair.
json allocation_state(const vector<KnapsackResult>& allocations) {
    json result = json::array();
    for (const auto& a : allocations)
        result.push_back({a.uav_id, a.allocatedList, a.allocatedBandwidth, a.allocatedValue,
            a.totalValue, a.totalWeight, a.hardValue, a.hardWeight, a.elasticValue, a.elasticWeight});
    return result;
}

/// Return a strictly checked handmade allocation admitting one Hard user; others remain unadmitted.
AlgorithmRunResult handmade(const SystemMd& estimated, int user_id = 0, bool elastic = false) {
    AlgorithmRunResult run;
    run.duration_ms = 1.25; run.status = AlgorithmRunStatus::Success;
    run.allocation.first.resize(estimated.uavs.size());
    for (size_t k = 0; k < estimated.uavs.size(); ++k) run.allocation.first[k].uav_id = static_cast<int>(k);
    auto& allocation = run.allocation.first[0];
    allocation.allocatedList = {user_id};
    allocation.allocatedBandwidth[user_id] = estimated.users[user_id].rMin / estimated.cap_list[0][user_id];
    if (elastic) { allocation.allocatedList.push_back(3); allocation.allocatedBandwidth[3] = 10.0; }
    run.metrics = compute_single_EXPResult(estimated, run.allocation.first, run.duration_ms);
    return run;
}

/// Exercise error generation, physical propagation, frozen evaluation, all-Hard rows, and NA cases.
json test_models_and_evaluation(const fs::path& root) {
    const auto truth = fixture();
    const auto original = model_state(truth);
    localization::Options options;
    check(localization::expected_calls(options) == 234, "approved matrix contains 234 calls");
    auto invalid = options; invalid.rmse_m = {0, 2, 2};
    expect_error([&] { localization::validate_options(invalid); }, "duplicate RMSE rejected");
    const auto field = localization::gaussian_field(truth.users.size(), localization::position_seed(options, 1, 1));
    check(field == localization::gaussian_field(truth.users.size(), localization::position_seed(options, 1, 1)), "reproducible normal field");
    check(field != localization::gaussian_field(truth.users.size(), localization::position_seed(options, 1, 2)), "distinct repetition field");
    const auto zero = localization::make_estimate(truth, field, 0);
    check(model_state(zero.model) == original && zero.actual_rmse_m == 0, "zero-error full state equality");
    const auto five = localization::make_estimate(truth, field, 5);
    const auto ten = localization::make_estimate(truth, field, 10);
    for (size_t i = 0; i < truth.users.size(); ++i) {
        check(std::abs((ten.model.users[i].X - truth.users[i].X) - 2 * (five.model.users[i].X - truth.users[i].X)) < 1e-10, "paired X scaling");
        check(std::abs((ten.model.users[i].Y - truth.users[i].Y) - 2 * (five.model.users[i].Y - truth.users[i].Y)) < 1e-10, "paired Y scaling");
    }
    check(allocation_near(ten.actual_rmse_m, 2 * five.actual_rmse_m), "empirical RMSE scales without renormalization");
    check(model_state(truth) == original, "generating estimates preserves truth");
    auto shifted = localization::GaussianField(truth.users.size(), {0.0, 0.0});
    shifted[0][0] = -1;
    const auto toward = localization::make_estimate(truth, shifted, std::sqrt(2.0));
    shifted[0][0] = 1;
    const auto away = localization::make_estimate(truth, shifted, std::sqrt(2.0));
    check(toward.model.dis_list[0][0] < truth.dis_list[0][0] && away.model.dis_list[0][0] > truth.dis_list[0][0], "radial geometry propagates");
    check(toward.model.cap_list[0][0] > truth.cap_list[0][0] && away.model.cap_list[0][0] < truth.cap_list[0][0], "physical reliable capacity propagates");
    shifted[0][0] = -2000;
    const auto outside_box = localization::make_estimate(truth, shifted, std::sqrt(2.0));
    check(outside_box.model.users[0].X < 0, "negative estimated coordinates are not clipped");
    check(truth.dis_list[0][2] > 600 && Point::cal_horizontal_distance(truth.uavs[0], truth.users[2]) < 600, "coverage remains 3D, not horizontal");

    const auto run = handmade(toward.model, 0, true);
    const auto allocation_before = allocation_state(run.allocation.first);
    const auto estimated_before = model_state(toward.model);
    const auto evaluation = localization::evaluate_truth(truth, toward.model, run);
    check(evaluation.realized.hard_num == 0 && evaluation.estimated.hard_num == 1, "small error keeps actual threshold violation");
    check(evaluation.hard_users[0].at("relative_shortfall").get<double>() > 0 &&
        evaluation.hard_users[0].at("relative_shortfall").get<double>() < 0.05, "small deficit measured without softening QoS");
    check(evaluation.hard_users[1].at("reason") == "NOT_ADMITTED" && evaluation.hard_users[1].at("true_qos_met").is_null(), "unadmitted is not a violation");
    check(evaluation.realized.hard_bandwidth == evaluation.estimated.hard_bandwidth, "violated user still consumes bandwidth");
    check(evaluation.realized.elastic_bandwidth == evaluation.estimated.elastic_bandwidth, "no redistribution to elastic");
    check(model_state(truth) == original && model_state(toward.model) == estimated_before &&
        allocation_state(run.allocation.first) == allocation_before, "evaluation freezes models and allocation");
    auto illegal = toward.model; illegal.users[0].weight += 1;
    expect_error([&] { localization::evaluate_truth(truth, illegal, run); }, "changed QoS/weight pairing rejected");

    auto crossing = truth; crossing.users[2].X = 500; crossing.init_SystemModel();
    const auto cross_run = handmade(crossing, 2);
    const auto cross = localization::evaluate_truth(truth, crossing, cross_run);
    check(cross.outside_coverage == 1 && cross.hard_outside_coverage == 1 &&
        cross.hard_users[2].at("relative_shortfall") == 1 && cross.hard_users[2].at("reason") == "OUTSIDE_COVERAGE", "false coverage gets separate full deficit");

    const auto key = localization::identity(1, 1, 1, 0);
    auto result = localization::make_result(key, options, 0, toward, run, evaluation, 100.0);
    auto hard = localization::identified_hard_rows(key, evaluation);
    const auto unit = root / "complete_unit";
    localization::write_case(unit, key, result, hard, truth.n1);
    check(localization::read_case(unit, key, truth.n1).value() == result, "complete pair round trip");
    expect_error([&] { localization::write_case(unit, key, result, hard, truth.n1); }, "complete unit cannot be overwritten");
    const auto partial = root / "partial_unit";
    localization::atomic_write(partial / "hard_users.csv", localization::csv_text(localization::hard_columns(), hard));
    check(!localization::read_case(partial, key, truth.n1), "detail without result is unfinished");
    localization::write_case(partial, key, result, hard, truth.n1);
    check(localization::read_case(partial, key, truth.n1).has_value(), "unfinished unit can be recomputed");
    const auto orphan = root / "orphan_summary";
    localization::atomic_write(orphan / "result.csv", localization::csv_text(localization::result_columns(), {result}));
    check(!localization::read_case(orphan, key, truth.n1), "summary without detail is unfinished");
    localization::write_case(orphan, key, result, hard, truth.n1);
    bool retained_summary = false;
    for (const auto& file : fs::directory_iterator(orphan))
        if (file.path().filename().string().rfind("result.csv.incomplete_", 0) == 0) retained_summary = true;
    check(retained_summary && localization::read_case(orphan, key, truth.n1).has_value(), "orphan marker retained before fresh pair publication");
    const auto truncated = root / "truncated_unit";
    localization::atomic_write(truncated / "result.csv", localization::csv_text(localization::result_columns(), {result}));
    localization::atomic_write(truncated / "hard_users.csv", "incomplete");
    expect_error([&] { localization::read_case(truncated, key, truth.n1); }, "truncated detail never counts as complete");
    auto failed = run; failed.status = AlgorithmRunStatus::SolverFailure;
    expect_error([&] { localization::evaluate_truth(truth, toward.model, failed); }, "solver failure rejected before result writing");
    auto invalid_run = run; invalid_run.allocation.first[0].allocatedBandwidth[0] *= 0.5;
    expect_error([&] { localization::evaluate_truth(truth, toward.model, invalid_run); }, "invalid estimated QoS remains a solver/allocation failure");
    localization::atomic_write(root / "not_a_directory", "test obstruction\n");
    expect_error([&] { localization::atomic_write(root / "not_a_directory" / "output.csv", "x"); }, "write failure throws");
    check(localization::read_case(unit, key, truth.n1).value() == result, "unrelated failure preserves complete data");
    auto negative = result;
    negative["baseline_utility"] = evaluation.realized.total_utility / 1.1;
    negative["utility_loss"] = 1.0 - evaluation.realized.total_utility / negative.at("baseline_utility").get<double>();
    localization::validate_bundle(negative, hard, key, truth.n1);
    localization::atomic_write(root / "negative.csv", localization::csv_text(localization::result_columns(), {negative}));
    check(localization::read_csv(root / "negative.csv", localization::result_columns())[0].at("utility_loss").get<double>() < 0, "negative loss survives serialization");

    SystemMd elastic_only({User(0, ELASTIC_UTILITY, 1, 100, 0, 0, 0, 0)}, truth.uavs);
    auto empty = AlgorithmRunResult{}; empty.status = AlgorithmRunStatus::ZeroAllocation;
    empty.allocation.first.resize(1); empty.allocation.first[0].uav_id = 0;
    empty.metrics = compute_single_EXPResult(elastic_only, empty.allocation.first, 0);
    const auto empty_eval = localization::evaluate_truth(elastic_only, elastic_only, empty);
    const auto empty_key = localization::identity(1, 0, 0, 0);
    const auto empty_result = localization::make_result(empty_key, options, 0,
        localization::EstimatedModel{elastic_only, 0}, empty, empty_eval, 0);
    localization::validate_bundle(empty_result, {}, empty_key, 0);
    check(empty_result.at("utility_loss").is_null() && empty_result.at("hard_service_ratio").is_null() &&
        empty_result.at("hard_admission_violation_ratio").is_null(), "zero baseline and denominators are NA");
    localization::write_case(root / "empty_hard", empty_key, empty_result, {}, 0);
    check(localization::read_case(root / "empty_hard", empty_key, 0).has_value(), "header-only Hard file is complete for no Hard users");
    return result;
}

/// Check two-stage equal-network means, NA counts, unique zero baselines, and metadata mismatch gates.
void test_aggregation(const fs::path& root, const json& prototype) {
    localization::Options options; options.instance_count = 2; options.rmse_m = {0, 5};
    vector<json> results;
    for (int id = 1; id <= 2; ++id) for (int rmse : options.rmse_m)
        for (int rep = rmse == 0 ? 0 : 1; rep <= (rmse == 0 ? 0 : 2); ++rep)
            for (size_t method = 0; method < method_name_list.size(); ++method) {
                json row = prototype; row.update(localization::identity(id, rmse, rep, method));
                // Aggregation-only synthetic values deliberately differ in available repeat counts.
                row["utility_loss"] = rmse == 0 ? json(0.0) : (id == 1 ? json(rep * 0.1) :
                    (rep == 1 ? json(0.8) : json(nullptr)));
                results.push_back(row);
            }
    localization::export_summaries(root / "aggregate", options, results);
    const auto overall = localization::read_csv(root / "aggregate" / "overall_means.csv",
        {"rmse_m", "method", "metric", "mean", "valid_networks", "expected_networks"});
    int matched = 0;
    for (const auto& row : overall) if (row.at("rmse_m") == 5 && row.at("metric") == "utility_loss") {
        check(allocation_near(row.at("mean"), 0.475) && row.at("valid_networks") == 2, "macro mean, not pooled repetitions");
        ++matched;
    }
    check(matched == 6, "six-method aggregate coverage");
    const auto network = localization::read_csv(root / "aggregate" / "network_means.csv",
        {"instance_id", "rmse_m", "method", "metric", "mean", "valid_error_repeats", "expected_error_repeats"});
    for (const auto& row : network) {
        if (row.at("rmse_m") == 0) check(row.at("expected_error_repeats") == 1, "zero baseline not duplicated");
        if (row.at("instance_id") == 2 && row.at("rmse_m") == 5 && row.at("metric") == "utility_loss")
            check(row.at("valid_error_repeats") == 1 && row.at("expected_error_repeats") == 2, "NA repeat count reported");
    }
    results.pop_back();
    expect_error([&] { localization::export_summaries(root / "incomplete", options, results); }, "incomplete experiment cannot publish summary");
    const auto metadata_root = root / "metadata";
    localization::inspect_destination(metadata_root, {{"version", 1}});
    localization::atomic_write(metadata_root / "run_info.json", json({{"settings", {{"version", 1}}}}).dump() + "\n");
    localization::inspect_destination(metadata_root, {{"version", 1}});
    expect_error([&] { localization::inspect_destination(metadata_root, {{"version", 2}}); }, "metadata/version drift rejected");
    const auto synthetic_users = root / "metadata_inputs" / "1_users.csv";
    const auto synthetic_uavs = root / "metadata_inputs" / "1_uavs.csv";
    localization::atomic_write(synthetic_users, "synthetic identity only\n");
    localization::atomic_write(synthetic_uavs, "synthetic identity only\n");
    localization::Options one; one.instance_count = 1;
    const ExperimentCondition condition{"3000", {synthetic_users.string()}, {synthetic_uavs.string()}, 40};
    const string config = (fs::path(experimentDataPath) / "ExperimentsResults" / "EXP1_user_num" / "def_config.json").string();
    const auto settings = localization::run_settings(one, condition, config);
    check(settings.at("solver_option_files").size() == 3 && settings.at("matching_defaults").at("max_outer_iter") == MatchingSQPConfig().max_outer_iter,
        "actual solver option files and default parameters recorded");
    check(settings.at("channel_config") == read_experiment_json(config) &&
        settings.at("input_files").at(0).at("id") == 1, "EXP1 physical config and input identities recorded");
}

/// Run each real allocator only on the tiny synthetic zero-error model, twice in opposite order.
void test_six_zero_error_methods() {
    const auto truth = fixture(); const auto original = model_state(truth);
    localization::Options options;
    const auto settings = localization::algorithm_options(options);
    const auto field = localization::gaussian_field(truth.users.size(), localization::position_seed(options, 1, 1));
    const auto estimate = localization::make_estimate(truth, field, 0);
    vector<json> allocations(6);
    for (int pass = 0; pass < 2; ++pass) for (size_t step = 0; step < 6; ++step) {
        const size_t method = pass == 0 ? step : 5 - step;
        const auto run = run_algorithm(estimate.model, method, settings, localization::method_seed(options, 1, method));
        if (!algorithm_status_valid(run.status))
            throw runtime_error("Synthetic solver failure / " + method_name_list[method] + " / " +
                (run.diagnostics.events.empty() ? "no diagnostic" : run.diagnostics.events.back()));
        const auto evaluation = localization::evaluate_truth(truth, estimate.model, run);
        const auto key = localization::identity(1, 0, 0, method);
        const auto result = localization::make_result(key, options, method, estimate, run, evaluation, evaluation.realized.total_utility);
        localization::validate_bundle(result, localization::identified_hard_rows(key, evaluation), key, truth.n1);
        check(result.at("hard_violated") == 0 && result.at("outside_coverage") == 0, "six-method zero-error truth consistency");
        if (pass == 0) allocations[method] = allocation_state(run.allocation.first);
        else check(allocations[method] == allocation_state(run.allocation.first), "method-order independent allocation");
        check(model_state(truth) == original && model_state(estimate.model) == original, "six solvers receive unchanged shared input");
        check(field == localization::gaussian_field(truth.users.size(), localization::position_seed(options, 1, 1)), "solver RNG cannot alter position RNG");
    }
}
/// Check wrapper mapping and real dispatch against selectors 3/1 on the tiny physical fixture only.
void test_latest_dispatch_and_wrapper() {
    ExperimentRunOptions common;
    common.instance_count = 10; common.output_name = "run_ton_01";
    const auto options = localization::from_experiment_options(common);
    check(localization::expected_calls(options) == 780, "ten networks contain 780 EXP5 units");
    check(localization::expected_calls(localization::Options{}) == 234, "legacy pilot defaults retained");
    check(options.instance_count == common.instance_count && options.algorithm_master_seed == common.master_seed &&
        options.rounding_trials == common.rounding_trials && options.epsilon == common.ton_epsilon &&
        options.input_root == common.input_root && options.output_name == common.output_name &&
        options.error_master_seed == 20260908u && options.error_repeats == 2, "all wrapper fields mapped");
    auto altered = common; altered.conditions = {"3000"};
    expect_error([&] { localization::from_experiment_options(altered); }, "EXP1 condition filter not accepted by EXP5");
    altered = common; altered.reuse_exp1_root = "old";
    expect_error([&] { localization::from_experiment_options(altered); }, "EXP5 cannot import old results");
    check(proposed_algorithm_policy().at("ApproBetter").at("selector") == 3 &&
        proposed_algorithm_policy().at("ApproFast").at("selector") == 1, "version metadata matches new dispatch");
    const auto model = fixture();
    for (size_t method = 0; method < 2; ++method) {
        auto routed = run_algorithm(model, method, common, 12345);
        auto direct = execute_allocation(model, 12345, [&](BAProblem& problem, AllocationDiagnostics&) {
            return problem.Appro_multiUAV_ToN(model.uavs, model.users, method == 0 ? 3 : 1, common.ton_epsilon);
        });
        check(algorithm_status_valid(routed.status) && routed.status == direct.status, "direct/routed dispatch succeeds");
        check(allocation_state(routed.allocation.first) == allocation_state(direct.allocation.first),
            "routed allocation exactly matches the selected current kernel");
        routed.metrics.duration = direct.metrics.duration = 0;
        check(expResultToCSVLine(routed.metrics) == expResultToCSVLine(direct.metrics),
            "all non-timing routed metrics match direct dispatch");
    }
    check(proposed_rerun::sha256("abc") ==
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        "SHA-256 matches published abc test vector");
}

/// Return a scalar-only synthetic successful record; no allocator or physical input file is involved.
AlgorithmRunResult rerun_row(const proposed_rerun::Run& run, size_t c, int id, size_t method, double value) {
    AlgorithmRunResult row;
    row.instance_id = id;
    row.seed = derive_algorithm_seed(run.options.master_seed, run.expected.at("experiment").get<string>(),
        run.expected.at("conditions").at(c).at("key").get<string>(), id, method_name_list.at(method));
    row.status = AlgorithmRunStatus::Success;
    row.duration_ms = row.metrics.duration = value;
    row.metrics.total_num = row.metrics.hard_num = 1;
    row.metrics.total_utility = row.metrics.hard_utility = value;
    row.metrics.hard_bandwidth = 1;
    row.metrics.total_throughput = row.metrics.hard_throughput = 1;
    return row;
}

/// Create one isolated two-ID/one-condition old run; input files are identity stubs, never loaded as models.
proposed_rerun::Run rerun_fixture(const fs::path& directory) {
    using namespace proposed_rerun;
    ExperimentRunOptions options; options.instance_count = 2; options.output_name = "synthetic";
    options.input_root = (directory / "inputs").string();
    ExperimentCondition condition{"1000", {}, {}, 40};
    for (int id = 1; id <= 2; ++id) {
        const fs::path user = directory / "inputs" / (to_string(id) + "_users.csv");
        const fs::path uav = directory / "inputs" / (to_string(id) + "_uavs.csv");
        atomic_save(user, "synthetic identity only\n"); atomic_save(uav, "synthetic identity only\n");
        condition.users.push_back(user.string()); condition.uavs.push_back(uav.string());
    }
    const string config = (fs::path(experimentDataPath) / "ExperimentsResults" / "EXP1_user_num" / "def_config.json").string();
    json expected = make_experiment_run_info("EXP1_user_num", {condition}, config, options);
    expected["summary"] = {{"status", "complete"}, {"instance_count", 2}};
    Run run{directory / "formal", {}, options, expected};
    run.source.emplace_back(6);
    for (size_t m = 0; m < 6; ++m) {
        for (int id = 1; id <= 2; ++id) run.source[0][m].emplace(id, rerun_row(run, 0, id, m, 10 + m + id));
        atomic_save(run.root / raw_name(run, 0, m), records_text(run.source[0][m]));
    }
    // Use the pre-existing public exporter so the temporary serializer is checked independently.
    vector<EXPResult> means;
    for (const auto& method : run.source[0]) means.push_back(average_valid_attempts(method, 2));
    exportAllSummaryCSV((run.root / "summary").string(), {"1000"}, {means});
    auto old = expected; old.erase("proposed_algorithms"); old["algorithm_version"] = SOURCE_VERSION;
    atomic_save(run.root / "run_info.json", old.dump(2) + "\n");
    return inspect(run.root, expected, options);
}

/// Exercise backup interruption, saved-unit resume, partial publication, final-metadata interruption and idempotence.
void test_rerun_recovery(const fs::path& root) {
    using namespace proposed_rerun;
    auto run = rerun_fixture(root / "ok");
    check(!fs::exists(run.side), "preflight creates no rerun directory");
    {
        SequenceLock lock(run.root);
        expect_error([&] { SequenceLock second(run.root); }, "second manual sequence is refused");
    }
    { SequenceLock reacquired(run.root); }
    int backed = 0;
    expect_error([&] { backup(run, [&](const string&) { if (++backed == 2) throw runtime_error("injected backup exit"); }); },
        "backup interruption stops before calculation");
    check(backed == 2, "backup test reached the injected interruption, not an unrelated I/O error");
    const auto saved_backup = run.side / "backup" / original_names(run)[1];
    const auto backup_time = fs::last_write_time(saved_backup);
    run = inspect(run.root, run.expected, run.options);
    backup(run);
    check(fs::last_write_time(saved_backup) == backup_time, "existing backup not rewritten on resume");
    int calls = 0, saved = 0;
    const Evaluate solver = [&](size_t c, int id, size_t m) {
        check(m < 2, "only two proposed methods evaluated");
        ++calls; return rerun_row(run, c, id, m, 100 + id + m);
    };
    expect_error([&] { perform(run, solver, [&](const string& event) {
        if (event.rfind("saved:", 0) == 0 && ++saved == 3) throw runtime_error("injected checkpoint exit");
    }); }, "per-instance interruption is recoverable");
    check(saved == 3 && calls == 3 && bytes(run.root / raw_name(run, 0, 0)) == run.originals.at(raw_name(run, 0, 0)),
        "old raw files stay unchanged while new checkpoints accumulate");
    check(read_experiment_json(run.root / "run_info.json").at("summary").at("status") == "stale",
        "formal summary is stale during new computation");
    run = inspect(run.root, run.expected, run.options);
    int published = 0;
    expect_error([&] { perform(run, solver, [&](const string& event) {
        if (event.rfind("published:", 0) == 0 && ++published == 1) throw runtime_error("injected publish exit");
    }); }, "publication interruption leaves recoverable old/new targets");
    check(published == 1 && calls == 4, "restart computed only one missing unit and reached publication");
    run = inspect(run.root, run.expected, run.options);
    const Evaluate no_solver = [&](size_t, int, size_t) -> AlgorithmRunResult {
        throw runtime_error("unexpected solver after all checkpoints complete");
    };
    expect_error([&] { perform(run, no_solver, [&](const string& event) {
        if (event == "metadata_published") throw runtime_error("injected final state exit");
    }); }, "final metadata/state gap is recoverable");
    check(read_experiment_json(run.root / "run_info.json") == run.expected,
        "final metadata test reached its intended injection boundary");
    run = inspect(run.root, run.expected, run.options);
    perform(run, no_solver);
    const auto formal_time = fs::last_write_time(run.root / "run_info.json");
    run = inspect(run.root, run.expected, run.options);
    perform(run, no_solver);
    check(run.manifest.at("phase") == "complete" && calls == 4, "complete rerun never computes again");
    check(fs::last_write_time(run.root / "run_info.json") == formal_time, "complete reentry does not rewrite metadata");
    for (const auto& item : run.originals)
        check(bytes(run.side / "backup" / item.first) == item.second, "every backup remains byte-identical");
    verify_baselines(run);
    check(read_experiment_json(run.root / "run_info.json") == run.expected, "canonical new metadata published last");
    check(inspect_experiment_destination(run.root, run.expected).at("algorithm_version") == SIMPLE_ALGORITHM_VERSION,
        "normal strict run-info gate accepts the fully migrated run");
}

/// Inject malformed records, drift, baseline edits, solver/output failures and foreign target contents in fresh fixtures.
void test_rerun_rejections(const fs::path& root) {
    using namespace proposed_rerun;
    for (const string scenario : {"duplicate", "truncated", "seed", "version", "parameters", "input",
        "baseline", "backup", "target", "candidate", "write", "solver"}) {
        auto run = rerun_fixture(root / scenario);
        backup(run);
        const string raw = raw_name(run, 0, 0);
        const auto checkpoint = run.side / "new_results" / raw;
        auto row = rerun_row(run, 0, 1, 0, 100);
        if (scenario == "duplicate") atomic_save(checkpoint, CSV_HEADER + "\n" +
            run_result_to_csv(row) + "\n" + run_result_to_csv(row) + "\n");
        if (scenario == "truncated") atomic_save(checkpoint, CSV_HEADER + "\n" + run_result_to_csv(row));
        if (scenario == "seed") { ++row.seed; atomic_save(checkpoint, CSV_HEADER + "\n" + run_result_to_csv(row) + "\n"); }
        if (scenario == "version") run.expected["algorithm_version"] = "different";
        if (scenario == "parameters") { run.options.ton_epsilon = 0.2; run.expected["ton_epsilon"] = 0.2; }
        if (scenario == "input") atomic_save(run.expected.at("conditions")[0].at("inputs")[0].at("user_path").get<string>(), "changed input\n");
        if (scenario == "baseline") atomic_save(run.root / raw_name(run, 0, 2), "changed baseline\n");
        if (scenario == "backup") atomic_save(run.side / "backup" / raw, "changed backup\n");
        if (scenario == "target") atomic_save(run.root / raw, "unrelated replacement\n");
        const Evaluate solver = [&](size_t c, int id, size_t m) { return rerun_row(run, c, id, m, 100 + id + m); };
        if (scenario == "candidate") {
            expect_error([&] { perform(run, solver, [&](const string& event) {
                if (event == "publishing") throw runtime_error("injected");
            }); }, "candidate fixture stops before publication");
            atomic_save(run.side / "candidate" / raw, "conflicting candidate\n");
        }
        if (scenario == "write" || scenario == "solver") {
            if (scenario == "write") atomic_save(run.side / "new_results", "file obstructs checkpoint directory\n");
            bool next_stage = false;
            expect_error([&] {
                perform(run, [&](size_t c, int id, size_t m) {
                    auto value = solver(c, id, m);
                    if (scenario == "solver") value.status = AlgorithmRunStatus::SolverFailure;
                    return value;
                });
                next_stage = true;
            }, "write/solver failure propagates without retry");
            check(!next_stage && bytes(run.root / raw) == run.originals.at(raw),
                "failure cannot enter EXP5 or overwrite old raw data");
            check(!fs::exists(checkpoint), "failed call has no completed performance row");
        } else expect_error([&] { inspect(run.root, run.expected, run.options); }, "reject " + scenario);
    }
    auto first = rerun_fixture(root / "pre1");
    auto second = rerun_fixture(root / "pre2");
    atomic_save(second.root / raw_name(second, 0, 2), "bad baseline\n");
    expect_error([&] {
        auto a = inspect(first.root, first.expected, first.options);
        auto b = inspect(second.root, second.expected, second.options);
        backup(a); backup(b);
    }, "later preflight failure prevents earlier writes");
    check(!fs::exists(first.side), "all experiments preflight before any backup");
}
} // namespace

/// Standalone test executable. Requires a new test directory argument; leaves all test artifacts for inspection.
int main(int argc, char* argv[]) {
    try {
        if (argc != 2) throw invalid_argument("Usage: localization_tests.exe <new-isolated-test-directory>");
        const fs::path root = fs::absolute(argv[1]).lexically_normal();
        if (fs::exists(root)) throw invalid_argument("Test output must not already exist");
        const string config = (fs::path(experimentDataPath) / "ExperimentsResults" / "EXP1_user_num" / "def_config.json").string();
        load_global_channel_config(config);
        const json prototype = test_models_and_evaluation(root);
        test_aggregation(root, prototype);
        test_six_zero_error_methods();
        test_latest_dispatch_and_wrapper();
        test_rerun_recovery(root / "rr");
        test_rerun_rejections(root / "rr");
        cout << "LOCALIZATION AND RERUN TESTS PASSED: " << checks
             << " checks; only tiny synthetic solver inputs; no formal rerun or EXP5 run.\n";
        return 0;
    } catch (const std::exception& error) {
        cerr << error.what() << '\n';
        return 1;
    }
}

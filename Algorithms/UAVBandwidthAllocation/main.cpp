/// @file main.cpp
/// @brief 保留五组multi-hard手动入口，仅启用EXP3的ApproBetter最低等级临时重跑。
/// 新变体使用独立CSV、元数据和汇总；不修改原六算法结果，不自动启动其他实验。

// Windows头文件先于项目头文件，避免std::byte与项目INFINITE宏冲突。
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#ifdef INFINITE
#undef INFINITE
#endif
#include "localization_experiments.h"

namespace exp3_approbetter_lowest {
const string METHOD = "ApproBetterLowest";
const string EXPERIMENT = "EXP3_hard_ratio";
const string VERSION = SIMPLE_ALGORITHM_VERSION + "-approbetter-lowest-v1";
const string INFO_FILE = "run_info_ApproBetterLowest.json";
const string PENDING = ".approbetter-lowest.pending";
const vector<string> CONDITION_KEYS = {"0", "2", "4", "6", "8", "10"};
const std::array<string, 12> METRIC_NAMES = {"Run_time_ms", "Total_Num", "Hard_Num", "Elastic_Num",
    "Total_Utility", "Hard_Utility", "Elastic_Utility", "Hard_Bandwidth", "Elastic_Bandwidth",
    "Hard_Throughput", "Elastic_Throughput", "Total_Throughput"};

/// 一个条件的已提交记录，以及需要完成CSV原子提交的已验证中断记录；不保存大体积分配。
struct Checkpoint {
    MethodRecords rows;
    bool publish_pending = false;
};

/// 返回一个明确目标的同目录暂存名；仅供本变体的写入/恢复使用。
fs::path pending_path(const fs::path& target) { return fs::path(target.string() + PENDING); }

/// 返回本变体逐用户文件路径；directory为比例目录，id为1起始实例编号。
fs::path user_path(const fs::path& directory, int id) {
    return directory / "users" / METHOD / (to_string(id) + ".csv");
}

/// 只读拒绝冒充文件的目录或符号链接；不存在的文件允许后续创建，无返回值。
void check_file(const fs::path& path) {
    if (fs::is_symlink(path) || (fs::exists(path) && !fs::is_regular_file(path)))
        throw runtime_error("Not a regular variant file: " + path.string());
}

/// 将content写入target的确定暂存文件并检查flush/close；不触碰target，不删除文件。
void stage_text(const fs::path& target, const string& content) {
    check_file(target);
    const fs::path pending = pending_path(target);
    check_file(pending);
    fs::create_directories(target.parent_path());
    ofstream output(pending, ios::binary | ios::trunc);
    if (!output) throw runtime_error("Cannot stage variant file: " + pending.string());
    output.write(content.data(), static_cast<std::streamsize>(content.size()));
    output.flush(); output.close();
    if (!output) throw runtime_error("Variant staging failed: " + pending.string());
}

/// 将已完整写入的确定暂存文件原子替换到本变体目标；失败保留暂存文件并抛错。
void publish_text(const fs::path& target) {
    check_file(target);
    const fs::path pending = pending_path(target);
    check_file(pending);
    if (!MoveFileExW(pending.c_str(), target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw runtime_error("Variant publication failed: " + target.string() +
            ", Windows error " + to_string(GetLastError()));
}

/// 原子写一个本变体元数据/汇总文件；target必须由调用方限定在新变体的文件范围内。
void write_text(const fs::path& target, const string& content) {
    stage_text(target, content);
    publish_text(target);
}

/// 在最低等级副本上调用selector=3；原模型只读，用原模型评价并保留Better的elastic重优化。
/// options提供epsilon，seed记录实例身份；返回含真实计时、分配、指标及失败诊断的执行结果。
AlgorithmRunResult run_lowest(const SystemMd& model, const ExperimentRunOptions& options, uint32_t seed) {
    return execute_allocation(model, seed, [&](BAProblem& problem, AllocationDiagnostics&) {
        const auto& view = problem.sysModel;
        bool any_budget = false;
        for (const auto& uav : view.uavs) any_budget = any_budget || uav.total_bandwidth > 0;
        if (view.users.empty() || view.uavs.empty() || !any_budget) {
            vector<KnapsackResult> empty(view.uavs.size());
            for (size_t k = 0; k < empty.size(); ++k) empty[k].uav_id = static_cast<int>(k);
            return AllocationPair{empty, problem.construct_user_results(empty)};
        }
        return problem.Appro_multiUAV_ToN(view.uavs, view.users, 3,
            options.ton_epsilon, options.reallocate_residual);
    }, HardBaselinePolicy::ClampLowestThreshold);
}

/// 核对已提交逐用户文件与汇总，并要求hard仅0/1级、类型数量与比例一致；不修改文件。
void check_users(const fs::path& path, const AlgorithmRunResult& run, const MultiHardCondition& condition) {
    check_multi_user_result(path, run, condition.user_count, condition.uav_count);
    ifstream input(path, ios::binary);
    string line;
    getline(input, line);
    int hard_count = 0;
    EXPResult totals;
    while (getline(input, line)) {
        const auto fields = csv_fields(line);
        const bool hard = !fields.at(4).empty();
        if (hard) {
            ++hard_count;
            if (fields[4] != (fields[2] == "-1" ? "0" : "1"))
                throw runtime_error("Variant hard level must be zero or one: " + path.string());
        }
        if (fields[2] == "-1") continue;
        const double bw = parse_finite_number(fields[3]), value = parse_finite_number(fields[5]);
        if (hard) { ++totals.hard_num; totals.hard_bandwidth += bw; totals.hard_utility += value; }
        else { ++totals.elastic_num; totals.elastic_bandwidth += bw; totals.elastic_utility += value; }
    }
    if (hard_count != exp3_expected_hard_count(condition.input.key, condition.user_count))
        throw runtime_error("Variant user types differ from EXP3 condition: " + path.string());
    for (auto member : {&EXPResult::hard_num, &EXPResult::elastic_num, &EXPResult::hard_bandwidth,
        &EXPResult::elastic_bandwidth, &EXPResult::hard_utility, &EXPResult::elastic_utility})
        if (!allocation_near(totals.*member, run.metrics.*member))
            throw runtime_error("Variant user metrics disagree: " + path.string());
}

/// 检查原始记录的实例范围及按新算法名派生的种子；condition为当前比例，不写文件。
void check_rows(const MethodRecords& rows, const MultiHardCondition& condition, const ExperimentRunOptions& options) {
    for (const auto& row : rows)
        if (row.first < 1 || row.first > options.instance_count || row.second.seed !=
            derive_algorithm_seed(options.master_seed, EXPERIMENT, condition.input.key, row.first, METHOD))
            throw runtime_error("Variant checkpoint ID/seed mismatch: " + condition.input.key);
}

/// 只读加载一个条件；已落盘明细必须配套已提交行或唯一完整暂存行，未知/损坏记录拒绝恢复。
/// 不完整暂存内容只可在尚无正式明细时重算；完整失败暂存行直接提交，避免自动重试失败。
Checkpoint read_checkpoint(const fs::path& directory, const MultiHardCondition& condition,
    const ExperimentRunOptions& options) {
    const fs::path csv = directory / (METHOD + ".csv"), pending = pending_path(csv);
    check_file(csv); check_file(pending);
    Checkpoint state{read_result_csv(csv, false, true), false};
    check_rows(state.rows, condition, options);
    int next_id = 1;
    while (next_id <= options.instance_count && state.rows.count(next_id)) ++next_id;
    MethodRecords candidate;
    bool complete_pending = false;
    if (fs::exists(pending)) {
        // 不完整尾行可重写；完整行中的非法字段必须报错，不能冒充正常中断。
        ifstream input(pending, ios::binary);
        ostringstream buffer;
        if (!input) throw runtime_error("Cannot read variant pending CSV");
        buffer << input.rdbuf();
        if (input.bad()) throw runtime_error("Variant pending CSV read failed");
        const string bytes = buffer.str();
        if (!bytes.empty() && bytes.back() == '\n') {
            candidate = read_result_csv(pending, true, true);
            check_rows(candidate, condition, options);
            // 写到旧记录间的换行处中断，也属于未完成暂存；已有行必须保持一致。
            for (const auto& row : candidate)
                if (row.first != next_id && (!state.rows.count(row.first) ||
                    run_result_to_csv(row.second) != run_result_to_csv(state.rows.at(row.first))))
                    throw runtime_error("Variant pending CSV changes an existing row");
            complete_pending = candidate.count(next_id) != 0;
            if (complete_pending && candidate.size() != state.rows.size() + 1)
                throw runtime_error("Unexpected variant pending instance set");
        }
    }
    const fs::path users = directory / "users" / METHOD;
    if (fs::exists(users)) {
        if (!fs::is_directory(users) || fs::is_symlink(users)) throw runtime_error("Invalid variant user directory");
        for (const auto& entry : fs::directory_iterator(users)) {
            check_file(entry.path());
            bool known = false;
            for (int id = 1; id <= options.instance_count; ++id) {
                const fs::path final = user_path(directory, id);
                if (entry.path() == pending_path(final)) { known = true; break; }
                if (entry.path() != final) continue;
                known = true;
                const auto committed = state.rows.find(id);
                if (committed != state.rows.end()) {
                    if (!algorithm_status_valid(committed->second.status))
                        throw runtime_error("Failed variant row has a user result");
                    check_users(final, committed->second, condition);
                } else {
                    if (!complete_pending || id != next_id || !algorithm_status_valid(candidate.at(id).status))
                        throw runtime_error("Orphan variant user result: " + final.string());
                    check_users(final, candidate.at(id), condition);
                }
                break;
            }
            if (!known) throw runtime_error("Unexpected variant user file: " + entry.path().string());
        }
    }
    for (const auto& row : state.rows)
        if (algorithm_status_valid(row.second.status))
            check_users(user_path(directory, row.first), row.second, condition);
    if (complete_pending && (!algorithm_status_valid(candidate.at(next_id).status) ||
        fs::exists(user_path(directory, next_id)))) {
        state.rows = std::move(candidate);
        state.publish_pending = true; // 所有条件预检通过后，调用方才允许提交这份原始表。
    }
    return state;
}

/// 将有序记录序列化为现有CSV字段；不输出分配对象，返回可原子发布的完整文本。
string render_rows(const MethodRecords& rows) {
    ostringstream output;
    output << CSV_HEADER << '\n';
    for (const auto& row : rows) output << run_result_to_csv(row.second) << '\n';
    return output.str();
}

/// 提交一个新的终止记录；先完整暂存CSV，再提交逐用户明细，最后提交CSV完成标记。
/// model只在成功时必需；失败不产生逐用户文件；旧ID不允许覆盖，rows仅在提交成功后更新。
void commit_attempt(const fs::path& directory, const MultiHardCondition& condition,
    const SystemMd* model, const AlgorithmRunResult& run, MethodRecords& rows) {
    if (rows.count(run.instance_id)) throw runtime_error("Refusing to overwrite a variant instance");
    MethodRecords updated = rows;
    updated.emplace(run.instance_id, run);
    updated.at(run.instance_id).allocation = AllocationPair{};
    const fs::path csv = directory / (METHOD + ".csv");
    if (fs::exists(user_path(directory, run.instance_id)))
        throw runtime_error("Recover the staged variant before running this instance again");
    stage_text(csv, render_rows(updated));
    if (algorithm_status_valid(run.status)) {
        if (!model) throw runtime_error("Successful variant needs its original model");
        // 暂存逐用户文件只写本变体的确定路径；不调用拒绝覆盖暂存文件的公共写入器。
        ostringstream output;
        output << setprecision(std::numeric_limits<double>::max_digits10) << MULTI_USER_RESULT_HEADER << '\n';
        for (const User& user : model->users) {
            const auto& result = run.allocation.second.at(user.ID);
            output << run.instance_id << ',' << csv_quote(user.source_user_id) << ',' << result.uav_id
                << ',' << result.allocated_bandwidth << ',';
            if (user.uType == HARD_UTILITY)
                output << (result.uav_id < 0 ? 0 : user.achieved_hard_level(result.allocated_bandwidth,
                    model->cap_list.at(result.uav_id).at(user.ID)));
            output << ',' << result.utility << '\n';
        }
        const fs::path detail = user_path(directory, run.instance_id);
        stage_text(detail, output.str());
        check_users(pending_path(detail), run, condition);
        publish_text(detail);
    }
    publish_text(csv);
    rows = std::move(updated);
}

/// 仅补本条件缺失ID；load_model接收1起始ID并返回原模型，便于隔离测试验证跳过与失败行为。
/// 输入/求解失败记为终止记录并继续，I/O失败向上抛错；已提交的成功和失败都不自动重试。
void run_condition(const fs::path& directory, const MultiHardCondition& condition,
    const ExperimentRunOptions& options, Checkpoint& state,
    const std::function<SystemMd(int)>& load_model) {
    if (state.publish_pending) {
        publish_text(directory / (METHOD + ".csv"));
        state.publish_pending = false;
    }
    for (int id = 1; id <= options.instance_count; ++id) {
        if (state.rows.count(id)) continue;
        const uint32_t seed = derive_algorithm_seed(options.master_seed, EXPERIMENT, condition.input.key, id, METHOD);
        std::unique_ptr<SystemMd> model;
        AlgorithmRunResult run;
        run.seed = seed;
        try {
            model = std::make_unique<SystemMd>(load_model(id));
            validate_allocation_model(*model);
            if (model->users.size() != static_cast<size_t>(condition.user_count) ||
                model->m != condition.uav_count ||
                model->n1 != exp3_expected_hard_count(condition.input.key, condition.user_count))
                throw runtime_error("EXP3 variant input dimensions/ratio mismatch");
            for (const auto& uav : model->uavs)
                if (!allocation_near(uav.total_bandwidth, condition.input.bandwidth_mhz * unit_para))
                    throw runtime_error("EXP3 variant UAV budget mismatch");
            run = run_lowest(*model, options, seed);
        } catch (const std::exception& error) {
            run.status = AlgorithmRunStatus::Exception;
            run.diagnostics.events.push_back(string("input_error: ") + error.what());
        }
        run.instance_id = id;
        commit_attempt(directory, condition, model.get(), run, state.rows);
        cout << EXPERIMENT << '/' << condition.input.key << "/ID" << id << '/' << METHOD
            << ": " << algorithm_status_name(run.status) << " ms=" << run.duration_ms << std::endl;
    }
}

/// 只读核对原EXP3批次与当前输入/参数；返回仅登记新方法的独立元数据，不改原run_info。
/// source为原六算法元数据，base由当前输入构造；任何版本或输入漂移均拒绝混写。
json variant_info(const json& source, const json& base, const fs::path& source_path) {
    if (source.at("schema") != "ton-multihard-run-v1" || source.at("experiment") != EXPERIMENT ||
        source.at("algorithm_version") != EXP3_BUDGET_ALGORITHM_VERSION ||
        source.at("methods") != method_name_list || source.at("proposed_algorithms") != proposed_algorithm_policy())
        throw runtime_error("ApproBetterLowest requires the current original EXP3 batch");
    for (const string field : {"instance_count", "master_seed", "rounding_trials", "ton_epsilon",
        "reallocate_residual", "conditions", "config_path", "channel_config", "unit_para",
        "abs_tolerance", "rel_tolerance", "build_profile", "smawk_verification",
        "input_schema", "input_generation", "application_profiles", "application_profiles_path"})
        if (source.at(field) != base.at(field))
            throw runtime_error("Original EXP3 input/parameter differs: " + field);
    json info = base;
    for (const char* field : {"hard_first_policy", "baseline_hard_policy", "baseline_hard_bandwidth_policy",
        "reuse_source", "reuse_methods"}) info.erase(field);
    info["schema"] = "ton-exp3-approbetter-lowest-v1";
    info["algorithm_version"] = VERSION;
    info["base_algorithm_version"] = SIMPLE_ALGORITHM_VERSION;
    info["source_algorithm_version"] = source.at("algorithm_version");
    info["source_run_info"] = experiment_absolute_path(source_path);
    info["methods"] = {METHOD};
    info["proposed_algorithms"] = {{METHOD, proposed_algorithm_policy().at("ApproBetter")}};
    info["hard_level_policy"] = "lowest_level_only";
    info["hard_bandwidth_policy"] = "clamp_to_lowest_threshold_no_reallocation";
    info["utility_evaluation"] = "original_model";
    info["seed_method"] = METHOD;
    info["failure_policy"] = "record_and_continue_no_automatic_retry";
    info["user_result_bandwidth_unit"] = "kHz";
    info["expected_attempt_count"] = base.at("conditions").size() * base.at("instance_count").get<size_t>();
    info["summary"] = {{"status", "not_generated"}, {"instance_count", 0}, {"attempt_count", 0}, {"success_count", 0}};
    return info;
}

/// 预检独立元数据和汇总目录；仅summary可变化，原六算法根目录的其他文件不由本函数管理。
/// 缺少元数据却已有新方法输出、未知汇总文件或参数变更时抛错；返回可续跑元数据。
json inspect_variant(const fs::path& root, const json& expected) {
    const fs::path marker = root / INFO_FILE, summary = root / "summary" / METHOD;
    check_file(marker); check_file(pending_path(marker));
    if (fs::exists(summary)) {
        if (!fs::is_directory(summary) || fs::is_symlink(summary)) throw runtime_error("Invalid variant summary directory");
        set<string> names{"Success_Rate.csv"};
        for (const auto& name : METRIC_NAMES) names.insert(name + ".csv");
        for (const auto& entry : fs::directory_iterator(summary)) {
            check_file(entry.path());
            string name = entry.path().filename().string();
            if (name.size() >= PENDING.size() && name.substr(name.size() - PENDING.size()) == PENDING)
                name.resize(name.size() - PENDING.size());
            if (!names.count(name)) throw runtime_error("Unexpected variant summary file: " + entry.path().string());
        }
    }
    if (!fs::exists(marker)) {
        for (const auto& condition : expected.at("conditions")) {
            const fs::path directory = root / condition.at("key").get<string>();
            const fs::path csv = directory / (METHOD + ".csv");
            if (fs::exists(csv) || fs::exists(pending_path(csv)) || fs::exists(directory / "users" / METHOD))
                throw runtime_error("Variant results exist without their metadata");
        }
        if (fs::exists(summary)) throw runtime_error("Variant summary exists without its metadata");
        return expected; // 首次元数据写入中断时，仅marker的确定暂存文件可覆盖重写。
    }
    const json old = read_experiment_json(marker);
    json prior = old, next = expected;
    prior.erase("summary"); next.erase("summary");
    if (prior != next) throw runtime_error("ApproBetterLowest settings/version changed");
    const auto& status = old.at("summary");
    const string phase = status.at("status").get<string>();
    if (phase != "not_generated" && phase != "stale" && phase != "writing" && phase != "complete")
        throw runtime_error("Invalid variant summary status");
    for (const char* key : {"instance_count", "attempt_count", "success_count"})
        if (!status.at(key).is_number_integer() || status.at(key).get<int64_t>() < 0)
            throw runtime_error("Invalid variant summary count");
    if (status.at("success_count").get<size_t>() > status.at("attempt_count").get<size_t>() ||
        status.at("attempt_count").get<size_t>() > expected.at("expected_attempt_count").get<size_t>() ||
        status.at("instance_count").get<int>() > expected.at("instance_count").get<int>())
        throw runtime_error("Inconsistent variant summary counts");
    if (phase == "complete" && (status.at("instance_count") != expected.at("instance_count") ||
        status.at("attempt_count") != expected.at("expected_attempt_count")))
        throw runtime_error("Complete variant metadata must include every selected attempt");
    return old;
}

/// 只在所有选定ID都有终止记录后发布本变体13张汇总；失败不当零值，均值只取有效记录。
/// 返回complete统计（含成功数），complete表示尝试齐全而非全成功；输出目录不得是原summary本身。
json export_summary(const fs::path& directory, const vector<MultiHardCondition>& conditions,
    const vector<Checkpoint>& states, const ExperimentRunOptions& options) {
    if (conditions.empty() || conditions.size() != states.size()) throw runtime_error("Variant summary dimensions");
    vector<EXPResult> means(conditions.size());
    vector<size_t> counts(conditions.size(), 0);
    size_t successes = 0;
    ostringstream rates;
    rates << "condition,method,success_count,attempt_count,success_rate\n" << setprecision(17);
    for (size_t c = 0; c < conditions.size(); ++c) {
        const auto& rows = states[c].rows;
        if (states[c].publish_pending || rows.size() != static_cast<size_t>(options.instance_count))
            throw runtime_error("Variant summary requires all committed attempts");
        check_rows(rows, conditions[c], options);
        for (int id = 1; id <= options.instance_count; ++id) {
            if (!rows.count(id)) throw runtime_error("Variant summary has a missing ID");
            if (algorithm_status_valid(rows.at(id).status)) {
                ++counts[c];
                for (auto member : EXP_METRICS) means[c].*member += rows.at(id).metrics.*member;
            }
        }
        if (counts[c]) for (auto member : EXP_METRICS) means[c].*member /= counts[c];
        successes += counts[c];
        rates << conditions[c].input.key << ',' << METHOD << ',' << counts[c] << ',' << rows.size()
            << ',' << static_cast<double>(counts[c]) / rows.size() << '\n';
    }
    write_text(directory / "Success_Rate.csv", rates.str());
    for (size_t metric = 0; metric < METRIC_NAMES.size(); ++metric) {
        ostringstream output;
        output << setprecision(17) << "User_Scale," << METHOD << '\n';
        for (size_t c = 0; c < conditions.size(); ++c) {
            output << conditions[c].input.key << ',';
            if (counts[c]) output << means[c].*EXP_METRICS[metric];
            output << '\n';
        }
        write_text(directory / (METRIC_NAMES[metric] + ".csv"), output.str());
    }
    return {{"status", "complete"}, {"instance_count", options.instance_count},
        {"attempt_count", conditions.size() * options.instance_count}, {"success_count", successes}};
}
} // namespace exp3_approbetter_lowest

/// 临时入口：仅向run_02的EXP3新增ApproBetterLowest，六比例各ID1--10；返回void，I/O异常上抛。
/// options必须匹配原批次；所有条件预检后才写独立输出，结束不调用普通六算法批处理。
void rerun_exp3_approbetter_lowest(const ExperimentRunOptions& options) {
    using namespace exp3_approbetter_lowest;
    validate_run_options(options);
    if (options.output_name != "run_02" || options.instance_count != 10 || !options.reuse_exp1_root.empty() ||
        experiment_condition_keys(options, CONDITION_KEYS) != CONDITION_KEYS)
        throw invalid_argument("Temporary ApproBetterLowest entry requires run_02, all six EXP3 conditions, IDs 1--10");
    const fs::path input_root = resolve_experiment_input_root(options);
    const json generation = read_experiment_json(input_root / "generation_config_ToN.json");
    if (generation.at("input_schema") != "ton-multihard-input-v1") throw runtime_error("Multi-hard inputs required");
    const fs::path physical = input_root / "physical_config.json", profiles = input_root / "application_profiles.json";
    const double bandwidth = read_experiment_json(physical).at("uav_bandwidth_mhz").get<double>();
    if (!std::isfinite(bandwidth) || bandwidth <= 0) throw runtime_error("Invalid EXP3 common bandwidth");
    const auto conditions = multi_hard_conditions(EXPERIMENT, input_root, generation, bandwidth, options);
    vector<ExperimentCondition> inputs;
    for (const auto& condition : conditions) {
        inputs.push_back(condition.input);
        for (int id = 0; id < options.instance_count; ++id)
            for (const string& path : {condition.input.users.at(id), condition.input.uavs.at(id)})
                if (!fs::is_regular_file(path) || !ifstream(path, ios::binary))
                    throw runtime_error("Missing/unreadable EXP3 input: " + path);
    }
    json base = make_experiment_run_info(EXPERIMENT, inputs, physical.string(), options);
    base["input_schema"] = generation.at("input_schema");
    base["input_generation"] = generation;
    base["application_profiles"] = read_experiment_json(profiles);
    base["application_profiles_path"] = experiment_absolute_path(profiles);
    const fs::path root = fs::path(experimentDataPath) / "ExperimentsResults" / "ToN_multiHard" / options.output_name / EXPERIMENT;
    const json expected = variant_info(read_experiment_json(root / "run_info.json"), base, root / "run_info.json");
    json info = inspect_variant(root, expected);
    vector<Checkpoint> states;
    size_t committed = 0, successes = 0;
    for (const auto& condition : conditions) {
        states.push_back(read_checkpoint(root / condition.input.key, condition, options));
        committed += states.back().rows.size();
        for (const auto& row : states.back().rows) if (algorithm_status_valid(row.second.status)) ++successes;
    }
    if (info.at("summary").at("status") == "complete" &&
        (committed != expected.at("expected_attempt_count").get<size_t>() ||
            successes != info.at("summary").at("success_count").get<size_t>()))
        throw runtime_error("Complete variant metadata disagrees with committed records");
    info["summary"]["status"] = "stale";
    write_text(root / INFO_FILE, info.dump(2) + "\n");
    for (size_t c = 0; c < conditions.size(); ++c) {
        const auto& condition = conditions[c];
        run_condition(root / condition.input.key, condition, options, states[c], [&](int id) {
            // 仅缺失实例才加载原始完整模型；路径和物理配置均沿用原EXP3批次。
            return SystemMd(condition.input.users.at(id - 1), condition.input.uavs.at(id - 1),
                physical.string(), profiles.string());
        });
    }
    info["summary"]["status"] = "writing";
    write_text(root / INFO_FILE, info.dump(2) + "\n");
    info["summary"] = export_summary(root / "summary" / METHOD, conditions, states, options);
    write_text(root / INFO_FILE, info.dump(2) + "\n");
    const size_t attempts = info["summary"]["attempt_count"].get<size_t>();
    successes = info["summary"]["success_count"].get<size_t>();
    cout << METHOD << ": " << attempts << " attempts, " << successes << " successful, "
        << attempts - successes << " failed; original six methods unchanged." << std::endl;
}

#ifndef TON_RERUN_TESTING
/// 配置当前十实例批次，只执行EXP3最低等级临时入口；无参数，成功返回0，异常返回1。
int main() {
    if (!SetConsoleOutputCP(CP_UTF8)) {
        std::cerr << "Failed to set console output to UTF-8. Error: " << GetLastError() << std::endl;
        return 1;
    }
    try {
        ExperimentRunOptions options;
        options.instance_count = 10; // 六比例各ID1--10，共60次新变体调用。
        options.master_seed = 20260905u;
        options.rounding_trials = 2;
        options.ton_epsilon = 0.083;
        options.reallocate_residual = false; // 历史兼容字段；Better仍按selector=3执行elastic重优化。
        options.input_root = (fs::path(experimentDataPath) / "data_ToN_multiHard" / "run_01").string();
        options.output_name = "run_02";
        options.conditions.clear();
        options.reuse_exp1_root.clear();

        rerun_exp3_approbetter_lowest(options); // 本轮唯一启用入口，不重算原六算法。
        // exp1_different_user_number(options);
        // exp2_different_uav_number(options);
        // exp3_different_hard_user_ratio(options);
        // exp4_real_requests_user_number(options);
        // exp5_real_requests_uav_number(options);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "实验执行失败：" << error.what() << std::endl;
        return 1;
    }
}
#endif // TON_RERUN_TESTING

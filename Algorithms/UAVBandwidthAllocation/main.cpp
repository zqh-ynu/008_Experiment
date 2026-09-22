// 本文件提供 ToN 实验的 Visual Studio 手动入口，集中设置公共参数并按需选择单个实验。
// 实际执行内容由main中的调用选择；正式重跑入口会覆盖对应方法的既有结果。

// Windows头文件放在项目头文件之前，避免std::byte等名称冲突。
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>

// 项目predefine.h已有自己的INFINITE定义，避免与Windows宏重复。
#ifdef INFINITE
#undef INFINITE
#endif

#include "localization_experiments.h"

/**
 * @brief 手动测试优化后Fast/Better在已有EXP1条件上的耗时；所有输出仅到控制台。
 * 无参数：固定run_01，实例数/条件/种子取现有元数据。不会保存新记录或修改任何旧结果。
 * 比较历史耗时与本次实测，受系统负载影响；应先测试，再调用正式覆盖函数。
 */
void test_optimized_proposed_exp1()
{
    const fs::path input_root = fs::path(experimentDataPath) / "data_ToN_multiHard" / "run_01";
    const fs::path batch_root = fs::path(experimentDataPath) / "ExperimentsResults" / "ToN_multiHard" / "run_01";
    const fs::path physical = input_root / "physical_config.json";
    const fs::path profiles = input_root / "application_profiles.json";
    const json generation = read_experiment_json(input_root / "generation_config_ToN.json");
    const json physical_json = read_experiment_json(physical);
    const json profile_json = read_experiment_json(profiles);
    if (generation.value("input_schema", string{}) != "ton-multihard-input-v1")
        throw runtime_error("Expected multi-hard run_01 inputs");
    const double bandwidth = physical_json.at("uav_bandwidth_mhz").get<double>();
    if (!std::isfinite(bandwidth) || bandwidth <= 0) throw runtime_error("Invalid common bandwidth");

    // 按已完成实验的记录恢复参数；不使用main中的运行数量，不扩展到尚未执行的50实例。
    auto read_options = [&](const json& info) {
        ExperimentRunOptions options;
        options.instance_count = info.at("instance_count").get<int>();
        options.master_seed = info.at("master_seed").get<uint32_t>();
        options.rounding_trials = info.at("rounding_trials").get<int>();
        options.ton_epsilon = info.at("ton_epsilon").get<double>();
        options.reallocate_residual = info.at("reallocate_residual").get<bool>();
        options.input_root = input_root.string();
        options.output_name = "run_01";
        options.conditions.clear();
        options.reuse_exp1_root.clear();
        for (const auto& condition : info.at("conditions"))
            options.conditions.push_back(condition.at("key").get<string>());
        validate_run_options(options);
        resolve_experiment_input_root(options); // 只读检查生成完成状态及实例前缀。
        return options;
    };
    // 复用既有元数据比较规则，检查算法、两份配置、来源及所有已记录输入路径一致。
    auto check_settings = [&](const string& experiment, const vector<MultiHardCondition>& conditions,
        const ExperimentRunOptions& options) {
        vector<ExperimentCondition> base;
        for (const auto& condition : conditions) {
            base.push_back(condition.input);
            for (size_t i = 0; i < condition.input.users.size(); ++i)
                if (!fs::is_regular_file(condition.input.users[i]) || !fs::is_regular_file(condition.input.uavs[i]))
                    throw runtime_error(experiment + "/" + condition.input.key + ": missing input ID " + to_string(i + 1));
        }
        json expected = make_experiment_run_info(experiment, base, physical.string(), options);
        expected["schema"] = "ton-multihard-run-v1";
        expected["input_schema"] = generation.at("input_schema");
        expected["input_generation"] = generation;
        expected["application_profiles"] = profile_json;
        expected["application_profiles_path"] = experiment_absolute_path(profiles);
        expected["user_result_bandwidth_unit"] = "kHz";
        expected["failure_policy"] = "record_and_continue_no_automatic_retry";
        inspect_experiment_destination(batch_root / experiment, expected);
    };
    // 构造和检查实例在算法计时之外，每个实例只构造一次供两个方法共用。
    auto load_model = [&](const MultiHardCondition& condition, int index) {
        SystemMd model(condition.input.users.at(index), condition.input.uavs.at(index),
            physical.string(), profiles.string());
        if (model.users.size() != static_cast<size_t>(condition.user_count) ||
            model.uavs.size() != static_cast<size_t>(condition.uav_count))
            throw runtime_error("Input size differs from recorded experiment");
        for (const auto& uav : model.uavs)
            if (!allocation_near(uav.total_bandwidth, bandwidth * unit_para))
                throw runtime_error("Input bandwidth differs from recorded configuration");
        validate_allocation_model(model);
        return model;
    };

    const string experiment = "EXP1_user_num";
    const fs::path root = batch_root / experiment;
    const json info = read_experiment_json(root / "run_info.json");
    if (info.at("summary").at("status") != "complete")
        throw runtime_error("EXP1 summary is not complete; cannot compare historical timings");
    const auto options = read_options(info);
    const auto conditions = multi_hard_conditions(experiment, input_root, generation, bandwidth, options);
    check_settings(experiment, conditions, options);

    // 只读取两个提出方法的旧CSV；完整检查后才开始真正调用算法。
    vector<vector<MethodRecords>> old(conditions.size(), vector<MethodRecords>(2));
    for (size_t c = 0; c < conditions.size(); ++c)
        for (size_t m = 0; m < 2; ++m) {
            old[c][m] = read_result_csv(root / conditions[c].input.key / (method_name_list[m] + ".csv"), true, true);
            if (old[c][m].size() != static_cast<size_t>(options.instance_count))
                throw runtime_error("EXP1 historical proposed-method rows are incomplete");
            for (int id = 1; id <= options.instance_count; ++id)
                if (!old[c][m].count(id) || old[c][m].at(id).seed !=
                    derive_algorithm_seed(options.master_seed, experiment, conditions[c].input.key, id, method_name_list[m]))
                    throw runtime_error("EXP1 historical ID/seed mismatch");
        }
    cout << "仅测试EXP1两个提出方法；不会保存结果。历史耗时与当前实测是跨时段比较，不是同负载计时对照。\n";
    if (info.at("summary").contains("proposed_refresh"))
        cout << "注意：参考目录已有提出方法替换标记，旧耗时可能已不是优化前版本。\n";
    cout << "计划调用：" << conditions.size() * options.instance_count * 2 << " 次\n";
    cout << setprecision(10) << "条件 / 实例ID / 方法 / 旧耗时ms / 新耗时ms / 旧耗时÷新耗时\n";
    auto print_ratio = [](double old_ms, double new_ms) {
        if (old_ms > 0 && new_ms > 0) cout << old_ms / new_ms;
        else cout << "N/A";
    };
    for (size_t c = 0; c < conditions.size(); ++c) {
        const auto& condition = conditions[c];
        double old_sum[2] = {}, new_sum[2] = {};
        int matched[2] = {}, new_failed[2] = {}, old_failed[2] = {};
        for (int index = 0; index < options.instance_count; ++index) {
            const int id = index + 1;
            std::unique_ptr<SystemMd> model;
            try { model = std::make_unique<SystemMd>(load_model(condition, index)); }
            catch (const std::exception& error) {
                cout << "[INPUT ERROR] " << condition.input.key << "/ID" << id << ": " << error.what() << '\n';
            }
            for (size_t m = 0; m < 2; ++m) {
                const auto& previous = old[c][m].at(id);
                if (!algorithm_status_valid(previous.status)) ++old_failed[m];
                if (!model) { ++new_failed[m]; continue; }
                const auto current = run_algorithm(*model, m, options, previous.seed);
                cout << condition.input.key << " / " << id << " / " << method_name_list[m] << " / ";
                if (algorithm_status_valid(previous.status)) cout << previous.duration_ms; else cout << "N/A";
                cout << " / ";
                if (algorithm_status_valid(current.status)) cout << current.duration_ms; else cout << "N/A";
                cout << " / ";
                if (algorithm_status_valid(previous.status) && algorithm_status_valid(current.status)) {
                    print_ratio(previous.duration_ms, current.duration_ms);
                    old_sum[m] += previous.duration_ms; new_sum[m] += current.duration_ms; ++matched[m];
                } else cout << "N/A";
                cout << '\n';
                if (!algorithm_status_valid(current.status)) {
                    ++new_failed[m];
                    cout << "[FAIL] " << algorithm_status_name(current.status);
                    for (const auto& event : current.diagnostics.events) cout << " / " << event;
                    cout << '\n';
                }
            }
        }
        for (size_t m = 0; m < 2; ++m) {
            cout << "[MEAN] " << condition.input.key << " / " << method_name_list[m]
                 << " / matched=" << matched[m] << " / old_failed=" << old_failed[m] << " / new_failed=" << new_failed[m];
            if (matched[m]) {
                const double old_mean = old_sum[m] / matched[m], new_mean = new_sum[m] / matched[m];
                cout << " / old_ms=" << old_mean << " / new_ms=" << new_mean << " / ratio=";
                print_ratio(old_mean, new_mean);
            } else cout << " / old_ms=N/A / new_ms=N/A / ratio=N/A";
            cout << '\n';
        }
    }
}

/**
 * @brief 手动覆盖run_01五实验的ApproBetter/ApproFast/AlgHardFirst，其余三个基线不重跑。
 * 无参数：读取各实验已记录的条件与实例数；每条件三个方法全成功后才覆盖该条件。
 * 不创建备份、不删文件、不绘图；写入失败保留stale标记，下次可从头修复自身的替换。
 */
void rerun_optimized_proposed_all_experiments()
{
    // 使用全局方法编号组织调用与文件覆盖，三个目标之外的原始记录始终保留。
    const vector<size_t> rerun_methods = {0, 1, 4};
    json rerun_method_names = json::array();
    for (size_t m : rerun_methods) rerun_method_names.push_back(method_name_list.at(m));
    const string implementation = "proposed-hardfirst-ls-v1";

    // 只接受当前策略或不含局部搜索的DA策略；其他字段必须完整一致。
    const json target_hard_first_policy = hard_first_da_policy();
    json source_da_policy = target_hard_first_policy;
    source_da_policy.erase("hard_local_search");
    source_da_policy["mechanism"] = "priority-aware-subchannel-da-v1";

    const fs::path input_root = fs::path(experimentDataPath) / "data_ToN_multiHard" / "run_01";
    const fs::path batch_root = fs::path(experimentDataPath) / "ExperimentsResults" / "ToN_multiHard" / "run_01";
    const fs::path physical = input_root / "physical_config.json";
    const fs::path profiles = input_root / "application_profiles.json";
    const json generation = read_experiment_json(input_root / "generation_config_ToN.json");
    const json physical_json = read_experiment_json(physical);
    const json profile_json = read_experiment_json(profiles);
    if (generation.value("input_schema", string{}) != "ton-multihard-input-v1")
        throw runtime_error("Expected multi-hard run_01 inputs");
    const double bandwidth = physical_json.at("uav_bandwidth_mhz").get<double>();
    if (!std::isfinite(bandwidth) || bandwidth <= 0) throw runtime_error("Invalid common bandwidth");

    // 按已完成实验的记录恢复参数；不使用main中的运行数量，不扩展到尚未执行的50实例。
    auto read_options = [&](const json& info) {
        ExperimentRunOptions options;
        options.instance_count = info.at("instance_count").get<int>();
        options.master_seed = info.at("master_seed").get<uint32_t>();
        options.rounding_trials = info.at("rounding_trials").get<int>();
        options.ton_epsilon = info.at("ton_epsilon").get<double>();
        options.reallocate_residual = info.at("reallocate_residual").get<bool>();
        options.input_root = input_root.string();
        options.output_name = "run_01";
        options.conditions.clear();
        options.reuse_exp1_root.clear();
        for (const auto& condition : info.at("conditions"))
            options.conditions.push_back(condition.at("key").get<string>());
        validate_run_options(options);
        resolve_experiment_input_root(options); // 只读检查生成完成状态及实例前缀。
        return options;
    };
    // 先核对来源HardFirst策略，再复用公共比较检查其他算法参数、配置和已记录输入路径。
    auto check_settings = [&](const string& experiment, const vector<MultiHardCondition>& conditions,
        const ExperimentRunOptions& options, const json& source_policy) {
        if (source_policy != target_hard_first_policy && source_policy != source_da_policy)
            throw runtime_error(experiment + ": unsupported HardFirst source policy");
        vector<ExperimentCondition> base;
        for (const auto& condition : conditions) {
            base.push_back(condition.input);
            for (size_t i = 0; i < condition.input.users.size(); ++i)
                if (!fs::is_regular_file(condition.input.users[i]) || !fs::is_regular_file(condition.input.uavs[i]))
                    throw runtime_error(experiment + "/" + condition.input.key + ": missing input ID " + to_string(i + 1));
        }
        json expected = make_experiment_run_info(experiment, base, physical.string(), options);
        expected["schema"] = "ton-multihard-run-v1";
        expected["input_schema"] = generation.at("input_schema");
        expected["input_generation"] = generation;
        expected["application_profiles"] = profile_json;
        expected["application_profiles_path"] = experiment_absolute_path(profiles);
        expected["user_result_bandwidth_unit"] = "kHz";
        expected["failure_policy"] = "record_and_continue_no_automatic_retry";
        // 仅比较对象采用已经核实的来源策略；实际元数据在本实验完全替换成功后才更新。
        expected["hard_first_da_policy"] = source_policy;
        inspect_experiment_destination(batch_root / experiment, expected);
    };
    // 构造和检查实例在算法计时之外，每个实例只构造一次供三个方法共用。
    auto load_model = [&](const MultiHardCondition& condition, int index) {
        SystemMd model(condition.input.users.at(index), condition.input.uavs.at(index),
            physical.string(), profiles.string());
        if (model.users.size() != static_cast<size_t>(condition.user_count) ||
            model.uavs.size() != static_cast<size_t>(condition.uav_count))
            throw runtime_error("Input size differs from recorded experiment");
        for (const auto& uav : model.uavs)
            if (!allocation_near(uav.total_bandwidth, bandwidth * unit_para))
                throw runtime_error("Input bandwidth differs from recorded configuration");
        validate_allocation_model(model);
        return model;
    };

    struct ExperimentState {
        string name;
        fs::path root;
        json info;
        ExperimentRunOptions options;
        vector<MultiHardCondition> conditions;
        vector<ConditionRecords> records; // 只在内存替换方法0/1/4，其余三个方法保留原始记录。
    };
    vector<ExperimentState> experiments;

    // 五组全部只读预检后才写任何标记；普通未完成实验不能与本函数同时执行。
    for (const string name : {"EXP1_user_num", "EXP2_uav_num", "EXP3_hard_ratio",
        "EXP4_real_user_num", "EXP5_real_uav_num"}) {
        ExperimentState state;
        state.name = name; state.root = batch_root / name;
        state.info = read_experiment_json(state.root / "run_info.json");
        const auto& summary = state.info.at("summary");
        const string status = summary.at("status").get<string>();
        const json refresh = summary.value("proposed_refresh", json::object());
        const bool own_incomplete = (status == "stale" || status == "writing") &&
            refresh.value("implementation", string{}) == implementation &&
            refresh.value("status", string{}) == "in_progress" &&
            refresh.value("methods", json::array()) == rerun_method_names &&
            refresh.value("target_hard_first_da_policy", json::object()) == target_hard_first_policy;
        if (status != "complete" && !own_incomplete)
            throw runtime_error(name + ": experiment is incomplete or refresh identity/policy differs");
        state.options = read_options(state.info);
        state.conditions = multi_hard_conditions(name, input_root, generation, bandwidth, state.options);
        check_settings(name, state.conditions, state.options, state.info.at("hard_first_da_policy"));
        for (const auto& condition : state.conditions) {
            const fs::path directory = state.root / condition.input.key;
            ConditionRecords records(method_name_list.size());
            if (!own_incomplete) {
                records = read_multi_records(directory, name, condition, state.options);
            } else {
                // 本函数写入中断可能使三个目标的CSV或逐用户文件不配套；它们会完整重算。
                // 非目标方法仍逐条检查，不尝试修复或重写其原始记录。
                for (size_t m = 0; m < method_name_list.size(); ++m) {
                    if (std::find(rerun_methods.begin(), rerun_methods.end(), m) != rerun_methods.end()) continue;
                    records[m] = read_result_csv(directory / (method_name_list[m] + ".csv"), true, true);
                    for (const auto& entry : records[m]) {
                        const auto detail = multi_user_result_path(directory, m, entry.first);
                        if (algorithm_status_valid(entry.second.status))
                            check_multi_user_result(detail, entry.second, condition.user_count, condition.uav_count);
                        else if (fs::exists(detail)) throw runtime_error("Baseline failure has unexpected user detail");
                    }
                    // 未重跑方法的目录仍需与CSV一一对应，孤立文件不能被恢复路径忽略。
                    const fs::path user_dir = directory / "users" / method_name_list[m];
                    if (fs::exists(user_dir))
                        for (const auto& file : fs::directory_iterator(user_dir)) {
                            if (!file.is_regular_file() || file.path().extension() != ".csv")
                                throw runtime_error("Unexpected user-result entry: " + file.path().string());
                            const double raw = parse_finite_number(file.path().stem().string());
                            if (raw < 1 || raw > INT_MAX || raw != floor(raw) ||
                                !records[m].count(static_cast<int>(raw)) ||
                                file.path().filename() != fs::path(to_string(static_cast<int>(raw)) + ".csv"))
                                throw runtime_error("Orphan user result: " + file.path().string());
                        }
                }
            }
            for (size_t m = 0; m < method_name_list.size(); ++m) {
                if (own_incomplete &&
                    std::find(rerun_methods.begin(), rerun_methods.end(), m) != rerun_methods.end()) continue;
                if (records[m].size() != static_cast<size_t>(state.options.instance_count))
                    throw runtime_error(name + "/" + condition.input.key + ": incomplete existing method records");
                for (int id = 1; id <= state.options.instance_count; ++id)
                    if (!records[m].count(id) || records[m].at(id).seed != derive_algorithm_seed(
                        state.options.master_seed, name, condition.input.key, id, method_name_list[m]))
                        throw runtime_error(name + ": existing ID/seed mismatch");
            }
            state.records.push_back(std::move(records));
        }
        experiments.push_back(std::move(state));
    }

    // 仅在内存序列化全部源用户结果，确保三个方法全部成功且文本已准备好才覆盖当前条件。
    auto user_text = [](const SystemMd& model, const AlgorithmRunResult& run) {
        ostringstream output;
        output << setprecision(std::numeric_limits<double>::max_digits10) << MULTI_USER_RESULT_HEADER << '\n';
        for (const auto& user : model.users) {
            const auto& result = run.allocation.second.at(user.ID);
            output << run.instance_id << ',' << csv_quote(user.source_user_id) << ',' << result.uav_id << ','
                   << result.allocated_bandwidth << ',';
            if (user.uType == HARD_UTILITY)
                output << (result.uav_id < 0 ? 0 :
                    user.achieved_hard_level(result.allocated_bandwidth, model.cap_list[result.uav_id][user.ID]));
            output << ',' << result.utility << '\n';
        }
        return output.str();
    };
    // 明确路径逐文件覆盖，仅写三个目标方法；不删除文件，任何写入失败立即向上抛出。
    auto overwrite = [](const fs::path& path, const string& text) {
        fs::create_directories(path.parent_path());
        ofstream output(path, ios::binary | ios::trunc);
        if (!output) throw runtime_error("Cannot overwrite: " + path.string());
        output << text; output.flush(); output.close();
        if (!output) throw runtime_error("Write/close failed: " + path.string());
    };

    size_t planned = 0;
    for (const auto& state : experiments)
        planned += state.conditions.size() * state.options.instance_count * rerun_methods.size();
    cout << "正式替换执行ApproBetter/ApproFast/AlgHardFirst，计划调用 " << planned
         << " 次。AlgRelaxRound/AlgSwapMatching/AlgSA-DD原始记录保留。\n"
         << "不得与同一批次的其他写入程序并行；本函数直接覆盖、不另存备份。\n";
    for (auto& state : experiments) {
        // 若预检后元数据被其他运行修改，拒绝覆盖其状态。
        if (read_experiment_json(state.root / "run_info.json") != state.info)
            throw runtime_error(state.name + ": metadata changed after preflight");
        state.info["summary"]["status"] = "stale";
        state.info["summary"]["proposed_refresh"] = {
            {"implementation", implementation}, {"status", "in_progress"},
            {"methods", rerun_method_names}, {"target_hard_first_da_policy", target_hard_first_policy}};
        write_experiment_run_info(state.root, state.info);
        for (size_t c = 0; c < state.conditions.size(); ++c) {
            const auto& condition = state.conditions[c];
            const fs::path directory = state.root / condition.input.key;
            // 缓存按全局方法编号索引，仅填充三个目标的位置，避免局部编号与方法编号混淆。
            vector<MethodRecords> replacement(method_name_list.size());
            vector<map<int, string>> details(method_name_list.size());
            // 当前条件的计算阶段仅缓存结果；任一方法失败时不发布该条件。
            for (int index = 0; index < state.options.instance_count; ++index) {
                const int id = index + 1;
                const string context = state.name + "/" + condition.input.key + "/ID" + to_string(id);
                try {
                    SystemMd model = load_model(condition, index);
                    for (size_t m : rerun_methods) {
                        const uint32_t seed = derive_algorithm_seed(state.options.master_seed, state.name,
                            condition.input.key, id, method_name_list[m]);
                        auto run = run_algorithm(model, m, state.options, seed);
                        run.instance_id = id;
                        if (!algorithm_status_valid(run.status)) {
                            string reason = method_name_list[m] + ": " + algorithm_status_name(run.status);
                            for (const auto& event : run.diagnostics.events) reason += " / " + event;
                            throw runtime_error(reason);
                        }
                        details[m].emplace(id, user_text(model, run));
                        cout << "[RERUN] " << context << '/' << method_name_list[m] << " / " << run.duration_ms << " ms\n";
                        run.allocation = AllocationPair{}; // 文本已保存，释放大分配map，只保留汇总记录。
                        replacement[m].emplace(id, std::move(run));
                    }
                } catch (const std::exception& error) {
                    throw runtime_error(context + ": " + error.what() + "；当前条件未发布，旧结果保留");
                }
            }
            vector<string> csv_texts(method_name_list.size());
            for (size_t m : rerun_methods) {
                ostringstream csv;
                csv << CSV_HEADER << '\n';
                for (const auto& entry : replacement[m]) csv << run_result_to_csv(entry.second) << '\n';
                csv_texts[m] = csv.str();
            }
            // 三个方法都成功才逐文件覆盖；不是跨文件原子提交，I/O中断后允许从头重算目标。
            for (size_t m : rerun_methods) {
                for (const auto& entry : details[m])
                    overwrite(multi_user_result_path(directory, m, entry.first), entry.second);
                overwrite(directory / (method_name_list[m] + ".csv"), csv_texts[m]);
                state.records[c][m] = std::move(replacement[m]);
            }
            cout << "[REPLACED] " << state.name << '/' << condition.input.key
                 << "（ApproBetter/ApproFast/AlgHardFirst）\n";
        }
        // 三个目标的新记录与其余三个基线原记录共同生成汇总，原失败记录仍参与成功率统计。
        state.info["summary"]["status"] = "writing";
        write_experiment_run_info(state.root, state.info);
        export_multi_summary(state.root / "summary", state.conditions, state.records);
        size_t attempts = 0, successes = 0;
        for (const auto& condition : state.records) for (const auto& method : condition)
            for (const auto& entry : method) { ++attempts; if (algorithm_status_valid(entry.second.status)) ++successes; }
        // 本实验全部条件与汇总均写入成功后，才声明顶层HardFirst策略及替换状态已完成更新。
        state.info["hard_first_da_policy"] = target_hard_first_policy;
        state.info["summary"]["status"] = "complete";
        state.info["summary"]["instance_count"] = state.options.instance_count;
        state.info["summary"]["attempt_count"] = attempts;
        state.info["summary"]["success_count"] = successes;
        state.info["summary"]["proposed_refresh"]["status"] = "complete";
        write_experiment_run_info(state.root, state.info);
        cout << "[COMPLETE] " << state.name
             << "：ApproBetter/ApproFast/AlgHardFirst已替换，其余三个基线保留；旧PDF需随后手动重新绘制。\n";
    }
}

#ifndef TON_RERUN_TESTING
/// @brief 配置固定 ToN 批次，并提供 EXP1--EXP5 的基础手动调用入口。
/// @return 实验正常结束或未选择实验时返回 0；实验抛出异常时返回 1。
int main() {

    // /utf-8决定字符串的编码；这里让控制台按同样的UTF-8编码显示。
    if (!SetConsoleOutputCP(CP_UTF8)) {
        std::cerr << "Failed to set console output to UTF-8. Error: "
            << GetLastError() << std::endl;
        return 1;
    }

    try {
        ExperimentRunOptions options;
        options.instance_count = 10;
        options.master_seed = 20260905u;
        options.rounding_trials = 2;
        options.ton_epsilon = 0.083;
        options.reallocate_residual = false; // 默认严格返回主算法结果；开启后只接纳尚未服务的用户。
        options.input_root = (fs::path(experimentDataPath) / "data_ToN_multiHard" / "run_01").string();
        options.output_name = "run_01"; // 新结果位于ExperimentsResults/ToN_multiHard/run_01，不写旧批次。
        options.conditions.clear();
        options.reuse_exp1_root.clear();

        // 先由Python准备50实例；这里先执行前10个，后续改为50即可续跑。每次只选择需要的实验。
        // exp1_different_user_number(options);
        // exp2_different_uav_number(options);
        // exp3_different_hard_user_ratio(options);
        // exp4_real_requests_user_number(options);
        // exp5_real_requests_uav_number(options);

        // 每次仅启用一个：先控制台测试，再按需正式覆盖。两个函数自行读取旧批次参数。
        // test_optimized_proposed_exp1();
        rerun_optimized_proposed_all_experiments();

        // 旧带宽/定位误差入口仍保留，但需要单独设置旧input_root及新的output_name，不能传新格式输入。
        // exp4_different_total_bandwidth(legacy_options);
        // exp5_different_location_error(legacy_options);

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "实验执行失败：" << error.what() << std::endl;
        return 1;
    }
}
#endif // TON_RERUN_TESTING

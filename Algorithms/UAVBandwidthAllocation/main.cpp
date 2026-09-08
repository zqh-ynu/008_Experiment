// Visual Studio 直接运行入口：集中设置实验参数，按条件/实例/算法保存结果并支持断点续跑。
// 不调用旧批次脚本、自动测试、监测或绘图；原有物理配置仍由各实验加载。
// 常规入口读取固定 ToN 批次，顺序执行 EXP1--EXP4；历史 HardFirst 重跑函数保留但默认不调用。
#include "experiments.h"

/// 临时重跑固定 run_01 的五个 EXP1 条件、各 ID 1--10，仅覆盖 AlgHardFirst.csv。
/// 无参数/返回值；读取原 run_info 的输入、种子和预算，其余五种方法只读。
/// 每条件全部成功后才用临时文件替换原 CSV，无备份；失败抛出条件/实例上下文。
/// 调用前必须确保没有其他进程写入 run_01；每次调用重跑全部条件，不做断点恢复。
void rerun_hardfirst_exp1_run01() {
    const fs::path root =
        "E:/Research/My_paper/2_Papers/008/008_Experiment/ExperimentsData/ExperimentsResults/"
        "EXP1_user_num/ToN_simple/run_01";
    const vector<string> keys = {"1000", "2000", "3000", "4000", "5000"};
    constexpr size_t hardfirst = 4;
    json info = read_experiment_json(root / "run_info.json");
    if (info.at("schema") != SIMPLE_RUN_SCHEMA || info.at("experiment") != "EXP1_user_num" ||
        info.at("methods") != method_name_list || method_name_list.at(hardfirst) != "AlgHardFirst" ||
        info.at("instance_count") != 10 || info.at("conditions").size() != keys.size())
        throw runtime_error("HardFirst rerun: expected run_01 EXP1, five conditions and IDs 1--10");

    ExperimentRunOptions options;
    options.instance_count = 10;
    const auto seed = info.at("master_seed").get<int64_t>();
    const auto trials = info.at("rounding_trials").get<int64_t>();
    if (!info.at("master_seed").is_number_integer() || seed < 0 || seed > UINT32_MAX ||
        !info.at("rounding_trials").is_number_integer() || trials <= 0 || trials > INT_MAX)
        throw runtime_error("HardFirst rerun: invalid recorded seed/trial count");
    options.master_seed = static_cast<uint32_t>(seed);
    options.rounding_trials = static_cast<int>(trials);
    options.ton_epsilon = info.at("ton_epsilon").get<double>();
    options.output_name = "run_01";
    options.conditions = keys;
    validate_run_options(options);
    const string config = info.at("config_path").get<string>();
    if (read_experiment_json(config) != info.at("channel_config") ||
        info.at("unit_para") != unit_para || info.at("abs_tolerance") != ALLOCATION_ABS_TOL ||
        info.at("rel_tolerance") != ALLOCATION_REL_TOL ||
        info.at("build_profile") != experiment_build_profile())
        throw runtime_error("HardFirst rerun: physical configuration, units, tolerances or build profile changed");
    load_global_channel_config(config);

    vector<ExperimentCondition> conditions;
    vector<vector<EXPResult>> means(keys.size(), vector<EXPResult>(method_name_list.size()));
    // 所有输入与保留结果在任何写入之前检查；不读取/导入旧 HardFirst 行。
    for (size_t c = 0; c < keys.size(); ++c) {
        const auto& recorded = info.at("conditions").at(c);
        if (recorded.at("key") != keys[c] || recorded.at("inputs").size() != 10 ||
            recorded.at("bandwidth_mhz") != 40.0)
            throw runtime_error("HardFirst rerun: unexpected condition/input count/bandwidth at " + keys[c]);
        ExperimentCondition condition{keys[c], {}, {}, recorded.at("bandwidth_mhz").get<double>()};
        for (int i = 0; i < 10; ++i) {
            const auto& input = recorded.at("inputs").at(i);
            const string user = input.at("user_path").get<string>(), uav = input.at("uav_path").get<string>();
            if (input.at("id") != i + 1 || allocation_instance_id(user) != i + 1 ||
                allocation_instance_id(uav) != i + 1 || !ifstream(user, ios::binary) || !ifstream(uav, ios::binary))
                throw runtime_error("HardFirst rerun: unreadable/mismatched input " + keys[c] + "/ID " + to_string(i + 1));
            condition.users.push_back(user); condition.uavs.push_back(uav);
        }
        if (!fs::is_regular_file(root / keys[c] / "AlgHardFirst.csv"))
            throw runtime_error("HardFirst rerun: missing original CSV in " + keys[c]);
        for (size_t m = 0; m < method_name_list.size(); ++m) {
            if (m == hardfirst) continue;
            const auto records = read_result_csv(root / keys[c] / (method_name_list[m] + ".csv"), true);
            if (records.size() != 10)
                throw runtime_error("HardFirst rerun: incomplete retained results " + keys[c] + "/" + method_name_list[m]);
            for (int id = 1; id <= 10; ++id)
                if (!records.count(id) || records.at(id).seed !=
                    derive_algorithm_seed(options.master_seed, "EXP1_user_num", keys[c], id, method_name_list[m]))
                    throw runtime_error("HardFirst rerun: retained ID/seed mismatch " + keys[c] + "/" + method_name_list[m]);
            means[c][m] = average_valid_attempts(records, 10);
        }
        conditions.push_back(std::move(condition));
    }

    const string stamp = to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    const fs::path info_temp = root / ("run_info.hardfirst_" + stamp + ".tmp");
    // 元数据也先完整关闭再重命名，避免中断写坏原 run_info；失败保留临时文件。
    const auto save_info = [&]() {
        if (fs::exists(info_temp)) throw runtime_error("HardFirst rerun: metadata temporary file already exists");
        ofstream output(info_temp, ios::binary);
        output << info.dump(2) << '\n';
        output.flush(); output.close();
        if (!output) throw runtime_error("HardFirst rerun: metadata write failed");
        fs::rename(info_temp, root / "run_info.json");
    };
    info["summary"]["status"] = "stale";
    // 不改全局旧 algorithm_version/reuse_source：仅第五种方法具有以下覆盖版本。
    info["hardfirst_rerun"] = {{"algorithm_version", SIMPLE_ALGORITHM_VERSION},
        {"da_policy", hard_first_da_policy()}, {"status", "incomplete"},
        {"completed_conditions", json::array()}, {"pending_condition", ""}, {"pending_temp_file", ""}};
    save_info();

    for (size_t c = 0; c < conditions.size(); ++c) {
        const auto& condition = conditions[c];
        const fs::path target = root / condition.key / "AlgHardFirst.csv";
        const fs::path temporary = root / condition.key / ("AlgHardFirst.rerun_" + stamp + ".tmp");
        int id = 0;
        try {
            if (fs::exists(temporary)) throw runtime_error("Temporary result file already exists");
            ofstream output(temporary, ios::binary);
            if (!output) throw runtime_error("Cannot open temporary result file");
            output << CSV_HEADER << '\n';
            for (id = 1; id <= 10; ++id) {
                cout << "[RERUN] EXP1 / " << condition.key << " / ID " << id << " / AlgHardFirst" << std::endl;
                SystemMd model(condition.users[id - 1], condition.uavs[id - 1], config);
                for (auto& uav : model.uavs) uav.total_bandwidth = condition.bandwidth_mhz * unit_para;
                model.init_SystemModel();
                validate_allocation_model(model);
                auto result = run_algorithm(model, hardfirst, options,
                    derive_algorithm_seed(options.master_seed, "EXP1_user_num", condition.key, id, "AlgHardFirst"));
                result.instance_id = id;
                if (!algorithm_status_valid(result.status)) {
                    const string reason = result.diagnostics.events.empty() ? "No diagnostic" : result.diagnostics.events.back();
                    throw runtime_error(algorithm_status_name(result.status) + ": " + reason);
                }
                output << run_result_to_csv(result) << '\n';
                if (!output) throw runtime_error("Temporary CSV write failed");
            }
            id = 10;
            output.flush(); output.close();
            if (!output) throw runtime_error("Temporary CSV flush/close failed");
            means[c][hardfirst] = average_valid_attempts(read_result_csv(temporary, true), 10);
            // 重命名前记下 pending；若恰在 CSV/JSON 更新间中断，可据临时文件是否仍在辨认。
            info["hardfirst_rerun"]["pending_condition"] = condition.key;
            info["hardfirst_rerun"]["pending_temp_file"] = temporary.filename().string();
            save_info();
            fs::rename(temporary, target); // 同目录 regular-file rename 替换；不先删除旧 CSV。
            info["hardfirst_rerun"]["completed_conditions"].push_back(condition.key);
            info["hardfirst_rerun"]["pending_condition"] = "";
            info["hardfirst_rerun"]["pending_temp_file"] = "";
            save_info();
            cout << "[REPLACED] " << target.string() << " (10 results)" << std::endl;
        } catch (const exception& error) {
            throw runtime_error("HardFirst rerun / " + condition.key + " / ID " + to_string(id) +
                ": " + error.what() + "; inspect hardfirst_rerun progress and retained temporary files");
        }
    }
    info["summary"]["status"] = "writing";
    save_info();
    exportAllSummaryCSV((root / "summary").string(), keys, means);
    info["summary"] = {{"status", "complete"}, {"instance_count", 10}};
    info["hardfirst_rerun"]["status"] = "complete";
    save_info();
    cout << "HardFirst rerun complete: 50 new results, other five methods unchanged, 12 summaries refreshed." << std::endl;
}

/// 使用下方固定批次和参数顺序执行 EXP1--EXP4；argc 必须为 1，不接收旧 CLI 参数。
/// 成功返回 0；遇到错误保留已写结果并返回 1，后续实验不再自动启动。
int main(int argc, char*[]) {
    try {
        if (argc != 1)
            throw invalid_argument("Legacy batch/test CLI is disabled. Select experiments and parameters in main.cpp.");

        // 临时只重跑 run_01 的 HardFirst：需要时同时取消下面两行注释。
        // rerun_hardfirst_exp1_run01();
        // return 0;  // 临时重跑完成后退出，不执行下面的常规实验。

        // ===== 日常只需修改这一参数区，再在 Visual Studio 中编译、运行 =====
        ExperimentRunOptions options;
        options.instance_count = 10;         // 每条件先算 ID 1--10；以后改为 30，只补算尚未保存的 ID 11--30。
        options.master_seed = 20260905u;     // 保持不变，使新算、续跑和旧记录使用一致的逐实例种子。
        options.rounding_trials = 2;         // AlgRelaxRound 的既有随机舍入次数。
        options.ton_epsilon = 0.1;           // ToN 近似参数，要求 0 < epsilon < 0.5。
        options.input_root = (fs::path(experimentDataPath) / "data_ToN" / "2026-09-07").string();
        options.output_name = "run_ton_01";  // 新 ToN 输入独立输出；保留旧 run_01，不导入旧指标或耗时。
        options.conditions.clear();         // 空列表使四个实验各自选择完整的标准条件，不能共用 EXP1 的条件键。
        options.reuse_exp1_root = "";

        // 先手动运行 generate_instances_ToN.py 备好固定 30 个输入，再由 Visual Studio 启动这里。
        // EXP1：1000/2000/3000/4000/5000 用户，10 UAV，每 UAV 40 MHz。
        exp1_different_user_number(options);

        // EXP2：5/10/15/20 UAV，3000 用户，每 UAV 40 MHz。
        exp2_different_uav_number(options);

        // EXP3：Hard 比例 0/0.2/0.4/0.6/0.8/1.0，3000 用户、10 UAV，每 UAV 40 MHz。
        exp3_different_hard_user_ratio(options);

        // EXP4：每 UAV 10/20/30/40/50 MHz，3000 用户、10 UAV。
        exp4_different_total_bandwidth(options);
        return 0;
    } catch (const exception& error) {
        cerr << "实验停止，已写入的结果保持不变。\n原因: " << error.what() << std::endl;
        return 1;
    }
}

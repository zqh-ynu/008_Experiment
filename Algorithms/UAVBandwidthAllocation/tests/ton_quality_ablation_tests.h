#pragma once
// 本文件提供手动启用的纯合成质量消融测试；不读正式输入、不写结果、不启动正式入口。
// main.cpp 先包含 ton_quality_ablation.h，再在 TON_QUALITY_ABLATION_TESTS 模式包含本文件。

namespace ton_quality_tests {

/// 记录成功断言数；失败抛异常使测试入口返回非零，不依赖会被 NDEBUG 关闭的 assert。
inline void check(bool condition, const string& message, size_t& count) {
    if (!condition) throw runtime_error("QUALITY TEST FAILED: " + message);
    ++count;
}

/// 创建归一化合成模型，直接指定有效信道与阈值，避免把物理配置差异混入算法机制测试。
/// caps 按 UAV/用户索引；所有用户位于覆盖内，hard 用户必须排在 elastic 用户之前。
inline SystemMd model(const vector<User>& users, const vector<vector<double>>& caps, double budget) {
    SystemMd result;
    result.users = users; result.n1 = 0; result.n2 = 0; result.m = static_cast<int>(caps.size());
    for (const auto& user : users) if (user.uType == HARD_UTILITY) ++result.n1; else ++result.n2;
    result.cap_list = caps;
    result.dis_list.assign(caps.size(), vector<double>(users.size(), 0));
    result.SNRave_list = result.dis_list; result.SNRth_list = result.dis_list; result.M_list = result.dis_list;
    result.Bth_list.assign(caps.size(), vector<double>(result.n1, 0));
    for (size_t k = 0; k < caps.size(); ++k) {
        result.uavs.emplace_back(static_cast<int>(k), 0, 0, 0, budget);
        result.uav_serviceable_users_map[static_cast<int>(k)] = users;
        for (int i = 0; i < result.n1; ++i) result.Bth_list[k][i] = users[i].rMin / caps[k][i];
    }
    validate_allocation_model(result);
    return result;
}

/// 构造预算为 10、成本/效用为 (6,12),(5,9.5),(5,9.5) 的三 Hard 用户背包。
inline SystemMd hard_fixture(double budget = 10) {
    return model({User(0, HARD_UTILITY, 12, 0, 0, 0, 1, 0.01),
        User(1, HARD_UTILITY, 9.5, 0, 0, 0, 1, 0.01), User(2, HARD_UTILITY, 9.5, 0, 0, 0, 1, 0.01)},
        {{1.0 / 6.0, 1.0 / 5.0, 1.0 / 5.0}}, budget);
}

/// 保留分配和全部缓存值及逐用户投影，以精确 JSON 数值对比诊断开关前后输出。
inline json allocation(const AllocationPair& result) {
    json out = {{"uavs", json::array()}, {"users", json::array()}};
    for (const auto& a : result.first)
        out["uavs"].push_back({a.uav_id, a.allocatedList, a.allocatedBandwidth, a.allocatedValue,
            a.totalValue, a.totalWeight, a.hardValue, a.hardWeight, a.elasticValue, a.elasticWeight});
    for (const auto& u : result.second)
        out["users"].push_back({u.first, u.second.uav_id, u.second.allocated_bandwidth, u.second.utility});
    return out;
}

/// 穷举八个子集验证真实机制最优值，再核对 Fast 舍入、Better DP 与不改变输出的诊断。
inline void test_mechanism(size_t& count) {
    const auto input = hard_fixture();
    double optimum = 0;
    for (unsigned mask = 0; mask < 8; ++mask) {
        double cost = 0, value = 0;
        for (int i = 0; i < 3; ++i) if (mask & (1u << i)) {
            cost += input.Bth_list[0][i]; value += input.users[i].weight * std::log2(1 + input.users[i].rMin);
        }
        if (cost <= 10) optimum = std::max(optimum, value);
    }
    BAProblem problem(input);
    vector<double> zero(3, 0);
    TonFastDiagnostics fast_d;
    const auto fast = problem.AlgFast_singleUAV_ToN(input.uavs[0], input.users, zero, zero, &fast_d);
    check(allocation_near(optimum, 19) && allocation_near(fast.totalValue, 12), "枚举 OPT=19、Fast=12", count);
    check(fast_d.unsafe_count == 1 && fast_d.return_reason == "round_without_unsafe", "unsafe 舍入分支", count);
    for (const auto& setting : ton_quality::variants()) if (setting.selector == 3) {
        TonFasterOptions options; options.certificate_early_return = setting.early_return;
        TonSingleUavDiagnostics diagnostic;
        const auto better = problem.AlgBetter_singleUAV_ToN_faster(input.uavs[0], input.users,
            zero, zero, setting.epsilon, options, &diagnostic);
        check(allocation_near(better.totalValue, optimum), "Better 找到两个较低密度用户组合", count);
        check(diagnostic.dp_entered && !diagnostic.certificate_satisfied, "困难实例不能提前认证", count);
        check(std::isfinite(diagnostic.best_upper) && diagnostic.best_upper + 1e-6 >= optimum, "记录有效对偶上界", count);
    }
    const auto easy = hard_fixture(16);
    BAProblem easy_problem(easy);
    TonSingleUavDiagnostics on, off;
    const auto a = easy_problem.AlgBetter_singleUAV_ToN_faster(easy.uavs[0], easy.users, zero, zero, 0.1, {}, &on);
    TonFasterOptions no_early; no_early.certificate_early_return = false;
    const auto b = easy_problem.AlgBetter_singleUAV_ToN_faster(easy.uavs[0], easy.users, zero, zero, 0.1, no_early, &off);
    check(on.certificate_satisfied && !on.dp_entered && on.return_reason == "certificate_fast", "默认早退", count);
    check(off.certificate_satisfied && off.dp_entered && allocation_near(a.totalValue, b.totalValue), "关闭早退仍保留证书观测", count);
}

/// 对 Hard、纯 Elastic、多 UAV 混合及零预算模型验证每个设置的诊断等价与物理契约。
inline void test_invariance(size_t& count) {
    const auto elastic = model({User(0, ELASTIC_UTILITY, 2, 0, 0, 0, 0, 0),
        User(1, ELASTIC_UTILITY, 3, 0, 0, 0, 0, 0)}, {{1, 2}}, 10);
    const auto mixed = model({User(0, HARD_UTILITY, 5, 0, 0, 0, 1, 0.01),
        User(1, ELASTIC_UTILITY, 2, 0, 0, 0, 0, 0), User(2, ELASTIC_UTILITY, 4, 0, 0, 0, 0, 0)},
        {{0.2, 1, 3}, {0.3, 3, 1}}, 10);
    for (const auto& input : {hard_fixture(), hard_fixture(0), elastic, mixed}) {
        for (const auto& setting : ton_quality::variants()) {
            TonNetworkDiagnostics diagnostic;
            auto plain = ton_quality::solve(input, setting, 1, nullptr);
            auto observed = ton_quality::solve(input, setting, 1, &diagnostic);
            check(algorithm_status_valid(plain.status) && algorithm_status_valid(observed.status), "合成求解均成功", count);
            check(allocation(plain.allocation) == allocation(observed.allocation), "诊断开关不改变分配与缓存", count);
            check(ton_quality::metric_comparison(plain.metrics, observed.metrics).at("exact").get<bool>(), "非耗时指标精确一致", count);
            ton_quality::validate_cached_results(input, observed);
            check(allocation_near(diagnostic.post_residual_utility, observed.metrics.total_utility), "残余后效用与重算一致", count);
            check(diagnostic.post_residual_utility + allocation_tolerance(diagnostic.post_residual_utility,
                diagnostic.pre_residual_utility) >= diagnostic.pre_residual_utility, "残余阶段不降低效用", count);
            for (const auto& call : diagnostic.calls)
                if (call.phase == "residual") check(call.selector == 1, "残余阶段始终为 Fast", count);
            if (input.n1 == 0) for (const auto& call : diagnostic.calls)
                check(!call.oracle.dp_entered, "纯凹模型不强制制造 DP", count);
            if (setting.name == "Fast" || setting.name == "Better_current") {
                BAProblem legacy_call(input);
                const auto default_result = legacy_call.Appro_multiUAV_ToN(input.uavs, input.users, setting.selector, setting.epsilon);
                check(allocation(default_result) == allocation(plain.allocation), "原有调用形式保持默认结果", count);
            }
        }
    }
}

/// 验证不同冻结状态不产生局部差值、零分母为 NA、五设置矩阵完整且非法 epsilon 抛异常。
inline void test_reporting(size_t& count) {
    check(ton_quality::variants().size() == 5, "固定五设置矩阵", count);
    TonOracleTrace a, b;
    a.uav_id = b.uav_id = 0; a.budget = b.budget = 10;
    a.current_utilities = {0, 1}; b.current_utilities = {0, 2};
    check(!ton_quality::same_state(a, b), "冻结状态不同不可直接比较", count);
    a.phase = b.phase = "greedy"; a.oracle.final_value = 2; b.oracle.final_value = 1;
    TonNetworkDiagnostics current, reference; current.calls = {a}; reference.calls = {b};
    const auto unmatched = ton_quality::trace_json(current, &reference).at("calls").at(0);
    check(!unmatched.at("same_state_as_fast").get<bool>() && unmatched.at("delta_vs_fast_path").is_null(),
        "路径分叉时导出 NA 而不是错误差值", count);
    reference.calls[0].current_utilities = a.current_utilities;
    const auto matched = ton_quality::trace_json(current, &reference).at("calls").at(0);
    check(matched.at("same_state_as_fast").get<bool>() && matched.at("delta_vs_fast_path") == 1,
        "相同冻结状态允许局部差值", count);
    EXPResult zero, positive; positive.total_utility = 1;
    const auto row = ton_quality::paired_row(1, ton_quality::variants()[1], positive, zero);
    check(row.at("relative_gain_pct").is_null() && row.at("outcome") == "win", "零分母与胜负分别表达", count);
    check(ton_quality::oracle_json(TonSingleUavDiagnostics()).at("best_upper").is_null(), "未计算上界不是零", count);
    const auto input = hard_fixture(); BAProblem problem(input); vector<double> z(3, 0);
    bool threw = false;
    try { problem.AlgBetter_singleUAV_ToN_faster(input.uavs[0], input.users, z, z, 0); }
    catch (const std::invalid_argument&) { threw = true; }
    check(threw, "非法 epsilon 明确拒绝", count);
}

/// 执行纯合成测试并在控制台输出断言数；没有文件副作用，失败由 main 捕获返回 1。
inline int run() {
    size_t count = 0;
    test_mechanism(count); test_invariance(count); test_reporting(count);
    cout << "[QUALITY SYNTHETIC PASS] assertions=" << count
        << "; synthetic only; formal networks not run" << std::endl;
    return 0;
}

} // namespace ton_quality_tests

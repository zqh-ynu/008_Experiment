#pragma once
// 本文件提供 ToN 质量消融的显式设置和只读诊断容器；不含全局开关或文件写入。
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

/// Faster 的实验设置；默认保留正式算法的证书早退行为。
struct TonFasterOptions {
    bool certificate_early_return = true;
};

/// Fast 单次调用的实际分支和舍入信息；空结果的真实值为零。
struct TonFastDiagnostics {
    std::size_t candidate_count = 0, retained_count = 0;
    std::size_t concave_count = 0, nonconcave_count = 0, unsafe_count = 0;
    int unsafe_user_id = -1;
    double unsafe_bandwidth = 0, unsafe_tau = 0, value = 0;
    std::string return_reason = "empty";
};

/// 单 UAV oracle 的观测值；NaN 表示未计算，不得导出为零或声称上界已验证。
struct TonSingleUavDiagnostics {
    TonFastDiagnostics fast;
    std::size_t candidate_count = 0, retained_count = 0;
    std::size_t concave_count = 0, nonconcave_count = 0;
    double epsilon = std::numeric_limits<double>::quiet_NaN();
    bool certificate_early_return = true, certificate_attempted = false;
    bool certificate_satisfied = false, dp_entered = false;
    std::size_t dual_evaluations = 0, active_bound = 0;
    double best_upper = std::numeric_limits<double>::quiet_NaN();
    double fast_value = std::numeric_limits<double>::quiet_NaN();
    double candidate_value = std::numeric_limits<double>::quiet_NaN();
    double final_value = std::numeric_limits<double>::quiet_NaN();
    double delta = std::numeric_limits<double>::quiet_NaN();
    int profit_limit = 0;
    std::string return_reason = "exception_or_incomplete";
};

/// 一次实际 oracle 调用；保留完整冻结向量，以便同实例内逐值核对状态而非仅相信哈希。
struct TonOracleTrace {
    std::string phase;
    std::size_t round = 0;
    int uav_id = -1, selector = 0;
    double budget = 0;
    bool selected = false;
    std::vector<int> candidate_ids;
    std::vector<double> current_utilities, base_bandwidths;
    TonSingleUavDiagnostics oracle;
};

/// 单次多 UAV 求解的局部轨迹和残余阶段前后真实效用；由调用方持有，无共享状态。
struct TonNetworkDiagnostics {
    std::vector<TonOracleTrace> calls;
    std::vector<int> selection_order;
    double pre_residual_utility = std::numeric_limits<double>::quiet_NaN();
    double post_residual_utility = std::numeric_limits<double>::quiet_NaN();
};

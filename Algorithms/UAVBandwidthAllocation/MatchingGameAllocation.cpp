#include "EntityDefinition.h"


using namespace Ipopt;

// ====================================================================
//  IPOPT NLP 子问题：单个UAV的Elastic用户带宽分配
//
//  max  Σ_j  w_j * log2(1 + b_j * cap_j)
//  s.t. Σ_j  b_j <= B_remain
//       b_j >= 0
//
//  等价于 min -Σ_j w_j * log2(1 + b_j * cap_j)
// ====================================================================
class ElasticBandwidthNLP : public TNLP
{
public:
    // 输入参数
    int n_users;                        // Elastic用户数
    std::vector<double> weights;        // w_j
    std::vector<double> capacities;     // cap_j = log2(1 + SNR_avg)
    double B_total;                     // 可用总带宽

    // 输出结果
    std::vector<double> opt_bandwidth;  // 最优带宽分配
    double opt_obj_value;               // 最优目标函数值（最大化值）

    ElasticBandwidthNLP(int n, const std::vector<double>& w,
        const std::vector<double>& cap, double B)
        : n_users(n), weights(w), capacities(cap), B_total(B),
        opt_bandwidth(n, 0.0), opt_obj_value(0.0)
    {
    }

    virtual ~ElasticBandwidthNLP() {}

    // ---- TNLP 接口实现 ----

    // 问题维度
    virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style) override
    {
        n = n_users;          // 变量数：每个用户一个带宽变量 b_j
        m = 1;                // 约束数：一个总带宽约束 Σ b_j <= B_total
        nnz_jac_g = n_users;  // 约束Jacobian非零元素数
        nnz_h_lag = n_users;  // Lagrangian Hessian非零元素数（对角矩阵）
        index_style = TNLP::C_STYLE;
        return true;
    }

    // 变量和约束的上下界
    virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u) override
    {
        // 变量界：b_j >= 0, b_j <= B_total
        for (Index j = 0; j < n; j++) {
            x_l[j] = 0.0;
            x_u[j] = B_total;
        }
        // 约束界：Σ b_j <= B_total  =>  g_l = -∞, g_u = B_total
        g_l[0] = 0.0;
        g_u[0] = B_total;
        return true;
    }

    // 初始点：均分带宽
    virtual bool get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m, bool init_lambda, Number* lambda) override
    {
        if (init_x) {
            double avg = B_total / n_users;
            for (Index j = 0; j < n; j++) {
                x[j] = avg;
            }
        }
        return true;
    }

    // 目标函数：min -Σ w_j * log2(1 + b_j * cap_j)
    virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) override
    {
        obj_value = 0.0;
        for (Index j = 0; j < n; j++) {
            double r = x[j] * capacities[j];
            obj_value -= weights[j] * log2(1.0 + r);
        }
        return true;
    }

    // 目标函数梯度：∂f/∂b_j = -w_j * cap_j / ((1 + b_j * cap_j) * ln2)
    virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) override
    {
        const double ln2 = log(2.0);
        for (Index j = 0; j < n; j++) {
            double r = x[j] * capacities[j];
            grad_f[j] = -weights[j] * capacities[j] / ((1.0 + r) * ln2);
        }
        return true;
    }

    // 约束函数值：g(x) = Σ b_j
    virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) override
    {
        g[0] = 0.0;
        for (Index j = 0; j < n; j++) {
            g[0] += x[j];
        }
        return true;
    }

    // 约束Jacobian结构和值
    virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
        Index m, Index nele_jac, Index* iRow, Index* jCol,
        Number* values) override
    {
        if (values == NULL) {
            // 返回稀疏结构：约束0对所有变量的偏导
            for (Index j = 0; j < n; j++) {
                iRow[j] = 0;
                jCol[j] = j;
            }
        }
        else {
            // ∂g/∂b_j = 1
            for (Index j = 0; j < n; j++) {
                values[j] = 1.0;
            }
        }
        return true;
    }

    // Lagrangian Hessian（下三角）
    // L = σ * f(x) + λ * g(x)
    // ∂²L/∂b_j² = σ * w_j * cap_j² / ((1 + b_j * cap_j)² * ln2)
    // 无交叉项，Hessian是对角矩阵
    virtual bool eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess,
        Index* iRow, Index* jCol, Number* values) override
    {
        if (values == NULL) {
            // 稀疏结构：对角元素
            for (Index j = 0; j < n; j++) {
                iRow[j] = j;
                jCol[j] = j;
            }
        }
        else {
            const double ln2 = log(2.0);
            for (Index j = 0; j < n; j++) {
                double r = x[j] * capacities[j];
                double denom = (1.0 + r) * (1.0 + r) * ln2;
                // 注意：f是最小化 -Σ(...)，所以二阶导为正
                values[j] = obj_factor * weights[j] * capacities[j] * capacities[j] / denom;
                // g是线性的，二阶导为0，无需加lambda项
            }
        }
        return true;
    }

    // 求解完成后的回调
    virtual void finalize_solution(SolverReturn status, Index n,
        const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda,
        Number obj_value, const IpoptData* ip_data,
        IpoptCalculatedQuantities* ip_cq) override
    {
        opt_obj_value = -obj_value; // 转回最大化值
        opt_bandwidth.resize(n);
        for (Index j = 0; j < n; j++) {
            opt_bandwidth[j] = x[j];
        }
    }
};


/**
 * @brief 基于匹配博弈与IPOPT优化的三阶段联合用户关联与带宽分配算法
 *
 * 算法流程：
 *   第一阶段：Hard (RT) 用户关联与保底资源分配 (DA matching)
 *   第二阶段：Elastic (BE) 用户关联 (DA matching，含UAV配额限制)
 *   第三阶段：Elastic (BE) 带宽分配 (IPOPT凸优化求解)
 *
 * @return pair<vector<KnapsackResult>, map<int, UserResult>>
 */
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::MatchingGameAllocation()
{
    const int M = sysModel.m;
    const int N = sysModel.n1 + sysModel.n2;
    const int N_hard = sysModel.n1;
    const int N_elastic = sysModel.n2;

    // Elastic用户配额：每个UAV最多接受 ceil(N_elastic / M) 个Elastic用户
    const int elastic_quota = (N_elastic + M - 1) / M;

    // ==================== 初始化结果容器 ====================
    std::vector<KnapsackResult> uav_results(M);
    for (int k = 0; k < M; k++) {
        uav_results[k].uav_id = sysModel.uavs[k].ID;
        clean_KnapsackResult(uav_results[k]);
        uav_results[k].uav_id = sysModel.uavs[k].ID;
    }
    std::map<int, UserResult> user_results;
    for (auto& user : sysModel.users) {
        UserResult ur;
        ur.uav_id = -1;
        ur.allocated_bandwidth = 0;
        ur.utility = 0;
        user_results[user.ID] = ur;
    }

    // ==================== 分离 Hard / Elastic 用户 ====================
    std::vector<User> all_hard_users, all_elastic_users;
    for (auto& user : sysModel.users) {
        if (user.uType == HARD_UTILITY)
            all_hard_users.push_back(user);
        else
            all_elastic_users.push_back(user);
    }

    // 每个UAV可服务的 Hard / Elastic 用户ID集合
    std::map<int, std::vector<int>> uav_serviceable_hard;
    std::map<int, std::vector<int>> uav_serviceable_elastic;
    for (int k = 0; k < M; k++) {
        int uav_id = sysModel.uavs[k].ID;
        if (sysModel.uav_serviceable_users_map.count(uav_id)) {
            for (auto& u : sysModel.uav_serviceable_users_map[uav_id]) {
                if (u.uType == HARD_UTILITY)
                    uav_serviceable_hard[k].push_back(u.ID);
                else
                    uav_serviceable_elastic[k].push_back(u.ID);
            }
        }
    }

    // 索引映射
    std::map<int, int> uid_to_idx;
    for (int i = 0; i < N; i++) {
        uid_to_idx[sysModel.users[i].ID] = i;
    }
    std::map<int, int> uav_id_to_idx;
    for (int k = 0; k < M; k++) {
        uav_id_to_idx[sysModel.uavs[k].ID] = k;
    }

    // ==================== 辅助 lambda ====================
    auto get_SNRave_dB = [&](int k, int uid) -> double {
        return sysModel.SNRave_list[k][uid_to_idx[uid]];
        };
    auto get_SNRave_linear = [&](int k, int uid) -> double {
        return pow(10.0, get_SNRave_dB(k, uid) / 10.0);
        };
    auto get_Bth = [&](int k, int uid) -> double {
        int j = uid_to_idx[uid];
        if (j < N_hard)
            return sysModel.Bth_list[k][j];
        return INFINITY;
        };
    auto get_hard_utility = [&](int k, int uid) -> double {
        int j = uid_to_idx[uid];
        double cap = sysModel.cap_list[k][j];
        double b_min = get_Bth(k, uid);
        double SNR_avg_dB = sysModel.SNRave_list[k][j];
        return sysModel.users[j].hard_utility(b_min, cap, SNR_avg_dB);
        };
    auto get_hard_efficiency = [&](int k, int uid) -> double {
        double uti = get_hard_utility(k, uid);
        double bth = get_Bth(k, uid);
        if (bth <= EPS || bth == INFINITY) return 0.0;
        return uti / bth;
        };
    // UAV对Elastic用户的偏好值
    auto get_elastic_orientation = [&](int k, int uid) -> double {
        int j = uid_to_idx[uid];
        double w_j = sysModel.users[j].weight;
        double snr_linear = get_SNRave_linear(k, uid);
        return w_j * log2(1.0 + log2(1.0 + snr_linear));
        };

    // ====================================================================
    //          第一阶段：Hard 用户 DA 匹配（与原版相同）
    // ====================================================================

    // 构建Hard用户偏好列表：按SNR降序
    std::map<int, std::vector<int>> preference_hard;
    for (auto& u : all_hard_users) {
        int uid = u.ID;
        std::vector<std::pair<double, int>> snr_uav_pairs;
        for (int k = 0; k < M; k++) {
            auto& slist = uav_serviceable_hard[k];
            if (std::find(slist.begin(), slist.end(), uid) != slist.end()) {
                double bth = get_Bth(k, uid);
                if (bth < INFINITY && bth > 0) {
                    snr_uav_pairs.push_back({ get_SNRave_dB(k, uid), k });
                }
            }
        }
        std::sort(snr_uav_pairs.begin(), snr_uav_pairs.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        for (auto& p : snr_uav_pairs) {
            preference_hard[uid].push_back(p.second);
        }
    }

    // DA匹配
    std::map<int, std::vector<int>> uav_accepted_hard;
    std::map<int, double> uav_hard_bw_used;
    for (int k = 0; k < M; k++) uav_hard_bw_used[k] = 0.0;

    std::map<int, int> hard_pref_ptr;
    std::map<int, bool> hard_matched;
    for (auto& u : all_hard_users) {
        hard_pref_ptr[u.ID] = 0;
        hard_matched[u.ID] = false;
    }

    bool has_proposal = true;
    while (has_proposal) {
        has_proposal = false;
        std::map<int, std::vector<int>> new_proposals;
        for (auto& u : all_hard_users) {
            int uid = u.ID;
            if (hard_matched[uid]) continue;
            if (hard_pref_ptr[uid] >= (int)preference_hard[uid].size()) continue;
            int target_uav = preference_hard[uid][hard_pref_ptr[uid]];
            new_proposals[target_uav].push_back(uid);
            has_proposal = true;
        }
        if (!has_proposal) break;

        for (int k = 0; k < M; k++) {
            double B_UAV = sysModel.uavs[k].total_bandwidth;
            std::vector<int> candidates = uav_accepted_hard[k];
            if (new_proposals.count(k)) {
                for (int uid : new_proposals[k])
                    candidates.push_back(uid);
            }
            if (candidates.empty()) continue;

            std::sort(candidates.begin(), candidates.end(),
                [&](int a, int b) {
                    return get_hard_efficiency(k, a) > get_hard_efficiency(k, b);
                });

            std::vector<int> accepted;
            double bw_sum = 0.0;
            for (int uid : candidates) {
                double bth = get_Bth(k, uid);
                if (bw_sum + bth <= B_UAV + EPS) {
                    accepted.push_back(uid);
                    bw_sum += bth;
                }
                else {
                    break;
                }
            }

            std::set<int> accepted_set(accepted.begin(), accepted.end());
            uav_accepted_hard[k] = accepted;
            uav_hard_bw_used[k] = bw_sum;

            for (int uid : candidates) {
                if (accepted_set.count(uid)) {
                    hard_matched[uid] = true;
                }
                else {
                    hard_matched[uid] = false;
                    hard_pref_ptr[uid]++;
                }
            }
        }
    }

    // 第一阶段结果写入 + 计算剩余带宽
    std::vector<double> B_remain(M, 0.0);
    for (int k = 0; k < M; k++) {
        double B_UAV = sysModel.uavs[k].total_bandwidth;
        B_remain[k] = B_UAV - uav_hard_bw_used[k];
        if (B_remain[k] < EPS) B_remain[k] = 0.0;

        for (int uid : uav_accepted_hard[k]) {
            int j = uid_to_idx[uid];
            double bth = get_Bth(k, uid);
            double cap = sysModel.cap_list[k][j];
            double SNR_avg_dB = sysModel.SNRave_list[k][j];
            double uti = sysModel.users[j].hard_utility(bth, cap, SNR_avg_dB);

            add_KnapsackResult(uav_results[k], sysModel.users[j], bth, uti);

            user_results[uid].uav_id = sysModel.uavs[k].ID;
            user_results[uid].allocated_bandwidth = bth;
            user_results[uid].utility = uti;
        }
        cout << "[Phase1] UAV " << k << ": Hard BW used = " << uav_hard_bw_used[k]
            << ", remaining = " << B_remain[k] << endl;
    }

    // ====================================================================
    //      第二阶段：Elastic 用户 DA 匹配（含UAV配额限制）
    // ====================================================================

    // 构建Elastic用户偏好列表：按SNR降序，仅考虑有剩余带宽的UAV
    std::map<int, std::vector<int>> preference_elastic;
    for (auto& u : all_elastic_users) {
        int uid = u.ID;
        std::vector<std::pair<double, int>> snr_uav_pairs;
        for (int k = 0; k < M; k++) {
            if (B_remain[k] <= EPS) continue;
            auto& slist = uav_serviceable_elastic[k];
            if (std::find(slist.begin(), slist.end(), uid) != slist.end()) {
                snr_uav_pairs.push_back({ get_SNRave_dB(k, uid), k });
            }
        }
        std::sort(snr_uav_pairs.begin(), snr_uav_pairs.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        for (auto& p : snr_uav_pairs) {
            preference_elastic[uid].push_back(p.second);
        }
    }

    // DA匹配（带配额限制）
    std::map<int, std::vector<int>> uav_accepted_elastic;
    std::map<int, bool> elastic_matched;
    std::map<int, int> elastic_pref_ptr;
    for (auto& u : all_elastic_users) {
        elastic_matched[u.ID] = false;
        elastic_pref_ptr[u.ID] = 0;
    }

    has_proposal = true;
    while (has_proposal) {
        has_proposal = false;

        // Step A: 未匹配的Elastic用户发起申请
        std::map<int, std::vector<int>> new_proposals;
        for (auto& u : all_elastic_users) {
            int uid = u.ID;
            if (elastic_matched[uid]) continue;
            if (elastic_pref_ptr[uid] >= (int)preference_elastic[uid].size()) continue;
            int target_uav = preference_elastic[uid][elastic_pref_ptr[uid]];
            new_proposals[target_uav].push_back(uid);
            has_proposal = true;
        }
        if (!has_proposal) break;

        // Step B: 每个UAV评估申请者（配额限制）
        for (int k = 0; k < M; k++) {
            // 合并已接受的用户和新申请者
            std::vector<int> candidates = uav_accepted_elastic[k];
            if (new_proposals.count(k)) {
                for (int uid : new_proposals[k])
                    candidates.push_back(uid);
            }
            if (candidates.empty()) continue;

            // 按orientation值降序排列（UAV的偏好）
            std::sort(candidates.begin(), candidates.end(),
                [&](int a, int b) {
                    return get_elastic_orientation(k, a) > get_elastic_orientation(k, b);
                });

            // 准入控制：最多接受 elastic_quota 个用户
            std::vector<int> accepted;
            for (int i = 0; i < (int)candidates.size() && i < elastic_quota; i++) {
                accepted.push_back(candidates[i]);
            }

            std::set<int> accepted_set(accepted.begin(), accepted.end());
            uav_accepted_elastic[k] = accepted;

            // 更新匹配状态
            for (int uid : candidates) {
                if (accepted_set.count(uid)) {
                    elastic_matched[uid] = true;
                }
                else {
                    // 被拒绝，偏好指针后移
                    elastic_matched[uid] = false;
                    elastic_pref_ptr[uid]++;
                }
            }
        }
    }

    // 打印第二阶段结果
    for (int k = 0; k < M; k++) {
        cout << "[Phase2] UAV " << k << ": accepted " << uav_accepted_elastic[k].size()
            << " elastic users (quota=" << elastic_quota << ")" << endl;
    }

    // ====================================================================
    //    第三阶段：Elastic 带宽分配（IPOPT 凸优化求解）
    // ====================================================================

    for (int k = 0; k < M; k++) {
        if (B_remain[k] <= EPS) continue;
        if (uav_accepted_elastic[k].empty()) continue;

        int n_elastic_k = (int)uav_accepted_elastic[k].size();
        cout << "[Phase3] UAV " << k << ": optimizing bandwidth for "
            << n_elastic_k << " elastic users, B_remain = " << B_remain[k] << endl;

        // 准备IPOPT输入
        std::vector<double> weights_vec(n_elastic_k);
        std::vector<double> cap_vec(n_elastic_k);
        for (int i = 0; i < n_elastic_k; i++) {
            int uid = uav_accepted_elastic[k][i];
            int j = uid_to_idx[uid];
            weights_vec[i] = sysModel.users[j].weight;
            cap_vec[i] = sysModel.cap_list[k][j]; // log2(1 + SNR_avg)
        }

        // 创建IPOPT NLP实例
        SmartPtr<TNLP> nlp = new ElasticBandwidthNLP(
            n_elastic_k, weights_vec, cap_vec, B_remain[k]);

        // 配置IPOPT求解器
        SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

        ApplicationReturnStatus status = app->Initialize("MatchGame_ipopt.opt");
        if (status != Solve_Succeeded) {
            cerr << "[IPOPT] Initialization failed for UAV " << k << endl;
            // 回退到均分方案
            double avg_bw = B_remain[k] / n_elastic_k;
            for (int i = 0; i < n_elastic_k; i++) {
                int uid = uav_accepted_elastic[k][i];
                int j = uid_to_idx[uid];
                double cap = sysModel.cap_list[k][j];
                double uti = sysModel.users[j].elastic_utility(avg_bw, cap);
                add_KnapsackResult(uav_results[k], sysModel.users[j], avg_bw, uti);
                user_results[uid].uav_id = sysModel.uavs[k].ID;
                user_results[uid].allocated_bandwidth = avg_bw;
                user_results[uid].utility = uti;
            }
            continue;
        }

        // 求解
        status = app->OptimizeTNLP(nlp);

        // 获取结果
        ElasticBandwidthNLP* nlp_ptr = dynamic_cast<ElasticBandwidthNLP*>(GetRawPtr(nlp));

        if (status == Solve_Succeeded || status == Solved_To_Acceptable_Level) {
            cout << "[Phase3] UAV " << k << ": IPOPT solved, total utility = "
                << nlp_ptr->opt_obj_value << endl;

            for (int i = 0; i < n_elastic_k; i++) {
                int uid = uav_accepted_elastic[k][i];
                int j = uid_to_idx[uid];
                double bw = nlp_ptr->opt_bandwidth[i];
                double cap = sysModel.cap_list[k][j];
                double uti = sysModel.users[j].elastic_utility(bw, cap);

                add_KnapsackResult(uav_results[k], sysModel.users[j], bw, uti);

                user_results[uid].uav_id = sysModel.uavs[k].ID;
                user_results[uid].allocated_bandwidth = bw;
                user_results[uid].utility = uti;
            }
        }
        else {
            cerr << "[IPOPT] Solve failed for UAV " << k
                << ", status = " << status << ". Falling back to equal split." << endl;
            // 回退到均分
            double avg_bw = B_remain[k] / n_elastic_k;
            for (int i = 0; i < n_elastic_k; i++) {
                int uid = uav_accepted_elastic[k][i];
                int j = uid_to_idx[uid];
                double cap = sysModel.cap_list[k][j];
                double uti = sysModel.users[j].elastic_utility(avg_bw, cap);
                add_KnapsackResult(uav_results[k], sysModel.users[j], avg_bw, uti);
                user_results[uid].uav_id = sysModel.uavs[k].ID;
                user_results[uid].allocated_bandwidth = avg_bw;
                user_results[uid].utility = uti;
            }
        }
    }

    return { uav_results, user_results };
}
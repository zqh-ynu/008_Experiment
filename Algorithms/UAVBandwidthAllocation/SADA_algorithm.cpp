#include "EntityDefinition.h"

// ============================================================================
// IPOPT 头文件
// ============================================================================
#include "IpIpoptApplication.hpp"
#include "IpTNLP.hpp"
#include "IpSmartPtr.hpp"

using namespace Ipopt;
// AlgSA-DD retains the original dual/IPOPT heuristic and association-initialization limitations.

// ============================================================================
// SADA-IPOPT算法 v3
//
// 核心修正:
//   1. 用原版对偶分解先求一个高质量初始解 (作为warm start)
//   2. IPOPT在此基础上做局部优化
//   3. 比较IPOPT解和原版解, 取更优者
//   4. 精确Hessian + Gauss-Newton正定修正
//   5. 为UH用户设置更紧的变量下界 (避免U≈0的数值困难区域)
// ============================================================================


// ============================================================================
// 辅助函数
// ============================================================================

static double sigmoid_utility_H_v3(double b_ki, double C_ki_s, double r_min, double V_i, double m) {
    if (b_ki < 1e-12) return 0.0;
    double R_ki = b_ki * C_ki_s;
    double exponent = -m * (R_ki - r_min);
    if (exponent > 500.0) return 0.0;
    if (exponent < -500.0) return V_i;
    return V_i / (1.0 + exp(exponent));
}

static double sigmoid_utility_H_deriv_v3(double b_ki, double C_ki_s, double r_min, double V_i, double m) {
    double U = sigmoid_utility_H_v3(b_ki, C_ki_s, r_min, V_i, m);
    if (V_i < 1e-12) return 0.0;
    return U * (1.0 - U / V_i) * m * C_ki_s;
}

static double sigmoid_utility_H_deriv2_v3(double b_ki, double C_ki_s, double r_min, double V_i, double m) {
    double U = sigmoid_utility_H_v3(b_ki, C_ki_s, r_min, V_i, m);
    if (V_i < 1e-12) return 0.0;
    double s = U / V_i;
    double mC = m * C_ki_s;
    return V_i * mC * mC * s * (1.0 - s) * (1.0 - 2.0 * s);
}

static double log_utility_E_v3(double b_kj, double C_kj_avg, double w_j) {
    if (b_kj < 1e-12) return 0.0;
    return w_j * log2(b_kj * C_kj_avg + 1.0);
}

static double log_utility_E_deriv_v3(double b_kj, double C_kj_avg, double w_j) {
    return (w_j * C_kj_avg) / ((b_kj * C_kj_avg + 1.0) * log(2.0));
}

static double log_utility_E_deriv2_v3(double b_kj, double C_kj_avg, double w_j) {
    double x = b_kj * C_kj_avg + 1.0;
    return -(w_j * C_kj_avg * C_kj_avg) / (x * x * log(2.0));
}


// ============================================================================
// IPOPT TNLP: 单UAV带宽分配子问题
//
//   min  -sum_i theta[i] * log( U_i(b_i) + eps_safe )
//   s.t. sum_i b_i <= B_UAV
//        b_lb_i <= b_i <= B_UAV
//
// 关键: b_lb_i 对UH用户设置为使 U > V*0.01 的值 (跳过Sigmoid凸区域)
//       这样 log(U) 的非凹区域被变量边界排除
// ============================================================================

class UAV_SubNLP_v3 : public TNLP {
public:
    int n_vars;
    double B_UAV;
    vector<double> theta_vec;
    vector<double> cap_vec;
    vector<bool> is_hard_vec;
    vector<double> rmin_vec;
    vector<double> Vi_vec;
    vector<double> weight_vec;
    double m_sigmoid;

    vector<double> b_lower;    // 每个变量的下界 (UH用户可能较大)
    vector<double> init_b;     // warm start
    vector<double> solution;
    double obj_value;

    static constexpr double eps_safe = 1e-20;

    UAV_SubNLP_v3() : n_vars(0), B_UAV(0), m_sigmoid(1.0), obj_value(0) {}

    // ---- 效用及导数 ----
    double U_raw(int i, double b) const {
        if (is_hard_vec[i])
            return sigmoid_utility_H_v3(b, cap_vec[i], rmin_vec[i], Vi_vec[i], m_sigmoid);
        else
            return log_utility_E_v3(b, cap_vec[i], weight_vec[i]);
    }
    double U_safe(int i, double b) const { return U_raw(i, b) + eps_safe; }
    double dU(int i, double b) const {
        if (is_hard_vec[i])
            return sigmoid_utility_H_deriv_v3(b, cap_vec[i], rmin_vec[i], Vi_vec[i], m_sigmoid);
        else
            return log_utility_E_deriv_v3(b, cap_vec[i], weight_vec[i]);
    }
    double d2U(int i, double b) const {
        if (is_hard_vec[i])
            return sigmoid_utility_H_deriv2_v3(b, cap_vec[i], rmin_vec[i], Vi_vec[i], m_sigmoid);
        else
            return log_utility_E_deriv2_v3(b, cap_vec[i], weight_vec[i]);
    }

    // ---- 计算代理目标函数值 (用于外部比较) ----
    double eval_surrogate_obj(const vector<double>& bvec) const {
        double val = 0.0;
        for (int i = 0; i < n_vars; i++) {
            double Us = U_safe(i, bvec[i]);
            val += theta_vec[i] * log(Us);
        }
        return val; // 越大越好
    }

    // ================================================================
    // TNLP 接口
    // ================================================================

    bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style) override
    {
        n = n_vars;
        m = 1;
        nnz_jac_g = n_vars;
        nnz_h_lag = n_vars;
        index_style = C_STYLE;
        return true;
    }

    bool get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u) override
    {
        for (Index i = 0; i < n; i++) {
            x_l[i] = b_lower[i];
            x_u[i] = B_UAV;
        }
        g_l[0] = -1e20;
        g_u[0] = B_UAV;
        return true;
    }

    bool get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m_con, bool init_lambda, Number* lambda) override
    {
        if (init_x) {
            // Warm start: 用对偶分解的解
            double sum_init = 0.0;
            for (Index i = 0; i < n; i++) {
                x[i] = max(init_b[i], b_lower[i]);
                sum_init += x[i];
            }
            // 约束修复
            if (sum_init > B_UAV + 1e-10) {
                double excess = sum_init - B_UAV;
                // 按比例缩减超出下界的部分
                double flex_sum = 0.0;
                for (Index i = 0; i < n; i++) flex_sum += (x[i] - b_lower[i]);
                if (flex_sum > 1e-15) {
                    for (Index i = 0; i < n; i++) {
                        double flex = x[i] - b_lower[i];
                        x[i] -= excess * (flex / flex_sum);
                        x[i] = max(x[i], b_lower[i]);
                    }
                }
            }
        }
        return true;
    }

    bool eval_f(Index n, const Number* x, bool new_x, Number& obj) override
    {
        obj = 0.0;
        for (Index i = 0; i < n; i++) {
            obj -= theta_vec[i] * log(U_safe(i, x[i]));
        }
        return true;
    }

    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) override
    {
        for (Index i = 0; i < n; i++) {
            double Us = U_safe(i, x[i]);
            double du = dU(i, x[i]);
            grad_f[i] = -theta_vec[i] * du / Us;
        }
        return true;
    }

    bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) override
    {
        g[0] = 0.0;
        for (Index i = 0; i < n; i++) g[0] += x[i];
        return true;
    }

    bool eval_jac_g(Index n, const Number* x, bool new_x,
        Index m, Index nele_jac,
        Index* iRow, Index* jCol, Number* values) override
    {
        if (values == nullptr) {
            for (Index i = 0; i < n; i++) { iRow[i] = 0; jCol[i] = i; }
        }
        else {
            for (Index i = 0; i < n; i++) values[i] = 1.0;
        }
        return true;
    }

    bool eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess,
        Index* iRow, Index* jCol, Number* values) override
    {
        if (values == nullptr) {
            for (Index i = 0; i < n; i++) { iRow[i] = i; jCol[i] = i; }
        }
        else {
            for (Index i = 0; i < n; i++) {
                double Us = U_safe(i, x[i]);
                double du = dU(i, x[i]);
                double d2u = d2U(i, x[i]);

                // d²[-theta*log(Us)]/db² = -theta * (d2u*Us - du²) / Us²
                double hess = -theta_vec[i] * (d2u * Us - du * du) / (Us * Us);

                // 正定修正: 我们求的是min, Hessian应 >= 0
                // Gauss-Newton近似: 用 theta*(du/Us)² >= 0
                if (hess < 1e-10) {
                    hess = theta_vec[i] * (du / Us) * (du / Us);
                    hess = max(hess, 1e-10); // 保证严格正定
                }

                values[i] = obj_factor * hess;
            }
        }
        return true;
    }

    void finalize_solution(SolverReturn status,
        Index n, const Number* x,
        const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda,
        Number obj_val,
        const IpoptData* ip_data,
        IpoptCalculatedQuantities* ip_cq) override
    {
        solution.resize(n);
        for (Index i = 0; i < n; i++) solution[i] = x[i];
        obj_value = obj_val;
    }
};


// ============================================================================
// 原版对偶分解求解器 (作为warm start和fallback)
// ============================================================================

/// @brief 对单个UAV k, 用原版对偶分解(二分法)求解带宽分配
/// @return 活跃用户的最优带宽向量 (按active_users顺序)
static vector<double> dual_decomposition_solve(
    int k, double B_UAV, double current_m,
    const vector<int>& active_users,
    int N1,
    const vector<vector<double>>& cap_list,
    const vector<User>& users,
    const vector<double>& V_i,
    const vector<vector<double>>& theta,
    int bisect_max_iter, double bisect_tol,
    int max_inner_iter, double dual_tol)
{
    int n_active = (int)active_users.size();
    vector<double> result(n_active, 0.0);

    // 定义: 给定mu, 求用户u的最优带宽
    auto solve_b_given_mu = [&](int u, double mu_k) -> double {
        if (mu_k < 1e-15) return B_UAV;

        double cap = cap_list[k][u];
        double th = theta[k][u];
        double r_min = (u < N1) ? users[u].rMin : 0.0;
        double Vi = (u < N1) ? V_i[u] : 0.0;

        if (u < N1) {
            // UH用户: 解析解
            double tmC = th * current_m * cap;
            if (tmC < 1e-15) return 0.0;

            double ratio = mu_k / tmC;
            if (ratio >= 1.0) return 0.0;

            double U_target = Vi * (1.0 - ratio);
            if (U_target < 1e-15) return 0.0;

            double VoverU = Vi / U_target;
            if (VoverU <= 1.0 + 1e-12) return B_UAV;

            double log_term = log(VoverU - 1.0);
            double b_new = (r_min - log_term / current_m) / cap;
            return max(0.0, min(b_new, B_UAV));
        }
        else {
            // UE用户: 二分法
            auto f_kkt = [&](double bval) -> double {
                double x = bval * cap + 1.0;
                if (x <= 1.0 + 1e-15) return 1e15;
                double lnx = log(x);
                if (lnx < 1e-15) return 1e15;
                return th * cap / (x * lnx);
                };

            double b_min_test = 1e-8;
            if (f_kkt(b_min_test) <= mu_k) return 0.0;
            if (f_kkt(B_UAV) >= mu_k) return B_UAV;

            double lo = b_min_test, hi = B_UAV;
            for (int bs = 0; bs < bisect_max_iter; bs++) {
                double mid = (lo + hi) * 0.5;
                if (f_kkt(mid) > mu_k) lo = mid;
                else hi = mid;
                if (hi - lo < bisect_tol) break;
            }
            return (lo + hi) * 0.5;
        }
        };

    auto calc_sum_b = [&](double mu_k) -> double {
        double sum = 0.0;
        for (int u : active_users) sum += solve_b_given_mu(u, mu_k);
        return sum;
        };

    double mu_lo = 0.0, mu_hi = 1.0;
    while (calc_sum_b(mu_hi) > B_UAV && mu_hi < 1e10) mu_hi *= 2.0;

    double sum_at_zero = calc_sum_b(1e-12);
    if (sum_at_zero <= B_UAV + 1e-6) {
        for (int i = 0; i < n_active; i++)
            result[i] = solve_b_given_mu(active_users[i], 1e-12);
    }
    else {
        for (int it = 0; it < max_inner_iter; it++) {
            double mu_mid = (mu_lo + mu_hi) * 0.5;
            double sum_mid = calc_sum_b(mu_mid);
            if (sum_mid > B_UAV) mu_lo = mu_mid;
            else mu_hi = mu_mid;
            if (abs(sum_mid - B_UAV) < dual_tol || mu_hi - mu_lo < 1e-12) break;
        }
        double mu_opt = (mu_lo + mu_hi) * 0.5;
        for (int i = 0; i < n_active; i++)
            result[i] = solve_b_given_mu(active_users[i], mu_opt);
    }

    return result;
}


// ============================================================================
// 主算法
// ============================================================================

/// Run the existing SA-DD heuristic; config controls its unchanged algorithmic defaults.
/// Return the legacy allocation pair, recording optional solver/fallback diagnostics and throwing on initialization failure.
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::SADA_Allocation(SADAConfig config, AllocationDiagnostics* diagnostics) {
    if (diagnostics) *diagnostics = AllocationDiagnostics();
    if (sysModel.users.empty() || sysModel.uavs.empty() ||
        std::none_of(sysModel.uavs.begin(), sysModel.uavs.end(), [](const Uav& uav) {return uav.total_bandwidth > 0;})) {
        vector<KnapsackResult> empty(sysModel.m);
        for (int k = 0; k < sysModel.m; ++k) empty[k].uav_id = k;
        return {empty, construct_user_results(empty)};
    }

    const int M = sysModel.m;
    const int N1 = sysModel.n1;
    const int N2 = sysModel.n2;
    const int N = N1 + N2;
    auto& users = sysModel.users;
    auto& uavs = sysModel.uavs;

    // if (config.verbose) {
    //     cout << "============================================" << endl;
    //     cout << "[SADA-IPOPT] Starting: M=" << M << ", N1=" << N1 << ", N2=" << N2 << endl;
    // }

    // ----------------------------------------------------------------
    // 1. 预计算
    // ----------------------------------------------------------------
    vector<double> V_i(N1, 0.0);
    for (int i = 0; i < N1; i++) {
        V_i[i] = users[i].weight * log2(1.0 + users[i].rMin);
    }

    vector<vector<bool>> serviceable(M, vector<bool>(N, false));
    for (int k = 0; k < M; k++) {
        for (int u = 0; u < N; u++) {
            if (sysModel.dis_list[k][u] <= max_coverage_distance &&
                sysModel.cap_list[k][u] > 1e-10) {
                serviceable[k][u] = true;
            }
        }
    }

    // ----------------------------------------------------------------
    // 2. 初始化: 贪心关联 + 均分带宽
    // ----------------------------------------------------------------
    vector<vector<double>> b(M, vector<double>(N, 0.0));

    vector<int> init_assoc(N, -1);
    for (int u = 0; u < N; u++) {
        double best_cap = -1.0;
        int best_k = -1;
        for (int k = 0; k < M; k++) {
            if (serviceable[k][u] && sysModel.cap_list[k][u] > best_cap) {
                best_cap = sysModel.cap_list[k][u];
                best_k = k;
            }
        }
        init_assoc[u] = best_k;
    }

    for (int k = 0; k < M; k++) {
        int count = 0;
        for (int u = 0; u < N; u++) {
            if (init_assoc[u] == k) count++;
        }
        if (count > 0) {
            double bw_each = uavs[k].total_bandwidth / count;
            for (int u = 0; u < N; u++) {
                if (init_assoc[u] == k) b[k][u] = bw_each;
            }
        }
    }

    // ----------------------------------------------------------------
    // 3. 初始化 theta
    // ----------------------------------------------------------------
    double current_m = config.m_init;
    vector<vector<double>> theta(M, vector<double>(N, 0.0));

    auto calc_utility = [&](int k, int u, double bw) -> double {
        if (!serviceable[k][u]) return 0.0;
        double cap = sysModel.cap_list[k][u];
        if (u < N1)
            return sigmoid_utility_H_v3(bw, cap, users[u].rMin, V_i[u], current_m);
        else
            return log_utility_E_v3(bw, cap, users[u].weight);
        };

    {
        double S = 0.0;
        for (int k = 0; k < M; k++)
            for (int u = 0; u < N; u++)
                S += calc_utility(k, u, b[k][u]);

        if (S > 1e-12) {
            for (int k = 0; k < M; k++)
                for (int u = 0; u < N; u++)
                    theta[k][u] = calc_utility(k, u, b[k][u]) / S;
        }
        else {
            if (diagnostics) {
                diagnostics->used_fallback = true;
                diagnostics->events.push_back("sadd.initialize_theta: zero-utility uniform-theta fallback");
            }
            double val = 1.0 / (double)(M * N);
            for (int k = 0; k < M; k++)
                for (int u = 0; u < N; u++)
                    theta[k][u] = val;
        }
    }

    // ----------------------------------------------------------------
    // 4. 预创建IPOPT Application
    // ----------------------------------------------------------------
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    if (IsNull(app))
        throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "SA-DD: IPOPT creation failed");
    ApplicationReturnStatus app_status = app->Initialize(algProjPath + "SADA_ipopt.opt");
    if (diagnostics) diagnostics->record("sadd.initialize", static_cast<int>(app_status));
    if (app_status != Solve_Succeeded) {
        throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "SA-DD: IPOPT initialization failed");
    }

    // ----------------------------------------------------------------
    // 5. 主循环
    // ----------------------------------------------------------------
    double prev_S = -1e30;

    for (int tau = 0; tau < config.max_outer_iter; tau++) {

        // ============================================================
        // 5.1 内层: 对每个UAV求解子问题
        // ============================================================

        for (int k = 0; k < M; k++) {
            double B_UAV = uavs[k].total_bandwidth;

            // 收集活跃用户
            vector<int> active_users;
            for (int u = 0; u < N; u++) {
                if (serviceable[k][u] && theta[k][u] > 1e-15)
                    active_users.push_back(u);
            }

            if (active_users.empty()) {
                for (int u = 0; u < N; u++) b[k][u] = 0.0;
                continue;
            }

            int n_active = (int)active_users.size();

            if (n_active == 1) {
                for (int u = 0; u < N; u++) b[k][u] = 0.0;
                b[k][active_users[0]] = B_UAV;
                continue;
            }

            // ---- Step A: 原版对偶分解求解 (作为baseline) ----
            vector<double> dual_sol = dual_decomposition_solve(
                k, B_UAV, current_m, active_users, N1,
                sysModel.cap_list, users, V_i, theta,
                config.bisect_max_iter, config.bisect_tol,
                config.max_inner_iter, config.dual_tol);

            // ---- Step B: 构造IPOPT子问题, 以dual_sol为warm start ----
            SmartPtr<UAV_SubNLP_v3> nlp = new UAV_SubNLP_v3();
            nlp->n_vars = n_active;
            nlp->B_UAV = B_UAV;
            nlp->m_sigmoid = current_m;
            nlp->theta_vec.resize(n_active);
            nlp->cap_vec.resize(n_active);
            nlp->is_hard_vec.resize(n_active);
            nlp->rmin_vec.resize(n_active);
            nlp->Vi_vec.resize(n_active);
            nlp->weight_vec.resize(n_active);
            nlp->b_lower.resize(n_active);
            nlp->init_b.resize(n_active);

            for (int i = 0; i < n_active; i++) {
                int u = active_users[i];
                nlp->theta_vec[i] = theta[k][u];
                nlp->cap_vec[i] = sysModel.cap_list[k][u];
                nlp->is_hard_vec[i] = (u < N1);
                nlp->rmin_vec[i] = (u < N1) ? users[u].rMin : 0.0;
                nlp->Vi_vec[i] = (u < N1) ? V_i[u] : 0.0;
                nlp->weight_vec[i] = users[u].weight;

                // ---- 关键: 为UH用户计算更紧的下界 ----
                // Sigmoid拐点在 b = r_min / C
                // 拐点左侧是凸区域, log(sigmoid)在此区域非凹
                // 设下界为拐点值的一半, 避免IPOPT进入非凹区域
                if (u < N1) {
                    double cap = sysModel.cap_list[k][u];
                    if (cap > 1e-10) {
                        // Sigmoid拐点: b_inflect = r_min / C
                        double b_inflect = users[u].rMin / cap;
                        // 下界取拐点的10%, 但不超过dual_sol的50%
                        double lb_candidate = b_inflect * 0.1;
                        // 不要设太大以免过度约束
                        nlp->b_lower[i] = max(1e-8, min(lb_candidate, dual_sol[i] * 0.5));
                    }
                    else {
                        nlp->b_lower[i] = 1e-8;
                    }
                }
                else {
                    nlp->b_lower[i] = 1e-8;
                }

                // warm start
                nlp->init_b[i] = max(dual_sol[i], nlp->b_lower[i]);
            }

            // ---- Step C: 求解IPOPT ----
            ApplicationReturnStatus status = app->OptimizeTNLP(nlp);

            // ---- Step D: 比较IPOPT解和对偶分解解, 取更优者 ----
            // 计算代理目标函数值 (越大越好)
            double dual_obj = nlp->eval_surrogate_obj(dual_sol);

            bool use_ipopt = false;
            if (status == Solve_Succeeded || status == Solved_To_Acceptable_Level) {
                double ipopt_obj = nlp->eval_surrogate_obj(nlp->solution);

                // 检查IPOPT解的可行性
                double ipopt_sum = 0.0;
                for (int i = 0; i < n_active; i++) ipopt_sum += nlp->solution[i];

                if (ipopt_sum <= B_UAV + 1e-6 && ipopt_obj >= dual_obj - 1e-10) {
                    use_ipopt = true;
                }
            }

            if (diagnostics) diagnostics->record("sadd.refine." + std::to_string(tau) +
                ".uav." + std::to_string(k), static_cast<int>(status), !use_ipopt);

            // 写入结果
            for (int u = 0; u < N; u++) b[k][u] = 0.0;

            if (use_ipopt) {
                for (int i = 0; i < n_active; i++) {
                    b[k][active_users[i]] = checked_solver_bandwidth(nlp->solution.at(i), "SA-DD.refinement");
                }
            }
            else {
                for (int i = 0; i < n_active; i++) {
                    b[k][active_users[i]] = checked_solver_bandwidth(dual_sol.at(i), "SA-DD.dual");
                }
            }

            // 清除非活跃用户
            for (int u = 0; u < N; u++) {
                if (!serviceable[k][u] || theta[k][u] <= 1e-15)
                    b[k][u] = 0.0;
            }

        } // end for each UAV k

        // ============================================================
        // 5.2 更新theta
        // ============================================================
        double S = 0.0;
        vector<vector<double>> U_mat(M, vector<double>(N, 0.0));
        for (int k = 0; k < M; k++) {
            for (int u = 0; u < N; u++) {
                U_mat[k][u] = calc_utility(k, u, b[k][u]);
                S += U_mat[k][u];
            }
        }

        if (config.verbose) {
            int h_count = 0, e_count = 0;
            double h_util = 0, e_util = 0;
            for (int k = 0; k < M; k++) {
                for (int u = 0; u < N; u++) {
                    if (b[k][u] > 1e-6) {
                        if (u < N1) { h_count++; h_util += U_mat[k][u]; }
                        else { e_count++; e_util += U_mat[k][u]; }
                    }
                }
            }
            // cout << "[SADA-IPOPT] Outer iter " << tau
            //     << ": S=" << fixed << setprecision(4) << S
            //     << " (H:" << h_util << "/" << h_count
            //     << ", E:" << e_util << "/" << e_count
            //     << "), m=" << current_m << endl;
        }

        if (S > 1e-12) {
            double max_theta_change = 0.0;
            for (int k = 0; k < M; k++) {
                for (int u = 0; u < N; u++) {
                    double new_theta = U_mat[k][u] / S;
                    max_theta_change = max(max_theta_change, abs(new_theta - theta[k][u]));
                    theta[k][u] = new_theta;
                }
            }
            if (max_theta_change < config.theta_tol && tau > 2) {
                if (config.verbose)
                    // cout << "[SADA-IPOPT] Outer loop converged at iter " << tau
                    // << ", max_theta_change=" << max_theta_change << endl;
                    break;
            }
        }
        else {
            // cout << "[SADA-IPOPT] Warning: total utility S ~ 0 at outer iter " << tau << endl;
            break;
        }

        current_m = min(current_m * config.m_scale, config.m_max);
        prev_S = S;

    } // end outer loop

    // ----------------------------------------------------------------
    // 6. 后处理
    // ----------------------------------------------------------------
    vector<int> user_to_uav(N, -1);
    vector<double> user_bw(N, 0.0);
    vector<double> user_util(N, 0.0);

    for (int u = 0; u < N; u++) {
        double best_util = -1.0;
        int best_k = -1;
        double best_bw = 0.0;
        for (int k = 0; k < M; k++) {
            if (b[k][u] < 1e-8) continue;
            double util = calc_utility(k, u, b[k][u]);
            if (util > best_util) {
                best_util = util;
                best_k = k;
                best_bw = b[k][u];
            }
        }
        if (best_k >= 0) {
            user_to_uav[u] = best_k;
            user_bw[u] = best_bw;
            user_util[u] = best_util;
        }
    }

    for (int u = 0; u < N1; u++) {
        if (user_to_uav[u] < 0) continue;
        int k = user_to_uav[u];
        double C_s = sysModel.cap_list[k][u];
        if (!hard_qos_satisfied(user_bw[u], C_s, users[u].rMin)) {
            double needed = users[u].rMin / C_s;
            user_bw[u] = needed;
        }
    }

    for (int k = 0; k < M; k++) {
        double B_UAV = uavs[k].total_bandwidth;
        double total_bw = 0.0;
        vector<int> assigned;
        for (int u = 0; u < N; u++) {
            if (user_to_uav[u] == k) {
                total_bw += user_bw[u];
                assigned.push_back(u);
            }
        }

        if (total_bw > B_UAV + EPS) {
            vector<pair<double, int>> density_list;
            for (int u : assigned) {
                double cap = sysModel.cap_list[k][u];
                double uti;
                double SNR_avg_dB = sysModel.SNRave_list[k][u];
                if (u < N1) uti = users[u].hard_utility(user_bw[u], cap, SNR_avg_dB);
                else uti = users[u].elastic_utility(user_bw[u], cap);
                double dens = (user_bw[u] > EPS) ? uti / user_bw[u] : 0.0;
                density_list.push_back({ dens, u });
            }
            sort(density_list.begin(), density_list.end(),
                [](const auto& a, const auto& b_) { return a.first > b_.first; });

            double cum_bw = 0.0;
            for (auto& [dens, u] : density_list) {
                if (cum_bw + user_bw[u] <= B_UAV + EPS) {
                    cum_bw += user_bw[u];
                }
                else {
                    if (u >= N1) {
                        double remain = B_UAV - cum_bw;
                        if (remain > EPS) { user_bw[u] = remain; cum_bw = B_UAV; }
                        else { user_to_uav[u] = -1; user_bw[u] = 0.0; }
                    }
                    else {
                        user_to_uav[u] = -1; user_bw[u] = 0.0;
                    }
                }
            }
        }
    }

    for (int u = 0; u < N1; u++) {
        if (user_to_uav[u] < 0) continue;
        int k = user_to_uav[u];
        double C_s = sysModel.cap_list[k][u];
        if (!hard_qos_satisfied(user_bw[u], C_s, users[u].rMin)) {
            user_to_uav[u] = -1; user_bw[u] = 0.0;
        }
    }

    for (int k = 0; k < M; k++) {
        double used_bw = 0.0;
        vector<int> elastic_on_k;
        for (int u = 0; u < N; u++) {
            if (user_to_uav[u] == k) {
                used_bw += user_bw[u];
                if (u >= N1) elastic_on_k.push_back(u);
            }
        }
        double remain = uavs[k].total_bandwidth - used_bw;
        if (remain > EPS && !elastic_on_k.empty()) {
            double wsum = 0.0;
            for (int u : elastic_on_k) wsum += users[u].weight;
            if (wsum > EPS) {
                for (int u : elastic_on_k)
                    user_bw[u] += remain * (users[u].weight / wsum);
            }
        }
    }

    // ----------------------------------------------------------------
    // 7. 构造输出
    // ----------------------------------------------------------------
    vector<KnapsackResult> allResults(M);
    for (int k = 0; k < M; k++) {
        allResults[k].uav_id = uavs[k].ID;
        clean_KnapsackResult(allResults[k]);
        allResults[k].uav_id = uavs[k].ID;
    }

    map<int, UserResult> userResults;
    double total_utility = 0.0;
    int connected_hard = 0, connected_elastic = 0;

    for (int u = 0; u < N; u++) {
        int k = user_to_uav[u];
        UserResult ur;
        ur.uav_id = k;
        ur.allocated_bandwidth = user_bw[u];

        if (k >= 0 && user_bw[u] > EPS) {
            double cap = sysModel.cap_list[k][u];
            double SNR_avg_dB = sysModel.SNRave_list[k][u];
            double final_util;
            if (u < N1) {
                final_util = users[u].hard_utility(user_bw[u], cap, SNR_avg_dB);
                if (final_util > 0) connected_hard++;
            }
            else {
                final_util = users[u].elastic_utility(user_bw[u], cap);
                if (final_util > 0) connected_elastic++;
            }
            ur.utility = final_util;
            total_utility += final_util;
            add_KnapsackResult(allResults[k], users[u], user_bw[u], final_util);
        }
        else {
            ur.uav_id = -1;
            ur.allocated_bandwidth = 0.0;
            ur.utility = 0.0;
        }
        userResults[users[u].ID] = ur;
    }

    // cout << "============================================" << endl;
    // cout << "[SADA-IPOPT] Algorithm completed." << endl;
    // cout << "[SADA-IPOPT] Total Utility: " << fixed << setprecision(4) << total_utility << endl;
    // cout << "[SADA-IPOPT] Connected Hard Users: " << connected_hard << " / " << N1 << endl;
    // cout << "[SADA-IPOPT] Connected Elastic Users: " << connected_elastic << " / " << N2 << endl;
    // cout << "============================================" << endl;

    return { allResults, userResults };
}

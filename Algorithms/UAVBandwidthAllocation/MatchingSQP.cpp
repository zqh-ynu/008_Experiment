#include "EntityDefinition.h"
#include <numeric>    // std::iota
// AlgSwapMatching: one-pass surrogate bandwidth optimization followed by beneficial swaps.
// Existing surrogate weights, hard recovery and disabled residual optimization are intentionally retained.
#include <unordered_set>
#include <unordered_map>

using namespace Ipopt;
// ============================================================================
// 锟姐法锟斤拷锟斤拷锟斤拷锟斤拷
// ============================================================================


// ============================================================================
// 锟斤拷锟斤拷锟斤拷锟斤拷锟姐单锟斤拷锟矫伙拷效锟斤拷 (Sigmoid 锟斤拷锟斤拷 / 锟斤拷一锟斤拷锟斤拷锟斤拷)
// ============================================================================

/// Hard锟矫伙拷 Sigmoid 锟斤拷锟斤拷效锟斤拷
static double sigmoid_hard_utility(double b_ki, double cap_ki, double r_min,
    double w_i, double nu)
{
    // R_i = b_ki * cap_ki (cap_ki = channel_capacity for hard user)
    double R_i = b_ki * cap_ki;
    double exponent = -nu * (R_i - r_min);
    // 锟斤拷止锟斤拷锟?
    if (exponent > 500.0) return 0.0;
    if (exponent < -500.0) return w_i;
    double sigma = 1.0 / (1.0 + exp(exponent));
    return w_i * sigma;
}

/// Hard锟矫伙拷 Sigmoid 效锟矫讹拷 b_ki 锟斤拷锟捷讹拷
static double sigmoid_hard_grad(double b_ki, double cap_ki, double r_min,
    double w_i, double nu)
{
    double R_i = b_ki * cap_ki;
    double exponent = -nu * (R_i - r_min);
    if (exponent > 500.0 || exponent < -500.0) return 0.0;
    double sigma = 1.0 / (1.0 + exp(exponent));
    return w_i * sigma * (1.0 - sigma) * nu * cap_ki;
}

/// Elastic锟矫伙拷 锟斤拷一锟斤拷锟斤拷锟斤拷效锟斤拷
/// U_j^E = w_j * log2(1 + b*Omega) / log2(1 + B_UAV*Omega)
static double normalized_elastic_utility(double b_kj, double cap_kj,
    double B_UAV, double w_j)
{
    // cap_kj 锟斤拷锟斤拷 log2(1 + SNR_avg), 锟斤拷 Omega^avg 锟斤拷应锟斤拷锟脚碉拷锟斤拷锟斤拷
    // 锟斤拷锟斤拷 elastic user: rate = b_kj * cap_kj
    // 效锟斤拷 = w_j * log2(1 + b_kj * cap_kj) / log2(1 + B_UAV * cap_kj)
    double r = b_kj * cap_kj;
    double r_max = B_UAV * cap_kj;
    double denom = log2(1.0 + r_max);
    if (denom < 1e-12) return 0.0;
    return w_j * log2(1.0 + r) / denom;
}

/// Elastic锟矫伙拷 锟斤拷一锟斤拷锟斤拷锟斤拷效锟矫讹拷 b_kj 锟斤拷锟捷讹拷
static double normalized_elastic_grad(double b_kj, double cap_kj,
    double B_UAV, double w_j)
{
    double r = b_kj * cap_kj;
    double r_max = B_UAV * cap_kj;
    double denom = log2(1.0 + r_max);
    if (denom < 1e-12) return 0.0;
    return w_j * cap_kj / (log(2.0) * (1.0 + r) * denom);
}

/// 通锟斤拷效锟矫硷拷锟姐（锟斤拷锟斤拷锟矫伙拷锟斤拷锟斤拷锟皆讹拷选锟斤拷
static double compute_user_utility(const User& user, int uav_id,
    double bandwidth, double cap,
    double B_UAV, double nu)
{
    if (user.uType == HARD_UTILITY) {
        return sigmoid_hard_utility(bandwidth, cap, user.rMin, user.weight, nu);
    }
    else {
        return normalized_elastic_utility(bandwidth, cap, B_UAV, user.weight);
    }
}


// ============================================================================
// IPOPT 锟斤拷锟斤拷锟斤拷锟斤拷锟? UAV 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 (Phase 1 / Phase 4.3)
// ============================================================================
// 锟斤拷 TNLP 锟斤拷锟斤拷为锟斤拷锟斤拷 UAV 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟脚伙拷锟斤拷锟斤拷
// 锟斤拷锟竭憋拷锟斤拷: b[0], b[1], ..., b[n-1] 锟斤拷应 UAV 锟斤拷锟斤拷锟? n 锟斤拷锟矫伙拷
// 目锟斤拷: max 锟斤拷 U_i(b_i) => min -锟斤拷 U_i(b_i)
// 约锟斤拷: 锟斤拷 b_i <= B_UAV (锟斤拷锟斤拷式约锟斤拷 g(x) <= 0 锟斤拷式: 锟斤拷b_i - B_UAV <= 0)
//        锟斤拷 hard 锟矫伙拷: b_i * cap_i >= r_min_i => r_min_i - b_i * cap_i <= 0
// 锟斤拷锟斤拷锟竭斤拷: b_i >= 0

class SingleUAV_BW_NLP : public TNLP {
public:
    // 锟斤拷锟斤拷锟斤拷锟斤拷
    int n_users;                    // 锟斤拷 UAV 锟斤拷锟斤拷锟斤拷没锟斤拷锟?
    vector<int> user_indices;       // 全锟斤拷锟矫伙拷锟斤拷锟斤拷
    vector<double> caps;            // 每锟斤拷锟矫伙拷锟斤拷应锟斤拷锟脚碉拷锟斤拷锟斤拷
    vector<double> r_mins;          // hard锟矫伙拷锟斤拷锟斤拷小锟斤拷锟斤拷(elastic锟斤拷为0)
    vector<double> weights;         // 锟矫伙拷权锟斤拷
    vector<int> is_hard;            // 锟角凤拷为hard锟矫伙拷
    double B_UAV;                   // UAV锟杰达拷锟斤拷
    double nu;                      // Sigmoid 锟斤拷锟斤拷锟斤拷锟斤拷
    bool enforce_qos;               // 锟角凤拷强锟斤拷QoS约锟斤拷(Phase 1: true, Phase 4.3: false)

    // 锟斤拷始锟斤拷 & 锟斤拷锟?
    vector<double> b_init;
    vector<double> b_opt;
    double obj_opt;

    // 约锟斤拷锟斤拷锟斤拷
    int n_hard_constraints;         // hard QoS 约锟斤拷锟斤拷锟斤拷

    SingleUAV_BW_NLP() : n_users(0), B_UAV(0), nu(10), enforce_qos(true),
        obj_opt(0), n_hard_constraints(0) {
    }

    // 锟斤拷取锟斤拷锟斤拷维锟斤拷
    bool get_nlp_info(Index& n, Index& m_con, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style) override
    {
        n = n_users;
        // 约锟斤拷锟斤拷: 1 (锟斤拷锟斤拷锟杰猴拷) + n_hard_constraints (hard QoS)
        n_hard_constraints = 0;
        if (enforce_qos) {
            for (int i = 0; i < n_users; i++) {
                if (is_hard[i]) n_hard_constraints++;
            }
        }
        m_con = 1 + n_hard_constraints;
        // Jacobian 锟斤拷锟斤拷元: 锟斤拷锟斤拷约锟斤拷锟斤拷n锟斤拷锟斤拷锟斤拷, 每锟斤拷QoS约锟斤拷锟斤拷1锟斤拷锟斤拷锟斤拷
        nnz_jac_g = n + n_hard_constraints;
        nnz_h_lag = 0; // 使锟斤拷L-BFGS锟斤拷锟斤拷, 锟斤拷锟斤拷要Hessian
        index_style = C_STYLE;
        return true;
    }

    // 锟斤拷锟斤拷锟斤拷约锟斤拷锟竭斤拷
    bool get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m_con, Number* g_l, Number* g_u) override
    {
        // 锟斤拷锟斤拷锟竭斤拷: b_i >= 0, b_i <= B_UAV
        for (int i = 0; i < n; i++) {
            x_l[i] = 0.0;
            x_u[i] = B_UAV;
        }
        // 约锟斤拷1: 锟斤拷b_i <= B_UAV => g_1 in (-inf, B_UAV]
        g_l[0] = -1e20;
        g_u[0] = B_UAV;

        // Hard QoS 约锟斤拷: b_i * cap_i >= r_min_i => g_k in [r_min_i, +inf)
        int con_idx = 1;
        for (int i = 0; i < n_users; i++) {
            if (enforce_qos && is_hard[i]) {
                g_l[con_idx] = r_mins[i];
                g_u[con_idx] = 1e20;
                con_idx++;
            }
        }
        return true;
    }

    // 锟斤拷始锟斤拷
    bool get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m_con, bool init_lambda, Number* lambda) override
    {
        if (init_x) {
            for (int i = 0; i < n; i++) {
                x[i] = b_init[i];
            }
        }
        return true;
    }

    // 目锟疥函锟斤拷 (锟斤拷小锟斤拷 -锟斤拷 U_i)
    bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) override
    {
        obj_value = 0.0;
        for (int i = 0; i < n; i++) {
            if (is_hard[i]) {
                obj_value -= sigmoid_hard_utility(x[i], caps[i], r_mins[i],
                    weights[i], nu);
            }
            else {
                obj_value -= normalized_elastic_utility(x[i], caps[i],
                    B_UAV, weights[i]);
            }
        }
        return true;
    }

    // 目锟疥函锟斤拷锟捷讹拷
    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) override
    {
        for (int i = 0; i < n; i++) {
            if (is_hard[i]) {
                grad_f[i] = -sigmoid_hard_grad(x[i], caps[i], r_mins[i],
                    weights[i], nu);
            }
            else {
                grad_f[i] = -normalized_elastic_grad(x[i], caps[i],
                    B_UAV, weights[i]);
            }
        }
        return true;
    }

    // 约锟斤拷值
    bool eval_g(Index n, const Number* x, bool new_x,
        Index m_con, Number* g) override
    {
        // 约锟斤拷1: 锟斤拷b_i
        g[0] = 0.0;
        for (int i = 0; i < n; i++) {
            g[0] += x[i];
        }
        // Hard QoS约锟斤拷: b_i * cap_i
        int con_idx = 1;
        for (int i = 0; i < n; i++) {
            if (enforce_qos && is_hard[i]) {
                g[con_idx] = x[i] * caps[i];
                con_idx++;
            }
        }
        return true;
    }

    // 约锟斤拷 Jacobian 锟结构锟斤拷值
    bool eval_jac_g(Index n, const Number* x, bool new_x,
        Index m_con, Index nele_jac,
        Index* iRow, Index* jCol, Number* values) override
    {
        if (values == nullptr) {
            // 锟结构锟斤拷稀锟斤拷模式锟斤拷
            int idx = 0;
            // 约锟斤拷1锟斤拷 Jacobian: 锟斤拷锟斤拷锟斤拷 b_i 锟斤拷偏锟斤拷锟斤拷锟斤拷 1
            for (int i = 0; i < n; i++) {
                iRow[idx] = 0;
                jCol[idx] = i;
                idx++;
            }
            // Hard QoS 约锟斤拷
            int con_idx = 1;
            for (int i = 0; i < n; i++) {
                if (enforce_qos && is_hard[i]) {
                    iRow[idx] = con_idx;
                    jCol[idx] = i;
                    idx++;
                    con_idx++;
                }
            }
        }
        else {
            // 锟斤拷值
            int idx = 0;
            for (int i = 0; i < n; i++) {
                values[idx] = 1.0;
                idx++;
            }
            int con_idx = 1;
            for (int i = 0; i < n; i++) {
                if (enforce_qos && is_hard[i]) {
                    values[idx] = caps[i];
                    idx++;
                    con_idx++;
                }
            }
        }
        return true;
    }

    // 锟斤拷锟结供 Hessian, 使锟斤拷 L-BFGS 锟斤拷锟斤拷
    bool eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m_con, const Number* lambda,
        bool new_lambda, Index nele_hess,
        Index* iRow, Index* jCol, Number* values) override
    {
        return false; // 使锟斤拷 L-BFGS
    }

    // 锟斤拷锟斤拷锟缴回碉拷
    void finalize_solution(SolverReturn status, Index n,
        const Number* x, const Number* z_L,
        const Number* z_U, Index m_con,
        const Number* g, const Number* lambda,
        Number obj_value, const IpoptData* ip_data,
        IpoptCalculatedQuantities* ip_cq) override
    {
        b_opt.resize(n);
        for (int i = 0; i < n; i++) {
            b_opt[i] = x[i];
        }
        obj_opt = -obj_value; // 锟街革拷为锟斤拷锟街?
    }
};


// ============================================================================
// Phase 4.3 专锟斤拷: 锟斤拷 Elastic 锟矫伙拷锟斤拷锟斤拷锟斤拷锟斤拷 (凸锟脚伙拷)
// ============================================================================
class ElasticOnly_BW_NLP : public TNLP {
public:
    int n_users;
    vector<double> caps;
    vector<double> weights;
    double B_avail;       // 锟斤拷锟矫达拷锟斤拷 (B_UAV - 锟斤拷 b_hard_th)
    double B_UAV;         // 锟斤拷锟节癸拷一锟斤拷

    vector<double> b_init;
    vector<double> b_opt;
    double obj_opt;

    ElasticOnly_BW_NLP() : n_users(0), B_avail(0), B_UAV(0), obj_opt(0) {}

    bool get_nlp_info(Index& n, Index& m_con, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style) override
    {
        n = n_users;
        m_con = 1; // 锟斤拷b_j <= B_avail
        nnz_jac_g = n;
        nnz_h_lag = 0;
        index_style = C_STYLE;
        return true;
    }

    bool get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m_con, Number* g_l, Number* g_u) override
    {
        for (int i = 0; i < n; i++) {
            x_l[i] = 0.0;
            x_u[i] = B_avail;
        }
        g_l[0] = -1e20;
        g_u[0] = B_avail;
        return true;
    }

    bool get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m_con, bool init_lambda, Number* lambda) override
    {
        if (init_x) {
            for (int i = 0; i < n; i++) x[i] = b_init[i];
        }
        return true;
    }

    bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value) override
    {
        obj_value = 0.0;
        for (int i = 0; i < n; i++) {
            obj_value -= normalized_elastic_utility(x[i], caps[i], B_UAV, weights[i]);
        }
        return true;
    }

    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) override
    {
        for (int i = 0; i < n; i++) {
            grad_f[i] = -normalized_elastic_grad(x[i], caps[i], B_UAV, weights[i]);
        }
        return true;
    }

    bool eval_g(Index n, const Number* x, bool new_x,
        Index m_con, Number* g) override
    {
        g[0] = 0.0;
        for (int i = 0; i < n; i++) g[0] += x[i];
        return true;
    }

    bool eval_jac_g(Index n, const Number* x, bool new_x,
        Index m_con, Index nele_jac,
        Index* iRow, Index* jCol, Number* values) override
    {
        if (values == nullptr) {
            for (int i = 0; i < n; i++) { iRow[i] = 0; jCol[i] = i; }
        }
        else {
            for (int i = 0; i < n; i++) values[i] = 1.0;
        }
        return true;
    }

    bool eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m_con, const Number* lambda,
        bool new_lambda, Index nele_hess,
        Index* iRow, Index* jCol, Number* values) override
    {
        return false;
    }

    void finalize_solution(SolverReturn status, Index n,
        const Number* x, const Number* z_L,
        const Number* z_U, Index m_con,
        const Number* g, const Number* lambda,
        Number obj_value, const IpoptData* ip_data,
        IpoptCalculatedQuantities* ip_cq) override
    {
        b_opt.resize(n);
        for (int i = 0; i < n; i++) b_opt[i] = x[i];
        obj_opt = -obj_value;
    }
};


// ============================================================================
// 锟斤拷锟姐法锟斤拷锟?: BAProblem::MatchingSQP_Allocation
// ============================================================================
/// Run the configured swap heuristic and return per-UAV/per-user allocations; diagnostics records fallbacks.
std::pair<std::vector<KnapsackResult>, map<int, UserResult>>
BAProblem::MatchingSQP_Allocation(MatchingSQPConfig config, AllocationDiagnostics* diagnostics)
{
    if (diagnostics) *diagnostics = AllocationDiagnostics();
    // Keep the legacy direct-call interface safe as well as the common experiment wrapper.
    if (sysModel.users.empty() || sysModel.uavs.empty() ||
        std::none_of(sysModel.uavs.begin(), sysModel.uavs.end(), [](const Uav& uav) {return uav.total_bandwidth > 0;})) {
        vector<KnapsackResult> empty(sysModel.m);
        for (int k = 0; k < sysModel.m; ++k) empty[k].uav_id = k;
        return {empty, construct_user_results(empty)};
    }
    const int M = sysModel.m;                   // UAV锟斤拷
    const int N = sysModel.n1 + sysModel.n2;    // 锟斤拷锟矫伙拷锟斤拷
    const int N1 = sysModel.n1;                 // Hard锟矫伙拷锟斤拷

    // allowable_matrix[k][i] = true 锟斤拷示 UAV k 锟斤拷锟皆凤拷锟斤拷锟矫伙拷 i
    vector<vector<bool>> allowable_matrix(M, vector<bool>(N, false));

    for (auto const& [uav_id, users_vec] : sysModel.uav_serviceable_users_map) {
        if (uav_id >= M) continue;
        for (const auto& user : users_vec) {
            // 锟斤拷锟斤拷 User 锟结构锟斤拷锟斤拷锟斤拷 id 锟街段讹拷应 0~N-1 锟斤拷锟斤拷锟斤拷
            // 锟斤拷锟? user.id 锟斤拷锟斤拷锟节ｏ拷锟斤拷锟斤拷锟绞碉拷锟斤拷锟斤拷锟斤拷取全锟斤拷锟斤拷锟斤拷
            int uid = user.ID;
            if (uid >= 0 && uid < N) {
                allowable_matrix[uav_id][uid] = true;
            }
        }
    }

    // ========================================================================
    // 锟斤拷锟捷结构
    // ========================================================================
    // pi[i] = 锟斤拷锟斤拷锟斤拷UAV锟斤拷锟?, -1锟斤拷示未锟斤拷锟斤拷
    vector<int> pi(N, -1);
    // U_k[k] = UAV k 锟斤拷锟斤拷锟斤拷没锟斤拷斜锟?
    vector<vector<int>> U_k(M);
    // b[k][i] = UAV k 锟斤拷锟斤拷锟斤拷没锟? i 锟侥达拷锟斤拷 (锟斤拷锟斤拷 i 锟斤拷 U_k[k] 时锟斤拷效)
    // 使锟斤拷 map 锟芥储稀锟斤拷锟斤拷锟斤拷
    vector<map<int, double>> b(M);

    double nu = config.nu_init;

    // ========================================================================
    // Phase 0: 贪锟侥筹拷始锟斤拷
    // ========================================================================
    // 每锟斤拷锟矫伙拷选锟斤拷锟脚碉拷锟斤拷锟斤拷锟斤拷锟斤拷UAV
    for (int i = 0; i < N; i++) {
        int best_k = -1;
        double best_cap = -1.0;
        for (int k = 0; k < M; k++) {
            // 锟斤拷锟? UAV k 锟角凤拷锟杰革拷知锟矫伙拷 i
            if (!allowable_matrix[k][i]) continue;

            double cap = sysModel.cap_list[k][i];
            if (cap > best_cap) {
                best_cap = cap;
                best_k = k;
            }
        }
        if (best_k >= 0 && best_cap > EPS) {
            pi[i] = best_k;
            U_k[best_k].push_back(i);
        }
    }

    // 锟斤拷锟街达拷锟斤拷锟斤拷始锟斤拷
    for (int k = 0; k < M; k++) {
        int n_k = (int)U_k[k].size();
        if (n_k == 0) continue;
        double bw_each = sysModel.uavs[k].total_bandwidth / n_k;
        for (int uid : U_k[k]) {
            b[k][uid] = bw_each;
        }
    }

    // ========================================================================
    // IPOPT App 锟斤拷锟斤拷锟斤拷锟斤拷锟矫ｏ拷
    // ========================================================================
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    if (IsNull(app))
        throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "SwapMatching: IPOPT creation failed");
    ApplicationReturnStatus status = app->Initialize(algProjPath + "Matching_SQP_ipopt.opt");
    if (diagnostics) diagnostics->record("matching.initialize", static_cast<int>(status));
    if (status != Solve_Succeeded) {
        throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "SwapMatching: IPOPT initialization failed");
    }

    double prev_total_utility = -1e20;

    // ========================================================================
    // 锟斤拷锟斤拷锟斤拷
    // ========================================================================
    for (int iter = 0; iter < config.max_outer_iter; iter++) {

        // ====================================================================
        // Phase 1 (Step A): 锟斤拷锟斤拷锟脚伙拷 - 锟教讹拷匹锟斤拷, 为每锟斤拷UAV锟斤拷锟斤拷锟斤拷锟?
        // ====================================================================
        auto start_time = chrono::high_resolution_clock::now();


        for (int k = 0; k < M; k++) {
            int n_k = (int)U_k[k].size();
            if (n_k == 0) continue;

            SmartPtr<SingleUAV_BW_NLP> nlp = new SingleUAV_BW_NLP();
            nlp->n_users = n_k;
            nlp->B_UAV = sysModel.uavs[k].total_bandwidth;
            nlp->nu = nu;
            nlp->enforce_qos = true;

            nlp->user_indices.resize(n_k);
            nlp->caps.resize(n_k);
            nlp->r_mins.resize(n_k);
            nlp->weights.resize(n_k);
            nlp->is_hard.resize(n_k);
            nlp->b_init.resize(n_k);

            for (int idx = 0; idx < n_k; idx++) {
                int uid = U_k[k][idx];
                nlp->user_indices[idx] = uid;
                nlp->caps[idx] = sysModel.cap_list[k][uid];
                nlp->weights[idx] = sysModel.users[uid].weight;
                nlp->is_hard[idx] = (sysModel.users[uid].uType == HARD_UTILITY) ? 1 : 0;
                nlp->r_mins[idx] = (nlp->is_hard[idx]) ? sysModel.users[uid].rMin : 0.0;

                // 使锟斤拷锟斤拷一锟街的斤拷锟斤拷锟轿拷锟绞硷拷锟?, 锟斤拷锟斤拷锟?
                if (b[k].count(uid))
                    nlp->b_init[idx] = b[k][uid];
                else
                    nlp->b_init[idx] = nlp->B_UAV / n_k;
            }

            // 锟斤拷锟斤拷欠锟斤拷锟节诧拷锟斤拷锟叫碉拷QoS约锟斤拷
            // 锟斤拷锟絟ard锟矫伙拷锟斤拷锟斤拷小锟斤拷锟斤拷锟斤拷锟斤拷之锟酵筹拷锟斤拷B_UAV锟斤拷锟斤拷锟斤拷时锟截憋拷QoS约锟斤拷
            double sum_bth = 0.0;
            for (int idx = 0; idx < n_k; idx++) {
                if (nlp->is_hard[idx] && nlp->caps[idx] > EPS) {
                    sum_bth += nlp->r_mins[idx] / nlp->caps[idx];
                }
            }
            if (sum_bth > nlp->B_UAV + EPS) {
                nlp->enforce_qos = false; // 锟斤拷锟斤拷锟叫ｏ拷锟斤拷强锟斤拷QoS
            }

            status = app->OptimizeTNLP(nlp);
            if (diagnostics) diagnostics->record("matching.uav." + std::to_string(k),
                static_cast<int>(status), status != Solve_Succeeded && status != Solved_To_Acceptable_Level);

            // 锟斤拷锟斤拷 b
            if (status == Solve_Succeeded || status == Solved_To_Acceptable_Level) {
                for (int idx = 0; idx < n_k; idx++) {
                    int uid = U_k[k][idx];
                    b[k][uid] = checked_solver_bandwidth(nlp->b_opt.at(idx), "SwapMatching.optimize");
                }
            }
        }
        auto end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

        // cout << "\nPhase 1 (Step A): 锟斤拷锟斤拷锟脚伙拷锟斤拷时: " << duration.count() << " ms" << endl;
        // ====================================================================
        // Phase 2 (Step B): 匹锟斤拷锟脚伙拷 - 锟缴对斤拷锟斤拷
        // ====================================================================
        bool swapped = true;
        int swapped_iter_count = 0;
        while (swapped) {
            swapped = false;
            for (int i = 0; i < N; i++) {
                if (pi[i] < 0) continue; // 未锟斤拷锟斤拷

                double best_gain = 0.0;
                int best_j = -1;

                for (int j = i + 1; j < N; j++) { // 只锟斤拷锟? j > i 锟斤拷锟斤拷锟截革拷
                    if (pi[j] < 0) continue;
                    if (pi[i] == pi[j]) continue; // 同一UAV

                    int k = pi[i];  // u_i 锟斤拷前锟斤拷 v_k
                    int kp = pi[j];  // u_j 锟斤拷前锟斤拷 v_k'

                    // 锟较凤拷锟皆硷拷椋猴拷锟斤拷锟角帮拷锟斤拷锟饺凤拷锟斤拷欠锟缴凤拷锟斤拷
                    // 锟斤拷锟? 1: UAV kp 锟角凤拷锟杰凤拷锟斤拷锟矫伙拷 i ? (i 锟斤拷要去 kp)
                    if (!allowable_matrix[kp][i]) continue;

                    // 锟斤拷锟? 2: UAV k 锟角凤拷锟杰凤拷锟斤拷锟矫伙拷 j ? (j 锟斤拷要去 k)
                    if (!allowable_matrix[k][j]) continue;

                    double B_UAV_k = sysModel.uavs[k].total_bandwidth;
                    double B_UAV_kp = sysModel.uavs[kp].total_bandwidth;

                    // 锟斤拷锟斤拷前效锟斤拷
                    double U_i_before = compute_user_utility(
                        sysModel.users[i], k, b[k][i],
                        sysModel.cap_list[k][i], B_UAV_k, nu);
                    double U_j_before = compute_user_utility(
                        sysModel.users[j], kp, b[kp][j],
                        sysModel.cap_list[kp][j], B_UAV_kp, nu);

                    // 锟斤拷锟斤拷锟斤拷: u_i -> v_k', 锟斤拷么锟斤拷锟? b[kp][j]
                    //          u_j -> v_k,  锟斤拷么锟斤拷锟? b[k][i]
                    double U_i_after = compute_user_utility(
                        sysModel.users[i], kp, b[kp][j],
                        sysModel.cap_list[kp][i], B_UAV_kp, nu);
                    double U_j_after = compute_user_utility(
                        sysModel.users[j], k, b[k][i],
                        sysModel.cap_list[k][j], B_UAV_k, nu);

                    double delta = (U_i_after + U_j_after) - (U_i_before + U_j_before);

                    if (delta > best_gain + EPS) {
                        best_gain = delta;
                        best_j = j;
                    }
                }

                // 执锟斤拷锟斤拷呀锟斤拷锟?
                if (best_j >= 0) {
                    int j = best_j;
                    int k = pi[i];
                    int kp = pi[j];

                    // 锟斤拷锟斤拷锟斤拷锟斤拷
                    double bw_i = b[k][i];
                    double bw_j = b[kp][j];

                    // 锟接撅拷UAV锟斤拷锟狡筹拷
                    b[k].erase(i);
                    b[kp].erase(j);
                    remove_first(U_k[k], i);
                    remove_first(U_k[kp], j);

                    // 锟斤拷锟接碉拷锟斤拷UAV
                    b[kp][i] = bw_j;  // u_i 锟斤拷锟? u_j 原锟斤拷锟侥达拷锟斤拷
                    b[k][j] = bw_i;   // u_j 锟斤拷锟? u_i 原锟斤拷锟侥达拷锟斤拷
                    U_k[kp].push_back(i);
                    U_k[k].push_back(j);

                    // 锟斤拷锟铰癸拷锟斤拷
                    pi[i] = kp;
                    pi[j] = k;

                    swapped = true;
                }
            }
            swapped_iter_count++;
        }
        //cout << " swapped_iter_count = " << swapped_iter_count << endl;

        // ====================================================================
        // 锟斤拷锟斤拷系统锟斤拷效锟斤拷 & 锟斤拷锟斤拷锟斤拷锟?
        // ====================================================================
        double total_utility = 0.0;
        for (int k = 0; k < M; k++) {
            double B_UAV_k = sysModel.uavs[k].total_bandwidth;
            for (int uid : U_k[k]) {
                total_utility += compute_user_utility(
                    sysModel.users[uid], k, b[k][uid],
                    sysModel.cap_list[k][uid], B_UAV_k, nu);
            }
        }

        // cout << "[MatchingSQP] Iter " << iter
        //     << " | nu=" << nu
        //     << " | Total Utility=" << total_utility << endl;

        double delta_u = abs(total_utility - prev_total_utility);
        if (delta_u < config.conv_eps && iter > 0) {
            //cout << "[MatchingSQP] Converged at iter " << iter << endl;
            break;
        }
        prev_total_utility = total_utility;

        // 锟斤拷锟斤拷 Sigmoid 锟斤拷锟斤拷锟斤拷锟斤拷
        nu = min(nu + config.nu_step, config.nu_max);
    }

    // ========================================================================
    // Phase 4: 锟斤拷锟叫斤拷指锟斤拷锟斤拷俜锟斤拷锟?
    // ========================================================================

    // Step 4.1: Hard锟矫伙拷锟斤拷锟斤拷锟皆恢革拷
    for (int k = 0; k < M; k++) {
        vector<int> to_remove;
        for (int uid : U_k[k]) {
            if (sysModel.users[uid].uType != HARD_UTILITY) continue;

            double cap = sysModel.cap_list[k][uid];
            if (cap < EPS) {
                // 锟脚碉拷锟斤拷锟斤拷为0, 锟睫凤拷锟斤拷锟斤拷
                to_remove.push_back(uid);
                continue;
            }
            double b_th = sysModel.users[uid].rMin / cap;

            if (hard_qos_satisfied(b[k][uid], cap, sysModel.users[uid].rMin)) {
                // 锟斤拷锟紸: 锟斤拷锟斤拷QoS, 锟斤拷锟诫到锟斤拷值
                b[k][uid] = b_th;
            }
            else {
                // 锟斤拷锟紹: 未锟斤拷锟斤拷QoS, 锟斤拷锟斤拷锟斤拷锟斤拷
                to_remove.push_back(uid);
            }
        }
        // 锟狡筹拷锟斤拷锟斤拷锟斤拷QoS锟斤拷hard锟矫伙拷
        for (int uid : to_remove) {
            b[k].erase(uid);
            remove_first(U_k[k], uid);
            pi[uid] = -1;
        }
    }

    // Step 4.2 & 4.3: 锟斤拷锟斤拷剩锟斤拷锟斤拷锟? & 锟劫凤拷锟斤拷锟紼lastic锟矫伙拷
    //for (int k = 0; k < M; k++) {
    //    double B_UAV_k = sysModel.uavs[k].total_bandwidth;

    //    // 锟斤拷锟斤拷hard锟矫伙拷锟斤拷占锟矫达拷锟斤拷
    //    double bw_hard = 0.0;
    //    vector<int> elastic_users_k;
    //    for (int uid : U_k[k]) {
    //        if (sysModel.users[uid].uType == HARD_UTILITY) {
    //            bw_hard += b[k][uid];
    //        }
    //        else {
    //            elastic_users_k.push_back(uid);
    //        }
    //    }

    //    double B_avail = B_UAV_k - bw_hard;
    //    if (B_avail < EPS || elastic_users_k.empty()) continue;

    //    int n_e = (int)elastic_users_k.size();

    //    // 使锟斤拷IPOPT锟斤拷獯縠lastic锟斤拷锟斤拷锟斤拷锟斤拷 (凸锟脚伙拷)
    //    SmartPtr<ElasticOnly_BW_NLP> nlp = new ElasticOnly_BW_NLP();
    //    nlp->n_users = n_e;
    //    nlp->B_avail = B_avail;
    //    nlp->B_UAV = B_UAV_k;
    //    nlp->caps.resize(n_e);
    //    nlp->weights.resize(n_e);
    //    nlp->b_init.resize(n_e);

    //    for (int idx = 0; idx < n_e; idx++) {
    //        int uid = elastic_users_k[idx];
    //        nlp->caps[idx] = sysModel.cap_list[k][uid];
    //        nlp->weights[idx] = sysModel.users[uid].weight;
    //        nlp->b_init[idx] = B_avail / n_e; // 锟斤拷锟街筹拷始锟斤拷
    //    }

    //    status = app->OptimizeTNLP(nlp);

    //    if (status == Solve_Succeeded || status == Solved_To_Acceptable_Level) {
    //        for (int idx = 0; idx < n_e; idx++) {
    //            int uid = elastic_users_k[idx];
    //            b[k][uid] = max(0.0, nlp->b_opt[idx]);
    //        }
    //    }
    //}

    // ========================================================================
    // 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
    // ========================================================================
    vector<KnapsackResult> allResults(M);

    for (int k = 0; k < M; k++) {
        clean_KnapsackResult(allResults[k]);
        allResults[k].uav_id = k;
        double B_UAV_k = sysModel.uavs[k].total_bandwidth;

        for (int uid : U_k[k]) {
            double bw = b[k][uid];
            if (bw == 0.0) continue; // An assigned zero-bandwidth user is not served.
            double cap = sysModel.cap_list[k][uid];
            double SNR_avg_dB = sysModel.SNRave_list[k][uid];
            // 锟斤拷锟斤拷锟斤拷锟秸碉拷原始效锟矫ｏ拷锟斤拷sigmoid锟斤拷锟狡ｏ拷
            double uti;
            if (sysModel.users[uid].uType == HARD_UTILITY) {
                uti = sysModel.users[uid].hard_utility(bw, cap, SNR_avg_dB);
            }
            else {
                uti = sysModel.users[uid].elastic_utility(bw, cap);
            }

            add_KnapsackResult(allResults[k], sysModel.users[uid], bw, uti);

        }
    }

    map<int, UserResult> userResults = construct_user_results(allResults);

    // 锟斤拷锟斤拷 BAProblem 锟斤拷 alloc_matrix 锟斤拷 connect_matrix
    alloc_matrix = vector<vector<double>>(M, vector<double>(N, 0.0));
    connect_matrix = vector<vector<int>>(M, vector<int>(N, 0));
    for (int k = 0; k < M; k++) {
        for (int uid : U_k[k]) {
            alloc_matrix[k][uid] = b[k][uid];
            connect_matrix[k][uid] = 1;
        }
    }

    // 锟斤拷印锟斤拷锟秸斤拷锟?
    double final_utility = 0.0;
    int served_hard = 0, served_elastic = 0;
    for (int i = 0; i < N; i++) {
        final_utility += userResults[i].utility;
        if (pi[i] >= 0) {
            if (sysModel.users[i].uType == HARD_UTILITY)
                served_hard++;
            else
                served_elastic++;
        }
    }
    // cout << "============================================" << endl;
    // cout << "[MatchingSQP] Final Results:" << endl;
    // cout << "  Total Utility = " << final_utility << endl;
    // cout << "  Served Hard Users   = " << served_hard << " / " << N1 << endl;
    // cout << "  Served Elastic Users = " << served_elastic << " / " << (N - N1) << endl;
    // cout << "============================================" << endl;

    return { allResults, userResults };
}

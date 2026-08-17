#include "EntityDefinition.h"
#include <numeric>    // std::iota
#include <unordered_set>
#include <unordered_map>

using namespace Ipopt;
// ============================================================================
// �㷨��������
// ============================================================================


// ============================================================================
// ���������㵥���û�Ч�� (Sigmoid ���� / ��һ������)
// ============================================================================

/// Hard�û� Sigmoid ����Ч��
static double sigmoid_hard_utility(double b_ki, double cap_ki, double r_min,
    double w_i, double nu)
{
    // R_i = b_ki * cap_ki (cap_ki = channel_capacity for hard user)
    double R_i = b_ki * cap_ki;
    double exponent = -nu * (R_i - r_min);
    // ��ֹ���?
    if (exponent > 500.0) return 0.0;
    if (exponent < -500.0) return w_i;
    double sigma = 1.0 / (1.0 + exp(exponent));
    return w_i * sigma;
}

/// Hard�û� Sigmoid Ч�ö� b_ki ���ݶ�
static double sigmoid_hard_grad(double b_ki, double cap_ki, double r_min,
    double w_i, double nu)
{
    double R_i = b_ki * cap_ki;
    double exponent = -nu * (R_i - r_min);
    if (exponent > 500.0 || exponent < -500.0) return 0.0;
    double sigma = 1.0 / (1.0 + exp(exponent));
    return w_i * sigma * (1.0 - sigma) * nu * cap_ki;
}

/// Elastic�û� ��һ������Ч��
/// U_j^E = w_j * log2(1 + b*Omega) / log2(1 + B_UAV*Omega)
static double normalized_elastic_utility(double b_kj, double cap_kj,
    double B_UAV, double w_j)
{
    // cap_kj ���� log2(1 + SNR_avg), �� Omega^avg ��Ӧ���ŵ�����
    // ���� elastic user: rate = b_kj * cap_kj
    // Ч�� = w_j * log2(1 + b_kj * cap_kj) / log2(1 + B_UAV * cap_kj)
    double r = b_kj * cap_kj;
    double r_max = B_UAV * cap_kj;
    double denom = log2(1.0 + r_max);
    if (denom < 1e-12) return 0.0;
    return w_j * log2(1.0 + r) / denom;
}

/// Elastic�û� ��һ������Ч�ö� b_kj ���ݶ�
static double normalized_elastic_grad(double b_kj, double cap_kj,
    double B_UAV, double w_j)
{
    double r = b_kj * cap_kj;
    double r_max = B_UAV * cap_kj;
    double denom = log2(1.0 + r_max);
    if (denom < 1e-12) return 0.0;
    return w_j * cap_kj / (log(2.0) * (1.0 + r) * denom);
}

/// ͨ��Ч�ü��㣨�����û������Զ�ѡ��
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
// IPOPT ���������? UAV �������������� (Phase 1 / Phase 4.3)
// ============================================================================
// �� TNLP ����Ϊ���� UAV �����������Ż�����
// ���߱���: b[0], b[1], ..., b[n-1] ��Ӧ UAV �����? n ���û�
// Ŀ��: max �� U_i(b_i) => min -�� U_i(b_i)
// Լ��: �� b_i <= B_UAV (����ʽԼ�� g(x) <= 0 ��ʽ: ��b_i - B_UAV <= 0)
//        �� hard �û�: b_i * cap_i >= r_min_i => r_min_i - b_i * cap_i <= 0
// �����߽�: b_i >= 0

class SingleUAV_BW_NLP : public TNLP {
public:
    // ��������
    int n_users;                    // �� UAV ������û���?
    vector<int> user_indices;       // ȫ���û�����
    vector<double> caps;            // ÿ���û���Ӧ���ŵ�����
    vector<double> r_mins;          // hard�û�����С����(elastic��Ϊ0)
    vector<double> weights;         // �û�Ȩ��
    vector<int> is_hard;            // �Ƿ�Ϊhard�û�
    double B_UAV;                   // UAV�ܴ���
    double nu;                      // Sigmoid ��������
    bool enforce_qos;               // �Ƿ�ǿ��QoSԼ��(Phase 1: true, Phase 4.3: false)

    // ��ʼ�� & ���?
    vector<double> b_init;
    vector<double> b_opt;
    double obj_opt;

    // Լ������
    int n_hard_constraints;         // hard QoS Լ������

    SingleUAV_BW_NLP() : n_users(0), B_UAV(0), nu(10), enforce_qos(true),
        obj_opt(0), n_hard_constraints(0) {
    }

    // ��ȡ����ά��
    bool get_nlp_info(Index& n, Index& m_con, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style) override
    {
        n = n_users;
        // Լ����: 1 (�����ܺ�) + n_hard_constraints (hard QoS)
        n_hard_constraints = 0;
        if (enforce_qos) {
            for (int i = 0; i < n_users; i++) {
                if (is_hard[i]) n_hard_constraints++;
            }
        }
        m_con = 1 + n_hard_constraints;
        // Jacobian ����Ԫ: ����Լ����n������, ÿ��QoSԼ����1������
        nnz_jac_g = n + n_hard_constraints;
        nnz_h_lag = 0; // ʹ��L-BFGS����, ����ҪHessian
        index_style = C_STYLE;
        return true;
    }

    // ������Լ���߽�
    bool get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m_con, Number* g_l, Number* g_u) override
    {
        // �����߽�: b_i >= 0, b_i <= B_UAV
        for (int i = 0; i < n; i++) {
            x_l[i] = 0.0;
            x_u[i] = B_UAV;
        }
        // Լ��1: ��b_i <= B_UAV => g_1 in (-inf, B_UAV]
        g_l[0] = -1e20;
        g_u[0] = B_UAV;

        // Hard QoS Լ��: b_i * cap_i >= r_min_i => g_k in [r_min_i, +inf)
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

    // ��ʼ��
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

    // Ŀ�꺯�� (��С�� -�� U_i)
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

    // Ŀ�꺯���ݶ�
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

    // Լ��ֵ
    bool eval_g(Index n, const Number* x, bool new_x,
        Index m_con, Number* g) override
    {
        // Լ��1: ��b_i
        g[0] = 0.0;
        for (int i = 0; i < n; i++) {
            g[0] += x[i];
        }
        // Hard QoSԼ��: b_i * cap_i
        int con_idx = 1;
        for (int i = 0; i < n; i++) {
            if (enforce_qos && is_hard[i]) {
                g[con_idx] = x[i] * caps[i];
                con_idx++;
            }
        }
        return true;
    }

    // Լ�� Jacobian �ṹ��ֵ
    bool eval_jac_g(Index n, const Number* x, bool new_x,
        Index m_con, Index nele_jac,
        Index* iRow, Index* jCol, Number* values) override
    {
        if (values == nullptr) {
            // �ṹ��ϡ��ģʽ��
            int idx = 0;
            // Լ��1�� Jacobian: ������ b_i ��ƫ������ 1
            for (int i = 0; i < n; i++) {
                iRow[idx] = 0;
                jCol[idx] = i;
                idx++;
            }
            // Hard QoS Լ��
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
            // ��ֵ
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

    // ���ṩ Hessian, ʹ�� L-BFGS ����
    bool eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m_con, const Number* lambda,
        bool new_lambda, Index nele_hess,
        Index* iRow, Index* jCol, Number* values) override
    {
        return false; // ʹ�� L-BFGS
    }

    // �����ɻص�
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
        obj_opt = -obj_value; // �ָ�Ϊ����?
    }
};


// ============================================================================
// Phase 4.3 ר��: �� Elastic �û��������� (͹�Ż�)
// ============================================================================
class ElasticOnly_BW_NLP : public TNLP {
public:
    int n_users;
    vector<double> caps;
    vector<double> weights;
    double B_avail;       // ���ô��� (B_UAV - �� b_hard_th)
    double B_UAV;         // ���ڹ�һ��

    vector<double> b_init;
    vector<double> b_opt;
    double obj_opt;

    ElasticOnly_BW_NLP() : n_users(0), B_avail(0), B_UAV(0), obj_opt(0) {}

    bool get_nlp_info(Index& n, Index& m_con, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style) override
    {
        n = n_users;
        m_con = 1; // ��b_j <= B_avail
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
// ���㷨���?: BAProblem::MatchingSQP_Allocation
// ============================================================================
std::pair<std::vector<KnapsackResult>, map<int, UserResult>>
BAProblem::MatchingSQP_Allocation(MatchingSQPConfig config)
{
    const int M = sysModel.m;                   // UAV��
    const int N = sysModel.n1 + sysModel.n2;    // ���û���
    const int N1 = sysModel.n1;                 // Hard�û���

    // allowable_matrix[k][i] = true ��ʾ UAV k ���Է����û� i
    vector<vector<bool>> allowable_matrix(M, vector<bool>(N, false));

    for (auto const& [uav_id, users_vec] : sysModel.uav_serviceable_users_map) {
        if (uav_id >= M) continue;
        for (const auto& user : users_vec) {
            // ���� User �ṹ������ id �ֶζ�Ӧ 0~N-1 ������
            // ���? user.id �����ڣ������ʵ�������ȡȫ������
            int uid = user.ID;
            if (uid >= 0 && uid < N) {
                allowable_matrix[uav_id][uid] = true;
            }
        }
    }

    // ========================================================================
    // ���ݽṹ
    // ========================================================================
    // pi[i] = ������UAV���?, -1��ʾδ����
    vector<int> pi(N, -1);
    // U_k[k] = UAV k ������û��б�?
    vector<vector<int>> U_k(M);
    // b[k][i] = UAV k ������û�? i �Ĵ��� (���� i �� U_k[k] ʱ��Ч)
    // ʹ�� map �洢ϡ������
    vector<map<int, double>> b(M);

    double nu = config.nu_init;

    // ========================================================================
    // Phase 0: ̰�ĳ�ʼ��
    // ========================================================================
    // ÿ���û�ѡ���ŵ���������UAV
    for (int i = 0; i < N; i++) {
        int best_k = -1;
        double best_cap = -1.0;
        for (int k = 0; k < M; k++) {
            // ���? UAV k �Ƿ��ܸ�֪�û� i
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

    // ���ִ�����ʼ��
    for (int k = 0; k < M; k++) {
        int n_k = (int)U_k[k].size();
        if (n_k == 0) continue;
        double bw_each = sysModel.uavs[k].total_bandwidth / n_k;
        for (int uid : U_k[k]) {
            b[k][uid] = bw_each;
        }
    }

    // ========================================================================
    // IPOPT App ���������ã�
    // ========================================================================
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    ApplicationReturnStatus status = app->Initialize("Matching_SQP_ipopt.opt");
    if (status != Solve_Succeeded) {
        cerr << "[MatchingSQP] IPOPT initialization failed!" << endl;
    }

    double prev_total_utility = -1e20;

    // ========================================================================
    // ������
    // ========================================================================
    for (int iter = 0; iter < config.max_outer_iter; iter++) {

        // ====================================================================
        // Phase 1 (Step A): �����Ż� - �̶�ƥ��, Ϊÿ��UAV�������?
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

                // ʹ����һ�ֵĽ����Ϊ��ʼ��?, �����?
                if (b[k].count(uid))
                    nlp->b_init[idx] = b[k][uid];
                else
                    nlp->b_init[idx] = nlp->B_UAV / n_k;
            }

            // ����Ƿ���ڲ����е�QoSԼ��
            // ���hard�û�����С��������֮�ͳ���B_UAV������ʱ�ر�QoSԼ��
            double sum_bth = 0.0;
            for (int idx = 0; idx < n_k; idx++) {
                if (nlp->is_hard[idx] && nlp->caps[idx] > EPS) {
                    sum_bth += nlp->r_mins[idx] / nlp->caps[idx];
                }
            }
            if (sum_bth > nlp->B_UAV + EPS) {
                nlp->enforce_qos = false; // �����У���ǿ��QoS
            }

            status = app->OptimizeTNLP(nlp);

            // ���� b
            if (status == Solve_Succeeded || status == Solved_To_Acceptable_Level) {
                for (int idx = 0; idx < n_k; idx++) {
                    int uid = U_k[k][idx];
                    b[k][uid] = max(0.0, nlp->b_opt[idx]);
                }
            }
        }
        auto end_time = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

        // cout << "\nPhase 1 (Step A): �����Ż���ʱ: " << duration.count() << " ms" << endl;
        // ====================================================================
        // Phase 2 (Step B): ƥ���Ż� - �ɶԽ���
        // ====================================================================
        bool swapped = true;
        int swapped_iter_count = 0;
        while (swapped) {
            swapped = false;
            for (int i = 0; i < N; i++) {
                if (pi[i] < 0) continue; // δ����

                double best_gain = 0.0;
                int best_j = -1;

                for (int j = i + 1; j < N; j++) { // ֻ���? j > i �����ظ�
                    if (pi[j] < 0) continue;
                    if (pi[i] == pi[j]) continue; // ͬһUAV

                    int k = pi[i];  // u_i ��ǰ�� v_k
                    int kp = pi[j];  // u_j ��ǰ�� v_k'

                    // �Ϸ��Լ�飺����ǰ����ȷ���Ƿ�ɷ���
                    // ���? 1: UAV kp �Ƿ��ܷ����û� i ? (i ��Ҫȥ kp)
                    if (!allowable_matrix[kp][i]) continue;

                    // ���? 2: UAV k �Ƿ��ܷ����û� j ? (j ��Ҫȥ k)
                    if (!allowable_matrix[k][j]) continue;

                    double B_UAV_k = sysModel.uavs[k].total_bandwidth;
                    double B_UAV_kp = sysModel.uavs[kp].total_bandwidth;

                    // ����ǰЧ��
                    double U_i_before = compute_user_utility(
                        sysModel.users[i], k, b[k][i],
                        sysModel.cap_list[k][i], B_UAV_k, nu);
                    double U_j_before = compute_user_utility(
                        sysModel.users[j], kp, b[kp][j],
                        sysModel.cap_list[kp][j], B_UAV_kp, nu);

                    // ������: u_i -> v_k', ��ô���? b[kp][j]
                    //          u_j -> v_k,  ��ô���? b[k][i]
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

                // ִ����ѽ���?
                if (best_j >= 0) {
                    int j = best_j;
                    int k = pi[i];
                    int kp = pi[j];

                    // ��������
                    double bw_i = b[k][i];
                    double bw_j = b[kp][j];

                    // �Ӿ�UAV���Ƴ�
                    b[k].erase(i);
                    b[kp].erase(j);
                    remove_first(U_k[k], i);
                    remove_first(U_k[kp], j);

                    // ���ӵ���UAV
                    b[kp][i] = bw_j;  // u_i ���? u_j ԭ���Ĵ���
                    b[k][j] = bw_i;   // u_j ���? u_i ԭ���Ĵ���
                    U_k[kp].push_back(i);
                    U_k[k].push_back(j);

                    // ���¹���
                    pi[i] = kp;
                    pi[j] = k;

                    swapped = true;
                }
            }
            swapped_iter_count++;
        }
        //cout << " swapped_iter_count = " << swapped_iter_count << endl;

        // ====================================================================
        // ����ϵͳ��Ч�� & �������?
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

        // ���� Sigmoid ��������
        nu = min(nu + config.nu_step, config.nu_max);
    }

    // ========================================================================
    // Phase 4: ���н�ָ����ٷ���?
    // ========================================================================

    // Step 4.1: Hard�û������Իָ�
    for (int k = 0; k < M; k++) {
        vector<int> to_remove;
        for (int uid : U_k[k]) {
            if (sysModel.users[uid].uType != HARD_UTILITY) continue;

            double cap = sysModel.cap_list[k][uid];
            if (cap < EPS) {
                // �ŵ�����Ϊ0, �޷�����
                to_remove.push_back(uid);
                continue;
            }
            double b_th = sysModel.users[uid].rMin / cap;

            if (b[k][uid] >= b_th - EPS) {
                // ���A: ����QoS, ���뵽��ֵ
                b[k][uid] = b_th;
            }
            else {
                // ���B: δ����QoS, ��������
                to_remove.push_back(uid);
            }
        }
        // �Ƴ�������QoS��hard�û�
        for (int uid : to_remove) {
            b[k].erase(uid);
            remove_first(U_k[k], uid);
            pi[uid] = -1;
        }
    }

    // Step 4.2 & 4.3: ����ʣ�����? & �ٷ����Elastic�û�
    //for (int k = 0; k < M; k++) {
    //    double B_UAV_k = sysModel.uavs[k].total_bandwidth;

    //    // ����hard�û���ռ�ô���
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

    //    // ʹ��IPOPT��ⴿelastic�������� (͹�Ż�)
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
    //        nlp->b_init[idx] = B_avail / n_e; // ���ֳ�ʼ��
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
    // ����������
    // ========================================================================
    vector<KnapsackResult> allResults(M);

    for (int k = 0; k < M; k++) {
        clean_KnapsackResult(allResults[k]);
        allResults[k].uav_id = k;
        double B_UAV_k = sysModel.uavs[k].total_bandwidth;

        for (int uid : U_k[k]) {
            double bw = b[k][uid];
            double cap = sysModel.cap_list[k][uid];
            double SNR_avg_dB = sysModel.SNRave_list[k][uid];
            // �������յ�ԭʼЧ�ã���sigmoid���ƣ�
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

    // ���� BAProblem �� alloc_matrix �� connect_matrix
    alloc_matrix = vector<vector<double>>(M, vector<double>(N, 0.0));
    connect_matrix = vector<vector<int>>(M, vector<int>(N, 0));
    for (int k = 0; k < M; k++) {
        for (int uid : U_k[k]) {
            alloc_matrix[k][uid] = b[k][uid];
            connect_matrix[k][uid] = 1;
        }
    }

    // ��ӡ���ս��?
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
#include "EntityDefinition.h"


// AlgRelaxRound: independent relaxation/rounding baseline, with small-model derivative verification.
// ==================== IPOPT NLP 问题定义 ====================

using namespace Ipopt;
/// <summary>
/// 松弛问题的IPOPT求解器：优化 x 和 b
/// </summary>
class RelaxedProblem_NLP : public TNLP {
public:
    // 系统参数
    const SystemMd& sysModel;
    int m;  // UAV数量
    int n;  // 总用户数量
    int n1; // 硬用户数量
    int n2; // 弹性用户数量


    // capacity线性值 (从dB转换)
    vector<vector<double>> hard_cap;   // m x n1
    vector<vector<double>> elastic_cap; // m x n2

    // 保存求解结果
    vector<vector<double>> solution_x;  // m x n
    vector<vector<double>> solution_b;  // m x n
    vector<vector<bool>> allowable_matrix; // 存储连通性矩阵，true表示UAV k可以服务用户i
    double solution_obj_value;

    RelaxedProblem_NLP(const SystemMd& sys) : sysModel(sys) {
        m = sysModel.m;
        n1 = sysModel.n1;
        n2 = sysModel.n2;
        n = n1 + n2;

        // 初始化capacity线性值
        hard_cap.resize(m, vector<double>(n1));
        elastic_cap.resize(m, vector<double>(n2));

        // 初始化解向量
        solution_x.resize(m, vector<double>(n, 0.0));
        solution_b.resize(m, vector<double>(n, 0.0));
        solution_obj_value = 0.0;
        // 【新增】: 初始化连通性矩阵
        allowable_matrix.assign(m, vector<bool>(n, false));
        // 遍历 sysModel 中的 map 填充矩阵
        // 假设 uav_serviceable_users_map 的 key 是 uav_id (int)
        for (auto const& [uav_id, users_vec] : sysModel.uav_serviceable_users_map) {
            if (uav_id < 0 || uav_id >= m) continue; // 安全检查
            for (const auto& user : users_vec) {
                // 注意：这里假设 user.id 对应于算法中 0 到 n-1 的索引
                // 如果你的 User 结构体没有 id，或者索引逻辑不同，请在此处修改
                int user_idx = user.ID;
                if (user_idx >= 0 && user_idx < n) {
                    allowable_matrix[uav_id][user_idx] = true;
                }
            }
        }

        // Use cap_list in the objective, constraints and all derivatives.
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                hard_cap[k][i] = sysModel.cap_list[k][i];

            }
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                elastic_cap[k][j] = sysModel.cap_list[k][user_idx];
            }
        }
    }

    virtual ~RelaxedProblem_NLP() {}

    // 返回变量数量: m*n个x + m*n个b
    virtual bool get_nlp_info(Index& n_vars, Index& n_constraints,
        Index& nnz_jac_g, Index& nnz_h_lag,
        IndexStyleEnum& index_style) {
        n_vars = 2 * m * n;  // x[m*n] + b[m*n]

        // 约束数量:
        // 1. 每个用户最多连1个UAV: n个约束
        // 2. b <= x*B_UAV: m*n个约束
        // 3. 每个UAV总带宽限制: m个约束
        // 4. 硬用户最小速率: n1个约束
        n_constraints = n + m * n + m + n1 * m;

        // 雅可比矩阵非零元素数量（粗略估计）
        nnz_jac_g = n * m + m * n * 2 + m * n + n1 * m * 2;

        // 因为是完全对角矩阵，非零元素数量等于变量总数
        nnz_h_lag = 2 * m * n;

        index_style = TNLP::C_STYLE;
        return true;
    }

    // 变量上下界
    virtual bool get_bounds_info(Index n_vars, Number* x_l, Number* x_u,
        Index n_constraints, Number* g_l, Number* g_u) {
        // 变量: [x[0,0], x[0,1], ..., x[m-1,n-1], b[0,0], b[0,1], ..., b[m-1,n-1]]

        // x的界: [0, 1]
        /*for (int i = 0; i < m * n; i++) {
            x_l[i] = 0.0;
            x_u[i] = 1.0;
        }*/
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n; i++) {
                int idx = k * n + i;
                x_l[idx] = 0.0;

                // 【修改点】: 如果不在服务范围内，强制上界为 0
                if (allowable_matrix[k][i]) {
                    x_u[idx] = 1.0;
                }
                else {
                    x_u[idx] = 0.0; // 强制 x = 0
                }
            }
        }

        // b的界: [0, +inf)
        for (int i = m * n; i < 2 * m * n; i++) {
            x_l[i] = 0.0;
            x_u[i] = 1e20;
        }

        int constraint_idx = 0;

        // 约束1: sum_k x[k][i] - 1 <= 0 for each user i
        for (int i = 0; i < n; i++) {
            g_l[constraint_idx] = -1e20;
            g_u[constraint_idx] = 0.0;
            constraint_idx++;
        }

        // 约束2: b[k][i] - x[k][i] * B_UAV <= 0 for each UAV k and user i
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n; i++) {
                g_l[constraint_idx] = -1e20;
                g_u[constraint_idx] = 0.0;
                constraint_idx++;
            }
        }

        // 约束3: sum_i b[k][i] - B_UAV <= 0 for each UAV k
        for (int k = 0; k < m; k++) {
            g_l[constraint_idx] = -1e20;
            g_u[constraint_idx] = 0;
            constraint_idx++;
        }

        // 约束4: x[k][i] * r_min - b[k][i] * log2(1 + capacity) <= 0 for hard users
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                g_l[constraint_idx] = -1e20;
                g_u[constraint_idx] = 0.0;
                constraint_idx++;
            }
        }

        return true;
    }

    // 初始值
    virtual bool get_starting_point(Index n_vars, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m_constr, bool init_lambda, Number* lambda) {
        if (init_x) {
            // 初始化x
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    int idx = k * n + i;

                    // 【修改点】: 仅对可服务的用户赋予非零初值
                    if (allowable_matrix[k][i]) {
                        x[idx] = 1.0 / m; // 或者更精细的初始化
                    }
                    else {
                        x[idx] = 0.0;     // 必须是 0
                    }
                }
            }

            // 初始化b为小值
            for (int i = m * n; i < 2 * m * n; i++) {
                x[i] = 0.0;
            }
        }
        return true;
    }

    // 目标函数
    virtual bool eval_f(Index n_vars, const Number* x, bool new_x,
        Number& obj_value) {
        obj_value = 0.0;
        double total_value = 0.0;
        // x变量: x[k*n + i] 表示 x[k][i]
        // b变量: x[m*n + k*n + i] 表示 b[k][i]

        // 硬用户贡献
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                int idx_x = k * n + i;
                double x_ki = x[idx_x];

                double bw = sysModel.Bth_list[k][i];
                double cap = sysModel.cap_list[k][i];
                double SNR_avg_dB = sysModel.SNRave_list[k][i];
                User user = sysModel.users[i];
                total_value += x_ki * user.utility(bw, cap, SNR_avg_dB);

            }
        }

        // 弹性用户贡献
        for (int k = 0; k < m; k++) {
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                int idx_b = m * n + k * n + user_idx;
                double b_kj = x[idx_b];

                double cap = sysModel.cap_list[k][user_idx];
                double SNR_avg_dB = sysModel.SNRave_list[k][user_idx];
                User user = sysModel.users[user_idx];

                total_value += user.utility(b_kj, cap, SNR_avg_dB);
            }
        }
        obj_value = -total_value; // 最大化效用等价于最小化负效用
        return true;
    }

    // 目标函数梯度
    virtual bool eval_grad_f(Index n_vars, const Number* x, bool new_x,
        Number* grad_f) {
        // 初始化梯度为0
        for (int i = 0; i < n_vars; i++) {
            grad_f[i] = 0.0;
        }

        // 硬用户部分: 对x求导
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                int idx_x = k * n + i;
                double bw = sysModel.Bth_list[k][i];
                double cap = sysModel.cap_list[k][i];
                double SNR_avg_dB = sysModel.SNRave_list[k][i];
                User user = sysModel.users[i];

                grad_f[idx_x] = -1 * user.utility(bw, cap, SNR_avg_dB);
            }
        }

        // 弹性用户部分: 对b求导
        for (int k = 0; k < m; k++) {
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                int idx_b = m * n + k * n + user_idx;
                double b_kj = x[idx_b];
                double weight = sysModel.users[user_idx].weight;
                double capacity = elastic_cap[k][j];

                double spectral_efficiency = capacity;
                double rate = b_kj * spectral_efficiency;

                // d/db[w * log2(rate + 1)] = w * spectral_efficiency / ((rate + 1) * ln(2))
                if (rate + 1.0 > 1e-4) {
                    grad_f[idx_b] = -1 * weight * spectral_efficiency / ((rate + 1.0) * log(2.0));
                }
            }
        }

        return true;
    }

    // 约束函数
    virtual bool eval_g(Index n_vars, const Number* x, bool new_x,
        Index m_constr, Number* g) {
        int constraint_idx = 0;

        // 约束1: sum_k x[k][i] - 1<= 0
        for (int i = 0; i < n; i++) {
            g[constraint_idx] = 0.0;
            for (int k = 0; k < m; k++) {
                int idx_x = k * n + i;
                g[constraint_idx] += x[idx_x];
            }
            g[constraint_idx] -= 1;
            constraint_idx++;
        }

        // 约束2: b[k][i] - x[k][i] * B_UAV <= 0
        for (int k = 0; k < m; k++) {
            double B_UAV = sysModel.uavs[k].total_bandwidth;
            for (int i = 0; i < n; i++) {
                g[constraint_idx] = 0.0;
                int idx_x = k * n + i;
                int idx_b = m * n + k * n + i;
                g[constraint_idx] += x[idx_b] - x[idx_x] * B_UAV;
                constraint_idx++;
            }
        }

        // 约束3: sum_i b[k][i] - B_UAV <= 0
        for (int k = 0; k < m; k++) {
            g[constraint_idx] = 0.0;
            for (int i = 0; i < n; i++) {
                int idx_b = m * n + k * n + i;
                g[constraint_idx] += x[idx_b];
            }
            g[constraint_idx] -= sysModel.uavs[k].total_bandwidth;
            constraint_idx++;
        }

        // 约束4: x[k][i] * r_min - b[k][i] * log2(1 + capacity) <= 0
        for (int k = 0; k < m; k++)
        {
            for (int i = 0; i < n1; i++) {

                double r_min = sysModel.users[i].rMin;
                g[constraint_idx] = 0.0;
                int idx_x = k * n + i;
                int idx_b = m * n + k * n + i;
                double capacity = hard_cap[k][i];

                g[constraint_idx] = (x[idx_x] * r_min - x[idx_b] * capacity);

                constraint_idx++;
            }
        }



        return true;
    }

    // 约束雅可比矩阵
    virtual bool eval_jac_g(Index n_vars, const Number* x, bool new_x,
        Index m_constr, Index nele_jac, Index* iRow,
        Index* jCol, Number* values) {
        if (values == NULL) {
            // 返回稀疏结构
            int nz_idx = 0;
            int constraint_idx = 0;

            // 约束1: sum_k x[k][i] <= 1
            for (int i = 0; i < n; i++) {
                for (int k = 0; k < m; k++) {
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = k * n + i;
                    nz_idx++;
                }
                constraint_idx++;
            }

            // 约束2: b[k][i] - x[k][i] * B_UAV <= 0
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    // b[k][i]项
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = m * n + k * n + i;
                    nz_idx++;
                    // -x[k][i] * B_UAV项
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = k * n + i;
                    nz_idx++;
                    constraint_idx++;
                }
            }

            // 约束3: sum_i b[k][i] <= B_UAV
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = m * n + k * n + i;
                    nz_idx++;
                }
                constraint_idx++;
            }

            // 约束4: x[k][i] * r_min - b[k][i] * log2(1 + capacity) <= 0
            // 硬用户部分
            for (int k = 0; k < m; k++)
            {
                for (int i = 0; i < n1; i++) {
                    // x[k][i] * r_min项
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = k * n + i;
                    nz_idx++;
                    // -b[k][i] * log2(1 + capacity)项
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = m * n + k * n + i;
                    nz_idx++;
                    constraint_idx++;
                }
            }
        }
        else {
            // 返回值
            int nz_idx = 0;
            int constraint_idx = 0;

            // 约束1
            for (int i = 0; i < n; i++) {
                for (int k = 0; k < m; k++) {
                    values[nz_idx] = 1.0;
                    nz_idx++;
                }
                constraint_idx++;
            }

            // 约束2
            for (int k = 0; k < m; k++) {
                double B_UAV = sysModel.uavs[k].total_bandwidth;
                for (int i = 0; i < n; i++) {
                    values[nz_idx] = 1.0;  // b项系数
                    nz_idx++;
                    values[nz_idx] = -B_UAV;  // -x*B_UAV项系数
                    nz_idx++;
                    constraint_idx++;
                }
            }

            // 约束3
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    values[nz_idx] = 1.0;
                    nz_idx++;
                }
                constraint_idx++;
            }

            // 约束4
            for (int k = 0; k < m; k++)
            {
                for (int i = 0; i < n1; i++) {
                    double r_min = sysModel.users[i].rMin;
                    values[nz_idx] = r_min;  // x*r_min项系数
                    nz_idx++;
                    double capacity = hard_cap[k][i];
                    values[nz_idx] = -capacity;  // -b*log2(1 + capacity)项系数
                    nz_idx++;
                    constraint_idx++;
                }
            }
        }

        return true;
    }

    virtual bool eval_h(Index n_vars, const Number* x, bool new_x,
        Number obj_factor, Index m_constr, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow,
        Index* jCol, Number* values) {

        // 模式 1: 返回稀疏结构 (values 为 NULL)
        if (values == NULL) {
            // 返回对角矩阵结构: H[i, i]
            // 尽管很多元素实际上是0，但声明为对角阵让索引处理最简单
            for (int i = 0; i < n_vars; i++) {
                iRow[i] = i;
                jCol[i] = i;
            }
        }
        // 模式 2: 返回具体的数值
        else {
            // 1. 初始化所有值为 0.0
            // 包括所有的 x 变量和硬用户的 b 变量，它们的二阶导都是 0
            for (int i = 0; i < n_vars; i++) {
                values[i] = 0.0;
            }

            // IPOPT 计算的是拉格朗日函数的二阶导: 
            // H = obj_factor * f''(x) + sum(lambda * g''(x))
            // 因为你的所有约束 g(x) 都是线性的，g''(x) 全为 0。
            // 所以我们只需要计算 obj_factor * f''(x)。

            // 2. 只需要计算弹性用户的 b 变量部分
            // 变量布局: [ x (0 ~ mn-1) | b (mn ~ 2mn-1) ]
            // b 的布局: [ b_uav0_user0, ..., b_uav0_usern, b_uav1_user0, ... ]

            double ln2 = log(2.0);

            for (int k = 0; k < m; k++) {
                for (int j = 0; j < n2; j++) {
                    // 计算索引
                    int user_idx = n1 + j;          // 弹性用户在全局用户列表中的索引
                    int idx_b = m * n + k * n + user_idx; // 在优化变量数组 x 中的索引

                    // 获取当前带宽值 b_kj
                    double b_kj = x[idx_b];

                    // 获取参数
                    double weight = sysModel.users[user_idx].weight;
                    double capacity = elastic_cap[k][j];

                    // 常数 C = log2(1 + capacity)
                    double C = capacity;

                    // 计算二阶导数
                    // f(b) = w * log2(1 + b*C)
                    // f'(b) = w * C / (ln2 * (1 + b*C))
                    // f''(b) = -w * C^2 / (ln2 * (1 + b*C)^2)

                    double term = 1.0 + b_kj * C;
                    double hess_val = weight * C * C / (ln2 * term * term);

                    // 赋值 (切记乘以 obj_factor)
                    values[idx_b] = obj_factor * hess_val;
                }
            }
        }

        return true;
    }

    virtual void finalize_solution(SolverReturn status, Index n_vars,
        const Number* x, const Number* z_L,
        const Number* z_U, Index m_constr,
        const Number* g, const Number* lambda,
        Number obj_value, const IpoptData* ip_data,
        IpoptCalculatedQuantities* ip_cq) {
        // 保存解
        solution_obj_value = obj_value;

        // 提取x和b
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n; i++) {
                int idx_x = k * n + i;
                int idx_b = m * n + k * n + i;
                solution_x[k][i] = x[idx_x];
                solution_b[k][i] = x[idx_b];
            }
        }
    }
};

/// <summary>
/// 固定x后的优化问题：只优化 b
/// </summary>
class FixedX_NLP : public TNLP {
public:
    const SystemMd& sysModel;
    int m, n, n1, n2;
    vector<vector<double>> hard_cap;
    vector<vector<double>> elastic_cap;
    vector<vector<int>> x_fixed;  // 固定的x值

    // 保存求解结果
    vector<vector<double>> solution_b;  // m x n
    double solution_obj_value;

    FixedX_NLP(const SystemMd& sys, const vector<vector<int>>& x_fix)
        : sysModel(sys), x_fixed(x_fix) {
        m = sysModel.m;
        n1 = sysModel.n1;
        n2 = sysModel.n2;
        n = n1 + n2;

        hard_cap.resize(m, vector<double>(n1));
        elastic_cap.resize(m, vector<double>(n2));
        solution_b.resize(m, vector<double>(n, 0.0));
        solution_obj_value = 0.0;

        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                hard_cap[k][i] = sysModel.cap_list[k][i];
            }
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                elastic_cap[k][j] = sysModel.cap_list[k][user_idx];
            }
        }
    }

    virtual ~FixedX_NLP() {}

    virtual bool get_nlp_info(Index& n_vars, Index& n_constraints,
        Index& nnz_jac_g, Index& nnz_h_lag,
        IndexStyleEnum& index_style) {
        n_vars = m * n;  // 只有b变量

        // 约束:
        // 1. b <= x*B_UAV: m*n个
        // 2. sum_i b[k][i] <= B_UAV: m个
        // 3. b[k][i] * log2(1+capacity) >= x[k][i] * r_min: n1 * m个
        n_constraints = m * n + m + n1 * m;

        nnz_jac_g = m * n + m * n + n1 * m;
        // 海塞矩阵非零元素数量 = 变量数量（因为是对角阵）
        nnz_h_lag = m * n;

        index_style = TNLP::C_STYLE;
        return true;
    }

    // 添加 eval_h 实现
    virtual bool eval_h(Index n_vars, const Number* x, bool new_x,
        Number obj_factor, Index m_constr, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow,
        Index* jCol, Number* values) {

        if (values == NULL) {
            // 1. 返回稀疏结构（对角矩阵）
            // iRow[k] = k, jCol[k] = k
            for (int i = 0; i < n_vars; i++) {
                iRow[i] = i;
                jCol[i] = i;
            }
        }
        else {
            // 2. 返回数值
            // 注意：IPOPT 求解的是 min (obj_factor * f(x) + lambda * g(x))
            // 你的约束都是线性的，线性约束的二阶导是 0，所以只需要考虑目标函数 f(x)

            // 初始化为 0
            for (int i = 0; i < n_vars; i++) values[i] = 0.0;

            // 遍历弹性用户，计算二阶导
            for (int k = 0; k < m; k++) {
                for (int j = 0; j < n2; j++) {
                    int user_idx = n1 + j;
                    int idx_b = k * n + user_idx; // 变量索引

                    double b_kj = x[idx_b];
                    double weight = sysModel.users[user_idx].weight;
                    double capacity = elastic_cap[k][j];
                    double spectral_efficiency = capacity; // 常数 C

                    // 计算二阶导数: - w * C^2 / ((1 + bC)^2 * ln(2))
                    double C = spectral_efficiency;
                    double term = 1.0 + b_kj * C;
                    double ln2 = log(2.0);

                    // f''(b)
                    double hess_val = weight * C * C / (term * term * ln2);

                    // 注意乘上 obj_factor (IPOPT 要求)
                    values[idx_b] = obj_factor * hess_val;
                }
            }

            // 硬用户的部分如果是线性的或者常数，二阶导就是0，不用管
        }
        return true;
    }

    virtual bool get_bounds_info(Index n_vars, Number* x_l, Number* x_u,
        Index n_constraints, Number* g_l, Number* g_u) {
        // b的界: [0, +inf)
        for (int i = 0; i < m * n; i++) {
            x_l[i] = 0.0;
            x_u[i] = 1e20;
        }

        int constraint_idx = 0;

        // 约束1: b - x*B_UAV <= 0
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n; i++) {
                g_l[constraint_idx] = -1e20;
                g_u[constraint_idx] = 0.0;
                constraint_idx++;
            }
        }

        // 约束2: sum_i b - B_UAV <= 0
        for (int k = 0; k < m; k++) {
            g_l[constraint_idx] = -1e20;
            g_u[constraint_idx] = 0.0;
            constraint_idx++;
        }
        // 约束3: x*r_min - b*log2(1+capacity) <= 0 for hard users
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                g_l[constraint_idx] = -1e20;
                g_u[constraint_idx] = 0.0;
                constraint_idx++;
            }
        }


        return true;
    }

    virtual bool get_starting_point(Index n_vars, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m_constr, bool init_lambda, Number* lambda) {
        if (init_x) {
            for (int i = 0; i < m * n; i++) {
                x[i] = 0.1;
            }
        }
        return true;
    }

    virtual bool eval_f(Index n_vars, const Number* x, bool new_x, Number& obj_value) {
        obj_value = 0.0;
        double total_value = 0.0;
        // 硬用户贡献
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                if (x_fixed[k][i] == 1) {
                    double bw = sysModel.Bth_list[k][i];
                    double cap = sysModel.cap_list[k][i];
                    double SNR_avg_dB = sysModel.SNRave_list[k][i];
                    double weight = sysModel.users[i].weight;
                    User user = sysModel.users[i];
                    total_value += user.utility(bw, cap, SNR_avg_dB);
                }
            }
        }

        // 弹性用户贡献
        for (int k = 0; k < m; k++) {
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                int idx_b = k * n + user_idx;
                double b_kj = x[idx_b];

                double cap = sysModel.cap_list[k][user_idx];
                double SNR_avg_dB = sysModel.SNRave_list[k][user_idx];
                double weight = sysModel.users[user_idx].weight;

                User user = sysModel.users[user_idx];
                total_value += user.utility(b_kj, cap, SNR_avg_dB);
            }
        }
        obj_value = -total_value;

        return true;
    }

    virtual bool eval_grad_f(Index n_vars, const Number* x, bool new_x, Number* grad_f) {
        for (int i = 0; i < n_vars; i++) {
            grad_f[i] = 0.0;
        }

        // 弹性用户部分
        for (int k = 0; k < m; k++) {
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                int idx_b = k * n + user_idx;
                double b_kj = x[idx_b];
                double weight = sysModel.users[user_idx].weight;
                double capacity = elastic_cap[k][j];

                double spectral_efficiency = capacity;
                double rate = b_kj * spectral_efficiency;

                if (rate + 1.0 > 1e-10) {
                    grad_f[idx_b] = -1 * (weight * spectral_efficiency / ((rate + 1.0) * log(2.0)));
                }
            }
        }

        return true;
    }

    virtual bool eval_g(Index n_vars, const Number* x, bool new_x,
        Index m_constr, Number* g) {
        int constraint_idx = 0;

        // 约束1: b - x_fixed*B_UAV <= 0
        for (int k = 0; k < m; k++) {
            double B_UAV = sysModel.uavs[k].total_bandwidth;
            for (int i = 0; i < n; i++) {
                int idx_b = k * n + i;
                g[constraint_idx] = x[idx_b] - x_fixed[k][i] * B_UAV;
                constraint_idx++;
            }
        }

        // 约束2: sum_i b <= B_UAV
        for (int k = 0; k < m; k++) {
            g[constraint_idx] = 0.0;
            for (int i = 0; i < n; i++) {
                int idx_b = k * n + i;
                g[constraint_idx] += x[idx_b];
            }
            g[constraint_idx] -= sysModel.uavs[k].total_bandwidth;
            constraint_idx++;
        }

        // 约束3:  x_fixed*r_min - b*log2(1+capacity)<= 0
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                g[constraint_idx] = 0.0;
                double r_min = sysModel.users[i].rMin;
                int idx_b = k * n + i;
                double capacity = hard_cap[k][i];

                g[constraint_idx] += x_fixed[k][i] * r_min - x[idx_b] * capacity;
                constraint_idx++;

            }
        }

        return true;
    }

    virtual bool eval_jac_g(Index n_vars, const Number* x, bool new_x,
        Index m_constr, Index nele_jac, Index* iRow,
        Index* jCol, Number* values) {
        if (values == NULL) {
            int nz_idx = 0;
            int constraint_idx = 0;

            // 约束1
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = k * n + i;
                    nz_idx++;
                    constraint_idx++;
                }
            }

            // 约束2
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = k * n + i;
                    nz_idx++;
                }
                constraint_idx++;
            }

            // 约束3 修正代码
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n1; i++) {
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = k * n + i;
                    nz_idx++;
                    constraint_idx++;
                }
            }
        }
        else {
            int nz_idx = 0;
            int constraint_idx = 0;

            // 约束1
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    values[nz_idx] = 1.0;
                    nz_idx++;
                    constraint_idx++;
                }
            }

            // 约束2
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n; i++) {
                    values[nz_idx] = 1.0;
                    nz_idx++;
                }
                constraint_idx++;
            }

            // 约束3 修正代码
            for (int k = 0; k < m; k++) {
                for (int i = 0; i < n1; i++) {
                    double capacity = hard_cap[k][i];
                    values[nz_idx] = -capacity;
                    nz_idx++;
                    constraint_idx++;
                }
            }
        }

        return true;
    }

    virtual void finalize_solution(SolverReturn status, Index n_vars,
        const Number* x, const Number* z_L,
        const Number* z_U, Index m_constr,
        const Number* g, const Number* lambda,
        Number obj_value, const IpoptData* ip_data,
        IpoptCalculatedQuantities* ip_cq) {
        // 保存解
        solution_obj_value = obj_value;

        // 提取b
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n; i++) {
                int idx_b = k * n + i;
                solution_b[k][i] = x[idx_b];
            }
        }
    }
};

// ==================== BAProblem 类的新算法实现 ====================

/// <summary>
/// 基于凸松弛和舍入的多无人机带宽分配算法
/// </summary>
/// Solve a continuous relaxation, perform seeded rounding, and return the best feasible refined allocation.
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::ConvexRelaxationAndRounding_multiUAV(double epsilon_tol,
    uint32_t seed, int max_trials, AllocationDiagnostics* diagnostics) {
    if (diagnostics) *diagnostics = AllocationDiagnostics();
    if (!std::isfinite(epsilon_tol) || epsilon_tol <= 0 || max_trials <= 0)
        throw std::invalid_argument("Invalid relax-round tolerance or trial count");
    const int m = sysModel.m;
    const int n = sysModel.n1 + sysModel.n2;
    vector<KnapsackResult> best_results(m);
    for (int k = 0; k < m; ++k) best_results[k].uav_id = k;
    if (m == 0 || n == 0)
        return {best_results, construct_user_results(best_results)};

    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    if (IsNull(app))
        throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "RelaxRound: IPOPT creation failed");
    ApplicationReturnStatus status = app->Initialize(algProjPath + "relax_rounding_ipopt.opt");
    if (diagnostics) diagnostics->record("relax.initialize", static_cast<int>(status));
    if (status != Solve_Succeeded)
        throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "RelaxRound: IPOPT initialization failed");
    SmartPtr<RelaxedProblem_NLP> relaxed_nlp = new RelaxedProblem_NLP(sysModel);
    status = app->OptimizeTNLP(relaxed_nlp);
    if (diagnostics) diagnostics->record("relax.optimize", static_cast<int>(status));
    if (status != Solve_Succeeded && status != Solved_To_Acceptable_Level)
        throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "RelaxRound: continuous relaxation failed");

    std::mt19937 generator(seed);
    bool found = false;
    bool solver_failed = false;
    double best_utility = -INFINITY;
    for (int trial = 0; trial < max_trials; ++trial) {
        vector<vector<int>> association(m, vector<int>(n, 0));
        for (int i = 0; i < n; ++i) {
            const double draw = allocation_uniform01(generator);
            double probability = 0.0;
            for (int k = 0; k < m; ++k) {
                if (!relaxed_nlp->allowable_matrix[k][i]) continue;
                const double x = relaxed_nlp->solution_x[k][i];
                if (!std::isfinite(x))
                    throw AllocationFailure(AlgorithmRunStatus::SolverFailure, "Non-finite relaxed association");
                if (x < 1e-6) continue; // Preserve the original rounding cutoff.
                probability += x;
                if (draw <= probability) { association[k][i] = 1; break; }
            }
        }
        bool candidate_ok = true;
        for (int k = 0; k < m; ++k) {
            double load = 0.0;
            for (int i = 0; i < sysModel.n1; ++i)
                if (association[k][i]) load += sysModel.Bth_list[k][i];
            const double budget = sysModel.uavs[k].total_bandwidth;
            if (!std::isfinite(load) || load > budget + allocation_tolerance(load, budget))
                candidate_ok = false;
        }
        if (!candidate_ok) {
            if (diagnostics) diagnostics->events.push_back("round." + std::to_string(trial) + ": hard-load precheck rejected");
            continue;
        }

        SmartPtr<FixedX_NLP> fixed = new FixedX_NLP(sysModel, association);
        status = app->OptimizeTNLP(fixed);
        if (diagnostics) diagnostics->record("fixed." + std::to_string(trial), static_cast<int>(status));
        if (status != Solve_Succeeded && status != Solved_To_Acceptable_Level) {
            solver_failed = true;
            continue;
        }
        vector<KnapsackResult> candidate(m);
        double utility_sum = 0.0;
        // Preserve the violated quantity in diagnostics instead of hiding infeasibility in an empty result.
        auto reject = [&](const string& reason, int k, int i, double actual, double required) {
            candidate_ok = false;
            if (diagnostics) {
                ostringstream message;
                message << setprecision(17) << "fixed." << trial << ": " << reason
                    << "; UAV=" << k << "; user=" << i << "; actual=" << actual << "; required=" << required;
                diagnostics->events.push_back(message.str());
            }
        };
        for (int k = 0; k < m; ++k) {
            candidate[k].uav_id = k;
            for (int i = 0; i < n; ++i) {
                const double raw_bw = fixed->solution_b[k][i];
                if (!std::isfinite(raw_bw) || raw_bw < -ALLOCATION_ABS_TOL) {
                    reject("invalid bandwidth", k, i, raw_bw, 0); continue;
                }
                // Remove numerical zero only; positive allocations are not repaired by validation.
                const double bw = std::max(0.0, raw_bw);
                if (!association[k][i]) {
                    if (bw > ALLOCATION_ABS_TOL) reject("unassociated bandwidth", k, i, bw, 0);
                    continue;
                }
                const double cap = sysModel.cap_list[k][i];
                if (i < sysModel.n1 && !hard_qos_satisfied(bw, cap, sysModel.users[i].rMin))
                    reject("hard rate insufficient", k, i, bw * cap, sysModel.users[i].rMin);
                if (bw <= 0.0) continue;
                const double value = sysModel.users[i].utility(bw, cap, sysModel.SNRave_list[k][i]);
                if (!std::isfinite(value)) { candidate_ok = false; continue; }
                add_KnapsackResult(candidate[k], sysModel.users[i], bw, value);
                utility_sum += value;
            }
            const double budget = sysModel.uavs[k].total_bandwidth;
            if (candidate[k].totalWeight > budget + allocation_tolerance(candidate[k].totalWeight, budget))
                reject("budget exceeded", k, -1, candidate[k].totalWeight, budget);
        }
        if (!candidate_ok) {
            if (diagnostics) diagnostics->events.push_back("fixed." + std::to_string(trial) + ": physical candidate validation rejected");
            continue;
        }
        if (!found || utility_sum > best_utility) {
            found = true;
            best_utility = utility_sum;
            best_results = std::move(candidate);
        }
    }
    if (!found)
        throw AllocationFailure(solver_failed ? AlgorithmRunStatus::SolverFailure :
            AlgorithmRunStatus::NoFeasibleCandidate, "RelaxRound: no feasible candidate in " +
            std::to_string(max_trials) + " recorded trials");
    return {best_results, construct_user_results(best_results)};
}

/// Check all objective, Jacobian and Hessian entries by central differences on a tiny NLP.
/// This diagnostic performs no optimization and produces no experiment files.
template<class Problem>
static void verify_relax_nlp_derivatives(Problem& problem) {
    Index nv, nc, nj, nh;
    TNLP::IndexStyleEnum style;
    if (!problem.get_nlp_info(nv, nc, nj, nh, style))
        throw runtime_error("Cannot read NLP dimensions");
    vector<Number> x(nv, 0.37), gradient(nv), jacobian(nj), hessian(nh), lambda(nc, 0.0);
    vector<Index> jr(nj), jc(nj), hr(nh), hc(nh);
    problem.eval_grad_f(nv, x.data(), true, gradient.data());
    problem.eval_jac_g(nv, x.data(), true, nc, nj, jr.data(), jc.data(), nullptr);
    problem.eval_jac_g(nv, x.data(), true, nc, nj, nullptr, nullptr, jacobian.data());
    problem.eval_h(nv, x.data(), true, 1.0, nc, lambda.data(), true, nh, hr.data(), hc.data(), nullptr);
    problem.eval_h(nv, x.data(), true, 1.0, nc, lambda.data(), true, nh, nullptr, nullptr, hessian.data());
    vector<vector<double>> dense_jac(nc, vector<double>(nv, 0)), dense_hess(nv, vector<double>(nv, 0));
    for (Index i = 0; i < nj; ++i) dense_jac.at(jr[i]).at(jc[i]) += jacobian[i];
    for (Index i = 0; i < nh; ++i) {
        dense_hess.at(hr[i]).at(hc[i]) += hessian[i];
        if (hr[i] != hc[i]) dense_hess.at(hc[i]).at(hr[i]) += hessian[i];
    }
    // Difference quotients intentionally use only the callback values, not repeated analytic formulas.
    auto check = [](double a, double b, const string& stage) {
        if (!std::isfinite(a) || !std::isfinite(b) ||
            abs(a - b) > 1e-5 * std::max(1.0, std::max(abs(a), abs(b))))
            throw runtime_error("RelaxRound derivative mismatch: " + stage);
    };
    constexpr double step = 1e-5;
    for (Index column = 0; column < nv; ++column) {
        auto plus = x, minus = x;
        plus[column] += step; minus[column] -= step;
        Number fp, fm;
        vector<Number> gp(nc), gm(nc), dp(nv), dm(nv);
        problem.eval_f(nv, plus.data(), true, fp);
        problem.eval_f(nv, minus.data(), true, fm);
        check(gradient[column], (fp - fm) / (2 * step), "objective");
        problem.eval_g(nv, plus.data(), true, nc, gp.data());
        problem.eval_g(nv, minus.data(), true, nc, gm.data());
        problem.eval_grad_f(nv, plus.data(), true, dp.data());
        problem.eval_grad_f(nv, minus.data(), true, dm.data());
        for (Index row = 0; row < nc; ++row) check(dense_jac[row][column], (gp[row] - gm[row]) / (2 * step), "Jacobian");
        for (Index row = 0; row < nv; ++row) check(dense_hess[row][column], (dp[row] - dm[row]) / (2 * step), "Hessian");
    }
}

/// Verify both relaxation and fixed-association derivatives using the supplied synthetic model.
void verify_relax_round_derivatives(const SystemMd& model) {
    if (model.m <= 0 || model.users.empty()) throw invalid_argument("Derivative test needs a nonempty model");
    RelaxedProblem_NLP relaxed(model);
    verify_relax_nlp_derivatives(relaxed);
    vector<vector<int>> association(model.m, vector<int>(model.users.size(), 0));
    for (size_t i = 0; i < model.users.size(); ++i) association[i % model.m][i] = 1;
    FixedX_NLP fixed(model, association);
    verify_relax_nlp_derivatives(fixed);
}

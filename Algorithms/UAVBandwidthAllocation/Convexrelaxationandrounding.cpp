#include "EntityDefinition.h"


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


    // SNR线性值 (从dB转换)
    vector<vector<double>> SNR_linear_hard;   // m x n1
    vector<vector<double>> SNR_linear_elastic; // m x n2

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

        // 初始化SNR线性值
        SNR_linear_hard.resize(m, vector<double>(n1));
        SNR_linear_elastic.resize(m, vector<double>(n2));

        // 初始化解向量
        solution_x.resize(m, vector<double>(n, 0.0));
        solution_b.resize(m, vector<double>(n, 0.0));
        solution_obj_value = 0.0;
        // 【新增】: 初始化连通性矩阵
        allowable_matrix.assign(m, vector<bool>(n, false));
        // 遍历 sysModel 中的 map 填充矩阵
        // 假设 uav_serviceable_users_map 的 key 是 uav_id (int)
        for (auto const& [uav_id, users_vec] : sysModel.uav_serviceable_users_map) {
            if (uav_id >= m) continue; // 安全检查
            for (const auto& user : users_vec) {
                // 注意：这里假设 user.id 对应于算法中 0 到 n-1 的索引
                // 如果你的 User 结构体没有 id，或者索引逻辑不同，请在此处修改
                int user_idx = user.ID;
                if (user_idx >= 0 && user_idx < n) {
                    allowable_matrix[uav_id][user_idx] = true;
                }
            }
        }

        // 从dB转换为线性值
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                SNR_linear_hard[k][i] = pow(10.0, sysModel.SNRth_list[k][i] / 10.0);

            }
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                SNR_linear_elastic[k][j] = pow(10.0, sysModel.SNRave_list[k][user_idx] / 10.0);
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

        // 约束4: x[k][i] * r_min - b[k][i] * log2(1 + SNR) <= 0 for hard users
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
                double SNR = SNR_linear_elastic[k][j];

                double log2_SNR = log2(1.0 + SNR);
                double rate = b_kj * log2_SNR;

                // d/db[w * log2(rate + 1)] = w * log2_SNR / ((rate + 1) * ln(2))
                if (rate + 1.0 > 1e-4) {
                    grad_f[idx_b] = -1 * weight * log2_SNR / ((rate + 1.0) * log(2.0));
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

        // 约束4: x[k][i] * r_min - b[k][i] * log2(1 + SNR) <= 0
        for (int k = 0; k < m; k++)
        {
            for (int i = 0; i < n1; i++) {

                double r_min = sysModel.users[i].rMin;
                g[constraint_idx] = 0.0;
                int idx_x = k * n + i;
                int idx_b = m * n + k * n + i;
                double SNR = SNR_linear_hard[k][i];

                g[constraint_idx] = (x[idx_x] * r_min - x[idx_b] * log2(1.0 + SNR));

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

            // 约束4: x[k][i] * r_min - b[k][i] * log2(1 + SNR) <= 0
            // 硬用户部分
            for (int k = 0; k < m; k++)
            {
                for (int i = 0; i < n1; i++) {
                    // x[k][i] * r_min项
                    iRow[nz_idx] = constraint_idx;
                    jCol[nz_idx] = k * n + i;
                    nz_idx++;
                    // -b[k][i] * log2(1 + SNR)项
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
                    double SNR = SNR_linear_hard[k][i];
                    values[nz_idx] = -log2(1.0 + SNR);  // -b*log2(1 + SNR)项系数
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
                    double SNR_linear = SNR_linear_elastic[k][j];

                    // 常数 C = log2(1 + SNR)
                    double C = log2(1.0 + SNR_linear);

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
    vector<vector<double>> SNR_linear_hard;
    vector<vector<double>> SNR_linear_elastic;
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

        SNR_linear_hard.resize(m, vector<double>(n1));
        SNR_linear_elastic.resize(m, vector<double>(n2));
        solution_b.resize(m, vector<double>(n, 0.0));
        solution_obj_value = 0.0;

        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                SNR_linear_hard[k][i] = pow(10.0, sysModel.SNRth_list[k][i] / 10.0);
            }
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                SNR_linear_elastic[k][j] = pow(10.0, sysModel.SNRave_list[k][user_idx] / 10.0);
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
        // 3. b[k][i] * log2(1+SNR) >= x[k][i] * r_min: n1 * m个
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
                    double SNR = SNR_linear_elastic[k][j];
                    double log2_SNR = log2(1.0 + SNR); // 常数 C

                    // 计算二阶导数: - w * C^2 / ((1 + bC)^2 * ln(2))
                    double C = log2_SNR;
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
        // 约束3: x*r_min - b*log2(1+SNR) <= 0 for hard users
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
                double SNR = SNR_linear_elastic[k][j];

                double log2_SNR = log2(1.0 + SNR);
                double rate = b_kj * log2_SNR;

                if (rate + 1.0 > 1e-10) {
                    grad_f[idx_b] = -1 * (weight * log2_SNR / ((rate + 1.0) * log(2.0)));
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

        // 约束3:  x_fixed*r_min - b*log2(1+SNR)<= 0
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                g[constraint_idx] = 0.0;
                double r_min = sysModel.users[i].rMin;
                int idx_b = k * n + i;
                double SNR = SNR_linear_hard[k][i];

                g[constraint_idx] += x_fixed[k][i] * r_min - x[idx_b] * log2(1.0 + SNR);
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
                    double SNR = SNR_linear_hard[k][i];
                    values[nz_idx] = -log2(1.0 + SNR);
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
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::ConvexRelaxationAndRounding_multiUAV(double epsilon_tol) {

    // cout << "========================================" << endl;
    // cout << "开始运行 ConvexRelaxationAndRounding 算法" << endl;
    // cout << "========================================" << endl;

    int m = sysModel.m;
    int n = sysModel.n1 + sysModel.n2;

    // cout << "系统规模: " << m << " UAVs, " << n << " users ("
    //     << sysModel.n1 << " hard, " << sysModel.n2 << " elastic)" << endl;

    // 步骤1 & 2 已在NLP类中定义

    // 步骤3: 求解松弛问题
    //cout << "\n步骤3: 创建IPOPT求解器..." << endl;

    SmartPtr<IpoptApplication> app;
    try {
        app = IpoptApplicationFactory();
    }
    catch (const std::exception& e) {
        cout << "错误: 创建IPOPT应用时发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult >());
    }
    catch (...) {
        cout << "错误: 创建IPOPT应用时发生未知异常" << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    // 检查app是否成功创建
    if (IsNull(app)) {
        cout << "错误: IpoptApplicationFactory() 返回空指针!" << endl;
        cout << "可能的原因:" << endl;
        cout << "  1. IPOPT库未正确安装" << endl;
        cout << "  2. IPOPT DLL文件缺失或路径不正确" << endl;
        cout << "  3. 链接器配置问题" << endl;
        cout << "\n请检查:" << endl;
        cout << "  - IPOPT库是否已安装" << endl;
        cout << "  - PATH环境变量是否包含IPOPT的bin目录" << endl;
        cout << "  - 项目配置是否正确链接了IPOPT库" << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    //cout << "IPOPT应用创建成功" << endl;

    // 先初始化IPOPT
    //cout << "初始化IPOPT..." << endl;
    ApplicationReturnStatus status;
    try {
        status = app->Initialize("relax_rounding_ipopt.opt");
    }
    catch (const std::exception& e) {
        cout << "错误: IPOPT初始化时发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    if (status != Solve_Succeeded) {
        cout << "IPOPT初始化失败! 状态码: " << status << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    //cout << "IPOPT初始化成功" << endl;

    // === 调试代码 ===
    if (IsNull(app)) {
        cout << "致命错误: app 指针突然变为空!" << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }
    else {
        //cout << "调试: app 指针地址有效: " << GetRawPtr(app) << endl;
    }

    // 尝试获取 Options 指针看看是否为空
    SmartPtr<OptionsList> opts = app->Options();
    if (IsNull(opts)) {
        cout << "致命错误: app->Options() 返回了空指针! 这通常意味着 Debug/Release 库不匹配。" << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }
    else {
        //cout << "调试: Options 指针获取成功." << endl;
    }
    // ===============



    // 创建松弛问题
    // cout << "创建松弛问题..." << endl;
    SmartPtr<TNLP> relaxed_nlp;
    try {
        relaxed_nlp = new RelaxedProblem_NLP(sysModel);
    }
    catch (const std::exception& e) {
        cout << "错误: 创建NLP问题时发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    if (IsNull(relaxed_nlp)) {
        cout << "错误: 创建NLP问题失败!" << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    //cout << "松弛问题创建成功" << endl;

    // 求解
    //cout << "开始求解松弛问题..." << endl;
    try {
        status = app->OptimizeTNLP(relaxed_nlp);
    }
    catch (const std::exception& e) {
        cout << "错误: 求解过程中发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    if (status != Solve_Succeeded && status != Solved_To_Acceptable_Level) {
        cout << "松弛问题求解失败!" << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    // 获取松弛解
    RelaxedProblem_NLP* nlp_ptr = dynamic_cast<RelaxedProblem_NLP*>(GetRawPtr(relaxed_nlp));
    vector<vector<double>> x_relaxed = nlp_ptr->solution_x;
    vector<vector<double>> b_relaxed = nlp_ptr->solution_b;

    // 输出前20个用户相关变量
    /*cout << "松弛解 (前20个用户):" << endl;
    for (int k = 0; k < m; k++) {
        cout << "UAV " << k << " 分配的 x 值: ";
        for (int i = sysModel.n1; i < std::min(sysModel.n1 + 100, n); i++) {
            cout << x_relaxed[k][i] << " ";
        }
        cout << endl;
        cout << "UAV " << k << " 分配的 b 值: ";
        for (int i = sysModel.n1; i < std::min(sysModel.n1 + 100, n); i++) {
            cout << b_relaxed[k][i] << " ";
        }
        cout << endl;
    }*/

    // cout << "松弛问题求解完成，目标函数值: " << nlp_ptr->solution_obj_value << endl;

    // 步骤4: 二分查找舍入阈值
    //cout << "\n步骤4: 开始随机舍入与二次优化..." << endl;

    int max_trials = 2; // 最大尝试次数
    double best_valid_obj_value = -1e20; // 记录找到的最大效用值
    bool found_any_feasible = false;

    // 保存全局最佳方案
    vector<vector<int>> global_best_x;
    vector<vector<double>> global_best_b;

    // 随机数生成器初始化
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    for (int trial = 0; trial < max_trials; trial++) {
        // 4.1 概率舍入 (保证每个用户最多连一个UAV)
        vector<vector<int>> current_x(m, vector<int>(n, 0));

        for (int i = 0; i < n; i++) {
            double rand_val = dis(gen);
            double cum_prob = 0.0;
            bool assigned = false;

            for (int k = 0; k < m; k++) {
                // 你需要在这里也能访问到类似的判断逻辑，或者直接信任 x_relaxed 为 0
                if (x_relaxed[k][i] < 1e-6) continue;
                cum_prob += x_relaxed[k][i];
                if (rand_val <= cum_prob) {
                    current_x[k][i] = 1; // 选中 UAV k
                    assigned = true;
                    break; // 跳出，确保单连接
                }
            }
            // 如果 rand_val > sum(x_relaxed), 则该用户不连接任何 UAV (x全为0)
        }

        // 4.2 可行性预检 (Pre-check) - 约束(5)
        bool is_trial_feasible = true;

        for (int k = 0; k < m; k++) {
            double current_uav_load = 0.0;
            for (int i = 0; i < sysModel.n1; i++) { // 只遍历硬用户
                if (current_x[k][i] == 1) {
                    double snr = pow(10.0, sysModel.SNRth_list[k][i] / 10.0);
                    double min_bw = sysModel.Bth_list[k][i];
                    current_uav_load += min_bw;
                }
            }

            // 如果硬用户的最小需求总和超过了 UAV 容量
            if (current_uav_load > sysModel.uavs[k].total_bandwidth) {
                is_trial_feasible = false;
                break; // 该分配方案无效，无需继续检查
            }
        }

        if (!is_trial_feasible) {
            // cout << "Trial " << trial << ": 预检失败 (带宽不足以满足硬用户需求)，跳过。" << endl;
            continue;
        }

        // 4.3 二次优化 (FixedX_NLP)
        // 只有预检通过才进行昂贵的 NLP 求解
        SmartPtr<TNLP> fixed_nlp = new FixedX_NLP(sysModel, current_x);
        // 注意：这里需要重新创建一个新的 app 实例或者 re-optimize，为简单起见建议复用 app 但需小心状态
        // 建议：在循环外创建 app，这里直接 OptimizeTNLP

        // *关键*: 重置上次求解的状态 (如果需要) 或者忽略错误继续
        status = app->OptimizeTNLP(fixed_nlp);

        if (status == Solve_Succeeded || status == Solved_To_Acceptable_Level) {
            FixedX_NLP* fixed_ptr = dynamic_cast<FixedX_NLP*>(GetRawPtr(fixed_nlp));
            double current_obj = fixed_ptr->solution_obj_value; // 注意：这是负值 (min -utility)

            // 注意：我们原本是 max utility，IPOPT 是 min -utility
            // 所以 solution_obj_value 越小越好

            // 为了方便比较，我们将 IPOPT 的负目标值转回正的效用值
            double current_utility = -current_obj;

            // 记录第一次可行解，或者更新更好的解
            if (!found_any_feasible || current_utility > best_valid_obj_value) {
                best_valid_obj_value = current_utility;
                global_best_x = current_x;
                global_best_b = fixed_ptr->solution_b;
                found_any_feasible = true;

                // cout << "Trial " << trial << ": 找到更优可行解! Utility = " << current_utility << endl;
            }
        }
    }

    // 步骤5: 结果构造 (如果没找到任何解，返回空)
    if (!found_any_feasible) {
        cout << "错误: 在 " << max_trials << " 次尝试后未找到满足硬约束的可行解。" << endl;
        return make_pair(vector<KnapsackResult>(), map<int, UserResult>());
    }

    // 构造返回对象 (使用 global_best_x 和 global_best_b)
    vector<KnapsackResult> uavResults(m);

    for (int k = 0; k < m; k++) {
        for (int i = 0; i < n; i++) {
            if (global_best_x[k][i] == 1) {
                uavResults[k].uav_id = sysModel.uavs[k].ID;
                uavResults[k].allocatedList.push_back(i);
                double bw = global_best_b[k][i];

                uavResults[k].allocatedBandwidth[i] = bw;
                double SNR_avg_dB = sysModel.SNRave_list[k][i];
                double cap = sysModel.cap_list[k][i];
                // 重新计算精确效用值用于统计
                double utility = 0.0;

                if (i < sysModel.n1) {
                    double bw_th = sysModel.Bth_list[k][i];
                    utility = sysModel.users[i].utility(bw_th, cap, SNR_avg_dB);
                    // cout << "user " << i << ", bw = " << bw << ", bw_th = " << bw_th << endl;
                    uavResults[k].allocatedValue[i] = utility;
                    uavResults[k].hardValue += utility;
                    uavResults[k].hardWeight += bw;
                }
                else {
                    utility = sysModel.users[i].utility(bw, cap, SNR_avg_dB);
                    uavResults[k].allocatedValue[i] = utility;
                    uavResults[k].elasticValue += utility;
                    uavResults[k].elasticWeight += bw;
                }

                uavResults[k].totalValue += utility;
                uavResults[k].totalWeight += bw;

            }
        }
    }

    map<int, UserResult> userResults = construct_user_results(uavResults);

    //cout << "算法结束，最优效用: " << best_valid_obj_value << endl;
    return make_pair(uavResults, userResults);
}
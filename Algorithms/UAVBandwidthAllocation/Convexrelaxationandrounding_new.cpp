#include "EntityDefinition.h"


// ==================== IPOPT NLP 问题定义 ====================

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

        // 从dB转换为线性值
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                SNR_linear_hard[k][i] = pow(10.0, sysModel.SNRt_list[k][i] / 10.0);
            }
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                SNR_linear_elastic[k][j] = pow(10.0, sysModel.SNRa_list[k][user_idx] / 10.0);
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
        for (int i = 0; i < m * n; i++) {
            x_l[i] = 0.0;
            x_u[i] = 1.0;
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
        for(int k=0; k<m; k++) {
            for(int i=0; i<n1; i++) {
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
            // 初始化x为均匀分配
            for (int i = 0; i < m * n; i++) {
                x[i] = 1.0 / m;  // 每个用户平均连接到所有UAV
            }
            // 初始化b为小值
            for (int i = m * n; i < 2 * m * n; i++) {
                x[i] = 0.1;
            }
        }
        return true;
    }

    // 目标函数
    virtual bool eval_f(Index n_vars, const Number* x, bool new_x,
        Number& obj_value) {
        obj_value = 0.0;

        // x变量: x[k*n + i] 表示 x[k][i]
        // b变量: x[m*n + k*n + i] 表示 b[k][i]

        // 硬用户贡献
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                int idx_x = k * n + i;
                double x_ki = x[idx_x];
                double weight = sysModel.users[i].weight;
                double r_min = sysModel.users[i].rMin;

                obj_value += x_ki * weight * log2(r_min + 1.0);
            }
        }

        // 弹性用户贡献
        for (int k = 0; k < m; k++) {
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                int idx_b = m * n + k * n + user_idx;
                double b_kj = x[idx_b];
                double weight = sysModel.users[user_idx].weight;
                double SNR = SNR_linear_elastic[k][j];

                double rate = b_kj * log2(1.0 + SNR);
                obj_value += weight * log2(rate + 1.0);
            }
        }

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
                double weight = sysModel.users[i].weight;
                double r_min = sysModel.users[i].rMin;

                grad_f[idx_x] = weight * log2(r_min + 1.0);
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
                if (rate + 1.0 > 1e-10) {
                    grad_f[idx_b] = weight * log2_SNR / ((rate + 1.0) * log(2.0));
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

                g[constraint_idx] += x[idx_x] * r_min - x[idx_b] * log2(1.0 + SNR);

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
                }
				constraint_idx++;
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
                }
				constraint_idx++;
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
                    double hess_val = -weight * C * C / (ln2 * term * term);

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
                SNR_linear_hard[k][i] = pow(10.0, sysModel.SNRt_list[k][i] / 10.0);
            }
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                SNR_linear_elastic[k][j] = pow(10.0, sysModel.SNRa_list[k][user_idx] / 10.0);
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
                    double hess_val = -weight * C * C / (term * term * ln2);

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

        // 硬用户贡献
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n1; i++) {
                if (x_fixed[k][i] == 1) {
                    double weight = sysModel.users[i].weight;
                    double r_min = sysModel.users[i].rMin;
                    obj_value += weight * log2(r_min + 1.0);
                }
            }
        }

        // 弹性用户贡献
        for (int k = 0; k < m; k++) {
            for (int j = 0; j < n2; j++) {
                int user_idx = n1 + j;
                int idx_b = k * n + user_idx;
                double b_kj = x[idx_b];
                double weight = sysModel.users[user_idx].weight;
                double SNR = SNR_linear_elastic[k][j];

                double rate = b_kj * log2(1.0 + SNR);
                obj_value += weight * log2(rate + 1.0);
            }
        }

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
                    grad_f[idx_b] = weight * log2_SNR / ((rate + 1.0) * log(2.0));
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
            for (int i = 0; i < n; i++) {
                g[constraint_idx] = 0.0;
                double r_min = sysModel.users[i].rMin;
                int idx_b = k * n + i;
                double SNR = SNR_linear_hard[k][i];

                g[constraint_idx] += x[idx_b] * log2(1.0 + SNR);
                g[constraint_idx] -= x_fixed[k][i] * r_min;
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
                    values[nz_idx] = log2(1.0 + SNR);
                    nz_idx++;
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
std::pair<std::vector<KnapsackResult>, std::vector<UserResult>>
BAProblem::ConvexRelaxationAndRounding_multiUAV(double epsilon_tol) {

    cout << "========================================" << endl;
    cout << "开始运行 ConvexRelaxationAndRounding 算法" << endl;
    cout << "========================================" << endl;

    int m = sysModel.m;
    int n = sysModel.n1 + sysModel.n2;

    cout << "系统规模: " << m << " UAVs, " << n << " users ("
        << sysModel.n1 << " hard, " << sysModel.n2 << " elastic)" << endl;

    // 步骤1 & 2 已在NLP类中定义

    // 步骤3: 求解松弛问题
    cout << "\n步骤3: 创建IPOPT求解器..." << endl;

    SmartPtr<IpoptApplication> app;
    try {
        app = IpoptApplicationFactory();
    }
    catch (const std::exception& e) {
        cout << "错误: 创建IPOPT应用时发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }
    catch (...) {
        cout << "错误: 创建IPOPT应用时发生未知异常" << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
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
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    cout << "IPOPT应用创建成功" << endl;

    // 先初始化IPOPT
    cout << "初始化IPOPT..." << endl;
    ApplicationReturnStatus status;
    try {
        status = app->Initialize();
    }
    catch (const std::exception& e) {
        cout << "错误: IPOPT初始化时发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    if (status != Solve_Succeeded) {
        cout << "IPOPT初始化失败! 状态码: " << status << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    cout << "IPOPT初始化成功" << endl;

    // === 调试代码 ===
    if (IsNull(app)) {
        cout << "致命错误: app 指针突然变为空!" << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }
    else {
        cout << "调试: app 指针地址有效: " << GetRawPtr(app) << endl;
    }

    // 尝试获取 Options 指针看看是否为空
    SmartPtr<OptionsList> opts = app->Options();
    if (IsNull(opts)) {
        cout << "致命错误: app->Options() 返回了空指针! 这通常意味着 Debug/Release 库不匹配。" << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }
    else {
        cout << "调试: Options 指针获取成功." << endl;
    }
    // ===============

    // 设置IPOPT选项
    // ... 继续你的代码
    // 设置IPOPT选项
    cout << "设置IPOPT参数..." << endl;
    try {
        // ✅ 修正 1: 不要重新声明 app，直接使用上面已经创建好的 app 指针
        // ✅ 修正 2: 显式设置 hessian_approximation 为 limited-memory
        // app->Options()->SetStringValue("hessian_approximation", "limited-memory");

        // 其他设置 (可选，为了保险起见建议加上)
        /*app->Options()->SetNumericValue("tol", 1e-7);
        app->Options()->SetStringValue("mu_strategy", "adaptive");
        app->Options()->SetIntegerValue("print_level", 3);
        app->Options()->SetIntegerValue("max_iter", 3000);*/

        cout << "参数设置完成" << endl;
    }
    catch (const std::exception& e) {
        cout << "警告: 设置IPOPT参数时发生异常: " << e.what() << endl;
        cout << "将使用默认参数继续..." << endl;
    }

    // 创建松弛问题
    cout << "创建松弛问题..." << endl;
    SmartPtr<TNLP> relaxed_nlp;
    try {
        relaxed_nlp = new RelaxedProblem_NLP(sysModel);
    }
    catch (const std::exception& e) {
        cout << "错误: 创建NLP问题时发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    if (IsNull(relaxed_nlp)) {
        cout << "错误: 创建NLP问题失败!" << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    cout << "松弛问题创建成功" << endl;

    // 求解
    cout << "开始求解松弛问题..." << endl;
    try {
        status = app->OptimizeTNLP(relaxed_nlp);
    }
    catch (const std::exception& e) {
        cout << "错误: 求解过程中发生异常: " << e.what() << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    if (status != Solve_Succeeded && status != Solved_To_Acceptable_Level) {
        cout << "松弛问题求解失败!" << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    // 获取松弛解
    RelaxedProblem_NLP* nlp_ptr = dynamic_cast<RelaxedProblem_NLP*>(GetRawPtr(relaxed_nlp));
    vector<vector<double>> x_relaxed = nlp_ptr->solution_x;
    vector<vector<double>> b_relaxed = nlp_ptr->solution_b;

    cout << "松弛问题求解完成，目标函数值: " << nlp_ptr->solution_obj_value << endl;

    // 步骤4: 二分查找舍入阈值
    double theta_low = 0.0;
    double theta_high = 1.0;

    vector<vector<int>> best_x_rounded;
    vector<vector<double>> best_b_allocated;
    bool found_feasible = false;

    while (theta_high - theta_low > epsilon_tol) {
        double theta_mid = (theta_low + theta_high) / 2.0;

        cout << "尝试阈值 theta = " << theta_mid << " [" << theta_low << ", " << theta_high << "]" << endl;

        // 根据阈值舍入x
        vector<vector<int>> x_rounded(m, vector<int>(n, 0));
        for (int k = 0; k < m; k++) {
            for (int i = 0; i < n; i++) {
                x_rounded[k][i] = (x_relaxed[k][i] >= theta_mid) ? 1 : 0;
            }
        }

        // 固定x，重新优化b
        SmartPtr<FixedX_NLP> fixed_x_nlp = new FixedX_NLP(sysModel, x_rounded);
        SmartPtr<IpoptApplication> app2 = IpoptApplicationFactory();
        app2->Initialize();
        ApplicationReturnStatus status2 = app2->OptimizeTNLP(GetRawPtr(fixed_x_nlp));

        // 检查是否可行
        if (status2 == Solve_Succeeded || status2 == Solved_To_Acceptable_Level) {
            // 可行解，尝试更低的阈值
            found_feasible = true;
            best_x_rounded = x_rounded;
            best_b_allocated = fixed_x_nlp->solution_b;

            cout << "  -> 可行，目标值: " << fixed_x_nlp->solution_obj_value << endl;

            theta_high = theta_mid;
        }
        else {
            // 不可行，提高阈值
            cout << "  -> 不可行" << endl;
            theta_low = theta_mid;
        }
    }

    // 如果没有找到可行解，返回空结果
    if (!found_feasible) {
        cout << "未找到可行的舍入解!" << endl;
        return make_pair(vector<KnapsackResult>(), vector<UserResult>());
    }

    cout << "找到最优阈值: " << theta_high << endl;

    // 步骤5: 构造返回结果
    vector<KnapsackResult> uavResults(m);
    vector<UserResult> userResults(n);

    for (int k = 0; k < m; k++) {
        for (int i = 0; i < n; i++) {
            if (best_x_rounded[k][i] == 1) {
                uavResults[k].allocatedList.push_back(i);
                uavResults[k].allocatedBandwidth[i] = best_b_allocated[k][i];

                // 计算效用
                double utility = 0.0;
                if (i < sysModel.n1) {
                    // 硬用户
                    utility = sysModel.users[i].weight * log2(sysModel.users[i].rMin + 1.0);
                    uavResults[k].hardValue += utility;
                }
                else {
                    // 弹性用户
                    int j = i - sysModel.n1;
                    double SNR = pow(10.0, sysModel.SNRa_list[k][i] / 10.0);
                    double rate = best_b_allocated[k][i] * log2(1.0 + SNR);
                    utility = sysModel.users[i].weight * log2(rate + 1.0);
                    uavResults[k].elasticValue += utility;
                }

                uavResults[k].allocatedValue[i] = utility;
                uavResults[k].totalValue += utility;
                uavResults[k].totalWeight += best_b_allocated[k][i];

                // 用户结果
                userResults[i].uav_id = k;
                userResults[i].allocated_bandwidth = best_b_allocated[k][i];
                userResults[i].utility = utility;
            }
        }
    }

    return make_pair(uavResults, userResults);
}
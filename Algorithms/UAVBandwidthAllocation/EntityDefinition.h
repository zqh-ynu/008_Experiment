#pragma once
// 本文件声明 UAV 带宽分配系统模型、结果结构以及各类算法接口。
#include "predefine.h"
#include "allocation_contract.h"
#include "ton_quality_diagnostics.h"


/// @brief AlgSA-DD 的连续近似、对偶带宽求解及现有后处理参数；不保证激活新的关联。
struct SADAConfig {
	// Sigmoid参数
	double m_init = 5.0;     // Sigmoid初始陡峭因子
	double m_scale = 1.5;     // 每轮外层迭代的放大系数
	double m_max = 100.0;   // Sigmoid陡峭因子上限

	// 外层循环参数
	int    max_outer_iter = 30;    // 最大外层迭代次数
	double theta_tol = 1e-8;  // theta收敛阈值

	// 内层对偶分解参数
	int    max_inner_iter = 200;   // 最大内层迭代次数
	double alpha_init = 0.1;   // 对偶变量初始步长
	double mu_init = 1.0;   // 对偶变量初始值
	double dual_tol = 1e-8;  // 对偶收敛阈值

	// 二分法参数 (用于求解UE用户KKT方程)
	int    bisect_max_iter = 5000;
	double bisect_tol = 1e-8;

	bool   verbose = true;
};
struct MatchingSQPConfig {
	double nu_init = 5.0;    // Sigmoid 初始陡峭因子
	double nu_step = 5.0;    // 每轮递增
	double nu_max = 50.0;   // 上限
	double conv_eps = 1e-4;   // 外层收敛阈值
	int    max_outer_iter = 1;    // 最大外层迭代次数
	int    max_ipopt_iter = 200;   // IPOPT 最大迭代
	double ipopt_tol = 1e-6;  // IPOPT 收敛容差
};

class Point;
class User;
class Uav;
class Channel;
class SystemMd;     // 系统模型
class BAProblem;    // 带宽分配问题类
#define AllocationResult map<double, KnapsackResult> // 带宽分配结果，key为总带宽，value为对应的分配结果
#define UtiFunc std::function<double(double, double)> // 用户效用函数类型




/// <summary>
/// 包含了背包问题的分配结果，包括选中的物品列表及其对应的分配值，总价值，总重量，连续部分的总价值和总重量，离散部分的总价值和总重量。
/// </summary>
struct KnapsackResult {
	int uav_id = -1; // 所属的UAV ID
	vector<int> allocatedList; // 选中的物品ID列表
	map<int, double> allocatedBandwidth; // 选中的物品对应的分配值（带宽）
	map<int, double> allocatedValue; // 选中的物品对应的价值
	double totalValue = 0;         // 总价值
	double totalWeight = 0;        // 总重量
	double elasticValue = 0;         // 仅连续部分的总价值
	double hardValue = 0;         // 仅离散部分的总价值
	double elasticWeight = 0;        // 仅连续部分的总重量
	double hardWeight = 0;        // 仅离散部分的总重量
};
struct UserResult {
	int uav_id = -1;
	double allocated_bandwidth = 0; // 分配的带宽
	double utility = 0; // 用户效用
};

/// <summary>
/// 该结构体是在函数BAProblem::WaterFillingAlgorithm_singleUAV和BAProblem::FPTAS_singleUAV中使用的。这两个函数都涉及到KKT条件和注水算法。
/// </summary>
struct KKT_parameters
{
	int user_id = -1;		// 用户下标
	double W_sum = 0.0;		// 当前下标之前所有elastic用户的权重之和
	double C_inv_sum = 0.0;	// 当前下标之前所有elastic用户的信道容量倒数之和
	double efficient = 0.0;	// 当前elastic用户在未分配带宽时(或分配全部带宽时）的边际效用
	double B_elastic_sum = 0.0; // 当前下标之前所有elastic用户的分配带宽之和
	double B_hard_sum = 0.0;	// 当前下标之前所有hard用户的最小带宽需求之和
};

void clean_KnapsackResult(KnapsackResult& result);
void add_KnapsackResult(KnapsackResult& result, User& user, double bandwidth, double value);

/// Count guard-inclusive slots from total/effective bandwidth in internal units; invalid/overflow throws.
int hard_first_slot_count(double total_bandwidth, double effective_bandwidth);
/// Minimum positive slots meeting reliable QoS, or zero when no bundle fits max_slots.
int hard_first_min_slots(int max_slots, double width, double capacity, double minimum_rate);
/// Independently check integer effective slots, guard budgets and minimal hard bundles; never repair.
void validate_hard_first_allocation(const SystemMd& model, const std::vector<KnapsackResult>& results);

/// Verify the relaxation/fixed-association objective, Jacobian and Hessian against finite differences.
void verify_relax_round_derivatives(const SystemMd& model);


// 删除 vector 中第一个等于 value 的元素，找到返回 true，否则返回 false
template<typename T>
bool remove_first(std::vector<T>& vec, const T& value) {
	auto it = std::find(vec.begin(), vec.end(), value);
	if (it == vec.end()) return false;
	vec.erase(it);
	return true;
}

// 删除 vector 中所有等于 value 的元素，返回删除的数量
template<typename T>
size_t remove_all(std::vector<T>& vec, const T& value) {
	auto new_end = std::remove(vec.begin(), vec.end(), value);
	size_t removed = std::distance(new_end, vec.end());
	vec.erase(new_end, vec.end());
	return removed;
}

// 按谓词删除（更通用），返回删除的数量
template<typename T, typename Pred>
size_t remove_if_pred(std::vector<T>& vec, Pred pred) {
	auto new_end = std::remove_if(vec.begin(), vec.end(), pred);
	size_t removed = std::distance(new_end, vec.end());
	vec.erase(new_end, vec.end());
	return removed;
}

/* 使用示例：
   vector<int> v = {1,2,3,2,4};
   remove_first(v, 2);        // v -> {1,3,2,4}
   remove_all(v, 2);          // v -> {1,3,4}
   remove_if_pred(v, [](int x){ return x % 2 == 0; }); // 删除偶数
*/

/* 注意事项：
 - 使用 erase-remove 惯用法（remove_all/remove_if_pred）是高效且安全的方式。
 - 如果 vector 元素不是可用 operator== 比较的类型，提供相应的谓词。
 - 若要同时从与之配套的 map/数组 中移除对应关联数据，需要先保存要删除的 key 列表，再统一删除，避免在遍历时修改容器导致未定义行为。
*/









class Point {
public:
	double X = 0;	// X坐标
	double Y = 0;	// Y坐标
	double Z = 0;

	int ID = -1;

	Point(double x, double y, double z);
	Point(int id, double x, double y, double z);
	Point() {}
	~Point() {}

	///<summary>
	/// 计算点s与点t的直线距离
	/// 备注：静态函数，通过类名调用
	/// </summary>
	/// <param name="s">The s.</param>
	/// <param name="t">The t.</param>
	/// <returns>点s与点t的直线距离</returns>
	static double cal_distance(const Point& s, const Point& t);
	/// <summary>
	/// 计算点s与点t的水平距离
	/// 备注：静态函数，通过类名调用
	/// </summary>
	/// <param name="s">The s.</param>
	/// <param name="t">The t.</param>
	/// <returns>点s与点t的水平距离</returns>
	static double cal_horizontal_distance(const Point& s, const Point& t);
	void set_location(double lx, double ly, double lz) { X = lx; Y = ly; Z = lz; }
	const int get_index() { return ID; }
	bool operator<(const Point& p) const;

	static void print_point();
};

class User : public Point {
public:
	int uType = HARD_UTILITY; // 用户效用类型，默认硬效用
	double weight = 1; // 用户权重，默认1
	double rData = 0;		// 需求的最小数据量
	double rMin = 0;		// 需求的最小数据速率 Mbps
	double pOut = 0;		// 需求的最大中断概率

	double B = 0.18; // 180 KHz
	double BSub = 0.18;   // 子载波带宽 180 KHz
	// double B = 20e6;    // 20 MHz

	User() {}

	User(int id_, int uType_, double weight_, double x1, double y1, double z1, double rD_, double rM_, double pO_) : Point(id_, x1, y1, z1)
	{
		uType = uType_;
		weight = weight_;
		set_communication_requirements(rD_, rM_, pO_);
		//set_Bandwidth();
	}

	User(int id_, int uType_, double weight_, double x1, double y1, double z1, double r_min_, double P_o_) : Point(id_, x1, y1, z1)
	{
		uType = uType_;
		weight = weight_;
		set_communication_requirements(r_min_, P_o_);
		//set_Bandwidth();
	}

	~User() {}

	void set_communication_requirements(double r_d_, double r_min_, double P_o_) { rData = r_d_; rMin = r_min_; pOut = P_o_; }
	void set_communication_requirements(double r_min_, double P_o_) { rMin = r_min_; pOut = P_o_; }
	void set_Bandwidth();

	/// <summary>
	/// 硬效用函数, 用户的效用函数是一个阶跃函数，当分配给用户的带宽和信道容量的乘积大于等于用户的最小速率需求时，用户的效用为其权重，否则为0。
	/// </summary>
	/// <param name="bandwidth_">用户被分配的带宽.</param>
	/// <param name="capacity_">用户与发送者之间的信道容量.</param>
	/// <returns>效用值</returns>
	double hard_utility(double bandwidth_, double capacity_, double SNR_avg_dB) const;

	double hard_utility_SNR_avg(double bandwidth_, double SNR_avg_dB) const;

	/// <summary>
	/// 以中断概率为自变量的硬效用函数, 用户的效用函数是一个阶跃函数，当用户的中断概率小于等于其最大中断概率时，用户的效用为其权重，否则为0。
	/// </summary>
	/// <param name="outage_">中断概率.</param>
	/// <returns>效用</returns>
	double hard_utility(double outage_) const;

	/// <summary>
	/// 弹性效用函数, 用户的效用函数是一个凹函数,
	/// weight * log(1 + r) / log(1 + rMin);
	/// </summary>
	/// <param name="bandwidth_">用户被分配的带宽.</param>
	/// <param name="capacity_">用户与发送者之间的信道容量.</param>
	/// <returns>效用值</returns>
	double elastic_utility(double bandwidth_, double capacity_) const;
	/// <summary>
	/// 右半边软效用函数, 用户的效用函数是一个右半边的软阶跃函数,
	/// 备注：当r 小于 rMin时，效用为0；当r 大于等于 rMin时，
	/// 具体定义参见tanUtilityMaximizationResource2015
	/// </summary>
	/// <param name="bandwidth_">用户被分配的带宽.</param>
	/// <param name="capacity_r">用户与发送者之间的信道容量.</param>
	/// <returns>效用值</returns>
	double halfsoft_utility(double bandwidth_, double capacity_r) const; // 右半边软效用函数

	double utility(double bandwidth_, double capacity_, double SNR_avg_dB); // 用户效用函数



	/// <summary>
	/// 在带宽为 bandwidth_，信道容量为 capacity_，且已经存在效用值 existed_utility 的情况下，用户的边际效用
	/// </summary>
	/// <param name="bandwidth_">The bandwidth.</param>
	/// <param name="capacity_">The capacity.</param>
	/// <param name="existed_utility">用户的已有效用.</param>
	/// <returns>边际效用</returns>
	double marginal_utility(double bandwidth_, double capacity_, double existed_utility);

	/// <summary>
	/// 用户在分配带宽为bandwidth_时的导数. 对于hard用户，参数capacity表示其需求的带宽阈值. 对于elastic用户，参数capacity表示信道容量，即log(1+SNR)
	/// </summary>
	/// <param name="bandwidth_">The bandwidth.</param>
	/// <param name="capacity_">The capacity.</param>
	/// <returns></returns>
	double utility_derivative(double bandwidth_, double capacity_);

	void print_user() const;
};

class Uav : public Point {
public:
	double total_bandwidth = 20;	// UAV的总带宽容量,20MHz
	double hard_bandwidth = 0;      // 为硬效用用户分配的带宽，这两个变量只有在带宽比例搜索算法中才会被使用
	double elastic_bandwidth = 0;      // 为弹性效用用户分配的带宽，这两个变量只有在带宽比例搜索算法中才会被使用
	double pTrans = 2;      // 发射功率 2W

	Uav() {}
	Uav(int id_, double x1, double y1, double z1, double total_bandwidth_) : Point(id_, x1, y1, z1)
	{
		total_bandwidth = total_bandwidth_;
	}
	~Uav() {}
	void print_UAV() const;
};

class Channel {
public:
	Uav P_tr;
	User P_re;
	double d = 0;
	double theta = 0;       // 仰角
	double P_LoS = 0;
	double P_NLoS = 0;
	double L_LoS = 0;
	double L_NLoS = 0;
	double PL = 0;
	double K_R = 0;         // 莱斯因子
	double M = 0;           // Nakagami-M分布的参数
	double beta = 0;
	double h_miu = 0;       // h服从复高斯分布，h_miu是其均值
	double h_sigma = 0;     // h服从复高斯分布，h_sigma是其方差
	double eta = 0;
	double h = 0;           // 计算h
	double g = 0;
	double SNRa_dB = 0;        // 平均信噪比 average SNR, in dB
	double SNRt_dB = 0;     // 信噪比阈值 SNR threshold, in dB
	double channel_capacity = 0;    // 信道容量 Capacity, in bit/s/Hz

	~Channel() {}

	/// <summary>
	/// 初始化类 <see= default;ref="Channel"/> 的一个新的实例.
	/// 计算发送者P_tr到接受者P_re之间的各种信道参数
	/// </summary>
	/// <param name="p_tr_">信号发送者，一般指无人机</param>
	/// <param name="p_re_">信号接收者，一般指用户</param>
	Channel(Uav& p_tr_, User& p_re_);

	/// <summary>
	/// Sets the p.
	/// </summary>
	/// <param name="p_tr_">The p tr.</param>
	/// <param name="p_re_">The p re.</param>
	void set_P(Uav& p_tr_, User& p_re_) { P_tr = p_tr_; P_re = p_re_; }

	/// <summary>
	/// 计算低点P_re到高点P_re的仰角
	/// theta = arctan((z1 - z2) / sqrt((x1 - x2)^2 + (y1 - y2)^2))
	/// </summary>
	/// <returns>P_re到P_tr的仰角，单位为度</returns>
	double cal_theta() const;
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的视距概率
	/// P_LoS = 1 / (1 + param_a * exp(-param_b * (theta - param_a)))
	/// </summary>
	/// <returns>P_tr到P_re的视距概率, 0-1之间</returns>
	double cal_P_LoS() const;
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的视距自由空间路径损耗L_LoS
	/// L_LoS = 20log10(4πfd/speed_light) + η_LoS
	/// </summary>
	/// <returns>P_tr到接受者P_re之间的视距自由空间路径损耗L_LoS，单位为dB</returns>
	double cal_L_LoS() const;
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的非视距自由空间路径损耗L_NLoS
	/// L_NLoS = 20log10(4πfd/speed_light) + η_NLoS
	/// </summary>
	/// <returns>P_tr到接受者P_re之间的非视距自由空间路径损耗L_LoS，单位为dB</returns>
	double cal_L_NLoS() const;
	/// <summary>
	/// 计算小尺度Nakagami-M衰落的参数M
	/// M = (K + 1)^2 / (2K + 1)，其中K = P_LoS / P_NLoS
	/// </summary>
	/// <returns>小尺度Nakagami-M衰落的参数M</returns>
	double cal_Nakagami_M() const;
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的大尺度衰减系数PL
	/// PL = P_LoS * L_LoS + P_NLoS * L_NLoS
	/// </summary>
	/// <returns>P_tr到接受者P_re之间的大尺度衰减系数PL，单位dB</returns>
	double cal_PL() const;
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的平均信噪比SNR
	/// SNR = (P_tr.pTrans * g^2) / (noise_dbm * PL)
	/// </summary>
	/// <returns>P_tr到接受者P_re之间的平均信噪比SNR，单位为dB</returns>
	double cal_average_SNR() const;
	/// <summary>
	/// 根据用户的最大中断概率pOut，计算P_tr满足P_re中断概率需求的信噪比阈值SNR_th。根据Nakagami衰落中的中断概率公式，通过求解反函数得到SNR_th。
	/// pOut = 1/Γ(M) * γ(M, (M * SNR_th) / SNR_avg)
	/// 如果发送者将其所有带宽分配给该用户时，仍然无法满足用户的中断概率需求，则返回负无穷。
	/// </summary>
	/// <returns>P_tr满足P_re中断概率需求的信噪比阈值SNR_th, 单位为dB</returns>
	double cal_SNR_th() const;
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的信道容量Capacity
	/// Hard: log2(1 + SNR_th); reliability is already encoded in SNR_th.
	/// </summary>
	/// <returns>P_tr到接受者P_re之间的信道容量Capacity, 单位为bit/s/Hz</returns>
	double cal_capacity() const;


	double cal_h() const;
	double cal_eta() const;

	double cal_beta() const;
	static double cal_Rayleigh();
	double cal_Rice();
	double cal_Rice_L1_2(double x);

	void cal_h_miu_sigma();
	double cal_h_old();
	double cal_g() const;
	double cal_SNR_old() const;

	void print_all();
};

//
/// <summary>
/// 系统模型类，包含了系统中的用户和无人机列表，以及它们之间的信道参数矩阵。
/// </summary>
class SystemMd {
public:
	int m = 0;      // 无人机数量
	int n1 = 0;     // 离散效用用户数量
	int n2 = 0;     // 连续效用用户数量
	double max_user_utility = 0; // 系统中用户的最大效用值

	vector<User> users;
	vector<Uav> uavs;

	vector<vector<double>> dis_list;    // m*(n1+n2) 记录各个用户与无人机之间的距离
	vector<vector<double>> SNRave_list;    // m*(n1+n2) 记录各个用户与无人机之间的信噪比
	vector<vector<double>> SNRth_list;    // m*(n1+n2) 记录各个用户与无人机之间的信噪比
	vector<vector<double>> M_list;      // m*(n1+n2) 记录各个用户与无人机之间的Nakagami-M参数
	vector<vector<double>> cap_list;    // m*(n1+n2) 记录各个用户与无人机之间的信道容量
	vector<vector<double>> Bth_list; // m*n1 记录各个离散效用用户与无人机之间的最小带宽需求
	map<int, vector<User>> uav_serviceable_users_map;	// 记录每个无人机可服务的用户列表，key为uav_id，value为该uav可服务的用户列表，分离硬效用用户和弹性效用用户

	SystemMd() {}
	/// <summary>
	/// 构造 SystemMD 类的实例，并初始化系统模型。
	///= default;/summary>
	/// <param name="u_">用户对象的 vector 引用，用于初始化系统中的用户列表。</param>
	/// <param name="a_">UAV（无人机）对象的 vector 引用，用于初始化系统中的 UAV 列表。</param>
	SystemMd(vector<User> u_, vector<Uav> a_);
	/// <summary>
	/// Initializes param_a new instance of the <see cref="SystemMD"/> class.
	/// </summary>
	/// <param name="user_file">The user file.</param>
	/// <param name="uav_file">The uav file.</param>
	SystemMd(string user_file, string uav_file, string config_file);
	/// <summary>
	/// Initializes the syetem model.
	/// </summary>
	void init_SystemModel();


	void print_SystemInfo(int n = 100) const;
	void print_all_users(int n = 100) const;
	void print_all_uavs() const;
	void print_dis_list(int n = 100) const;
	void print_SNRa_list(int n = 100) const;
	void print_SNRt_list(int n = 100) const;
	void print_M_list(int n = 100) const;
	void print_cap_list(int n = 100) const;
	void print_min_bw_list(int n = 100) const;
};

class BAProblem {
public:
	SystemMd sysModel;
	double epsilon = 0.001;	// FPTAS精度参数
	double delta = 0.1;		// 搜索算法步长
	vector<vector<double>> alloc_matrix; // m*(n1+n2) 记录各个用户从各个无人机分配到的带宽
	vector<vector<int>> connect_matrix;  // m*(n1+n2) 记录各个用户与无人机的连接关系，1表示连接，0表示不连接



	BAProblem() {}
	/// <summary>
	/// Initializes param_a new instance of the <see cref="BAProblem"/> class.
	/// </summary>
	/// <param name="sys_">The system.</param>
	BAProblem(const SystemMd sys_) : sysModel(sys_)
	{
		alloc_matrix = vector<vector<double>>(sysModel.m, vector<double>(sysModel.n1 + sysModel.n2, 0.0));
		connect_matrix = vector<vector<int>>(sysModel.m, vector<int>(sysModel.n1 + sysModel.n2, 0));
	}
	~BAProblem() {}

	/// <summary>
	/// 根据分配矩阵计= default;统的总效用值
	/// </summary>
	/// <param name="allResult"> 每个无人机的分配结果列表.</param>
	/// <returns> 系统的总效用值</returns>
	static double get_total_utility(const vector<KnapsackResult>& allResult);

	/// <summary>
	/// 实现局部搜索算法，(1+alpha)近似带宽分配算法
	/// 参考论文："Knapsack problems with sigmoid utilities: Approximation algorithms via hybrid optimization"
	/// </summary>
	/// <param name="epsilon_"> 子问题FPTAS精度参数</param>
	/// <param name="delta_"> 搜索算法步长</param>
	/// <returns> 算法结果</returns>
	std::pair< std::vector<KnapsackResult>, std::vector<UserResult> > local_search_allocation(double epsilon_ = 0.1, double delta_ = 0.5);

	/// <summary>
	/// 确定无人机uav_id的临时分配结果，是一个递归函数
	/// 参考论文："Knapsack problems with sigmoid utilities: Approximation algorithms via hybrid optimization"
	/// </summary>
	/// <param name="uav_id">当前分配的UAV.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <param name="allResult">最终分配结果，递归过程中按下表倒序填充</param>
	/// <returns> 无人机uav_id的分配结果</returns>
	KnapsackResult GAP(int uav_id, vector<double>& uti_max, vector<KnapsackResult>& allResult);

	map<int, UserResult> construct_user_results(const vector<KnapsackResult>& allResults);

	// ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	// 单无人机分配问题的算法

	/// <summary>
	/// 基于资源比例搜索的子问题近似算法
	/// 备注：该函数通过遍历不同的连续资源，分别求解离散部分和连续部分的分配问题，并将结果合并，最终返回不同连续资源比例下的分配结果。
	/// 该算法离散用户未使用的资源不会被连续用户使用
	/// </summary>
	/// <param name="uav_id">当前求解的UAV的ID.</param>
	/// <param name="capacity">uav_id对应uav的剩余容量</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>key: 连续部分比例，value: 不同连续资源比例下的分配结果</returns>
	map<double, KnapsackResult> RP_based_subproblem_allocation_experiment1(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 基于资源比例搜索的子问题近似算法, 返回所有断点处的分配结果
	/// 备注：该函数通过遍历不同的连续资源比例，分别求解离散部分和连续部分的分配问题，并将结果合并，最终返回不同连续资源比例下的分配结果。
	/// 该算法离散用户未使用的资源会被连续用户使用
	/// </summary>
	/// <param name="uav_id">当前求解的UAV的ID.</param>
	/// <param name="capacity">uav_id对应uav的剩余容量</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>key: 所有断点对应的连续资源比例，value: 不同连续资源比例下的分配结果</returns>
	map<double, KnapsackResult> RP_based_subproblem_allocation_experiment2(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 基于资源比例搜索的子问题近似算法
	/// 备注：该函数通过遍历不同的连续资源，分别求解离散部分和连续部分的分配问题，并将结果合并，最终返回不同连续资源比例下的分配结果。
	/// 该算法是RP_based_subproblem_allocation_experiment1的松弛版本，离散用户的效用函数不再是阶跃函数，而是一个线性函数，表示离散用户可以部分分配资源，从而获得部分效用。
	/// </summary>
	/// <param name="uav_id">当前求解的UAV的ID.</param>
	/// <param name="capacity">uav_id对应uav的剩余容量</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>key: 连续部分比例，value: 不同连续资源比例下的分配结果</returns>
	map<double, KnapsackResult> RP_based_subproblem_allocation_experiment1_relaxed(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 基于资源比例搜索的子问题近似算法, 返回最大效用处的分配结果
	/// 备注：该函数通过遍历不同的连续资源比例，分别求解离散部分和连续部分的分配问题，并将结果合并，最终返回不同连续资源比例下的分配结果。离散部分使用FPTAS算法。
	/// 该算法离散用户未使用的资源会被连续用户使用
	/// </summary>
	/// <param name="uav_id">当前求解的UAV的ID.</param>
	/// <param name="capacity">uav_id对应uav的剩余容量</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>单无人机分配问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult RP_based_subproblem_allocation_FPTAS(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 基于资源比例搜索的子问题近似算法, 返回最大效用处的分配结果
	/// 备注：该函数通过遍历不同的连续资源比例，分别求解离散部分和连续部分的分配问题，并将结果合并，最终返回不同连续资源比例下的分配结果。离散部分使用贪心算法。
	/// 该算法离散用户未使用的资源会被连续用户使用
	/// </summary>
	/// <param name="uav_id">当前求解的UAV的ID.</param>
	/// <param name="capacity">uav_id对应uav的剩余容量</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>单无人机分配问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult RP_based_subproblem_allocation_Greedy(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 对于hard_utility用户的带宽分配问题，是一个0-1背包问题，FPTAS算法. 时间复杂度为 O(n^3 / ε)
	/// 给定无人机uav_id，用该算法为该无人机分配带宽
	/// 在该问题中，用户的最小带宽需求对应于背包问题中的物品重量，用户的权重对应于物品价值
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="unproc_users">待处理的硬效用用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult Fptas01Knapsack(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max);




	/// <summary>
	/// 贪心算法解决0-1背包问题，时间复杂度为O(n log n)
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="sorted_unproc_users">待处理的硬效用用户集合, 已按单位带宽效用排序.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns></returns>
	KnapsackResult Greedy01Knapsack(Uav& uav, vector<User>& sorted_unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 这是一个基于KKT条件和注水原理 (Water-filling) 的资源分配算法。该算法的核心思想是通过边际效用排序，找到“水位”（即拉格朗日乘子 $\lambda$）的临界点，从而确定哪些用户可以获得资源。
	/// </summary>
	/// <param name="uav_id"> 当前无人机的ID </param>
	/// <param name="capacity"> 当前无人机的剩余容量 </param>
	/// <param name="unproc_users">待处理的用户集合</param>
	/// <param name="is_rounding"> 是否对分配结果中的最后一个hard用户进行舍入，默认为1，表示进行舍入 </param>
	/// <returns> 背包问题分配结果，KnapsackResult 类型 </returns>
	KnapsackResult WaterFillingAlgorithm_singleUAV(Uav uav, vector<User> unproc_users, int is_rounding = 1);

	/// <summary>
	/// 该方法与WaterFillingAlgorithm_singleUAV的唯一区别是，在搜索水位λ时，采用二分法进行搜索，从而提高了搜索效率。
	/// </summary>
	/// <param name="uav_id"> 当前无人机的ID </param>
	/// <param name="capacity"> 当前无人机的剩余容量 </param>
	/// <param name="unproc_users">待处理的用户集合</param>
	/// <param name="is_rounding"> 是否对分配结果中的最后一个hard用户进行舍入，默认为1，表示进行舍入 </param>
	/// <returns> 背包问题分配结果，KnapsackResult 类型 </returns>
	KnapsackResult WaterFillingAlgorithm_singleUAV_new(Uav uav, vector<User> unproc_users, int is_rounding = 1);

	/// <summary>
	/// 一个新的FPTAS算法，用于解决单无人机的带宽分配问题，时间复杂度为O(n^2 / ε)。考虑了用户的混合效用函数（硬效用和弹性效用）。
	/// </summary>
	/// <param name="uav_id"> 当前无人机的ID </param>
	/// <param name="capacity"> 当前无人机的剩余容量 </param>
	/// <param name="unproc_users">待处理的用户集合</param>
	/// <param name="epsilon"> 精度要求 </param>
	/// <returns> 背包问题分配结果，KnapsackResult 类型 </returns>
	KnapsackResult FPTAS_singleUAV(Uav& uav, vector<User>& unproc_users, double epsilon);

	/// <summary>
	/// 该方法与FPTAS_singleUAV的唯一区别是，动态规划表格的值不再是恰好等于价值v的最小重量，而是价值至少为v的最小重量
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="unproc_users">待处理的硬效用用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult FPTAS_singleUAV_new(Uav uav, vector<User> unproc_users, double epsilon = 0.02);

	/**
	 * @brief 使用 LCM 与二候选舍入求解 ToN 快速单 UAV 边际效用问题。
	 * @param diagnostics 可选 Fast 分支接收器；空指针关闭诊断。
	 * @param uav 待评估 UAV；total_bandwidth 表示本次可用的新增带宽预算。
	 * @param candidate_users 该 UAV 在本次调用中可以服务的候选用户。
	 * @param current_utilities 按用户 ID 索引的当前网络绝对效用 m_j。
	 * @param base_bandwidths 按用户 ID 索引的同一 UAV 已有基准带宽 b_{kj}^{old}。
	 * @return allocatedBandwidth 为新增带宽，allocatedValue/totalValue 为真实边际效用。
	 */
	KnapsackResult AlgFast_singleUAV_ToN(
		const Uav& uav,
		const vector<User>& candidate_users,
		const vector<double>& current_utilities,
		const vector<double>& base_bandwidths,
		TonFastDiagnostics* diagnostics = nullptr); // 可选观测接收器；不改变求解规则。

	/**
	 * @brief 使用缩放利润 DP 与 SMAWK 求解 ToN 改进单 UAV 边际效用问题。
	 * @param uav 待评估 UAV；total_bandwidth 表示本次可用的新增带宽预算。
	 * @param candidate_users 该 UAV 在本次调用中可以服务的候选用户。
	 * @param current_utilities 按用户 ID 索引的当前网络绝对效用 m_j。
	 * @param base_bandwidths 按用户 ID 索引的同一 UAV 已有基准带宽 b_{kj}^{old}。
	 * @param epsilon 近似参数，必须满足 0 < epsilon < 1/2，默认值为 0.1。
	 * @return allocatedBandwidth 为新增带宽，allocatedValue/totalValue 为真实边际效用。
	 */
	KnapsackResult AlgBetter_singleUAV_ToN(
		const Uav& uav,
		const vector<User>& candidate_users,
		const vector<double>& current_utilities,
		const vector<double>& base_bandwidths,
		double epsilon = 0.1);


	// Faster 新接口开始：保留旧接口和正式调度，供后续显式切换使用。
	/**
	 * @brief 通过对偶认证、非凹用户 DP 与凹用户精确注水实现改进的 ToN 单 UAV 近似算法。
	 * @param options 显式证书早退设置；关闭时仍观测证书，但不据此返回。
	 * @param diagnostics 可选单次调用诊断；未计算的值保持 NaN。
	 * @param uav 本次可用的新增带宽预算。
	 * @param candidate_users 本次可服务的候选用户子集，输入顺序不被修改。
	 * @param current_utilities 按全局用户 ID 索引的当前网络绝对效用。
	 * @param base_bandwidths 同一 UAV 已有的基准带宽，按全局用户 ID 索引。
	 * @param epsilon 误差参数，要求 0<epsilon<1/2，默认 0.1。
	 * @return 新增带宽及真实边际效用；保持 (1-epsilon) 保证但不承诺与旧版输出一致。
	 */
	KnapsackResult AlgBetter_singleUAV_ToN_faster(
		const Uav& uav,
		const vector<User>& candidate_users,
		const vector<double>& current_utilities,
		const vector<double>& base_bandwidths,
		double epsilon = 0.1,
		const TonFasterOptions& options = TonFasterOptions(),
		TonSingleUavDiagnostics* diagnostics = nullptr);
	// Faster 新接口结束。

	/// <summary>
	/// 在当前状态下，计算无人机uav_id子分配问题的KKT参数
	/// </summary>
	/// <param name="uav_id"> 当前无人机的ID </param>
	/// <param name="unproc_users">待处理的用户集合</param>
	/// <returns> KKT参数结构体 </returns>
	vector<KKT_parameters> compute_KKT_parameters(int uav_id, const vector<User>& unproc_users);

	void print_KKT_parameters(const vector<KKT_parameters>& kkt_params);

	/// <summary>
	/// 对于elastic_utility类型的用户，带宽分配问题是一个凸优化问题，直接根据KKT条件求解 给定无人机uav_id，用该算法为该无人机分配带宽
	/// 算法记录与笔记：[[003 算法设计2 混合效用的单无人机资源分配问题#2 3 2 对于连续部分：KKT条件找最优解]]
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="unproc_users">待处理的软效用用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult KktBasedElasticUtility(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// Pegging算法，算法参考笔记：[[003 算法设计2 混合效用的单无人机资源分配问题#2 3 3 对于连续部分的新方法：Pegging算法]]
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult PeggingAlgorithm(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max);



	/// <summary>
	/// 提出的多无人机带宽分配近似算法
	/// </summary>
	/// <param name="uavs">系统中无人机集合</param>
	/// <param name="users">系统中用户的集合</param>
	/// <param name="used_single_alg"> 单无人机算法指示变量，1为使用1/2近似算法，2为使用FPTAS算法</param>
	/// <param name="epsilon">精度要求</param>
	/// <returns></returns>
	pair<vector<KnapsackResult>, map<int, UserResult>>  approposed_multiUAV_allocation(vector<Uav> uavs, vector<User> users, int used_single_alg = 2, double parameter = 0.083);

	/// <summary>
	/// 提出的多无人机带宽分配近似算法; 与原算法相比，不是每次选择每个效用最大的无人机，而是先为每个无人机分配一遍。
	/// 用户选择效用最大的无人机
	/// </summary>
	/// <param name="uavs">系统中无人机集合</param>
	/// <param name="users">系统中用户的集合</param>
	/// <param name="used_single_alg"> 单无人机算法指示变量，1为使用1/2近似算法，2为使用FPTAS算法</param>
	/// <param name="epsilon">精度要求</param>
	/// <returns></returns>
	pair<vector<KnapsackResult>, map<int, UserResult>>  approposed_multiUAV_allocation_new(vector<Uav> uavs, vector<User> users, int used_single_alg = 2, double parameter = 0.083);

	/**
	 * @brief 执行 ToN 固定状态贪心多 UAV 分配及残余带宽阶段。
	 * @param uavs 参与选择的 UAV 集合，每架 UAV 恰好处理一次。
	 * @param users 当前系统模型中的完整用户集合。
	 * @param used_single_alg 取 1 时调用 AlgFast_singleUAV_ToN，取 2 时调用 AlgBetter_singleUAV_ToN，
	 *        取 3 时调用 AlgBetter_singleUAV_ToN_faster。
	 * @param epsilon 传给两种 AlgBetter 单 UAV 算法的近似参数，必须满足 0 < epsilon < 1/2。
	 * @param options 仅控制 Faster 的证书早退；默认不改变正式流程。
	 * @param diagnostics 可选轨迹容器；非空时重置并记录本次调用，不改动输入状态。
	 * @return 最终绝对结果：allocatedBandwidth 为总带宽，allocatedValue/totalValue 为绝对效用；
	 *         同时返回与 UAV 结果一致的逐用户投影。
	 */
	pair<vector<KnapsackResult>, map<int, UserResult>> Appro_multiUAV_ToN(
		const vector<Uav>& uavs,
		const vector<User>& users,
		int used_single_alg = 2,
		double epsilon = 0.1,
		const TonFasterOptions& options = TonFasterOptions(),
		TonNetworkDiagnostics* diagnostics = nullptr);


	/**
	* @brief 打印单个 KnapsackResult 结构体
	* * @param result 要打印的结果对象
	* @param maxItemsToPrint 控制打印详细物品的最大数量，默认打印前10个，-1表示打印所有
	* @param indent 缩进空格数，用于美化层级显示
	*/
	void PrintKnapsackResult(const KnapsackResult& result, int uav_id, int maxItemsToPrint = 10, int indent = 0);

	/**
	* @brief 打印 vector<KnapsackResult>
	* * @param results 结果列表
	* @param maxItemsPerResult 每个结果中详细打印的物品数限制
	*/
	void PrintKnapsackResultList(const vector<KnapsackResult>& results, int maxItemsPerResult = 5);



	/// AlgRelaxRound: continuous relaxation, seeded rounding and fixed-association refinement.
	/// Independently constructed; not Tian's TD3/preemption method.
	/// epsilon_tol is retained for source compatibility (the shared feasibility contract takes precedence).
	/// seed controls mt19937; max_trials bounds candidate attempts; optional diagnostics records solver stages.
	/// Returns per-UAV allocations and per-user projections; typed AllocationFailure reports unrecoverable stages.
	std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
		ConvexRelaxationAndRounding_multiUAV(double epsilon_tol = 1e-4,
			uint32_t seed = 20260905u, int max_trials = 2,
			AllocationDiagnostics* diagnostics = nullptr);

	/// AlgSwapMatching: strongest-link initialization, per-UAV IPOPT optimization and beneficial swaps.
	/// Inspired by Han--Wang, not a DEI or maximum-weight-matching reproduction.
	/// The default config keeps one outer pass: swaps are not followed by another bandwidth optimization.
	/// Hard sigmoid and normalized elastic-log surrogates differ from reported utility. Hard admission constraints
	/// are relaxed on overloaded UAVs; unmet hard users are dropped in recovery. Residual reoptimization is disabled.
	/// config controls the existing heuristic; diagnostics records optimizer status and feasible fallbacks.
	/// Returns the original pair of per-UAV allocations and per-user projections.
	std::pair<std::vector<KnapsackResult>, map<int, UserResult>>
		MatchingSQP_Allocation(MatchingSQPConfig config = MatchingSQPConfig(),
			AllocationDiagnostics* diagnostics = nullptr);


	std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
		MatchingGameAllocation();

	/// AlgSA-DD: successive-approximation/dual-price bandwidth heuristic adapted from Li et al. (2024).
	/// Retains theta updates, per-UAV dual/KKT solving, IPOPT refinement and original feasibility recovery.
	/// Positive-utility strongest-link initialization normally leaves other links at theta=0; the existing
	/// zero-utility initialization branch assigns uniform theta. This is not a general joint-association solver.
	/// config retains all original heuristic defaults; diagnostics reports solver stages and retained fallbacks.
	/// Returns per-UAV allocations and per-user projections, with unserved users at UAV=-1 and zero resources.
	std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
		SADA_Allocation(SADAConfig config = SADAConfig(),
			AllocationDiagnostics* diagnostics = nullptr);


	/// AlgHardFirst: per-subchannel priority-aware DA, finite hard recovery, then elastic DA.
	/// Common BSub is effective MHz; each slot occupies BSub*10/9 including guard overhead.
	/// Returns effective bandwidth; normalized-demand/dynamic preferences are adaptations, not a stability claim.
	std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
		HardFirstPriorityMatchingAllocation();
	/// Same DA allocation with one aggregate proposal/replacement/recovery JSON diagnostics event.
	std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
		HardFirstPriorityMatchingAllocation(AllocationDiagnostics* diagnostics);
	/// Compatibility alias only: forwards to HardFirstPriorityMatchingAllocation, not Hungarian/KM.
	std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
		HungarianMatchingAllocation();
};

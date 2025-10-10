#pragma once
#include "predefine.h"

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
	vector<int> allocatedList; // 选中的物品ID列表
	map<int, double> allocatedBandwidth; // 选中的物品对应的分配值（带宽）
	map<int, double> allocatedValue; // 选中的物品对应的价值
	double totalValue = 0;         // 总价值
	double totalWeight = 0;        // 总重量
	double softValue = 0;         // 仅连续部分的总价值
	double hardValue = 0;         // 仅离散部分的总价值
	double softWeight = 0;        // 仅连续部分的总重量
	double hardWeight = 0;        // 仅离散部分的总重量
};
struct UserResult {
	int uav_id = -1;
	double allocated_bandwidth = 0; // 分配的带宽
	double utility = 0; // 用户效用
};





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
	int weight = 1; // 用户权重，默认1
	double rData = 0;		// 需求的最小数据量
	double rMin = 0;		// 需求的最小数据速率
	double pOut = 0;		// 需求的最大中断概率

	double B = 0.18; // 180 KHz
	double BSub = 0.18;   // 子载波带宽 180 KHz
	// double B = 20e6;    // 20 MHz

	User() {}

	User(int id_, int uType_, int weight_, double x1, double y1, double z1, double rD_, double rM_, double pO_) : Point(id_, x1, y1, z1)
	{
		uType = uType_;
		weight = weight_;
		set_communication_requirements(rD_, rM_, pO_);
		//set_Bandwidth();
	}

	User(int id_, int uType_, int weight_, double x1, double y1, double z1, double r_min_, double P_o_) : Point(id_, x1, y1, z1)
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
	double hard_utility(double bandwidth_, double capacity_) const; 	
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
	double utility(double bandwidth_, double capacity_); // 用户效用函数

	/// <summary>
	/// 在带宽为 bandwidth_，信道容量为 capacity_，且已经存在效用值 existed_utility 的情况下，用户的边际效用
	/// </summary>
	/// <param name="bandwidth_">The bandwidth.</param>
	/// <param name="capacity_">The capacity.</param>
	/// <param name="existed_utility">用户的已有效用.</param>
	/// <returns>边际效用</returns>
	double marginal_utility(double bandwidth_, double capacity_, double existed_utility);

	/// <summary>
	/// Elastic_utility 的导数函数
	/// </summary>
	/// <param name="bandwidth_">The bandwidth.</param>
	/// <param name="capacity_">The capacity.</param>
	/// <returns></returns>
	double elastic_utility_derivative(double bandwidth_, double capacity_) const;

	void print_user() const;
};

class Uav : public Point {
public:
	double total_bandwidth = 20e6;	// UAV的总带宽容量,20MHz
	double hard_bandwidth = 0;      // 为硬效用用户分配的带宽，这两个变量只有在带宽比例搜索算法中才会被使用
	double soft_bandwidth = 0;      // 为软效用用户分配的带宽，这两个变量只有在带宽比例搜索算法中才会被使用
	double pTrans = 2;      // 发射功率 2W

	Uav() {}
	Uav(int id_, double x1, double y1, double z1, int total_bandwidth_) : Point(id_, x1, y1, z1)
	{
		total_bandwidth = total_bandwidth_;
	}
	~Uav() {}
	void print_UAV() const;
};

class Channel{
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
	/// P_LoS = 1 / (1 + a * exp(-b * (theta - a)))
	/// </summary>
	/// <returns>P_tr到P_re的视距概率, 0-1之间</returns>
	double cal_P_LoS() const;	
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的视距自由空间路径损耗L_LoS
	/// L_LoS = 20log10(4πfd/c) + η_LoS
	/// </summary>
	/// <returns>P_tr到接受者P_re之间的视距自由空间路径损耗L_LoS，单位为dB</returns>
	double cal_L_LoS() const;	
	/// <summary>
	/// 计算发送者P_tr到接受者P_re之间的非视距自由空间路径损耗L_NLoS
	/// L_NLoS = 20log10(4πfd/c) + η_NLoS
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
	/// SNR = (P_tr.pTrans * g^2) / (P_N * PL)
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
	/// 公式: Capacity = (1 - pOut) * log2(1 + SNR_th)
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
	vector<vector<double>> SNRa_list;    // m*(n1+n2) 记录各个用户与无人机之间的信噪比
	vector<vector<double>> SNRt_list;    // m*(n1+n2) 记录各个用户与无人机之间的信噪比
	vector<vector<double>> M_list;      // m*(n1+n2) 记录各个用户与无人机之间的Nakagami-M参数
	vector<vector<double>> cap_list;    // m*(n1+n2) 记录各个用户与无人机之间的信道容量

	SystemMd() {}
	/// <summary>
	/// 构造 SystemMD 类的实例，并初始化系统模型。
	///= default;/summary>
	/// <param name="u_">用户对象的 vector 引用，用于初始化系统中的用户列表。</param>
	/// <param name="a_">UAV（无人机）对象的 vector 引用，用于初始化系统中的 UAV 列表。</param>
	SystemMd(vector<User> u_, vector<Uav> a_);	
	/// <summary>
	/// Initializes a new instance of the <see cref="SystemMD"/> class.
	/// </summary>
	/// <param name="user_file">The user file.</param>
	/// <param name="uav_file">The uav file.</param>
	SystemMd(static string user_file, string uav_file);
	
	/// <summary>
	/// Initializes the syetem model.
	/// </summary>
	void init_SyetemModel();

	void print_all_users();
	void print_all_uavs();
	void print_dis_list() const;
	void print_SNRa_list() const;
	void print_SNRt_list() const;
	void print_M_list() const;
	void print_cap_list() const;
};

class BAProblem {
public:
	SystemMd sysModel;
	double epsilon = 0.1;	// FPTAS精度参数
	double delta = 0.1;		// 搜索算法步长
	vector<vector<double>> alloc_matrix; // m*(n1+n2) 记录各个用户从各个无人机分配到的带宽
	vector<vector<int>> connect_matrix;  // m*(n1+n2) 记录各个用户与无人机的连接关系，1表示连接，0表示不连接


	BAProblem() {}	
	/// <summary>
	/// Initializes a new instance of the <= default;e cref="BAProblem"/> class.
	/// </summary>
	/// <param name="sys_">The system.</param>
	BAProblem(const static SystemMd sys_) : sysModel(sys_)
	{
		alloc_matrix = vector<vector<double>>(sysModel.m, vector<double>(sysModel.n1 + sysModel.n2, 0.0));
		connect_matrix = vector<vector<int>>(sysModel.m, vector<int>(sysModel.n1 + sysModel.n2, 0));
	}
	~BAProblem() {}

	void init_allocation();	
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

	// 基于线性规划的(1+ε)α近似的带宽分配算法
	void LP_based_allocation(double epsilon, double alpha);

	// 基于动态规划的子问题近似算法
	void DP_based_subproblem_allocation(double epsilon, double alpha);



	// ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	// 单无人机分配问题的算法

	/// <summary>
	/// 基于资源比例搜索的子问题近似算法
	/// 备注：该函数通过遍历不同的连续资源比例，分别求解离散部分和连续部分的分配问题，并将结果合并，最终返回不同连续资源比例下的分配结果。
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
	/// 基于资源比例搜索的子问题近似算法, 返回最大效用处的分配结果
	/// 备注：该函数通过遍历不同的连续资源比例，分别求解离散部分和连续部分的分配问题，并将结果合并，最终返回不同连续资源比例下的分配结果。
	/// 该算法离散用户未使用的资源会被连续用户使用
	/// </summary>
	/// <param name="uav_id">当前求解的UAV的ID.</param>
	/// <param name="capacity">uav_id对应uav的剩余容量</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>单无人机分配问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult RP_based_subproblem_allocation(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 对于hard_utility用户的带宽分配问题，是一个0-1背包问题，FPTAS算法.
	/// 给定无人机uav_id，用该算法为该无人机分配带宽
	/// 在该问题中，用户的最小带宽需求对应于背包问题中的物品重量，用户的权重对应于物品价值	
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="unproc_users">待处理的硬效用用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult Fptas01Knapsack(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 对于elastic_utility类型的用户，带宽分配问题是一个凸优化问题，直接根据KKT条件求解 给定无人机uav_id，用该算法为该无人机分配带宽	
	/// 算法记录与笔记：![[000-工作日志-讨论日志-9月2#对于连续部分：KKT条件找最优解]]
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="unproc_users">待处理的软效用用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult KktBasedElasticUtility(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// Pegging算法，算法参考论文："The nonlinear knapsack problem – algorithms and applications"
	/// </summary>
	/// <param name="uav">为uav进行分配决策，其中包含了容量信息.</param>
	/// <param name="unproc_users">待处理的用户集合.</param>
	/// <param name="uti_max">用户当前的最大效用，用户的边际效用函数参数</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult PeggingAlgorithm(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max);

	/// <summary>
	/// 用CPLEX求解凸优化问题，验证KKT_based_elastic_utility的正确性	
	/// </summary>
	/// <param name="uav_id">当前求解的UAV的ID.</param>
	/// <returns>背包问题的分配结果，KnapsackResult 类型</returns>
	KnapsackResult CplexBasedElasticUtility(int uav_id);
	
	/// <summary>
	/// Prints the knapsack result.
	/// </summary>
	/// <param name="knapsackResult">The knapsack result.</param>
	/// <param name="uav_id">id of uav</param>
	static void PrintKnapsackResult(KnapsackResult& knapsackResult, int uav_id = -1);
};

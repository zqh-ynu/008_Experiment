#include "EntityDefinition.h"

Point::Point(double x, double y, double z)
{
	X = x;
	Y = y;
	Z = z;
}

Point::Point(int id, double x, double y, double z)
{
	ID = id;
	X = x;
	Y = y;
	Z = z;
}

double Point::cal_distance(const Point& s, const Point& t)
{
	return sqrt((s.X - t.X) * (s.X - t.X) + (s.Y - t.Y) * (s.Y - t.Y) + (s.Z - t.Z) * (s.Z - t.Z));
}

double Point::cal_horizontal_distance(const Point& s, const Point& t)
{
	return sqrt((s.X - t.X) * (s.X - t.X) + (s.Y - t.Y) * (s.Y - t.Y));
}

void Point::print_point()
{
}

double User::hard_utility(double bandwidth_, double capacity_) const
{
	if (bandwidth_ * capacity_ >= rMin)
		return weight * 1.0;
	else
		return 0.0;
}

double User::elastic_utility(double bandwidth_, double capacity_) const
{
	double r = bandwidth_ * capacity_;
	// 参数p的选取会影响函数的增长速度，p越大，函数增长越快
	// 备注：当r < rMin时，效用为0；当r >= rMin时，
	// 具体定义参见tanUtilityMaximizationResource2015
	/*double p = 0.2;
	if(r < rMin)
		return 0.0;
	else
		return weight * (1 - exp(-p * (r - rMin)));*/

	// 定义2：参见shiNetworkUtilityMaximization2008 公式(4)
	// u(r) = w * log(1 + r) / log(1 + rMin)
	return weight * log(1 + r) / log(1 + rMin);
}

double User::halfsoft_utility(double bandwidth_, double capacity_) const
{
	double p = 0.2;
	double beta = 0.6;
	double r = bandwidth_ * capacity_;
	if (r < rMin)
		return 0.0;
	else
		return weight * (1 - (1 - beta) * exp(-p * (r - rMin)));
}

double User::utility(double bandwidth_, double capacity_)
{
	if (uType == HARD_UTILITY)
		return hard_utility(bandwidth_, capacity_);
	else if (uType == ELASTIC_UTILITY)
		return elastic_utility(bandwidth_, capacity_);
	else if (uType == HALFSOFT_UTILITY)
		return halfsoft_utility(bandwidth_, capacity_);
	else
	{
		cout << "Error: undefined user utility type!" << endl;
		return -1;
	}
}

double User::marginal_utility(double bandwidth_, double capacity_, double existed_utility)
{
	double new_utility = utility(bandwidth_, capacity_) - existed_utility;
	/*if (new_utility <0) 
		return 0;*/
	return new_utility;
}

double User::elastic_utility_derivative(double bandwidth_, double capacity_) const
{
	double r = bandwidth_ * capacity_;
	return weight * (capacity_ / ((1 + r) * log(1 + rMin)));
}

void User::print_user() const
{
	cout << "User ID: " << ID << ", Type: " << uType << ", Weight: " << weight << ", Location: (" << X << ", " << Y << ", " << Z << "), rData: " << rData << ", rMin: " << rMin << ", pOut: " << pOut << endl;
}

void Uav::print_UAV() const
{
	cout << "UAV ID: " << ID << ", Location: (" << X << ", " << Y << ", " << Z << "), Bandwidth: " << total_bandwidth << "MHz, pTrans: " << pTrans << "W" << endl;
}

Channel::Channel(Uav& p_tr_, User& p_re_)
{
	set_P(p_tr_, p_re_);
	d = User::cal_distance(P_tr, P_re); // 计算发送者p1到接受者p2之间的距离

	theta = cal_theta();   // 计算低点p2到高点p1的仰角
	P_LoS = cal_P_LoS(); // 计算发送者p1到接受者p2之间的视距概率
	P_NLoS = 1 - P_LoS;
	M = cal_Nakagami_M();
	L_LoS = cal_L_LoS(); // 计算发送者p1到接受者p2之间的L_LoS
	L_NLoS = cal_L_NLoS(); // 计算发送者p1到接受者p2之间的L_NLoS
	PL = cal_PL();    // 计算发送者p1到接受者p2之间的大范围衰减系数PL
	SNRa_dB = cal_average_SNR(); // 计算发送者p1到接受者p2之间的平均信噪比SNR
	SNRt_dB = cal_SNR_th(); // 信噪比阈值
	channel_capacity = cal_capacity(); // 信道容量
}

double Channel::cal_theta() const
{
	double x1 = P_tr.X;
	double y1 = P_tr.Y;
	double z1 = P_tr.Z;

	double x2 = P_re.X;
	double y2 = P_re.Y;
	double z2 = P_re.Z;

	double fenzi = abs(z1 - z2);
	double fenmu = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
	// cout << "fenzi = " << fenzi << "\tfenmu = " << fenmu << '\n';
	double theta_ = atan(fenzi / fenmu);
	double rad_to_deg = 180.0 / pi;
	theta_ = theta_ * rad_to_deg;
	// cout << "theta_ = " << theta_ << '\n';
	return theta_;
}

double Channel::cal_P_LoS() const
{
	double P_LoS_ = pow(1 + a * exp(-b * (theta - a)), -1);
	// 将P_LoS保留3位小数
	// P_LoS_ = ((int)(P_LoS_ * 1000)) / 1000.0;
	// cout << "P_LOS = " << P_LoS_ << '\n';
	return P_LoS_;
}

double Channel::cal_L_LoS() const
{
	// in dB
	double L_LoS_ = 20 * log10((4 * pi * f * d) / c) + eta_LoS;
	return L_LoS_;
}

double Channel::cal_L_NLoS() const
{
	double L_NLoS_ = 20 * log10((4 * pi * f * d) / c) + eta_NLoS;
	return L_NLoS_;
}

double Channel::cal_PL() const
{
	// 将dB转换为线性值计算
	double L_LoS_ = pow(10, L_LoS / 10.0);
	double L_NLoS_ = pow(10, L_NLoS / 10.0);
	double PL_ = P_LoS * L_LoS_ + P_NLoS * L_NLoS_;
	// 将PL转换为dB值
	PL_ = 10 * log10(PL_);
	return PL_;
}

double Channel::cal_average_SNR() const
{
	// 将各个变量转换为线性值计算
	double P_tr_linear = P_tr.pTrans; // W
	double P_N_linear = pow(10, P_N / 10.0); // W
	double g_UAV_linear = pow(10, g_UAV / 10.0);
	double PL_linear = pow(10, PL / 10.0);
	double SNR_linear = (P_tr_linear * g_UAV_linear) / (P_N_linear * PL_linear);
	double SNR_ = 10 * log10(SNR_linear);
	// cout << "SNR_ = " << SNR_ << endl;
	return SNR_;
}

double Channel::cal_h() const
{
	double g_ = 0;
	/*if (P_tr.Type == IBS)
		g_ = pow(10, g_BS / 10.0);
	else
		g_ = pow(10, g_UAV / 10.0);*/
	g_ = pow(10, g_UAV / 10.0);
	double h_ = PL * g_;
	// cout << "h_ = " << h_ << endl;
	return h_;
}

double Channel::cal_eta() const
{
	double eta_ = P_LoS * eta_LoS + P_NLoS * eta_NLoS;
	return eta_;
}

double Channel::cal_beta() const
{
	double beta_ = pow(10, -PL / 10.0);
	return beta_;
}

double Channel::cal_Rayleigh()
{
	return pi / 2.0;
}

double Channel::cal_h_old()
{
	cal_h_miu_sigma();
	double h_ = 0;
	/*if (P_tr.Type == "UAV")
		h_ = cal_Rice();
	else
		h_ = cal_Rayleigh();*/
	return h_;
}

double Channel::cal_g() const
{
	double g_ = pow(beta, 0.5) * h;
	return g_;
}

double Channel::cal_SNR_old() const
{
	double SNR_ = (P_tr.pTrans * pow(g, 2)) / pow(10, P_N / 10);
	return SNR_;
}

//double Channel::cal_Rice()
//{
//	double x = (-pow(h_miu, 2) / (2 * pow(h_sigma, 2)));
//	double h_ = h_sigma * pow(pi / 2, 0.5) * cal_Rice_L1_2(x);
//	return h_;
//}

//double Channel::cal_Rice_L1_2(double x)
//{
//
//	double I0 = cyl_bessel_i(0, -x / 2.0);
//	double I1 = cyl_bessel_i(1, -x / 2.0);
//	double L1_2 = exp(x / 2.0) * ((1 - x) * I0 - x * I1);
//	return L1_2;
//}

double Channel::cal_Nakagami_M() const
{
	double K = 0;
	if (P_NLoS != 0)
		K = P_LoS / P_NLoS;

	double M_ = pow(K + 1, 2) / (2 * K + 1.0);
	/*cout << "P_LoS = " << P_LoS << ", P_NLoS = " << P_NLoS << endl;
	cout << "K = " << K << ", M_ = " << M_ << endl;*/
	//M_ = 4;
	return M_;
}

double Channel::cal_SNR_th() const
{
	// 已知中断概率公式，通过数值方法求解反函数
	// pOut = 1/Γ(M) * γ(M, (M * SNR_th) / SNR_avg)
	// 先判断发送者将其所有带宽分配给该用户时，是否能满足用户的中断概率要求
	double SNR_th_test = pow(2, (P_re.rMin / P_tr.total_bandwidth)) - 1;

	double SNR_avg_linear = pow(10, SNRa_dB / 10.0);
	// boost::math::gamma_p = 1/Γ(M) * γ(M, (M * SNR_th) / SNR_avg) 是下不完全伽马函数的归一化形式
	// 文档地址：https://www.boost.org/doc/libs/1_64_0/libs/math/doc/html/math_toolkit/sf_gamma/igamma.html
	double p_test = boost::math::gamma_p(M, (M * SNR_th_test) / SNR_avg_linear);

	if (p_test > P_re.pOut)
	{
		/*cout << "Error: uav_id = " << P_tr.ID << ", user_id = " << P_re.ID << ", \t" << endl;
		cout << "The user outage probability requirement cannot be met even if all the bandwidth is allocated to it!" << endl;
		*/// 返回负无穷
		return -INFINITY;
	}

	try {
		// boost::math::gamma_p_inv 是下不完全伽马函数的归一化形式的反函数
		// 文档地址：https://www.boost.org/doc/libs/1_64_0/libs/math/doc/html/math_toolkit/sf_gamma/igamma_inv.html
		double root = boost::math::gamma_p_inv(M, P_re.pOut) * (SNR_avg_linear / M);
		// 将根转换为dB值
		root = 10 * log10(root);

		//// 输出求解结果
		//std::cout << "根的区间为： [" << result.first << ", " << result.second << "]" << std::endl;
		//std::cout << "根的近似值为： " << root << std::endl;

		//// 输出方程在根处的值，理论上应该接近于零
		//std::cout << "在根处的函数值： " << f(root) << std::endl;

		//// 输出迭代次数
		//std::cout << "迭代次数： " << max_iter << std::endl;

		return root;
	}
	catch (const std::exception& e) {
		// 捕获异常并输出错误信息
		cout << "uav_id = " << P_tr.ID << ", user_id = " << P_re.ID << endl;
		std::cerr << "错误： " << e.what() << std::endl;
	}
	return 0.0;
}

double Channel::cal_capacity() const
{
	// 计算信道容量
	if (SNRt_dB == -INFINITY)
		return 0.0;
	double SNR_th_linear = pow(10, SNRt_dB / 10.0);
	double capacity_ = (1 - P_re.pOut) * log2(1 + SNR_th_linear);
	return capacity_;
}

void Channel::cal_h_miu_sigma()
{
	h_miu = 0.0;
	h_sigma = 1.0;
	//if (P_tr.Type == "UAV")
	//{
	//	double K_r = P_LoS / P_NLoS;
	//	h_miu = pow(K_r / (K_r + 1), 0.5);
	//	h_sigma = pow(1 / (K_r + 1), 0.5);
	//	// cout << "\ncal_h_miu_sigma_test\n";
	//	// cout << "K_r: " << K_r << "\thmiu: " << h_miu << "\thsigma: " << h_sigma << '\n';
	//}
}

void Channel::print_all()
{
	cout << "++++++++++++++++++++++++++++++++++++++++++++\n";
	P_tr.print_point();
	cout << "--------------------------------------------\n";
	P_re.print_point();
	cout << "--------------------------------------------\n";
	// 输出类的所有属性，每行输出4个属性
	cout << "Distance (m): " << d << "\tTheta (deg): " << theta << "\n";
	cout << "P_LoS: " << P_LoS << "\tP_NLoS: " << P_NLoS << "\n";
	cout << "L_LoS (dB): " << L_LoS << "\tL_NLoS (dB): " << L_NLoS << "\n";
	cout << "PL (dB): " << PL << "\tK_R: " << K_R << "\n";
	cout << "Nakagami-M: " << M << "\tBeta: " << beta << "\n";
	cout << "h_miu: " << h_miu << "\th_sigma: " << h_sigma << "\n";
	cout << "h: " << h << "\tg: " << g << "\n";
	cout << "SNRa (dB): " << SNRa_dB << "\tSNRt (dB): " << SNRt_dB << "\n";
	cout << "Capacity (bit/s/Hz): " << channel_capacity << "\tpOut: " << P_re.pOut << "\n";
	cout << "++++++++++++++++++++++++++++++++++++++++++++\n";
}

SystemMd::SystemMd(vector<User> u_, vector<Uav> a_)
{
	users = u_;
	uavs = a_;
	n1 = 0;
	n2 = 0;
	for (auto& user : users)
	{
		if (user.uType == HARD_UTILITY)
			n1++;
		else
			n2++;
	}
	m = uavs.size();

	init_SyetemModel();
}

SystemMd::SystemMd(string user_file, string uav_file)
{
	// 先要从文件中读取用户和无人机的信息

	// 根据读取到的信息初始化信息
	init_SyetemModel();
}

void SystemMd::init_SyetemModel()
{
	dis_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	SNRa_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	SNRt_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	M_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	cap_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n1 + n2; j++)
		{
			Channel ch(uavs[i], users[j]);
			dis_list[i][j] = ch.d;
			SNRa_list[i][j] = ch.SNRa_dB;
			SNRt_list[i][j] = ch.SNRt_dB;
			M_list[i][j] = ch.M;
			cap_list[i][j] = ch.channel_capacity;
			// cout << "i=" << i << "\tj=" << j << "\tdis=" << dis_list[i][j] << "\tSNR=" << SNR_list[i][j] << "\tM=" << M_list[i][j] << "\tcap=" << cap_list[i][j] << '\n';
		}
	}

	// 计算用户的最大权重
	for (auto& user: users)
	{
		if(max_user_utility < user.weight)
			max_user_utility = user.weight;
	}
}

void SystemMd::print_all_users()
{
	cout << fixed << setprecision(2);
	cout << ".................all users...................." << '\n';
	for (auto& user : users)
	{
		user.print_user();
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_all_uavs()
{
	cout << fixed << setprecision(2);
	cout << ".................all UAVs....................." << '\n';
	for (auto& uav : uavs)
	{
		uav.print_UAV();
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_dis_list() const
{
	cout << fixed << setprecision(2);
	cout << ".................dis_list....................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n1 + n2; j++)
		{
			cout << dis_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_SNRa_list() const
{
	cout << fixed << setprecision(2);
	cout << ".................SNRa_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n1 + n2; j++)
		{
			cout << SNRa_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_SNRt_list() const
{
	cout << fixed << setprecision(2);
	cout << ".................SNRt_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n1 + n2; j++)
		{
			cout << SNRt_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_M_list() const
{
	cout << fixed << setprecision(2);
	cout << ".................M_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n1 + n2; j++)
		{
			cout << M_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_cap_list() const
{
	cout << fixed << setprecision(2);
	cout << ".................cap_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n1 + n2; j++)
		{
			cout << cap_list[i][j] << "\t";
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

double BAProblem::get_total_utility(const vector<KnapsackResult>& allResult)
{
	double total_utility = 0.0;
	for (auto result : allResult)
	{
		total_utility += result.totalValue;
	}
	return total_utility;
}

std::pair < std::vector<KnapsackResult>, std::vector<UserResult>> BAProblem::local_search_allocation(double epsilon_, double delta_)
{
	// 初始化算法输入
	epsilon = epsilon_;
    delta = delta_;
	vector<Uav> uavs = sysModel.uavs;
	vector<User> users = sysModel.users;
	vector<vector<double>> cap_list = sysModel.cap_list;
	int uav_num = uavs.size();
	int hard_user_num = sysModel.n1;
	int soft_user_num = sysModel.n2;
	vector<double> user_weights;

	vector<double> remaining_bandwidth;
	for(auto& uav : uavs)
	{
		remaining_bandwidth.push_back(uav.total_bandwidth);
	}
	
	// 用户新的效用函数 f(x) = f(x) - uti_max[j]
	// 用函数User::marginal_utility()计算
	// uti_max记录用户在当前分配情况下的最大效用值，初始化为0
	vector<double> uti_max = vector<double>(users.size(), 0.0);
	vector<KnapsackResult> allResult = vector<KnapsackResult>(uav_num);

	// userResult<uav_id, <bandwidth, value>> 记录用户对应的无人机及被分配的带宽
	vector<UserResult> user_results = vector<UserResult>(users.size());
	// 对userResult进行初始化
	for (int user_id = 0; user_id < users.size(); user_id ++)
	{
		user_results[user_id].uav_id = -1;

		user_results[user_id].allocated_bandwidth = 0;
		user_results[user_id].utility = 0.0;
	}

	// 通过递归调用GAP来对所有UAV进行分配
	allResult[0] = GAP(0, uti_max, allResult);

	
	
	// 根据无人机的分配结果allResult得到用户的分配结果
	// 这一步是为了在后续步骤中更高效，注意要同步更新结果
	for (int uav_id = 0; uav_id<uav_num; uav_id++)
	{
		KnapsackResult& uav_result = allResult[uav_id];
		for (auto user_id : uav_result.allocatedList)
		{
			// 遍历被uav_id分配的用户
			double bandwidth = uav_result.allocatedBandwidth[user_id];
			double value = uav_result.allocatedValue[user_id];

			if (user_results[user_id].uav_id == -1)
			{
				// 说明用户user_id还没有被分配
				user_results[user_id].uav_id = uav_id;
				user_results[user_id].allocated_bandwidth = bandwidth;
				user_results[user_id].utility = value;
			}
			else
				cout << "user " << user_id << " 被重复分配。\n";
		}
	}
	
	vector<User> unpro_users; // 记录未被分配的用户
	// 构造unpro_users
	for (int j = 0; j < users.size(); j++)
	{
		bool is_allocated = false;
		for (auto result : allResult)
		{
			// 如果用户j在result的allocatedList中，说明用户j已经被分配
			for (auto user_id : result.allocatedList)
			{
				if (user_id == j)
				{
					is_allocated = true;
					break;
				}
			}
		}
		if (!is_allocated)
			unpro_users.push_back(sysModel.users[j]);
	}

	// 启发式的提升性能
	// 将未被分配的用户分配到不饱和的UAV上
	for (int uav_id = 0; uav_id < uav_num; uav_id++)
	{
		cout << "uav " << uav_id << ": " << allResult[uav_id].totalValue << '\n';
		double remain_bw = uavs[uav_id].total_bandwidth - allResult[uav_id].totalWeight;
		if (remain_bw > 0 and !unpro_users.empty())
		{
			// 如果UAV还有剩余资源，且存在未被分配的用户

			// 用剩余资源对未服务的用户进行分配
			KnapsackResult result = RP_based_subproblem_allocation(uav_id, remain_bw, unpro_users, uti_max);

			// 在allResult中添加此次分配结果
			for (auto user_id : result.allocatedList)
			{
				allResult[uav_id].allocatedList.push_back(user_id);

				// 更新userResult
				double bandwidth = result.allocatedBandwidth[user_id];
				double value = result.allocatedValue[user_id];

				// 输出检查user_id
				cout << "检查user_id: " << user_id << '\n';

				if (user_results[user_id].uav_id == -1)
				{
					user_results[user_id].uav_id = uav_id;
					user_results[user_id].allocated_bandwidth = bandwidth;
					user_results[user_id].utility = value;
				}
				else
					cout << "user " << user_id << " 被重复分配。\n";
			}
			allResult[uav_id].hardValue += result.hardValue;
			allResult[uav_id].hardWeight+= result.hardWeight;

			allResult[uav_id].softValue += result.softValue;
			allResult[uav_id].softWeight += result.softWeight;

			allResult[uav_id].totalValue += result.totalValue;
			allResult[uav_id].totalWeight += result.totalWeight;


			// 并在unpro_users中删除被分配的用户
			auto it = unpro_users.begin();
			while (it != unpro_users.end())
			{
				
				int find_user_id = it->ID;
				bool is_find = false;

				auto it_id = std::find(result.allocatedList.begin(), result.allocatedList.end(), find_user_id);
				if (it_id != result.allocatedList.end()) is_find = true;

				if (is_find)
				{
					// 说明it->ID对应的用户被分配
					// 从unpro_users中删除
					it = unpro_users.erase(it);
				}
				else
				{
					// 如果没有删除元素，手动将 迭代器后移
					++it;
				}
			}
		}
		else if (remain_bw > 0 and unpro_users.empty())
		{
			// 如果UAV还有剩余资源，但不存在未被分配的用户


			// 将剩余资源分配给收益最高的软效用用户
			double max_margin_utility = 0;
			int max_user_id = -1;


			for (auto user_id : allResult[uav_id].allocatedList)
			{
				// 输出检查user_id
				cout << "检查user_id: " << user_id << '\n';
				User& user = users[user_id];
				if (user.uType != HARD_UTILITY)
				{
					double allcated_band = user_results[user_id].allocated_bandwidth;	// 用户j已经被分配的带宽
					double margin_utility = user.utility(allcated_band + remain_bw, cap_list[uav_id][user_id]);

					// 输出检查margin_utility
					cout << "检查margin_utility: " << margin_utility << '\n';
					if (max_margin_utility < margin_utility)
					{
						max_margin_utility = margin_utility;
						max_user_id = user_id;
					}
				}
			}

			// 输出检查max_user_id
			cout << "检查max_user_id: " << max_user_id << '\n';

			// 将剩余带宽分配给带来边际效用最大的用户
			// 更新结果allResult, userResult
			if (max_user_id != -1)
			{
				allResult[uav_id].allocatedBandwidth[max_user_id] += remain_bw;
				allResult[uav_id].allocatedValue[max_user_id] += max_margin_utility;
				allResult[uav_id].softWeight += remain_bw;
				allResult[uav_id].softValue += max_margin_utility;
				allResult[uav_id].totalWeight += remain_bw;
				allResult[uav_id].totalValue += max_margin_utility;

				user_results[max_user_id].allocated_bandwidth += remain_bw;
			}
		}

	}

	return {allResult, user_results};
}

KnapsackResult BAProblem::GAP(int uav_id, vector<double>& uti_max, vector<KnapsackResult>& allResult)
{
	// 输出uti_max
	cout << "uti_max: ";
	for (auto uti : uti_max)
		cout << uti << "\t";
	cout << endl;

	Uav& uav = sysModel.uavs[uav_id];
	KnapsackResult temp_result = RP_based_subproblem_allocation(uav_id, uav.total_bandwidth, sysModel.users, uti_max);

	// 检查参数
	cout << "BAProblem::GAP : \n";
	cout << "UAV " << uav_id << " : total_bandwidth = " << uav.total_bandwidth << "\n";
	// 输出temp_result
    PrintKnapsackResult(temp_result, uav_id);

	// 更新用户的最大效用值
	for (auto user_id : temp_result.allocatedList)
	{
		double bw = temp_result.allocatedBandwidth[user_id];
		double cap = sysModel.cap_list[uav_id][user_id];
		double new_utility = sysModel.users[user_id].utility(bw, cap);
		temp_result.allocatedValue[user_id] = new_utility;

		if (new_utility > uti_max[user_id])
		{
			uti_max[user_id] = new_utility;
		}
	}
	// 更新后
	cout << "After updating user maximum utility:\n";
	PrintKnapsackResult(temp_result, uav_id);

	if (uav_id < sysModel.m - 1)
	{
		allResult[uav_id + 1] = GAP(uav_id + 1, uti_max, allResult);

		// 由于是递归调用，下列代码在回溯时执行
		// 最先确定的分配为allResult[sysModel.m - 1]

		// 从temp_result排除allResult[uav_id + 1]至allResult[sysModel.m - 1]中已经分配的用户
		for(int i = uav_id + 1; i < sysModel.m; i++)
		{

			for (auto user_id : allResult[i].allocatedList)
			{
				// 在temp_result中查找user_id
				// 这里只删除用户，KnapsackResult中的其他属性不更新，在最后合并结果时更新
				if (bool is_find = remove_first(temp_result.allocatedList, user_id))
				{
					// 说明user_id在temp_result中被分配
					// 删除该用户的分配
					temp_result.allocatedBandwidth.erase(user_id);
					temp_result.allocatedValue.erase(user_id);
				}
			}
		}
		// 更新temp_result的其他属性
		temp_result.hardValue = 0.0;
		temp_result.softValue = 0.0;
		temp_result.hardWeight = 0.0;
		temp_result.softWeight = 0.0;
		for (auto user_id : temp_result.allocatedList)
		{
			double bw = temp_result.allocatedBandwidth[user_id];
			double value = temp_result.allocatedValue[user_id];

			// 根据用户类型更新
			if (sysModel.users[user_id].uType == HARD_UTILITY)
			{
				temp_result.hardWeight += bw;
				temp_result.hardValue += value;
			}
			else
			{
				temp_result.softWeight += bw;
				temp_result.softValue += value;
			}
		}
		temp_result.totalWeight = temp_result.hardWeight + temp_result.softWeight;
		temp_result.totalValue = temp_result.hardValue + temp_result.softValue;

		return temp_result;
	}
	else
		return temp_result;
}

map<double, KnapsackResult> BAProblem::RP_based_subproblem_allocation_experiment1(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{
	Uav uav = sysModel.uavs[uav_id];
	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0;
	uav.soft_bandwidth = 0;

	vector<User> unproc_hard_users;
	vector<User> unproc_soft_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_soft_users.push_back(user);

	map<double, KnapsackResult> result_list;
	double max_utility_ratio = 1; // 效用最大时，对应的连续部分比例

	double ratio_step = 0.01;		// 每次增加的比例
	double continuous_ratio = 1; // 连续部分比例

	while (continuous_ratio >= 0)
	{
		double discrete_ratio = 1 - continuous_ratio;
		uav.soft_bandwidth = uav.total_bandwidth * continuous_ratio;
		uav.hard_bandwidth = uav.total_bandwidth * discrete_ratio;
		//cout << "continuous_ratio = " << continuous_ratio << "\tdiscrete_ratio = " << discrete_ratio << '\n';

		// 1. 离散部分：0-1背包问题
		KnapsackResult discrete_result = Fptas01Knapsack(uav, unproc_hard_users, uti_max);
		double remain_resource = uav.hard_bandwidth - discrete_result.totalWeight;
		uav.soft_bandwidth += remain_resource;
		cout << "remain resource: " << remain_resource << '\n';
		// 2. 连续部分：KKT条件
		KnapsackResult continuous_result = KktBasedElasticUtility(uav, unproc_soft_users, uti_max);
		// 3. 合并结果
		KnapsackResult current_result;
		current_result.softValue = continuous_result.totalValue;
		current_result.hardValue = discrete_result.totalValue;
		current_result.softWeight = continuous_result.totalWeight;
		current_result.hardWeight = discrete_result.totalWeight;

		current_result.totalWeight = discrete_result.totalWeight + continuous_result.totalWeight;
		current_result.totalValue = discrete_result.totalValue + continuous_result.totalValue;

		/*cout << "**soft_bandwidth = " << uavs[uav_id].soft_bandwidth << "\thard_bandwidth = " << uavs[uav_id].hard_bandwidth << '\n';
		cout << "**Discrete Part: totalWeight = " << discrete_result.totalWeight << "\ttotalValue = " << discrete_result.totalValue << "\n";
		cout << "**Continuous Part: totalWeight = " << continuous_result.totalWeight << "\ttotalValue = " << continuous_result.totalValue << "\n";
		cout << "**totalWeight = " << current_result.totalWeight << "\totalValue = " << current_result.totalValue << '\n';*/
		for (auto item_id : discrete_result.allocatedList)
		{
			current_result.allocatedList.push_back(item_id);
			current_result.allocatedBandwidth[item_id] = discrete_result.allocatedBandwidth[item_id];
		}
		for (auto item_id : continuous_result.allocatedList)
		{
			current_result.allocatedList.push_back(item_id);
			current_result.allocatedBandwidth[item_id] = continuous_result.allocatedBandwidth[item_id];
		}

		result_list.insert({ continuous_ratio, current_result });

		if (current_result.totalValue > result_list[max_utility_ratio].totalValue)
			max_utility_ratio = continuous_ratio;

		continuous_ratio -= ratio_step;
	}
	//cout << "uav_id = " << uav_id << "\tmax_utility_ratio = " << max_utility_ratio << '\n';

	return result_list;
}

map<double, KnapsackResult> BAProblem::RP_based_subproblem_allocation_experiment2(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{
	Uav uav = sysModel.uavs[uav_id];
	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0; // 初始化
	uav.soft_bandwidth = 0;

	vector<User> unproc_hard_users;
	vector<User> unproc_soft_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_soft_users.push_back(user);

	double hard_bandwidth = capacity;

	map<double, KnapsackResult> result_list;
	double max_utility_bandwidth = hard_bandwidth; // 效用最大时，对应的硬资源量
	int count = 0;	 // 计数器，防止死循环

	while (hard_bandwidth > 0)
	{
		uav.hard_bandwidth = hard_bandwidth;
		uav.soft_bandwidth = uav.total_bandwidth - hard_bandwidth;
		cout << "Iteration " << count << ": hard_bandwidth = " << hard_bandwidth << "\tsoft_bandwidth = " << uav.soft_bandwidth << '\n';

		// 1. 离散部分：0-1背包问题

		KnapsackResult discrete_result = Fptas01Knapsack(uav, unproc_hard_users, uti_max);
		// 输出离散部分的结果
		/*cout << "**Discrete Part: totalWeight = " << discrete_result.totalWeight << "\ttotalValue = " << discrete_result.totalValue << "\n";
		cout << "Allocated Users in Discrete Part: ";
		for (auto entry : discrete_result.allocatedList)
		{
			cout << " user_id=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		// 离散部分剩余的资源分配给连续部分
		double remain_resource = uav.hard_bandwidth - discrete_result.totalWeight;
		uav.soft_bandwidth += remain_resource;
		// cout << "remain resource: " << remain_resource << '\n';

		// 2. 连续部分：KKT条件
		KnapsackResult continuous_result = KktBasedElasticUtility(uav, unproc_soft_users, uti_max);
		// 输出连续部分的结果
		/*cout << "**Continuous Part: totalWeight = " << continuous_result.totalWeight << "\ttotalValue = " << continuous_result.totalValue << "\n";
		cout << "Allocated Users in Continuous Part: ";
		for (auto entry : continuous_result.allocatedList)
		{
			cout << " user_id=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		// 3. 合并结果
		KnapsackResult current_result;
		current_result.softValue = continuous_result.totalValue;
		current_result.hardValue = discrete_result.totalValue;
		current_result.softWeight = continuous_result.totalWeight;
		current_result.hardWeight = discrete_result.totalWeight;

		current_result.totalWeight = discrete_result.totalWeight + continuous_result.totalWeight;
		current_result.totalValue = discrete_result.totalValue + continuous_result.totalValue;
		for (auto item_id : discrete_result.allocatedList)
		{
			current_result.allocatedList.push_back(item_id);
			current_result.allocatedBandwidth[item_id] = discrete_result.allocatedBandwidth[item_id];
		}
		for (auto item_id : continuous_result.allocatedList)
		{
			current_result.allocatedList.push_back(item_id);
			current_result.allocatedBandwidth[item_id] = continuous_result.allocatedBandwidth[item_id];
		}

		result_list.insert({ hard_bandwidth, current_result });

		if (current_result.totalValue > result_list[max_utility_bandwidth].totalValue)
			max_utility_bandwidth = hard_bandwidth;

		// 更新hard_bandwidth
		hard_bandwidth = discrete_result.totalWeight - delta;
		count++;
	}
	//cout << "uav_id = " << uav_id << "\tmax_utility_ratio = " << max_utility_ratio << '\n';

	return result_list;
}

KnapsackResult BAProblem::RP_based_subproblem_allocation(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{
	Uav uav = sysModel.uavs[uav_id];

	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0; // 初始化
	uav.soft_bandwidth = 0;
	KnapsackResult max_result;

	int count = 0;	 // 计数器，防止死循环

	vector<User> unproc_hard_users;
	vector<User> unproc_soft_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_soft_users.push_back(user);

	/*cout << "***\t in BAProblem::RP_based_subproblem_allocation ***\n";
	cout << "\t unproc_users num = " << unproc_users.size() << endl;*/

	double hard_bandwidth = capacity;
	while (hard_bandwidth > 0)
	{
		uav.hard_bandwidth = hard_bandwidth;
		uav.soft_bandwidth = uav.total_bandwidth - uav.hard_bandwidth;
		//cout << "Iteration " << count << ": hard_bandwidth = " << hard_bandwidth << "\tsoft_bandwidth = " << uav.soft_bandwidth << '\n';

		// 1. 离散部分：0-1背包问题

		KnapsackResult discrete_result = Fptas01Knapsack(uav, unproc_hard_users, uti_max);
		// 输出离散部分的结果
		/*cout << "\t\t unproc_hard_users num = " << unproc_hard_users.size() << endl;
		cout << "\t\t**Discrete Part: totalWeight = " << discrete_result.totalWeight << "\ttotalValue = " << discrete_result.totalValue << "\n";
		cout << "\t\tAllocated Users in Discrete Part: ";
		for (auto entry : discrete_result.allocatedList)
		{
			cout << "\t\t\tuser_id=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		// 离散部分剩余的资源分配给连续部分
		double remain_resource = uav.hard_bandwidth - discrete_result.totalWeight;
		uav.soft_bandwidth += remain_resource;
		// cout << "remain resource: " << remain_resource << '\n';

		// 2. 连续部分：KKT条件
		KnapsackResult continuous_result = KktBasedElasticUtility(uav, unproc_soft_users, uti_max);
		// 输出连续部分的结果
		/*cout << "**Continuous Part: totalWeight = " << continuous_result.totalWeight << "\ttotalValue = " << continuous_result.totalValue << "\n";
		cout << "Allocated Users in Continuous Part: ";
		for (auto entry : continuous_result.allocatedList)
		{
			cout << " user_id=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		double total_value = discrete_result.totalValue + continuous_result.totalValue;
		if (total_value > max_result.totalValue)
		{
			// 3. 合并结果
			KnapsackResult current_result;
			current_result.softValue = continuous_result.totalValue;
			current_result.hardValue = discrete_result.totalValue;
			current_result.softWeight = continuous_result.totalWeight;
			current_result.hardWeight = discrete_result.totalWeight;

			current_result.totalWeight = discrete_result.totalWeight + continuous_result.totalWeight;
			current_result.totalValue = discrete_result.totalValue + continuous_result.totalValue;

			for (auto item_id : discrete_result.allocatedList)
			{
				current_result.allocatedList.push_back(item_id);
				current_result.allocatedBandwidth[item_id] = discrete_result.allocatedBandwidth[item_id];
				current_result.allocatedValue[item_id] = discrete_result.allocatedValue[item_id];
			}
			for (auto item_id : continuous_result.allocatedList)
			{
				current_result.allocatedList.push_back(item_id);
				current_result.allocatedBandwidth[item_id] = continuous_result.allocatedBandwidth[item_id];
				current_result.allocatedValue[item_id] = continuous_result.allocatedValue[item_id];
			}

			max_result = current_result;

		}
	

		// 更新hard_bandwidth
		hard_bandwidth = discrete_result.totalWeight - delta;
		count++;
	}
	//cout << "uav_id = " << uav_id << "\tmax_utility_ratio = " << max_utility_ratio << '\n';
	return max_result;

}

KnapsackResult BAProblem::Fptas01Knapsack(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max)
{
	//cout << "\t\t\t\t in BAProblem::FPTAS_0_1_knapsack \n";

	// 参考文献：A Fast Polynomial-Time Approximation Scheme for the 0-1 Knapsack Problem
	// 该算法是一个针对0-1背包问题的快速多项式时间近似方案
	// 输入：物品的价值和重量，背包的容量，误差参数ε
	// 输出：近似最优解
	int uav_id = uav.ID;
	double capacity = uav.hard_bandwidth;

	vector<User>& hard_users = unproc_users;
	vector<vector<double>>& cap_list = sysModel.cap_list;

	// 获取当前UAV对应的用户列表
	vector<int> user_indices;
	vector<double> weights;
	vector<double> values;


	for (auto& user : hard_users)
	{
		int user_id = user.ID;
		double cap = cap_list[uav_id][user_id];
		
		if (cap > 0)
		{
			double rMin = user.rMin;
			double bandwidth = rMin / cap; // 用户j的最小带宽需求
			weights.push_back(bandwidth); // 物品的重量：用户j的最小速率需求转换为带宽需求

			// utility_funcs[user_id] 是用户user_id的效用函数指针
			// 在多背包问题中，用户若已被其它UAV分配资源，则其效用函数需调整为边际效用函数
			double mutility = user.marginal_utility(bandwidth, cap, uti_max[user_id]);

			values.push_back(mutility); // 物品的价值：用户j的权重
			user_indices.push_back(user_id);
		}
	}

	// uid = user_indices[i] 对应 weights[i], values[i]
	// 输出所有信息
	/*for (size_t i = 0; i < weights.size(); i++)
	{
		cout << "User ID: " << user_indices[i] << "\tWeight: " << weights[i] << "\tValue: " << values[i] << '\n';
	}*/

	int n = weights.size();
	KnapsackResult knapsack_result;
	// 检查变量n和capacity
	// 输出n和capacity
    // cout << "\t\t\t\tn = " << n << "\tcapacity = " << capacity << '\n';
	if (n == 0 || capacity <= 0) return knapsack_result;

	// 1. 计算缩放因子K
	int v_max = *max_element(values.begin(), values.end());
	double K = max(1.0, (epsilon * v_max) / n); // 避免K=0

	// 2. 缩放价值并计算新价值总和
	vector<int> scaled_values(n);
	int scaled_V = 0;
	for (int i = 0; i < n; i++) {
		scaled_values[i] = (int)floor(values[i] / K);
		scaled_V += scaled_values[i];
	}

	if (scaled_V == 0) {
		// 所有物品缩放价值为0，无法通过DP区分，返回空结果
		return knapsack_result;
	}

	// 3. 动态规划求解：dp[v] = 达到价值v的最小重量（使用double避免截断误差）
	constexpr double INF = std::numeric_limits<double>::infinity();
	vector<double> dp(scaled_V + 1, INF);
	// selected[i][v] 表示在处理到第 i 件物品时，是否选择第 i 件以达到价值 v
	vector<vector<char>> selected(n, vector<char>(scaled_V + 1, 0));
	dp[0] = 0.0;

	// 填充DP表并记录选择决策
	for (int i = 0; i < n; i++) {
		int sv = scaled_values[i];
		if (sv <= 0) continue; // 缩放价值为0的物品对DP无影响，可跳过
		// 遍历时从高到低，保证每件物品只使用一次
		for (int v = scaled_V; v >= sv; v--) {
			if (dp[v - sv] != INF) {
				double new_weight = dp[v - sv] + weights[i];
				// 仅当新的重量在容量范围内并且更优时更新
				if (new_weight <= capacity && new_weight < dp[v]) {
					dp[v] = new_weight;
					selected[i][v] = 1; // 标记选择当前物品
				}
			}
		}
	}

	// 4. 找到缩放价值最大的有效解
	int max_scaled_value = 0;
	for (int v = scaled_V; v >= 0; v--) {
		if (dp[v] <= capacity) {
			max_scaled_value = v;
			break;
		}
	}

	// 5. 回溯找出选择的物品
	int current_value = max_scaled_value;
	for (int i = n - 1; i >= 0 && current_value > 0; i--) {
		int sv = scaled_values[i];
		if (sv <= 0) continue;
		if (current_value >= sv && selected[i][current_value]) {
			// 选择了物品 i
			knapsack_result.allocatedList.push_back(i); // 记录物品的重量
			knapsack_result.allocatedBandwidth.insert({ i, weights[i] });
			knapsack_result.allocatedValue.insert({ i, values[i] });
			knapsack_result.totalWeight += weights[i];
			knapsack_result.totalValue += values[i];
			current_value -= sv;
		}
	}

	// 最后做一次安全检查，确保总重量不超过容量
	if (knapsack_result.totalWeight > capacity + 1e-9) {
		// 如果由于数值误差超过了容量，则移除一些物品直到符合容量限制
		// 以最小价值密度优先移除（value/weight 最小）
		vector<pair<int, double>> items;
		for (auto& idx : knapsack_result.allocatedList) {
			double w = knapsack_result.allocatedBandwidth[idx];
			double v = knapsack_result.allocatedValue[idx];
			double density = (v / (w > 0 ? w : 1e-12));
			items.emplace_back(idx, density);
		}
		// 按密度升序移除低密度物品（对总价值影响最小）
		sort(items.begin(), items.end(), [](const pair<int, double>& a, const pair<int, double>& b) { return a.second < b.second; });
		for (auto& it : items) {
			if (knapsack_result.totalWeight <= capacity + 1e-9) break;
			int idx = it.first;
			// remove
			knapsack_result.totalWeight -= weights[idx];
			knapsack_result.totalValue -= values[idx];

			bool is_removed = remove_first(knapsack_result.allocatedList, idx);
		}
	}

	KnapsackResult result;
	// 将物品索引转换回用户ID
	for (auto item_idx : knapsack_result.allocatedList) {
		int user_id = user_indices[item_idx];
		double weight = knapsack_result.allocatedBandwidth[item_idx];
		double value = knapsack_result.allocatedValue[item_idx];
		result.allocatedList.push_back(user_id);
		result.allocatedBandwidth.insert({ user_id, weight });
		result.allocatedValue.insert({ user_id, value });
	}
	if (result.allocatedList.size() != knapsack_result.allocatedList.size())
	{
		cout << "Error in BAProblem::FPTAS_0_1_knapsack: allocatedList size mismatch!\n";
	}
	// 重新计算总重量和总价值，确保准确
	result.hardWeight = knapsack_result.totalWeight;
	result.hardValue = knapsack_result.totalValue;
	result.totalWeight = knapsack_result.totalWeight;
	result.totalValue = knapsack_result.totalValue;

	/*cout << "\t\t\t\t in BAProblem::FPTAS_0_1_knapsack \n";
	print_KnapsackResult(result, uav_id);*/

	return result;
}

KnapsackResult BAProblem::KktBasedElasticUtility(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max)
{
	// 根据obsidian笔记"000-工作日志-讨论日志-9月 对于连续部分：KKT条件找最优解"
	// 当中推导出了最优拉格朗日乘子λ的表达式
	int uav_id = uav.ID;
	double capacity = uav.hard_bandwidth;
	vector<User>& allusers = sysModel.users;
	vector<vector<double>>& cap_list = sysModel.cap_list;
	int n1 = sysModel.n1;
	int n2 = sysModel.n2;

	double lambda = 0.0;
	double sum_S = 0;
	double sum_C = 0;

	if (capacity <= 0)
	{
		KnapsackResult empty_result;
		return empty_result;
	}

	

	map<int, double> allocated_users; // 记录被分配带宽的{用户ID, 带宽}
	// 初始化allocated_users，假设所有用户都被分配带宽
	for (auto& user : unproc_users)
	{
		int user_id = user.ID;
		if (cap_list[uav_id][user_id] < 1e-10)
			continue; // 避免除以0

		allocated_users.insert({ user_id, 0 });
	}

	int is_alloc = 0;	// 标记是否完成分配
	while (!is_alloc)
	{
		for (auto entry : allocated_users)
		{
			int user_id = entry.first;
			User& u = allusers[user_id];
			sum_S += u.weight / log(u.rMin + 1);
			sum_C += 1 / cap_list[uav_id][user_id];
		}

		// 计算λ的值
		lambda = sum_S / (uav.soft_bandwidth + sum_C);

		// 记录最小的带宽值，初始化为无穷大
		double min_bandwidth = numeric_limits<double>::max();
		int min_band_user = -1; // 记录最小带宽对应的用户ID

		// 

		for (auto entry : allocated_users)
		{
			int user_id = entry.first;
			User& u = allusers[user_id];
			
			double Sij = u.weight / log(u.rMin + 1);
			double Cij = 1 / cap_list[uav_id][user_id];
			double bandwidth_ij = (Sij / lambda - Cij);
			if (bandwidth_ij < min_bandwidth)
			{
				min_bandwidth = bandwidth_ij;
				min_band_user = user_id;
			}
			allocated_users[user_id] = bandwidth_ij;
			//cout << "j = " << j << "\tSij = " << Sij << "\tSij/lambda = " << Sij / lambda << "\tCij = " << Cij << "\tbandwidth_ij = " << bandwidth_ij << '\n';
		}

		if (min_bandwidth < 0)
		{
			// 将该用户从allocated_users中移除
			allocated_users.erase(min_band_user);
			// 重新计算sum_S和sum_C
			sum_S = 0;
			sum_C = 0;
			//cout << "User " << min_user << " removed from allocation due to negative bandwidth." << '\n';
		}
		else
		{
			is_alloc = 1; // 分配完成
		}
	}

	// 根据λ的值计算每个用户的分配带宽
	KnapsackResult alloc_result;
	for (auto entry : allocated_users)
	{
		int user_id = entry.first;
		double bandwidth_ij = entry.second;
		double value = allusers[user_id].marginal_utility(bandwidth_ij, cap_list[uav_id][user_id], uti_max[user_id]);

		alloc_result.allocatedList.push_back(user_id);
		alloc_result.allocatedBandwidth.insert({ user_id, bandwidth_ij });
		alloc_result.allocatedValue.insert({ user_id, value });

		alloc_result.totalWeight += bandwidth_ij;
		// 在多背包问题中，用户若已被其它UAV分配资源，则其效用函数需调整为边际效用函数 
		alloc_result.totalValue += value;
		// cout << "Final allocation - User " << j << ": Bandwidth = " << bandwidth_ij << ", Utility = " << users[j].elastic_utility(bandwidth_ij, cap_list[uav_id][j]) << '\n';
	}

	return alloc_result;
}


KnapsackResult BAProblem::PeggingAlgorithm(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max)
{
	KnapsackResult result;
	// 实现Pegging算法的具体步骤

	int uav_id = uav.ID;
	double capacity = uav.hard_bandwidth;
	vector<User>& allusers = sysModel.users;
	vector<vector<double>>& cap_list = sysModel.cap_list;
	int n1 = sysModel.n1;
	int n2 = sysModel.n2;





	return result;
}


KnapsackResult BAProblem::CplexBasedElasticUtility(int uav_id)
{
	vector<Uav>& uavs = sysModel.uavs;
	vector<User>& users = sysModel.users;
	vector<vector<double>>& cap_list = sysModel.cap_list;
	int n1 = sysModel.n1;
	int n2 = sysModel.n2;

	// 尝试用 CPLEX 求解；若提取/求解失败则回退到 KKT 解析解（更稳健）
	KnapsackResult alloc_result;
	IloEnv env;
	try {
		IloModel model(env);
		// 构造变量（使用 n2 长度的变量数组），确保用同一个 env
		IloNumVarArray bij(env, n2, 0.0, IloInfinity, ILOFLOAT);

		// 约束：总带宽
		IloExpr constraint_expr(env);
		for (IloInt j = 0; j < n2; j++) constraint_expr += bij[j];
		model.add(constraint_expr <= uavs[uav_id].soft_bandwidth);
		constraint_expr.end();

		// 构造目标（注意：IloLog 是非线性，可能在某些 CPLEX 安装下无法提取）
		IloExpr objective_expr(env);
		for (IloInt j = n1; j < n1 + n2; j++) {
			User& u = users[j];
			double c_ij = cap_list[uav_id][j];
			objective_expr += (u.weight * IloLog(1 + bij[j - n1] * c_ij)) / log(1 + u.rMin);
		}
		model.add(IloMaximize(env, objective_expr));
		objective_expr.end();

		// 构造求解器（这里可能抛出“cannot extract extractables...”）
		IloCplex cplex(model);
		if (!cplex.solve()) {
			env.error() << "Failed to optimize (CPLEX returned no solution)." << std::endl;
			env.end();
			// 回退
			return alloc_result;
		}

		// 读取解
		for (int user_id = n1; user_id < n1 + n2; ++user_id) {
			double bandwidth_ij = cplex.getValue(bij[user_id - n1]);
			alloc_result.allocatedList.push_back(user_id);
			alloc_result.allocatedBandwidth[user_id] = bandwidth_ij;
			alloc_result.allocatedValue[user_id] = users[user_id].elastic_utility(bandwidth_ij, cap_list[uav_id][user_id]);
			alloc_result.totalWeight += bandwidth_ij;
			alloc_result.totalValue += alloc_result.allocatedValue[user_id];
		}
		env.end();
		return alloc_result;
	}
	catch (IloException& e) {
		std::cerr << "IloException: " << e.getMessage() << std::endl;
		// 清理 env 并回退到解析解
		env.end();
		return alloc_result;
	}
	catch (...) {
		std::cerr << "Unknown exception in CPLEX_based_elastic_utility" << std::endl;
		env.end();
		return alloc_result;
	}
}

void BAProblem::PrintKnapsackResult(KnapsackResult& knapsackResult, int uav_id)
{
	if (uav_id!=-1)
        cout << "---------------UAV " << uav_id << "--------------- " << endl;
	else
		cout << "------------------------------------" << endl;


    cout << "Total Value: " << knapsackResult.totalValue << endl;
    cout << "Total Weight: " << knapsackResult.totalWeight << endl;
	cout << "Soft Value: " << knapsackResult.softValue << endl;
	cout << "Soft Weight: " << knapsackResult.softWeight << endl;
	cout << "Hard Value: " << knapsackResult.hardValue << endl;
	cout << "Hard Weight: " << knapsackResult.hardWeight << endl;
    cout << "Allocated List: " << endl;
    for (const auto user_id : knapsackResult.allocatedList) {
        cout << "User ID: " << user_id << ", Bandwidth: " << knapsackResult.allocatedBandwidth[user_id] << ", Utility: " << knapsackResult.allocatedValue[user_id] << endl;
    }
    cout << endl;
    

}

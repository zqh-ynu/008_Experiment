#include "EntityDefinition.h"



// 辅助函数：分割字符串
vector<string> split(const string& s, char delimiter) {
	vector<string> tokens;
	string token;
	istringstream tokenStream(s);
	while (getline(tokenStream, token, delimiter)) {
		tokens.push_back(token);
	}
	return tokens;
}



void clean_KnapsackResult(KnapsackResult& result)
{
	result.allocatedList.clear();
	result.allocatedBandwidth.clear();
	result.allocatedValue.clear();
	result.totalValue = 0;
	result.totalWeight = 0;
	result.elasticValue = 0;
	result.hardValue = 0;
	result.elasticWeight = 0;
	result.hardWeight = 0;
}

void add_KnapsackResult(KnapsackResult& result, User& user, double bandwidth, double value)
{
	int user_id = user.ID;
	result.allocatedList.push_back(user_id);
	result.allocatedBandwidth.insert({ user_id, bandwidth });
	result.allocatedValue.insert({ user_id, value });
	result.totalValue += value;
	result.totalWeight += bandwidth;
	if (user.uType == HARD_UTILITY)
	{
		result.hardValue += value;
		result.hardWeight += bandwidth;
	}
	else
	{
		result.elasticValue += value;
		result.elasticWeight += bandwidth;
	}
}

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
	double data_rate = bandwidth_ * capacity_;
	if (data_rate >= rMin - EPS)
	{
		double uti = weight * log2(1 + rMin);
		return uti;
	}
	else
		return 0.0;
}

double User::hard_utility(double outage_) const
{
	if (outage_ <= pOut)
		return weight * log2(1 + rMin);
	else
		return 0.0;
}

double User::elastic_utility(double bandwidth_, double capacity_) const
{
	double r = bandwidth_ * capacity_;
	// 参数p的选取会影响函数的增长速度，p越大，函数增长越快
	// 备注：当r < rMin时，效用为0；当r >= rMin时，
	// 具体定义参见tanUtilityMaximizationResource2015
	/*double v = 0.2;
	if(r < rMin)
		return 0.0;
	else
		return weight * (1 - exp(-v * (r - rMin)));*/

		// 定义2：参见shiNetworkUtilityMaximization2008 公式(4)
		// user(r) = w * log(1 + r) / log(1 + rMin)
		// return weight * log(1 + r) / log(1 + rMin);
	return weight * log2(1 + r);
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

double User::utility_derivative(double bandwidth_, double capacity_)
{
	if (uType == HARD_UTILITY)
	{
		// 对于hard用户，参数capacity表示其需求的带宽阈值
		double uti = weight * log2(1 + rMin);
		double B_th = capacity_;
		if (B_th > 1e-9)
		{
			return uti / B_th;
		}
		return INF;
	}
	else
	{
		// 对于elastic用户，参数capacity表示信道容量，即log(1+SNR)
		return (weight * capacity_) / ((bandwidth_ * capacity_ + 1) * log(2));
	}
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
	double P_LoS_ = pow(1 + param_a * exp(-param_b * (theta - param_a)), -1);

	// 打印上述涉及到的所有变量
	/*cout << "param_a = " << param_a << ", param_b = " << param_b << ", theta = " << theta << endl;*/

	// 将P_LoS保留3位小数
	// P_LoS_ = ((int)(P_LoS_ * 1000)) / 1000.0;
	// cout << "P_LOS = " << P_LoS_ << '\n';
	return P_LoS_;
}

double Channel::cal_L_LoS() const
{
	// in dB
	double L_LoS_ = 20 * log10((4 * pi * freq_hz * d) / speed_light) + los_loss_db;

	// 打印上述涉及到的所有变量
	/*cout << "freq_hz = " << freq_hz << ", d = " << d << ", speed_light = " << speed_light << ", los_loss_db = " << los_loss_db << endl;*/
	return L_LoS_;
}

double Channel::cal_L_NLoS() const
{
	double L_NLoS_ = 20 * log10((4 * pi * freq_hz * d) / speed_light) + nlos_loss_db;
	return L_NLoS_;
}

double Channel::cal_PL() const
{
	// 将dB转换为线性值计算
	double L_LoS_ = pow(10, L_LoS / 10.0);
	double L_NLoS_ = pow(10, L_NLoS / 10.0);
	double PL_ = P_LoS * L_LoS_ + P_NLoS * L_NLoS_;

	// 打印上述涉及到的所有变量
	/*cout << "L_LoS_ = " << L_LoS_ << ", L_NLoS_ = " << L_NLoS_ << endl;
	cout << "P_LoS = " << P_LoS << ", P_NLoS = " << P_NLoS << endl;*/

	// 将PL转换为dB值
	PL_ = 10 * log10(PL_);
	return PL_;
}

double Channel::cal_average_SNR() const
{
	// 将各个变量转换为线性值计算
	double P_tr_linear = P_tr.pTrans * 1000; // mW
	double P_N_linear = pow(10, noise_dbm / 10.0); // mW
	double g_UAV_linear = pow(10, gain_uav_db / 10.0);
	double PL_linear = pow(10, PL / 10.0);
	double SNR_linear = (P_tr_linear * g_UAV_linear) / (P_N_linear * PL_linear);
	double SNR_ = 10 * log10(SNR_linear);

	// 打印上述涉及到的所有参数
	/*cout << "P_tr_linear = " << P_tr.pTrans << " W, P_N_linear = " << noise_dbm << " W, g_UAV_linear = " << gain_uav_db << ", PL_linear = " << PL << endl;
	cout << "SNR_linear = " << SNR_linear << endl;
	cout << "SNRa_dB = " << SNR_ << " dB" << endl;*/
	// cout << "SNR_ = " << SNR_ << endl;
	return SNR_;
}

double Channel::cal_h() const
{
	double g_ = 0;
	/*if (P_tr.Type == IBS)
		g_ = pow(10, gain_bs_db / 10.0);
	else
		g_ = pow(10, gain_uav_db / 10.0);*/
	g_ = pow(10, gain_uav_db / 10.0);
	double h_ = PL * g_;
	// cout << "h_ = " << h_ << endl;
	return h_;
}

double Channel::cal_eta() const
{
	double eta_ = P_LoS * los_loss_db + P_NLoS * nlos_loss_db;
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
	double SNR_ = (P_tr.pTrans * pow(g, 2)) / pow(10, noise_dbm / 10);
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
	// 文档地址：https://www.boost.org/doc/libs/1_64_0/libs/math/doc/html/math_toolkit/sf_gamma/jgamma.html
	double p_test = boost::math::gamma_p(M, (M * SNR_th_test) / SNR_avg_linear);

	if (p_test > P_re.pOut)
	{
		/*cout << "Error: uav_id = " << P_tr.ID << ", user_id_J1 = " << P_re.ID << ", \t" << endl;
		cout << "The user outage probability requirement cannot be met even if all the min_band is allocated to it!" << endl;
		*/// 返回负无穷
		// return -INFINITY;
	}
	try {
		// boost::math::gamma_p_inv 是下不完全伽马函数的归一化形式的反函数
		// 文档地址：https://www.boost.org/doc/libs/1_64_0/libs/math/doc/html/math_toolkit/sf_gamma/igamma_inv.html
		double root = boost::math::gamma_p_inv(M, P_re.pOut) * (SNR_avg_linear / M);

		/*cout << "uav_id = " << P_tr.ID << ", user_id_J1 = " << P_re.ID << ", \t" << endl;
		cout << "SNRa_dB = " << SNRa_dB << ", SNR_avg_linear = " << SNR_avg_linear << ", p_out_target = " << P_re.pOut << ", snr_th_linear = " << root << endl;*/

		// 将根转换为 dB 值
		root = 10 * log10(root);


		//// 输出求解结果
		//std::cout << "根的区间为： [" << result.first << ", " << result.second << "]" << std::endl;
		//std::cout << "根的近似值为： " << root << std::endl;

		//// 输出方程在根处的值，理论上应该接近于零
		//std::cout << "在根处的函数值： " << freq_hz(root) << std::endl;

		//// 输出迭代次数
		//std::cout << "迭代次数： " << max_iter << std::endl;

		return root;
	}
	catch (const std::exception& e) {
		// 捕获异常并输出错误信息
		cout << "uav_id = " << P_tr.ID << ", user_id_J1 = " << P_re.ID << endl;
		std::cerr << "错误： " << e.what() << std::endl;
	}
	return 0.0;
}

double Channel::cal_capacity() const
{
	if (P_re.uType == HARD_UTILITY)
	{
		// 计算信道容量
		if (SNRt_dB == -INFINITY)
			return 0.0;
		double SNR_th_linear = pow(10, SNRt_dB / 10.0);
		double capacity_ = (1 - P_re.pOut) * log2(1 + SNR_th_linear);
		return capacity_;
	}
	else if (P_re.uType == ELASTIC_UTILITY)
	{
		double SNRa_linear = pow(10, SNRa_dB / 10.0);
		double capacity_ = log2(1 + SNRa_linear);
		return capacity_;
	}
	return 0;
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

	init_SystemModel();
}

// 辅助结构体，用于暂存读取的数据以便进行坐标转换
struct RawUserData {
	double lon, lat;
	string type;
	double weight;
	double req1; // rMin (Kbps)
	double req2; // pOut or ignored
};

struct RawUavData {
	double lon, lat;
	double bandwidth; // MHz
};
SystemMd::SystemMd(string user_file, string uav_file, string config_file)
{
	// 1. 加载全局配置
	load_global_channel_config(config_file);

	// 2. 读取 User 文件
	vector<RawUserData> raw_users;
	ifstream user_fs(user_file);
	if (!user_fs.is_open()) {
		cerr << "Error: Cannot open user file " << user_file << endl;
		exit(1);
	}

	string line;
	// 跳过表头
	getline(user_fs, line);
	while (getline(user_fs, line)) {
		if (line.empty()) continue;
		vector<string> tokens = split(line, ',');
		if (tokens.size() < 8) continue;

		RawUserData d;
		// token[0] 是 id，忽略
		d.lon = stod(tokens[1]);
		d.lat = stod(tokens[2]);
		d.type = tokens[3];
		d.weight = stod(tokens[4]);
		d.req1 = stod(tokens[5]);
		d.req2 = stod(tokens[6]);
		// token[7] 是 app_category，忽略

		raw_users.push_back(d);
	}
	user_fs.close();

	// 3. 读取 UAV 文件
	vector<RawUavData> raw_uavs;
	ifstream uav_fs(uav_file);
	if (!uav_fs.is_open()) {
		cerr << "Error: Cannot open uav file " << uav_file << endl;
		exit(1);
	}

	// 跳过表头
	getline(uav_fs, line);
	while (getline(uav_fs, line)) {
		if (line.empty()) continue;
		vector<string> tokens = split(line, ',');
		if (tokens.size() < 4) continue;

		RawUavData d;
		// token[0] 是 id，忽略
		d.lon = stod(tokens[1]);
		d.lat = stod(tokens[2]);
		d.bandwidth = stod(tokens[3]);

		raw_uavs.push_back(d);
	}
	uav_fs.close();

	// 4. 计算坐标原点 (最小经纬度)
	double min_lon = 180.0;
	double min_lat = 90.0;

	for (const auto& u : raw_users) {
		if (u.lon < min_lon) min_lon = u.lon;
		if (u.lat < min_lat) min_lat = u.lat;
	}
	for (const auto& u : raw_uavs) {
		if (u.lon < min_lon) min_lon = u.lon;
		if (u.lat < min_lat) min_lat = u.lat;
	}
	// cout << "min_lon = " << min_lon << ", min_lat = " << min_lat << endl;

	// 5. 坐标转换系数 (简单的等距投影，适用于小范围)
	// 纬度 1度 = pi * R / 180 米
	// 经度 1度 = pi * R * cos(lat) / 180 米
	double lat_to_meter = pi * EARTH_RADIUS / 180.0;
	double lon_to_meter = pi * EARTH_RADIUS * cos(min_lat * pi / 180.0) / 180.0;

	// 6. 生成 User 对象
	vector<User> hard_users;
	vector<User> elastic_users;
	int user_id_counter = 0;
	for (const auto& raw : raw_users) {
		double x = (raw.lon - min_lon) * lon_to_meter;
		double y = (raw.lat - min_lat) * lat_to_meter;
		double z = 0.0; // 用户默认为地面 0

		int type = (raw.type == "hard") ? HARD_UTILITY : ELASTIC_UTILITY;
		// 权重取整? User构造函数里weight是int，但文件里是double(0.606)。
		// 建议修改User类支持double权重，或者这里暂时放大/取整。
		// 根据User类定义 `int weight`，这里只能取整或者修改User类。
		// 为了保持逻辑，这里暂时强制转int，建议后续优化User类支持double权重。
		// *修正*：为了保持精度，这里假设User类未修改，若权重非常小(0.xxx)，int会变成0。
		// 假设User类的weight应该改为double。如果不能改，这里建议 * 10 或者向上取整。
		// 鉴于上下文是PhD研究，通常权重是相对值，这里暂时使用 ceil 确保不为0，或者假设User类已改为double。
		// 这里按照 int 转换 (如果User::weight是int):
		double w = raw.weight;
		// 如果User::weight已经改为了double (推荐)，则直接传 raw.weight。
		// 假设这里按照代码现状 int 处理。

		// 速率转换：Kbps -> Mbps (因为带宽是MHz，对应速率单位通常是Mbps)
		double rMin_mbps = raw.req1 / 1000.0;

		double pOut = 0.0;
		if (type == HARD_UTILITY) {
			pOut = raw.req2;
			hard_users.emplace_back(user_id_counter++, type, w, x, y, z, rMin_mbps, pOut);
		}
		else {
			// Elastic 用户也有 rMin (user_requirement_1), req2 忽略
			elastic_users.emplace_back(user_id_counter++, type, w, x, y, z, rMin_mbps, pOut);
		}
	}
	// 拼接两个vector
	this->users = hard_users;
	this->users.insert(this->users.end(), elastic_users.begin(), elastic_users.end());
	// 重设id
	user_id_counter = 0;
	for (auto& user : this->users) {
		user.ID = user_id_counter++;
	}

	// 7. 生成 UAV 对象
	this->uavs.clear();
	int uav_id_counter = 0;
	for (const auto& raw : raw_uavs) {
		double x = (raw.lon - min_lon) * lon_to_meter;
		double y = (raw.lat - min_lat) * lat_to_meter;

		// 带宽：文件中是 50 (MHz)。
		// 系统默认带宽往往是 20. 如果 config.h 或 predefine.h 有默认值，这里会覆盖。
		// 保持单位一致，内部计算使用 MHz，则存 50。
		double bw = raw.bandwidth;

		// Uav(int id_, double x1, double y1, double z1, int total_bandwidth_)
		// 注意 Uav 构造函数最后一个参数是 int，如果 bandwidth 是 double，这里也会截断。
		// 同样建议 Uav 类支持 double bandwidth。
		this->uavs.emplace_back(uav_id_counter++, x, y, uav_alt, bw);

		// 强行赋值以防构造函数参数类型限制
		this->uavs.back().pTrans = uav_trans_power;
	}

	// 8. 初始化统计变量和系统模型
	this->n1 = 0;
	this->n2 = 0;
	for (const auto& u : this->users) {
		if (u.uType == HARD_UTILITY) this->n1++;
		else this->n2++;
	}
	this->m = this->uavs.size();

	cout << "biaoji--" << endl;

	// 初始化信道矩阵
	init_SystemModel();

	cout << "System initialized from files." << endl;
	cout << "Origin (Min Lon, Min Lat): (" << min_lon << ", " << min_lat << ")" << endl;
	cout << "Users: " << users.size() << " (Hard: " << n1 << ", Elastic: " << n2 << ")" << endl;
	cout << "UAVs: " << uavs.size() << endl;
}

void SystemMd::init_SystemModel()
{
	dis_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	SNRave_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	SNRth_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	M_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	cap_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	Bth_list = vector<vector<double>>(m, vector<double>(n1, 0.0));

	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n1 + n2; j++)
		{
			Channel ch(uavs[i], users[j]);
			dis_list[i][j] = ch.d;
			SNRave_list[i][j] = ch.SNRa_dB;
			SNRth_list[i][j] = ch.SNRt_dB;
			M_list[i][j] = ch.M;
			cap_list[i][j] = ch.channel_capacity;
			// cout << "j=" << j << "\tj=" << j << "\tdis=" << dis_list[j][j] << "\tSNR=" << SNR_list[j][j] << "\tM=" << M_list[j][j] << "\tcap=" << cap_list[j][j] << '\n';

			if (dis_list[i][j] <= max_coverage_distance)
			{
				User user = users[j];
				uav_serviceable_users_map[i].push_back(user);
			}
		}

		for (int j = 0; j < n1; j++)
		{
			// 计算每个硬性用户的最小带宽需求
			double cap = cap_list[i][j];
			if (cap > 0)
				Bth_list[i][j] = users[j].rMin / cap;
			else
				Bth_list[i][j] = INFINITY;
		}
	}

	// 计算用户的最大权重
	for (auto& user : users)
	{
		if (max_user_utility < user.weight)
			max_user_utility = user.weight;
	}
}



void SystemMd::print_SystemInfo(int n) const
{
	if (n < 0)
		n = n1 + n2;
	else if (n > n1 + n2)
		n = n1 + n2;

	// 打印所有参数
	cout << "System Model Information:" << '\n';
	print_all_uavs();
	print_all_users(n);
	print_dis_list(n);
	print_SNRa_list(n);
	print_SNRt_list(n);
	print_M_list(n);
	print_cap_list(n);
	print_min_bw_list(n);

}

void SystemMd::print_all_users(int n) const
{
	cout << fixed << setprecision(5);
	cout << ".................all users...................." << '\n';
	for (auto& user : users)
	{
		if (user.ID > n) break;
		user.print_user();

	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_all_uavs() const
{
	cout << fixed << setprecision(5);
	cout << ".................all UAVs....................." << '\n';
	for (auto& uav : uavs)
	{
		uav.print_UAV();
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_dis_list(int n) const
{
	cout << fixed << setprecision(5);
	cout << ".................dis_list....................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cout << dis_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_SNRa_list(int n) const
{
	cout << fixed << setprecision(5);
	cout << ".................SNRave_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cout << SNRave_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_SNRt_list(int n) const
{
	cout << fixed << setprecision(5);
	cout << ".................SNRth_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cout << SNRth_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_M_list(int n) const
{
	cout << fixed << setprecision(5);
	cout << ".................M_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cout << M_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_cap_list(int n) const
{
	cout << fixed << setprecision(5);
	cout << ".................cap_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cout << cap_list[i][j] << "\t";
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_min_bw_list(int n) const
{
	if (n > n1)
		n = n1;
	cout << fixed << setprecision(5);
	cout << ".................Bth_list...................." << '\n';
	for (int i = 0; i < m; i++)
	{
		for (int j = 0; j < n; j++)
		{
			cout << Bth_list[i][j] << "\t";
			if (j % 10 == 9)
				cout << '\n';
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}
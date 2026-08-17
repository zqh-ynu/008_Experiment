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

double User::hard_utility(double bandwidth_, double capacity_, double SNR_avg_dB) const
{
	double data_rate = bandwidth_ * capacity_;
	if (data_rate >= rMin - EPS)
	{
		double uti = weight * log2(1 + rMin);
		//double uti = hard_utility_SNR_avg(bandwidth_, SNR_avg_dB);
		return uti;
	}
	else
		return 0.0;
}

double User::hard_utility_SNR_avg(double bandwidth_, double SNR_avg_dB) const
{
	double SNR_avg_linear = pow(10.0, SNR_avg_dB / 10.0);
	double data_rate = bandwidth_ * log2(1 + SNR_avg_linear);
	if (data_rate > EPS)
		return weight * log2(1 + data_rate);
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

double User::utility(double bandwidth_, double capacity_, double SNR_avg_dB)
{
	if (uType == HARD_UTILITY)
		return hard_utility(bandwidth_, capacity_, SNR_avg_dB);
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
	return 0.0;
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
		//std::cout << "根的区间为： [" << uav_result.first << ", " << uav_result.second << "]" << std::endl;
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

		double rMin_mbps = raw.req1 * unit_para;

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
		double bw = raw.bandwidth * unit_para;

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
	// cout << "uav num " << this->m << endl;

	// cout << "biaoji--" << endl;

	// 测试 放大hard用户的rmin需求
	for (int i = 0; i < n1; i++)
	{
		users[i].rMin *= 1;
	}

	// 初始化信道矩阵
	init_SystemModel();

	cout << "System initialized from files:" << endl;
	cout << "\t" << uav_file << endl;
	cout << "\t" << user_file << endl;
	// cout << "Origin (Min Lon, Min Lat): (" << min_lon << ", " << min_lat << ")" << endl;
	// cout << "Users: " << users.size() << " (Hard: " << n1 << ", Elastic: " << n2 << ")" << endl;
	// cout << "UAVs: " << uavs.size() << endl;
}

void SystemMd::init_SystemModel()
{

	dis_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	SNRave_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	SNRth_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	M_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	cap_list = vector<vector<double>>(m, vector<double>(n1 + n2, 0.0));
	Bth_list = vector<vector<double>>(m, vector<double>(n1, 0.0));

	int bandwidth_per_uav = 0;
	if (unit_para == 1)
	{
		bandwidth_per_uav = uavs[0].total_bandwidth * 1e6; // Hz
	}
	else
	{
		bandwidth_per_uav = uavs[0].total_bandwidth * 1e3; // Hz
	}


	noise_dbm = noise_dbm + 10 * log10(bandwidth_per_uav); // 调整噪声功率到指定带宽下

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
	cout << fixed << setprecision(3);
	cout << ".................dis_list....................." << '\n';

	for (int j = 0; j < n; j++)
	{
		cout << "user" << j << ":\t";
		for (int i = 0; i < m; i++)
		{
			cout << dis_list[i][j] << "\t";
			if (i % 10 == 9)
				cout << "\n\t";
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_SNRa_list(int n) const
{
	cout << fixed << setprecision(3);
	cout << ".................SNRave_list...................." << '\n';
	for (int j = 0; j < n; j++)
	{
		cout << "user" << j << ":\t";
		for (int i = 0; i < m; i++)
		{
			cout << SNRave_list[i][j] << "\t";
			if (i % 10 == 9)
				cout << "\n\t";
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_SNRt_list(int n) const
{
	cout << fixed << setprecision(3);
	cout << ".................SNRth_list...................." << '\n';
	for (int j = 0; j < n; j++)
	{
		cout << "user" << j << ":\t";
		for (int i = 0; i < m; i++)
		{
			cout << SNRth_list[i][j] << "\t";
			if (i % 10 == 9)
				cout << "\n\t";
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_M_list(int n) const
{
	cout << fixed << setprecision(3);
	cout << ".................M_list...................." << '\n';
	for (int j = 0; j < n; j++)
	{
		cout << "user" << j << ":\t";
		for (int i = 0; i < m; i++)
		{
			cout << M_list[i][j] << "\t";
			if (i % 10 == 9)
				cout << "\n\t";
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_cap_list(int n) const
{
	cout << fixed << setprecision(3);
	cout << ".................cap_list...................." << '\n';
	for (int j = 0; j < n; j++)
	{
		cout << "user" << j << ":\t";
		for (int i = 0; i < m; i++)
		{
			cout << cap_list[i][j] << "\t";
			if (i % 10 == 9)
				cout << "\n\t";
		}
		cout << '\n';
	}
	cout << ".............................................." << '\n';
}

void SystemMd::print_min_bw_list(int n) const
{
	if (n > n1)
		n = n1;
	cout << fixed << setprecision(3);
	cout << ".................Bth_list...................." << '\n';
	for (int j = 0; j < n; j++)
	{
		cout << "user" << j << ":\t";
		for (int i = 0; i < m; i++)
		{
			cout << Bth_list[i][j] << "\t";
			if (i % 10 == 9)
				cout << "\n\t";
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
	int elastic_user_num = sysModel.n2;
	vector<double> user_weights;

	vector<double> remaining_bandwidth;
	for (auto& uav : uavs)
	{
		remaining_bandwidth.push_back(uav.total_bandwidth);
	}

	// 用户新的效用函数 freq_hz(x) = freq_hz(x) - uti_max[j]
	// 用函数User::marginal_utility()计算
	// uti_max记录用户在当前分配情况下的最大效用值，初始化为0
	vector<double> uti_max = vector<double>(users.size(), 0.0);
	vector<KnapsackResult> allResult = vector<KnapsackResult>(uav_num);

	// userResult<uav_id, <min_band, value>> 记录用户对应的无人机及被分配的带宽
	vector<UserResult> user_results = vector<UserResult>(users.size());
	// 对userResult进行初始化
	for (int user_id = 0; user_id < users.size(); user_id++)
	{
		user_results[user_id].uav_id = -1;

		user_results[user_id].allocated_bandwidth = 0;
		user_results[user_id].utility = 0.0;
	}

	// 通过递归调用GAP来对所有UAV进行分配
	allResult[0] = GAP(0, uti_max, allResult);



	// 根据无人机的分配结果allResult得到用户的分配结果
	// 这一步是为了在后续步骤中更高效，注意要同步更新结果
	for (int uav_id = 0; uav_id < uav_num; uav_id++)
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
			KnapsackResult result = RP_based_subproblem_allocation_FPTAS(uav_id, remain_bw, unpro_users, uti_max);

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
			allResult[uav_id].hardWeight += result.hardWeight;

			allResult[uav_id].elasticValue += result.elasticValue;
			allResult[uav_id].elasticWeight += result.elasticWeight;

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
					double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
					double margin_utility = user.utility(allcated_band + remain_bw, cap_list[uav_id][user_id], SNR_avg_dB);

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
				allResult[uav_id].elasticWeight += remain_bw;
				allResult[uav_id].elasticValue += max_margin_utility;
				allResult[uav_id].totalWeight += remain_bw;
				allResult[uav_id].totalValue += max_margin_utility;

				user_results[max_user_id].allocated_bandwidth += remain_bw;
			}
		}

	}

	return { allResult, user_results };
}

KnapsackResult BAProblem::GAP(int uav_id, vector<double>& uti_max, vector<KnapsackResult>& allResult)
{
	// 输出uti_max
	cout << "uti_max: ";
	for (auto uti : uti_max)
		cout << uti << "\t";
	cout << endl;

	Uav& uav = sysModel.uavs[uav_id];
	KnapsackResult temp_result = RP_based_subproblem_allocation_FPTAS(uav_id, uav.total_bandwidth, sysModel.users, uti_max);

	// 检查参数
	cout << "BAProblem::GAP : \n";
	cout << "UAV " << uav_id << " : total_bandwidth_consume = " << uav.total_bandwidth << "\n";
	// 输出temp_result
	PrintKnapsackResult(temp_result, uav_id);

	// 更新用户的最大效用值
	for (auto user_id : temp_result.allocatedList)
	{
		double bw = temp_result.allocatedBandwidth[user_id];
		double cap = sysModel.cap_list[uav_id][user_id];
		double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
		double new_utility = sysModel.users[user_id].utility(bw, cap, SNR_avg_dB);
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
		for (int i = uav_id + 1; i < sysModel.m; i++)
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
		temp_result.elasticValue = 0.0;
		temp_result.hardWeight = 0.0;
		temp_result.elasticWeight = 0.0;
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
				temp_result.elasticWeight += bw;
				temp_result.elasticValue += value;
			}
		}
		temp_result.totalWeight = temp_result.hardWeight + temp_result.elasticWeight;
		temp_result.totalValue = temp_result.hardValue + temp_result.elasticValue;

		return temp_result;
	}
	else
		return temp_result;
}

map<int, UserResult> BAProblem::construct_user_results(const vector<KnapsackResult>& allResults)
{
	map<int, UserResult> user_results;
	for (auto& user : sysModel.users)
	{
		int user_id = user.ID;
		UserResult user_result;
		user_results[user_id] = user_result;
	}

	for (auto uavResult : allResults)
	{
		int uav_id = uavResult.uav_id;
		for (auto user_id : uavResult.allocatedList)
		{
			user_results[user_id].uav_id = uav_id;
			user_results[user_id].allocated_bandwidth = uavResult.allocatedBandwidth[user_id];
			user_results[user_id].utility = uavResult.allocatedValue[user_id];
		}
	}
	return user_results;
}

map<double, KnapsackResult> BAProblem::RP_based_subproblem_allocation_experiment1(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{
	Uav uav = sysModel.uavs[uav_id];
	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0;
	uav.elastic_bandwidth = 0;

	vector<User> unproc_hard_users;
	vector<User> unproc_elastic_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_elastic_users.push_back(user);

	map<double, KnapsackResult> result_map;
	double max_utility_ratio = 0; // 效用最大时，对应的连续部分的带宽资源量

	double bandwidth_step = 1e-2;		// 每次增加的带宽
	double elastic_bandwidth = uav.total_bandwidth;		// 初始化elastic用户的总带宽
	// double elastic_bandwidth = 0.4;		// 初始化elastic用户的总带宽

	while (elastic_bandwidth >= 0)
	{
		double hard_bandwidth = uav.total_bandwidth - elastic_bandwidth;
		uav.elastic_bandwidth = elastic_bandwidth;
		uav.hard_bandwidth = hard_bandwidth;
		//cout << "elastic_bandwidth = " << elastic_bandwidth << "\tdiscrete_ratio = " << hard_bandwidth << '\n';

		// 1. 离散部分：0-1背包问题
		KnapsackResult hard_result = Fptas01Knapsack(uav, unproc_hard_users, uti_max);

		// 将离散部分剩余的资源分配给连续部分
		// double remain_resource = uav.hard_bandwidth - hard_result.totalWeight;
		// uav.elastic_bandwidth += remain_resource;
		// cout << "remain resource: " << remain_resource << '\n';

		// 2. 连续部分：KKT条件
		Uav uav_elastic_only = uav;
		uav_elastic_only.total_bandwidth = uav.elastic_bandwidth;
		KnapsackResult elastic_result = WaterFillingAlgorithm_singleUAV_new(uav_elastic_only, unproc_elastic_users);

		// 3. 合并结果
		KnapsackResult current_result;
		current_result.elasticValue = elastic_result.totalValue;
		current_result.hardValue = hard_result.totalValue;
		current_result.elasticWeight = elastic_result.totalWeight;
		current_result.hardWeight = hard_result.totalWeight;

		current_result.totalWeight = hard_result.totalWeight + elastic_result.totalWeight;
		current_result.totalValue = hard_result.totalValue + elastic_result.totalValue;

		current_result.allocatedBandwidth = hard_result.allocatedBandwidth;
		current_result.allocatedBandwidth.merge(elastic_result.allocatedBandwidth);

		current_result.allocatedValue = hard_result.allocatedValue;
		current_result.allocatedValue.merge(elastic_result.allocatedValue);

		current_result.allocatedList = hard_result.allocatedList;
		current_result.allocatedList.insert(current_result.allocatedList.end(),
			elastic_result.allocatedList.begin(), elastic_result.allocatedList.end());
		/*cout << "**elastic_bandwidth = " << uavs[uav_id].elastic_bandwidth << "\thard_bandwidth = " << uavs[uav_id].hard_bandwidth << '\n';
		cout << "**Discrete Part: totalWeight = " << hard_result.totalWeight << "\ttotalValue = " << hard_result.totalValue << "\n";
		cout << "**Continuous Part: totalWeight = " << elastic_result.totalWeight << "\ttotalValue = " << elastic_result.totalValue << "\n";
		cout << "**totalWeight = " << current_result.totalWeight << "\totalValue = " << current_result.totalValue << '\n';*/
		/*for (auto user_id_J1 : hard_result.allocatedList)
		{
			current_result.allocatedList.push_back(user_id_J1);
			current_result.allocatedBandwidth[user_id_J1] = hard_result.allocatedBandwidth[user_id_J1];
		}
		for (auto item_id : elastic_result.allocatedList)
		{
			current_result.allocatedList.push_back(item_id);
			current_result.allocatedBandwidth[item_id] = elastic_result.allocatedBandwidth[item_id];
		}*/

		if (elastic_bandwidth == 1.0)
		{
			// 输出检查
			cout << "uav_id = " << uav_id << "\tElastic Only EXPResult: totalWeight = " << current_result.totalWeight << "\ttotalValue = " << current_result.totalValue << '\n';
		}

		result_map.insert({ elastic_bandwidth, current_result });

		if (current_result.totalValue > result_map[max_utility_ratio].totalValue)
			max_utility_ratio = elastic_bandwidth;

		elastic_bandwidth -= bandwidth_step;
	}
	//cout << "uav_id = " << uav_id << "\tmax_utility_ratio = " << max_utility_ratio << '\n';

	return result_map;
}

map<double, KnapsackResult> BAProblem::RP_based_subproblem_allocation_experiment2(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{
	Uav uav = sysModel.uavs[uav_id];
	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0; // 初始化
	uav.elastic_bandwidth = 0;

	vector<User> unproc_hard_users;
	vector<User> unproc_elastic_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_elastic_users.push_back(user);

	double hard_bandwidth = capacity;

	map<double, KnapsackResult> result_list;
	double max_utility_bandwidth = hard_bandwidth; // 效用最大时，对应的硬资源量
	int count = 0;	 // 计数器，防止死循环

	while (hard_bandwidth > 0)
	{
		uav.hard_bandwidth = hard_bandwidth;
		uav.elastic_bandwidth = uav.total_bandwidth - hard_bandwidth;
		cout << "Iteration " << count << ": hard_bandwidth = " << hard_bandwidth << "\telastic_bandwidth = " << uav.elastic_bandwidth << '\n';

		// 1. 离散部分：0-1背包问题

		KnapsackResult discrete_result = Fptas01Knapsack(uav, unproc_hard_users, uti_max);
		// 输出离散部分的结果
		/*cout << "**Discrete Part: totalWeight = " << hard_result.totalWeight << "\ttotalValue = " << hard_result.totalValue << "\n";
		cout << "Allocated Users in Discrete Part: ";
		for (auto entry : hard_result.allocatedList)
		{
			cout << " user_id_J1=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		// 离散部分剩余的资源分配给连续部分
		double remain_resource = uav.hard_bandwidth - discrete_result.totalWeight;
		uav.elastic_bandwidth += remain_resource;
		// cout << "remain resource: " << remain_resource << '\n';

		// 2. 连续部分：KKT条件
		KnapsackResult continuous_result = KktBasedElasticUtility(uav, unproc_elastic_users, uti_max);
		// 输出连续部分的结果
		/*cout << "**Continuous Part: totalWeight = " << elastic_result.totalWeight << "\ttotalValue = " << elastic_result.totalValue << "\n";
		cout << "Allocated Users in Continuous Part: ";
		for (auto entry : elastic_result.allocatedList)
		{
			cout << " user_id_J1=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		// 3. 合并结果
		KnapsackResult current_result;
		current_result.elasticValue = continuous_result.totalValue;
		current_result.hardValue = discrete_result.totalValue;
		current_result.elasticWeight = continuous_result.totalWeight;
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

map<double, KnapsackResult> BAProblem::RP_based_subproblem_allocation_experiment1_relaxed(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{

	Uav uav = sysModel.uavs[uav_id];
	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0;
	uav.elastic_bandwidth = 0;

	vector<User> unproc_hard_users;
	vector<User> unproc_elastic_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_elastic_users.push_back(user);

	map<double, KnapsackResult> result_map;
	double max_utility_ratio = 0; // 效用最大时，对应的连续部分的带宽资源量

	double bandwidth_step = 1e-2;		// 每次增加的带宽
	double elastic_bandwidth = uav.total_bandwidth;		// 初始化elastic用户的总带宽
	// double elastic_bandwidth = 0.4;		// 初始化elastic用户的总带宽

	while (elastic_bandwidth >= 0)
	{
		double hard_bandwidth = uav.total_bandwidth - elastic_bandwidth;
		uav.elastic_bandwidth = elastic_bandwidth;
		uav.hard_bandwidth = hard_bandwidth;
		//cout << "elastic_bandwidth = " << elastic_bandwidth << "\tdiscrete_ratio = " << hard_bandwidth << '\n';

		// 1. 离散部分：0-1背包问题
		Uav uav_hard_only = uav;
		uav_hard_only.total_bandwidth = uav.hard_bandwidth;
		KnapsackResult hard_result = WaterFillingAlgorithm_singleUAV_new(uav_hard_only, unproc_hard_users, 0);

		// 将离散部分剩余的资源分配给连续部分
		// double remain_resource = uav.hard_bandwidth - hard_result.totalWeight;
		// uav.elastic_bandwidth += remain_resource;
		// cout << "remain resource: " << remain_resource << '\n';

		// 2. 连续部分：KKT条件
		Uav uav_elastic_only = uav;
		uav_elastic_only.total_bandwidth = uav.elastic_bandwidth;
		KnapsackResult elastic_result = WaterFillingAlgorithm_singleUAV_new(uav_elastic_only, unproc_elastic_users);

		// 3. 合并结果
		KnapsackResult current_result;
		current_result.elasticValue = elastic_result.totalValue;
		current_result.hardValue = hard_result.totalValue;
		current_result.elasticWeight = elastic_result.totalWeight;
		current_result.hardWeight = hard_result.totalWeight;

		current_result.totalWeight = hard_result.totalWeight + elastic_result.totalWeight;
		current_result.totalValue = hard_result.totalValue + elastic_result.totalValue;

		current_result.allocatedBandwidth = hard_result.allocatedBandwidth;
		current_result.allocatedBandwidth.merge(elastic_result.allocatedBandwidth);

		current_result.allocatedValue = hard_result.allocatedValue;
		current_result.allocatedValue.merge(elastic_result.allocatedValue);

		current_result.allocatedList = hard_result.allocatedList;
		current_result.allocatedList.insert(current_result.allocatedList.end(),
			elastic_result.allocatedList.begin(), elastic_result.allocatedList.end());
		/*cout << "**elastic_bandwidth = " << uavs[uav_id].elastic_bandwidth << "\thard_bandwidth = " << uavs[uav_id].hard_bandwidth << '\n';
		cout << "**Discrete Part: totalWeight = " << hard_result.totalWeight << "\ttotalValue = " << hard_result.totalValue << "\n";
		cout << "**Continuous Part: totalWeight = " << elastic_result.totalWeight << "\ttotalValue = " << elastic_result.totalValue << "\n";
		cout << "**totalWeight = " << current_result.totalWeight << "\totalValue = " << current_result.totalValue << '\n';*/
		/*for (auto user_id_J1 : hard_result.allocatedList)
		{
			current_result.allocatedList.push_back(user_id_J1);
			current_result.allocatedBandwidth[user_id_J1] = hard_result.allocatedBandwidth[user_id_J1];
		}
		for (auto item_id : elastic_result.allocatedList)
		{
			current_result.allocatedList.push_back(item_id);
			current_result.allocatedBandwidth[item_id] = elastic_result.allocatedBandwidth[item_id];
		}*/

		if (elastic_bandwidth == 1.0)
		{
			// 输出检查
			cout << "uav_id = " << uav_id << "\tElastic Only EXPResult: totalWeight = " << current_result.totalWeight << "\ttotalValue = " << current_result.totalValue << '\n';
		}

		result_map.insert({ elastic_bandwidth, current_result });

		if (current_result.totalValue > result_map[max_utility_ratio].totalValue)
			max_utility_ratio = elastic_bandwidth;

		elastic_bandwidth -= bandwidth_step;
	}
	//cout << "uav_id = " << uav_id << "\tmax_utility_ratio = " << max_utility_ratio << '\n';

	return result_map;
}

KnapsackResult BAProblem::RP_based_subproblem_allocation_FPTAS(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{
	Uav uav = sysModel.uavs[uav_id];

	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0; // 初始化
	uav.elastic_bandwidth = 0;
	KnapsackResult max_result;

	int count = 0;	 // 计数器，防止死循环

	vector<User> unproc_hard_users;
	vector<User> unproc_elastic_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_elastic_users.push_back(user);

	/*cout << "***\t in BAProblem::RP_based_subproblem_allocation ***\n";
	cout << "\t unproc_users num = " << unproc_users.size() << endl;*/

	// 初始化hard_bandwidth为无人机当前剩余的总带宽capacity
	// 从capacity开始，逐步减少hard_bandwidth
	double hard_bandwidth = capacity;
	while (hard_bandwidth > 0)
	{
		uav.hard_bandwidth = hard_bandwidth;
		uav.elastic_bandwidth = uav.total_bandwidth - uav.hard_bandwidth;
		//cout << "Iteration " << count << ": hard_bandwidth = " << hard_bandwidth << "\telastic_bandwidth = " << uav.elastic_bandwidth << '\n';

		// 1. 离散部分：0-1背包问题
		KnapsackResult discrete_result = Fptas01Knapsack(uav, unproc_hard_users, uti_max);
		// 输出离散部分的结果
		/*cout << "\t\t unproc_hard_users num = " << unproc_hard_users.size() << endl;
		cout << "\t\t**Discrete Part: totalWeight = " << hard_result.totalWeight << "\ttotalValue = " << hard_result.totalValue << "\n";
		cout << "\t\tAllocated Users in Discrete Part: ";
		for (auto entry : hard_result.allocatedList)
		{
			cout << "\t\t\tuser_id=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		// 离散部分剩余的资源分配给连续部分
		double remain_resource = uav.hard_bandwidth - discrete_result.totalWeight;
		uav.elastic_bandwidth += remain_resource;
		// cout << "remain resource: " << remain_resource << '\n';

		// 2. 连续部分：KKT条件
		KnapsackResult continuous_result = KktBasedElasticUtility(uav, unproc_elastic_users, uti_max);
		// 输出连续部分的结果
		/*cout << "**Continuous Part: totalWeight = " << elastic_result.totalWeight << "\ttotalValue = " << elastic_result.totalValue << "\n";
		cout << "Allocated Users in Continuous Part: ";
		for (auto entry : elastic_result.allocatedList)
		{
			cout << " user_id_J1=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		double total_value = discrete_result.totalValue + continuous_result.totalValue;
		if (total_value > max_result.totalValue)
		{
			// 3. 合并结果
			KnapsackResult current_result;
			current_result.elasticValue = continuous_result.totalValue;
			current_result.hardValue = discrete_result.totalValue;
			current_result.elasticWeight = continuous_result.totalWeight;
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

KnapsackResult BAProblem::RP_based_subproblem_allocation_Greedy(int uav_id, double capacity, vector<User>& unproc_users, vector<double>& uti_max)
{
	Uav uav = sysModel.uavs[uav_id];

	uav.total_bandwidth = capacity;
	uav.hard_bandwidth = 0; // 初始化
	uav.elastic_bandwidth = 0;
	KnapsackResult max_result;

	int count = 0;	 // 计数器，防止死循环

	vector<User> unproc_hard_users;
	vector<User> unproc_elastic_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_elastic_users.push_back(user);

	// 将unproc_hard_users按照单位带宽边际效用从大到小排序
	vector<pair<User, double>> user_utility_list; // <user_id_J1, unit_utility>
	for (auto& user : unproc_hard_users)
	{
		int user_id = user.ID;
		double cap = sysModel.cap_list[uav_id][user_id]; // 用户j的信道容量
		double bandwidth = sysModel.Bth_list[uav_id][user_id]; // 用户j的最小带宽需求

		// utility_funcs[user_id_J1] 是用户user_id的效用函数指针
		// 在多背包问题中，用户若已被其它UAV分配资源，则其效用函数需调整为边际效用函数
		double utility = user.marginal_utility(bandwidth, cap, uti_max[user_id]);

		double unit_utility = utility / bandwidth; // 单位带宽边际效用
		// 输出utility，bandwidth，和unit_utility以供检查

		user_utility_list.push_back({ user, unit_utility });
	}
	// 按照unit_utility从大到小排序
	sort(user_utility_list.begin(), user_utility_list.end(),
		[](const pair<User, double>& param_a, const pair<User, double>& param_b) {
			return param_a.second > param_b.second;
		});
	// 输出排序结果
	cout << "User unit marginal utility ranking:\n";
	for (const auto& entry : user_utility_list)
	{
		cout << "User ID: " << entry.first.ID << "\tUnit Marginal Utility: " << entry.second << '\n';
	}
	// 按照排序结果重构unproc_hard_users
	unproc_hard_users.clear();
	for (auto& entry : user_utility_list)
	{
		unproc_hard_users.push_back(entry.first);
	}

	// 初始化hard_bandwidth为无人机当前剩余的总带宽capacity
	// 从capacity开始，逐步减少hard_bandwidth
	double hard_bandwidth = capacity;
	while (hard_bandwidth > 0)
	{
		uav.hard_bandwidth = hard_bandwidth;
		uav.elastic_bandwidth = uav.total_bandwidth - uav.hard_bandwidth;
		//cout << "Iteration " << count << ": hard_bandwidth = " << hard_bandwidth << "\telastic_bandwidth = " << uav.elastic_bandwidth << '\n';

		// 1. 离散部分：0-1背包问题
		KnapsackResult discrete_result = Greedy01Knapsack(uav, unproc_hard_users, uti_max);
		// 输出离散部分的结果
		// cout << "\t\t unproc_hard_users num = " << unproc_hard_users.size() << endl;
		cout << "\t\t**Discrete Part: totalWeight = " << discrete_result.totalWeight << "\ttotalValue = " << discrete_result.totalValue << "\n";
		/*cout << "\t\tAllocated Users in Discrete Part: ";
		for (int j = 0; j < hard_result.allocatedList.size(); j++)
		{
			int user_id_J1 = hard_result.allocatedList[j];
			cout << "\t\t\tuser_id=" << user_id_J1 << "(bw=" << hard_result.allocatedBandwidth[user_id_J1] << ") ";
		}
		cout << '\n';*/

		// 离散部分剩余的资源分配给连续部分
		double remain_resource = uav.hard_bandwidth - discrete_result.totalWeight;
		uav.elastic_bandwidth += remain_resource;
		// cout << "remain resource: " << remain_resource << '\n';

		// 2. 连续部分：KKT条件
		KnapsackResult continuous_result = KktBasedElasticUtility(uav, unproc_elastic_users, uti_max);
		// 输出连续部分的结果
		/*cout << "**Continuous Part: totalWeight = " << elastic_result.totalWeight << "\ttotalValue = " << elastic_result.totalValue << "\n";
		cout << "Allocated Users in Continuous Part: ";
		for (auto entry : elastic_result.allocatedList)
		{
			cout << " user_id_J1=" << entry.first << "(bw=" << entry.second << ") ";
		}
		cout << '\n';*/

		double total_value = discrete_result.totalValue + continuous_result.totalValue;
		if (total_value > max_result.totalValue)
		{
			// 3. 合并结果
			KnapsackResult current_result;
			current_result.elasticValue = continuous_result.totalValue;
			current_result.hardValue = discrete_result.totalValue;
			current_result.elasticWeight = continuous_result.totalWeight;
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
		/*cout << "totalWeight = " << hard_result.totalWeight << '\n';
		cout << "当前hard_bandwidth更新为: " << hard_bandwidth << '\n';*/
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
		double bandwidth = sysModel.Bth_list[uav_id][user_id]; // 用户j的最小带宽需求
		weights.push_back(bandwidth); // 物品的重量：用户j的最小速率需求转换为带宽需求

		// utility_funcs[user_id_J1] 是用户user_id的效用函数指针
		// 在多背包问题中，用户若已被其它UAV分配资源，则其效用函数需调整为边际效用函数
		double mutility = user.marginal_utility(bandwidth, cap, uti_max[user_id]);

		values.push_back(mutility); // 物品的价值：用户j的权重
		user_indices.push_back(user_id);
	}

	// user_id_J1 = user_indices[j] 对应 weights[j], values[i]
	// 输出所有信息
	/*for (size_t j = 0; j < weights.size(); j++)
	{
		cout << "User ID: " << user_indices[j] << "\tWeight: " << weights[j] << "\tValue: " << values[j] << '\n';
	}*/

	int n = weights.size();
	KnapsackResult knapsack_result;
	// 检查变量n和capacity
	// 输出n和capacity
	// cout << "\t\t\t\tn = " << n << "\tcapacity = " << value << '\n';
	if (n == 0 || capacity <= 0) return knapsack_result;

	// 1. 计算缩放因子K
	int v_max = *max_element(values.begin(), values.end());
	epsilon = 0.0001;
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

	vector<double> dp(scaled_V + 1, INF);
	// selected[j][v] 表示在处理到第 j 件物品时，是否选择第 i 件以达到价值 v
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
		sort(items.begin(), items.end(), [](const pair<int, double>& param_a, const pair<int, double>& param_b) { return param_a.second < param_b.second; });
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
	print_KnapsackResult(uav_result, uav_id);*/

	return result;
}

KnapsackResult BAProblem::Greedy01Knapsack(Uav& uav, vector<User>& sorted_unproc_users, vector<double>& uti_max)
{
	int uav_id = uav.ID;
	double capacity = uav.hard_bandwidth; // 无人机当前带宽容量
	int n = sorted_unproc_users.size();
	vector<vector<double>>& cap_list = sysModel.cap_list;
	KnapsackResult result;	// 算法的解

	// 贪心地按顺序选择sorted_unproc_users中的用户，直到容量用尽
	double total_weight = 0.0;
	int last_index = -1;
	for (int i = 0; i < n; i++)
	{
		int user_id = sorted_unproc_users[i].ID; // 当前用户id

		double cap = cap_list[uav_id][user_id]; // 用户user_id与无人机uav_id之间的信道容量
		if (cap <= 0)
			continue; // 避免除以0
		double min_band = sysModel.Bth_list[uav_id][user_id]; // 用户的最小带宽需求
		if (total_weight + min_band <= capacity + 1e-9) // 考虑数值误差
		{
			// 选择该用户
			double value = sorted_unproc_users[i].marginal_utility(min_band, cap, uti_max[user_id]);
			result.allocatedList.push_back(user_id);
			result.allocatedBandwidth.insert({ user_id, min_band });
			result.allocatedValue.insert({ user_id, value });
			total_weight += min_band;
			result.totalWeight += min_band;
			result.totalValue += value;
			last_index = i;
		}
		else
		{
			// 容量已满，停止选择
			break;
		}
	}

	// 检查是否可以通过替换last_index后一个用户来获得更优解
	// 首先确保last_index有效且不是最后一个用户
	if (last_index >= 0 && last_index < n - 1)
	{
		// 判断前last_index个用户与第last_index+1个用户之间是否存在更优解
		int next_index = last_index + 1;
		int user_id = sorted_unproc_users[next_index].ID;
		double cap = cap_list[uav_id][user_id];

		double min_band = sysModel.Bth_list[uav_id][user_id]; // 用户的最小带宽需求

		if (min_band <= capacity)
		{
			double next_value = sorted_unproc_users[next_index].marginal_utility(min_band, cap, uti_max[user_id]);


			if (next_value > result.totalValue)
			{
				clean_KnapsackResult(result);
				result.allocatedList.push_back(user_id);
				result.allocatedBandwidth.insert({ user_id, min_band });
				result.allocatedValue.insert({ user_id, next_value });
				result.totalValue += next_value;
				result.totalWeight += min_band;
			}
		}

	}

	return result;
}



//KnapsackResult BAProblem::WaterFillingAlgorithm_singleUAV(Uav uav, vector<User> unproc_users, int is_rounding)
//{
//	// elastic 用户的效用函数定义为 U(param_b) = w * log2(1 + bandwidth * log( 1 + SNR)), 其中 w 为用户权重，令C = log2(1 + SNR)
//
//
//	int uav_id = uav.ID;
//	double capacity = uav.total_bandwidth;
//	cout << "In WaterFillingAlgorithm_singleUAV: total bandwidth: " << capacity << endl;
//	// 记录所有注水法涉及的参数
//	// <user_id_J1, KKT_parameters>
//	vector<KKT_parameters> WF_Params = compute_KKT_parameters(uav_id, unproc_users);
//	
//	// 输出所有KKT参数以供检查
//	//print_KKT_parameters(WF_Params);
//
//
//	int l_last = -1;	// 记录中断循环时的l值
//	for (int l = 0; l < WF_Params.size(); l++)
//	{
//		// 对于每一个可能的l值，计算对应的λ值
//		KKT_parameters& params = WF_Params[l].second;
//		double W_sum = params.W_sum;
//		double C_inv_sum = params.C_inv_sum;
//		double B_hard_sum = 0;
//		if (l >= 1)
//			B_hard_sum = WF_Params[l - 1].second.B_hard_sum; // 遍历到第l个用户时，前l-1个用户的硬需求带宽之和（当前用户还不分配带宽，只是使用第l个用户在资源为0时的效率）
//		
//
//		
//		// 假设lambda的值为当前第l个用户的边际效用
//		double lambda = params.efficient;
//
//		// 计算此时的总带宽需求
//		double total_bandwidth_consume = B_hard_sum + (W_sum / lambda) - C_inv_sum;
//
//		// 打印各个参数以供检查
//		cout << "For l = " << l << ": W_sum = " << W_sum
//			<< "\tC_inv_sum = " << C_inv_sum
//			<< "\tB_hard_sum = " << B_hard_sum
//			<< "\tlambda = " << lambda
//			<< "\tTotal Bandwidth Consume = " << total_bandwidth_consume << '\n';
//
//		
//		/*cout << "For l = " << l << ": lambda = " << lambda
//			<< "\tTotal Bandwidth Consume = " << total_bandwidth_consume << '\n';*/
//		if (total_bandwidth_consume > uav.total_bandwidth)
//		{
//			// 找到第一个使得总带宽需求超过UAV总带宽的l值，停止
//			// cout << "total_bandwidth_consume = " << total_bandwidth_consume << " > uav.total_bandwidth = " << uav.total_bandwidth << '\n';
//			l_last = l;
//			break;
//		}
//		l_last = l + 1;
//	}
//
//	// l_last = 1;
//
//	cout << "Determined l_last = " << l_last << '\n';
//	if(l_last <= 1)
//		cout << "WatterFillingAlgorithm_singleUAV: l_last <= 1, maybe something wrong.\n";
//
//	// 根据l_last计算最优lambda值
//	// 在l_last 停止说明最优lambda在 WF_Params[l_last - 1] 和 WF_Params[l_last] 之间。第l_last个用户不会被分配资源
//
//	int last_user_id = WF_Params[l_last - 1].first;
//	User& last_user = sysModel.users[last_user_id];
//
//	double lambda_opt = 0;
//	KKT_parameters& params = WF_Params[l_last - 1].second;
//	double W_sum = params.W_sum;
//	double C_inv_sum = params.C_inv_sum;
//	double B_hard_sum_1 = WF_Params[l_last - 1].second.B_hard_sum;
//	double B_hard_sum = params.B_hard_sum;
//	double B_sum = 0;
//	double excess_bandwidth = 0;
//
//
//	if (last_user.uType == HARD_UTILITY)
//	{
//		// 先判断一下完全服务last_user时能否满足容量约束
//		lambda_opt = WF_Params[l_last - 1].second.efficient;
//
//		// 计算完全服务last_user时的总带宽需求
//		// 在搜索l_last时，last_user还没有被分配带宽
//		// 此处是考虑了last_user被完全分配带宽的情况
//		B_sum = B_hard_sum + (W_sum / lambda_opt) - C_inv_sum;
//
//		//cout << "B_sum when fully serving last hard user = " << B_sum << '\n';
//
//		// 要先判断是否选择第l_last个hard用户
//		if(B_sum > uav.total_bandwidth)
//		{
//			// 完全服务last_user超过了容量，说明只满足last_user部分需求
//			// 且最优lambda就是last_user的效用函数的斜率
//			excess_bandwidth = B_sum - uav.total_bandwidth;
//			//cout << "lambda_opt is the marginal utility of last hard user: " << last_user.ID << "lambda_opt = " << lambda_opt << '\n';
//		}
//		else
//		{
//			// 完全服务last_user没超过容量，
//			// 说明最优lambda在 WF_Params[l_last - 1] 和 WF_Params[l_last] 之间
//			lambda_opt = W_sum / (uav.total_bandwidth - B_hard_sum + C_inv_sum);
//			//cout << " lambda_opt between two users: " << lambda_opt << '\n';
//		} 
//
//		/*cout << "Calculated B_sum = " << B_sum << '\n';
//		cout << "Excess bandwidth = " << excess_bandwidth << '\n';
//		cout << "Optimal lambda = " << lambda_opt << '\n';*/
//
//	}
//	else if (last_user.uType == ELASTIC_UTILITY)
//	{
//		lambda_opt = W_sum / (uav.total_bandwidth - B_hard_sum + C_inv_sum);
//		//cout << "Elastic user at l_last, lambda_opt = " << lambda_opt << '\n';
//	}
//
//	// 计算在最优lambda下的总效用（用于检查）
//	/*double total_utility = 0.0;
//	double total_hard_utility = 0.0;
//	double total_elastic_utility = 0.0;
//	double total_bandwidth_used = 0.0;
//
//
//	for (int i = 0; i < l_last; i++)
//	{
//		int user_id_J1 = WF_Params[i].first;
//		User& user = sysModel.users[user_id_J1];
//		if (user.uType == HARD_UTILITY)
//		{
//			double B_th = sysModel.Bth_list[uav_id][user_id_J1];
//			double hard_utility = user.utility(B_th, sysModel.cap_list[uav_id][user_id_J1]);
//			total_utility += hard_utility;
//			total_hard_utility += hard_utility;
//			total_bandwidth_used += B_th;
//			cout << "User " << user_id_J1 << " bandwidth = " << B_th << " (HARD) utility = " << hard_utility << '\n';
//		}
//		else
//		{
//			double cap = sysModel.cap_list[uav_id][user_id_J1];
//			double bandwidth = (user.weight / (lambda_opt * log(2))) - (1.0 / cap);
//			if (bandwidth < 0)
//				bandwidth = 0.0;
//			double elastic_utility = user.utility(bandwidth, cap);
//			total_elastic_utility += elastic_utility;
//			total_utility += elastic_utility;
//			total_bandwidth_used += bandwidth;
//			cout << "User " << user_id_J1 << " bandwidth = " << bandwidth << " (ELASTIC) utility = " << elastic_utility << '\n';
//		}
//	}
//	cout << "Total utility with optimal lambda = " << total_utility
//		<< "\tTotal hard utility = " << total_hard_utility
//		<< "\tTotal elastic utility = " << total_elastic_utility << '\n';
//	cout << "Total bandwidth used = " << total_bandwidth_used << "\tUAV value = " << uav.total_bandwidth << '\n';*/
//
//
//	// 根据最优lambda计算每个用户的分配带宽
//	// 上述过程确定了要分配带宽给前l_last-1个用户
//	// 第l_last个用户可能部分分配，根据用户类型决定
//	KnapsackResult alloc_result;
//	// 当l_last个用户是hard用户时，要贪心地选择前面的hard用户，还是第l_last个hard用户。才能保证1 / 2近似
//	map<int, int> hard_user_map; // 记录所有hard用户在efficient_list中的索引
//	double first_l_utility = 0.0;	// 记录前l_last-1个用户中的hard用户的总效用
//	
//	if (l_last >= 2)
//	{
//		for (int i = 0; i <= l_last - 2; i++)
//		{
//			int user_id_J1 = WF_Params[i].first;
//			User& user = sysModel.users[user_id_J1];
//			if (user.uType == HARD_UTILITY)
//			{
//				hard_user_map.insert({ i, user_id_J1 });
//				double B_th = sysModel.Bth_list[uav_id][user_id_J1];
//				double value = user.utility(B_th, sysModel.cap_list[uav_id][user_id_J1]);
//				first_l_utility += value;
//			}
//			else if (user.uType == ELASTIC_UTILITY)
//			{
//				double cap = sysModel.cap_list[uav_id][user_id_J1];
//				double bandwidth = (user.weight / (lambda_opt * log(2))) - (1.0 / cap);
//				if (bandwidth < 0)
//					bandwidth = 0.0;
//				double value = user.utility(bandwidth, cap);
//				alloc_result.allocatedList.push_back(user_id_J1);
//				alloc_result.allocatedBandwidth.insert({ user_id_J1, bandwidth });
//				alloc_result.allocatedValue.insert({ user_id_J1, value });
//				alloc_result.totalWeight += bandwidth;
//				alloc_result.totalValue += value;
//				alloc_result.elasticValue += value;
//				alloc_result.elasticWeight += bandwidth;
//			}
//		}
//	}
//
//	// 输出hard_user_map和first_l_utility以供检查
//	/*cout << "Hard user indices before l_last:\n";
//	for (auto entry : hard_user_map)
//	{
//		cout << "Index: " << entry.first << "\tUser ID: " << entry.second << '\n';
//	}
//	cout << "First l_last-1 hard users total utility: " << first_l_utility << '\n';*/
//
//	
//	// 处理第l_last - 1个用户	
//	if(last_user.uType == HARD_UTILITY)
//	{
//		// 计算仅选择第l_last - 1个hard用户的效用
//
//		// 计算last_user的效用
//		double last_user_B_min = sysModel.Bth_list[uav_id][last_user_id];
//		double last_user_utility = last_user.utility(last_user_B_min, sysModel.cap_list[uav_id][last_user_id]);
//		double hard_bandwidth = uav.total_bandwidth - alloc_result.totalWeight;
//		//cout << "Last hard user utility: " << last_user_utility << "\tB_min: " << last_user_B_min << "\thard_bandwidth: " << hard_bandwidth << '\n';
//		// 判断选择第l_last个hard用户，还是前l_last-1个hard用户
//		// 如果是舍入方案，则当选择last_user的效用更大时，选择last_user
//		if (is_rounding == 1 and last_user_utility > first_l_utility and last_user_B_min < hard_bandwidth)
//		{
//			// 选择第l_last个hard用户
//			alloc_result.allocatedList.push_back(last_user_id);
//			alloc_result.allocatedBandwidth.insert({ last_user_id, last_user_B_min });
//			alloc_result.allocatedValue.insert({ last_user_id, last_user_utility });
//			alloc_result.totalWeight += last_user_B_min;
//			alloc_result.totalValue += last_user_utility;
//			alloc_result.hardValue += last_user_utility;
//			alloc_result.hardWeight += last_user_B_min;
//			//cout << "Selecting last hard user_id_J1 = " << last_user_id << '\n';
//		}
//		else
//		{
//			// cout << "Selecting first l_last-1 hard users.\n";
//			// 选择前l_last-1个用户中的hard用户
//			for (auto idx : hard_user_map)
//			{
//				int user_id_J1 = idx.second;
//				User& user = sysModel.users[user_id_J1];
//				double B_th = sysModel.Bth_list[uav_id][user_id_J1];
//				double value = user.utility(B_th, sysModel.cap_list[uav_id][user_id_J1]);
//				alloc_result.allocatedList.push_back(user_id_J1);
//				alloc_result.allocatedBandwidth.insert({ user_id_J1, B_th });
//				alloc_result.allocatedValue.insert({ user_id_J1, value });
//				alloc_result.totalWeight += B_th;
//				alloc_result.totalValue += value;
//				alloc_result.hardValue += value;
//				alloc_result.hardWeight += B_th;
//			}
//			if (is_rounding == 0)
//			{
//				// 如果不是舍入方案，则对last_user进行部分分配
//
//				// 最后一个hard用户部分分配
//				double bandwidth = hard_bandwidth - alloc_result.hardWeight;
//				if (bandwidth < 0)
//					bandwidth = 0.0;
//				double frac_last_user_utility = (bandwidth / last_user_B_min) * last_user_utility;
//
//				alloc_result.allocatedList.push_back(last_user_id);
//				alloc_result.allocatedBandwidth.insert({ last_user_id, bandwidth });
//				alloc_result.allocatedValue.insert({ last_user_id, frac_last_user_utility });
//				alloc_result.totalValue += frac_last_user_utility;
//				alloc_result.totalWeight += bandwidth;
//				alloc_result.hardValue += frac_last_user_utility;
//				alloc_result.hardWeight += bandwidth;
//			}
//		}
//	}
//	else if(last_user.uType == ELASTIC_UTILITY)
//	{
//		double cap = sysModel.cap_list[uav_id][last_user_id];
//		double bandwidth = (last_user.weight / (lambda_opt * log(2))) - (1.0 / cap);
//		if (bandwidth < 0)
//			bandwidth = 0.0;
//		double value = last_user.utility(bandwidth, cap);
//		alloc_result.allocatedList.push_back(last_user_id);
//		alloc_result.allocatedBandwidth.insert({ last_user_id, bandwidth });
//		alloc_result.allocatedValue.insert({ last_user_id, value });
//		alloc_result.totalWeight += bandwidth;
//		alloc_result.totalValue += value;
//		alloc_result.elasticValue += value;
//		alloc_result.elasticWeight += bandwidth;
//
//		for (auto idx : hard_user_map)
//		{
//			int user_id_J1 = idx.second;
//			User& user = sysModel.users[user_id_J1];
//			double B_th = sysModel.Bth_list[uav_id][user_id_J1];
//			double value = user.utility(B_th, sysModel.cap_list[uav_id][user_id_J1]);
//			alloc_result.allocatedList.push_back(user_id_J1);
//			alloc_result.allocatedBandwidth.insert({ user_id_J1, B_th });
//			alloc_result.allocatedValue.insert({ user_id_J1, value });
//			alloc_result.totalWeight += B_th;
//			alloc_result.totalValue += value;
//			alloc_result.hardValue += value;
//			alloc_result.hardWeight += B_th;
//		}
//	}
//	// 输出最终的分配结果以供检查
//	/*cout << "Water Filling Algorithm EXPResult for UAV ID " << uav_id << ":\n";
//	PrintKnapsackResult(alloc_result);*/
//	
//	// 如果最后还有剩余带宽，则分配给elastic用户
//	double remaining_bandwidth = uav.total_bandwidth - alloc_result.totalWeight;
//	if(remaining_bandwidth > 1e-9)
//	{
//		//cout << "Remaining bandwidth after initial allocation: " << remaining_bandwidth << '\n';
//		// 重新计算lambda_opt，以便分配剩余带宽
//		//cout << "Original lambda_opt = " << lambda_opt << '\n';
//		lambda_opt = W_sum / (uav.total_bandwidth - alloc_result.hardWeight + C_inv_sum);
//		//cout << "new lambda_opt = " << lambda_opt << '\n';
//		// 分配给elastic用户
//		for (auto& entry : alloc_result.allocatedList)
//		{
//			int user_id_J1 = entry;
//			User& user = sysModel.users[user_id_J1];
//			if (user.uType == ELASTIC_UTILITY)
//			{
//				double cap = sysModel.cap_list[uav_id][user_id_J1];
//				double current_bandwidth = alloc_result.allocatedBandwidth[user_id_J1];
//				double new_bandwidth = (user.weight / (lambda_opt * log(2))) - (1.0 / cap);
//				if (new_bandwidth > current_bandwidth)
//				{
//					double new_utility = user.utility(new_bandwidth, cap);
//					double current_utility = alloc_result.allocatedValue[user_id_J1];
//					double delta_utility = new_utility - current_utility;
//					// 更新分配结果
//					alloc_result.allocatedBandwidth[user_id_J1] = new_bandwidth;
//					alloc_result.allocatedValue[user_id_J1] = new_utility;
//					alloc_result.totalWeight += (new_bandwidth - current_bandwidth);
//					alloc_result.totalValue += delta_utility;
//					alloc_result.elasticWeight += (new_bandwidth - current_bandwidth);
//					alloc_result.elasticValue += delta_utility;
//				}
//			}
//		}
//	}
//
//	return alloc_result;
//}

KnapsackResult BAProblem::FPTAS_singleUAV(Uav& uav, vector<User>& unproc_users, double epsilon)
{
	// 该方法实现了针对单无人机的FPTAS算法
	int uav_id = uav.ID;
	double capacity = uav.total_bandwidth;
	// 首先使用WaterFillingAlgorithm_singleUAV方法，得到一个1/2近似解
	KnapsackResult wf_result = WaterFillingAlgorithm_singleUAV_new(uav, unproc_users);
	double wf_utility = wf_result.totalValue;
	// PrintKnapsackResult(wf_result, uav_id);


	vector<User> unproc_hard_users;
	vector<User> unproc_elastic_users;
	for (auto& user : unproc_users)
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_elastic_users.push_back(user);

	// 计算所有elastic用户的KKT_parameters_elastic
	vector<KKT_parameters> WF_Params_elastic = compute_KKT_parameters(uav_id, unproc_elastic_users);

	// print_KKT_parameters(WF_Params_elastic);


	//-------------------------------------------------------------------
	// 对hard用户使用动态规划的方法求解0-1背包问题，输出动态规划表格的最后一行
	// 构造环境
	// Uav uav = sysModel.uavs[uav_id];
	uav.total_bandwidth = capacity;

	// 获取当前UAV对应的用户列表
	vector<int> user_indices;
	vector<double> weights;
	vector<double> values;

	for (auto& user : unproc_hard_users)
	{
		int user_id = user.ID;
		double cap = sysModel.cap_list[uav_id][user_id];
		double bandwidth = sysModel.Bth_list[uav_id][user_id]; // 用户j的最小带宽需求
		weights.push_back(bandwidth); // 物品的重量：用户j的最小速率需求转换为带宽需求
		// utility_funcs[user_id_J1] 是用户user_id的效用函数指针
		// 在多背包问题中，用户若已被其它UAV分配资源，则其效用函数需调整为边际效用函数
		double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
		double utility = user.utility(bandwidth, cap, SNR_avg_dB);
		values.push_back(utility); // 物品的价值：用户j的权重
		user_indices.push_back(user_id);
	}
	// user_id_J1 = user_indices[j] 对应 weights[j], values[i]
	// 输出所有信息
	/*for (size_t i = 0; i < weights.size(); i++)
	{
		cout << "User ID: " << user_indices[i] << "\tWeight: " << weights[i] << "\tValue: " << values[i] << '\n';
	}*/

	int n1 = weights.size();
	// 检查变量n和capacity
	// 输出n和capacity
	cout << "\t\t\t\tn = " << n1 << "\tcapacity = " << capacity << '\n';

	// 1. 计算缩放因子delta = ε * wf_utility / n1
	double delta = (epsilon * wf_utility) / n1;

	// 2. 缩放价值
	vector<int> scaled_values(n1);
	// 最优解上界为 2 * wf_utility，因此缩放后最大价值为 2 * wf_utility / delta 向上取整。
	int scaled_V = ceil((2 * n1) / epsilon); // 这里最优解的上界是1/2解的两倍； 在后续中，可以优化为松弛后的最优解。
	for (int i = 0; i < n1; i++)
	{
		scaled_values[i] = (values[i] / delta); // 这里已经暗含了向下取整
	}
	if (scaled_V == 0) {
		// 所有物品缩放价值为0，无法通过DP区分，输出错误警告
		std::cout << "Error in BAProblem::FPTAS_singleUAV: All scaled values are zero!" << std::endl;
	}

	// 3. 动态规划求解：dp[v] = 达到价值v的最小重量（使用double避免截断误差）
	constexpr double INF = std::numeric_limits<double>::infinity();
	vector<double> dp(scaled_V + 1, INF);
	// selected[j][v] 表示在处理到第 j 件物品时，是否选择第 i 件以达到价值 v
	vector<vector<char>> selected(n1, vector<char>(scaled_V + 1, 0));
	dp[0] = 0.0;

	// 填充DP表并记录选择决策
	int current_max_v = 0;	// 维护一个 current_max_v 变量，记录当前已达到的最大价值。内层循环只需从 current_max_v + sv 开始向下遍历
	for (int i = 0; i < n1; i++) {
		int sv = scaled_values[i];
		if (sv <= 0) continue; // 缩放价值为0的物品对DP无影响，可跳过

		// 优化：只遍历可能更新的范围
		int upper_bound = std::min(scaled_V, current_max_v + sv);

		// 遍历时从高到低，保证每件物品只使用一次
		for (int v = upper_bound; v >= sv; v--) {
			if (dp[v - sv] != INF) {
				double new_weight = dp[v - sv] + weights[i];
				// 仅当新的重量在容量范围内并且更优时更新
				if (new_weight <= capacity && new_weight < dp[v]) {
					dp[v] = new_weight;
					selected[i][v] = 1; // 标记选择当前物品

					// 更新 current_max_v
					if (v > current_max_v)
					{
						current_max_v = v; // 更新当前已达到的最大价值
					}
				}
			}
		}
	}

	// 截止目前，已经得到了hard用户的动态规划表dp
	// dp[v]表示考虑所有hard用户，达到缩放价值v所需的最小带宽

	// 4. 从dp表尾开始遍历，当前价值为v时，消耗的资源为dp[v]，那么剩余资源为elastic_cap = capacity - dp[v]，分配给elastic用户。
	// 在WF_Params_elastic列表中找到使得总带宽消耗超过elastic_cap的最小l值
	double max_utility = 0.0;		// 记录最大总效用
	int max_hard_utility = 0;  // 记录在最大效用时，对应的hard用户效用. 用于从dp表中回溯选择的用户
	int max_elastic_utility_l = 0;	// 记录在最大效用时，对应的WF_Params_elastic 中的l值
	double max_elastic_lambda = 0.0; // 记录在最大效用时，对应的elastic用户的最优lambda值
	for (int v = scaled_V; v >= 0; v--)
	{
		double hard_bandwidth_consume = dp[v];
		if (hard_bandwidth_consume == INF)
			continue; // 该价值不可达，跳过
		double elastic_cap = capacity - hard_bandwidth_consume;
		// 在WF_Params_elastic中找到最小的l值使得总带宽消耗超过elastic_cap
		// 顺序查找
		/*int l_elastic_last = -1;
		for (int l = 0; l < WF_Params_elastic.size(); l++)
		{
			KKT_parameters& params = WF_Params_elastic[l].second;
			double total_bandwidth_consume = params.B_elastic_sum;
			if (total_bandwidth_consume > elastic_cap)
			{
				l_elastic_last = l;
				break;
			}
		}*/

		// 优化后：二分查找 (使用 std::upper_bound 或 std::lower_bound)
		// 构造一个比较用的临时对象或lambda
		auto it = std::lower_bound(WF_Params_elastic.begin(), WF_Params_elastic.end(), elastic_cap,
			[](const KKT_parameters& elem, double cap) {
				return elem.B_elastic_sum < cap;
			});

		int l_elastic_last = -1;
		if (it != WF_Params_elastic.end()) {
			l_elastic_last = std::distance(WF_Params_elastic.begin(), it);
		}
		else {
			// 如果所有带宽加起来都不够(或者刚好够)，说明可以使用全部elastic用户，
			// 但需注意原逻辑中 break 的条件是 > elastic_cap。
			// 如果找不到比elastic_cap大的，说明可以用尽所有用户
			l_elastic_last = -1; // 保持你原有的逻辑标记
		}

		// 根据l_elastic_last计算对应的λ值，并计算总效用
		if (l_elastic_last == -1)
		{
			// 即使选择所有elastic用户，带宽消耗也未超过elastic_cap，说明elastic用户可以全部满足
			// 此时lambda取最小值，即最后一个用户的efficient
			l_elastic_last = WF_Params_elastic.size() - 1;
		}
		// 计算lambda_opt
		KKT_parameters& params_elastic = WF_Params_elastic[l_elastic_last];
		double W_sum = params_elastic.W_sum;
		double C_inv_sum = params_elastic.C_inv_sum;
		double lambda_opt = W_sum / (elastic_cap + C_inv_sum);

		// 根据lambda_opt计算elastic用户的总效用
		double elastic_utility = 0.0;
		for (int i = 0; i <= l_elastic_last; i++)
		{
			int user_id = WF_Params_elastic[i].user_id;
			User& u = sysModel.users[user_id];
			double cap = sysModel.cap_list[uav_id][user_id];
			double bandwidth = (u.weight / (lambda_opt * log(2))) - (1.0 / cap);
			if (bandwidth < 0)
				bandwidth = 0.0;
			double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
			double value = u.utility(bandwidth, cap, SNR_avg_dB);
			elastic_utility += value;
		}

		double hard_utility = v * delta; // 还原回原始价值
		double total_utility = hard_utility + elastic_utility;


		if (total_utility > max_utility)
		{
			max_utility = total_utility;
			max_hard_utility = v;
			max_elastic_utility_l = l_elastic_last;
			max_elastic_lambda = lambda_opt;

		}

	}

	// 根据max_hard_utility和max_elastic_utility_l回溯选择的用户
	KnapsackResult final_result;
	// 回溯hard用户
	for (int i = n1 - 1; i >= 0 && max_hard_utility > 0; i--)
	{
		int sv = scaled_values[i];
		if (sv <= 0) continue;
		if (max_hard_utility >= sv && selected[i][max_hard_utility])
		{
			// 选择了物品 i
			int user_id = user_indices[i];
			double bandwidth = weights[i];
			double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
			double value = unproc_hard_users[i].utility(bandwidth, sysModel.cap_list[uav_id][user_id], SNR_avg_dB);
			final_result.allocatedList.push_back(user_id);
			final_result.allocatedBandwidth.insert({ user_id, bandwidth });
			final_result.allocatedValue.insert({ user_id, value });
			final_result.totalWeight += bandwidth;
			final_result.totalValue += value;
			final_result.hardValue += value;
			final_result.hardWeight += bandwidth;
			max_hard_utility -= sv;
		}
	}
	// 回溯elastic用户
	for (int j = 0; j <= max_elastic_utility_l; j++)
	{
		int user_id = WF_Params_elastic[j].user_id;
		User& u = sysModel.users[user_id];
		double cap = sysModel.cap_list[uav_id][user_id];
		double bandwidth = (u.weight / (max_elastic_lambda * log(2))) - (1.0 / cap);
		if (bandwidth < 0)
			bandwidth = 0.0;
		double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
		double value = u.utility(bandwidth, cap, SNR_avg_dB);
		final_result.allocatedList.push_back(user_id);
		final_result.allocatedBandwidth.insert({ user_id, bandwidth });
		final_result.allocatedValue.insert({ user_id, value });
		final_result.totalWeight += bandwidth;
		final_result.totalValue += value;
		final_result.elasticValue += value;
		final_result.elasticWeight += bandwidth;
	}
	return final_result;
}

KnapsackResult BAProblem::WaterFillingAlgorithm_singleUAV_new(Uav uav, vector<User> unproc_users, int is_rounding)
{
	// elastic 用户的效用函数定义为 U(b) = w * log2(1 + bandwidth * log2( 1 + SNR)), 其中 w 为用户权重，而C = log2(1 + SNR)

	int uav_id = uav.ID;
	double capacity = uav.total_bandwidth;
	// cout << "WaterFillingAlgorithm_singleUAV_new uav id " << uav.ID << endl;
	// 记录所有注水算法涉及的参数
	// <user_id_J1, KKT_parameters>
	vector<KKT_parameters> WF_Params = compute_KKT_parameters(uav_id, unproc_users);
	// cout << "uav id " << uav.ID << endl;
	// print_KKT_parameters(WF_Params);

	// 二分查找确定 l_last
	int lb = 0;
	int ub = WF_Params.size() - 1;
	double break_band = 0;
	while (lb + 1 < ub)
	{
		int l = (lb + ub) / 2;
		KKT_parameters& params = WF_Params[l];
		double W_sum = params.W_sum;
		double C_inv_sum = params.C_inv_sum;

		// 当前注水水平下的总带宽消耗，但是不包括第l个用户
		double B_hard_sum = params.B_hard_sum;

		double lambda = params.efficient;
		double total_bandwidth_consume = B_hard_sum + (W_sum / lambda) - C_inv_sum;
		//cout << " l = " << l << ",\tW_sum = " << W_sum << ",\tC_inv_sum = " << C_inv_sum << ",\tB_hard_sum = " << B_hard_sum << ",\ttotal_bandwidth_consume = " << total_bandwidth_consume << endl;

		if (total_bandwidth_consume > uav.total_bandwidth)
		{
			ub = l;
		}
		else
			lb = l;
		break_band = total_bandwidth_consume;
	}
	int l_last = lb;
	//cout << "l_last = " << l_last << ", total_bandwidth_consume = " << break_band << endl;

	/*if (l_last <= 1)
		cout << "WatterFillingAlgorithm_singleUAV: l_last <= 1, maybe something wrong.\n";*/

		// 最优lambda在 l_last 和 l_last + 1 之间
		// 计算 lambda = WF_Params[l_last].efficient 时，被完全服务的用户总带宽消耗
	double total_B_max = 0.0;
	double lambda = WF_Params[l_last].efficient;
	//cout << "init lambda = " << lambda << endl;

	vector<int> is_counted(sysModel.n1 + sysModel.n2, 0); // 记录用户是否已被遍历过
	// 计算当lambda时的分配
	for (int j = WF_Params.size() - 1; j >= 0; j--)
	{
		KKT_parameters& params = WF_Params[j];
		int user_id = params.user_id;
		if (user_id >= 0 and user_id < sysModel.n1 + sysModel.n2)
		{
			// 用户存在. 在params中每个用户有两个记录，分别对应最小和最大的效用斜率
			// 如果当前对象的斜率大于lambda，说明该用户在lambda下被完全服务
			if (params.efficient > lambda and is_counted[user_id] == 0)
			{
				if (sysModel.users[user_id].uType == HARD_UTILITY)
				{

					double B_th = sysModel.Bth_list[uav_id][user_id];
					total_B_max += B_th;

					//cout << "user_id_J1 :" << user_id << ",\tB_th = " << B_th << ",\ttotal_B_max = " << total_B_max << endl;
				}
				else
				{
					total_B_max += uav.total_bandwidth; // elastic用户在该lambda下被完全服务
				}
			}
			// WF_Params是按照斜率降序排序，当倒序遍历时，遇到某个用户的第一个记录是斜率较小的那一个。当遍历过一次该用户后，不管与lambda的关系如何，都不再重复计算。
			// 因为只需比较斜率较小的那一个，就能判断该用户是否被完全服务
			is_counted[user_id] = 1;
		}
	}

	int is_l_last_opt_lambda = 0; // 记lambda_opt是否为第l_last个hard用户的斜率
	if (sysModel.users[WF_Params[l_last].user_id].uType == HARD_UTILITY)
	{
		// 如果终止的这个用户是hard用户，判断一下完全服务它是否会超过容量
		// 
		int user_id = WF_Params[l_last].user_id;
		double B_th = sysModel.Bth_list[uav_id][user_id];
		if (capacity - total_B_max - B_th > 1e-9)
		{
			// 说明容得下user_id
			total_B_max += B_th;
		}
		else
		{
			is_l_last_opt_lambda = 1; // 如果l_last用户是hard用户，且加入之后就违反容量约束了，那么lambda_opt为他的斜率
		}
	}

	//cout << "Total bandwidth consumption when fully serving users up to l_last: " << total_B_max << '\n';
	double B_res = uav.total_bandwidth - total_B_max;

	double lambda_opt = 0;
	KKT_parameters& params = WF_Params[l_last];
	double W_sum = params.W_sum;
	double C_inv_sum = params.C_inv_sum;
	double B_hard_sum = params.B_hard_sum;
	if (is_l_last_opt_lambda == 1)
	{
		lambda_opt = params.efficient;
	}
	else
	{
		// 当有剩余带宽时，说明最优lambda在 WF_Params[l_last] 和 WF_Params[l_last + 1] 之间

		// 本质上，此时的total_B_max均有hard用户贡献。
		// 输出total_B_max和B_hard_sum以供检查
		//cout << "total_B_max: " << total_B_max << "\tB_hard_sum: " << B_hard_sum << "\tB_res：" << B_res << endl;

		lambda_opt = W_sum / (B_res + C_inv_sum);
	}
	//cout << "lambda_opt = " << lambda_opt << endl;

	// 根据lambda_opt为所有用户分配带宽
	KnapsackResult temp_result;	//temp_result 是松弛问题的最优解
	temp_result.uav_id = uav_id;
	int user_id_eq_lambda_opt = -1;		// 斜率刚好等于lambda_opt的用户id
	for (auto& user : unproc_users)
	{
		int user_id = user.ID;
		double min_efficient = 0.0;
		double max_efficient = 0.0;

		if (user.uType == HARD_UTILITY)
		{
			double B_th = sysModel.Bth_list[uav_id][user_id];
			max_efficient = user.utility_derivative(0, B_th);
			min_efficient = user.utility_derivative(B_th, B_th);

			if (max_efficient > lambda_opt)
			{
				// 此时该hard用户被完全服务
				double cap = sysModel.cap_list[uav_id][user_id];
				double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
				double uti = user.utility(B_th, cap, SNR_avg_dB);
				add_KnapsackResult(temp_result, user, B_th, uti);
				//cout << "user " << user_id_J1 << ", max_efficient = " << max_efficient << ", min_efficient = " << min_efficient << ", B_th = " << B_th << endl;
			}
		}
		else
		{
			double cap = sysModel.cap_list[uav_id][user_id];
			max_efficient = user.utility_derivative(0, cap);
			//cout << "user " << user_id_J1 << ", max_efficient = " << max_efficient << ", min_efficient = " << min_efficient << endl;
			if (max_efficient > lambda_opt)
			{
				// 只有当elastic位于0点的斜率大于lambda_opt才能被服务
				// 经过这个条件判断，能排除cap十分小的情况。
				double bandwidth = user.weight / (lambda_opt * log(2)) - 1 / cap;
				double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
				double uti = user.utility(bandwidth, cap, SNR_avg_dB);
				add_KnapsackResult(temp_result, user, bandwidth, uti);
			}
		}
		if (max_efficient - lambda_opt > 1e-9)
			user_id_eq_lambda_opt = user_id;
	}


	// 将剩余资源分配给user_id_eq_lambda_opt
	if (temp_result.totalWeight - uav.total_bandwidth < 1e-9)
	{
		double res_band = uav.total_bandwidth - temp_result.totalWeight;
		User user = sysModel.users[user_id_eq_lambda_opt];
		if (user.uType == HARD_UTILITY)
		{
			double B_th = sysModel.Bth_list[uav_id][user_id_eq_lambda_opt];
			double cap = sysModel.cap_list[uav_id][user_id_eq_lambda_opt];
			double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id_eq_lambda_opt];

			double full_uti = user.utility(B_th, cap, SNR_avg_dB);
			double partial_uti = (res_band / B_th) * full_uti;
			// 在面搜索最优lambda区间时，就已经知道，加入了该用户会导致破坏容量限制，因此此处(res_band / B_th)原则上应该是小于1的;
			add_KnapsackResult(temp_result, user, res_band, partial_uti);
		}
	}

	// temp_result是松弛问题的最优解
	//cout << "Temp uav_result: \n";
	// PrintKnapsackResult(temp_result, uav_id);

	if (is_rounding != 1)
	{
		return temp_result;
	}
	else
	{
		// 根据temp_result构造最终原问题的可行解
		KnapsackResult final_result;
		final_result.uav_id = uav_id;
		for (auto user_id : temp_result.allocatedList)
		{
			User user = sysModel.users[user_id];
			if (user.uType == ELASTIC_UTILITY)
			{
				double bandwidth = temp_result.allocatedBandwidth[user_id];
				double value = temp_result.allocatedValue[user_id];
				add_KnapsackResult(final_result, user, bandwidth, value);
			}
		}
		double res_hard_bandwidth = uav.total_bandwidth - final_result.totalWeight;
		/*cout << "res_hard_bandwidth = " << res_hard_bandwidth << endl;
		cout << "Final middle uav_result: \n";
		PrintKnapsackResult(final_result);*/


		if (res_hard_bandwidth > 1e-9)
		{
			// 用剩余资源分配给hard用户

			// 1. 构造斜率队列
			vector<pair<int, double>> efficient_list;
			for (auto& user : unproc_users)
			{
				int user_id = user.ID;
				if (user.uType == HARD_UTILITY)
				{
					double B_th = sysModel.Bth_list[uav_id][user_id];
					double efficient = user.utility_derivative(0, B_th);
					efficient_list.push_back({ user_id, efficient });
				}
			}



			//2. efficient_list按照efficient降序排序
			sort(efficient_list.begin(), efficient_list.end(),
				[](const pair<int, double>& param_a, const pair<int, double>& param_b) {
					return param_a.second > param_b.second;
				});


			// 根据efficient_list构造一个累积消耗前缀和列表
			vector<double> accumulated_bandwidth_list;
			vector<double> accu_uti_list;
			double ac_band = 0.0;
			double ac_uti = 0.0;
			for (auto& entry : efficient_list)
			{
				int user_id = entry.first;
				User user = sysModel.users[user_id];
				double B_th = sysModel.Bth_list[uav_id][user_id];
				double cap = sysModel.cap_list[uav_id][user_id];
				double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
				double uti = user.utility(B_th, cap, SNR_avg_dB);
				ac_band += sysModel.Bth_list[uav_id][user_id];
				ac_uti += uti;
				accumulated_bandwidth_list.push_back(ac_band);
				accu_uti_list.push_back(ac_uti);
				// cout << "user_id_J1 = " << user_id << ", ac_band = " << ac_band << ", ac_uti = " << ac_uti << endl;
			}


			// 3. 找到最大的下标J，使得前J个hard用户不超过预算，而J+1个超过了预算
			int lb = 0;
			int ub = efficient_list.size() - 1;
			int J = 0;
			while (lb + 1 < ub)
			{
				J = (lb + ub) / 2;
				if (accumulated_bandwidth_list[J] - res_hard_bandwidth > 1e-9)
				{
					ub = J;
				}
				else
				{
					lb = J;
				}
			}
			J = lb;


			double uti_J_1 = 0.0;
			double user_id_J1 = efficient_list[J].first;
			if (J + 1 >= efficient_list.size())
			{
				// 如果accu_uti_list中只有一个用户

			}
			else
			{
				uti_J_1 = accu_uti_list[J + 1] - accu_uti_list[J];
				user_id_J1 = efficient_list[J + 1].first;
			}
			// 判断前J个累积效用大，还是第J+1个大
			//cout << "J = " << J << ", uti_J+1 = " << uti_J_1 << ", accu_uti_list[J] = " << accu_uti_list[J] << endl;
			// 第J+1个用户的信息
			//cout << "user_id_J1 = " << user_id_J1 << endl;
			User user_J1 = sysModel.users[user_id_J1];
			double B_th_J1 = sysModel.Bth_list[uav_id][user_id_J1];
			if (uti_J_1 - accu_uti_list[J] >= 1e-9 and res_hard_bandwidth - B_th_J1 > 1e-9)
			{
				// 服务第J+1个
				double cap = sysModel.cap_list[uav_id][user_id_J1];
				double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id_J1];
				double uti = user_J1.utility(B_th_J1, cap, SNR_avg_dB);
				add_KnapsackResult(final_result, user_J1, B_th_J1, uti);

			}
			else
			{
				// 服务前J个
				for (int j = 0; j <= J; j++)
				{
					int user_id = efficient_list[j].first;
					User user = sysModel.users[user_id];
					double B_th = sysModel.Bth_list[uav_id][user_id];
					double cap = sysModel.cap_list[uav_id][user_id];
					double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];

					double uti = user.utility(B_th, cap, SNR_avg_dB);
					add_KnapsackResult(final_result, user, B_th, uti);
				}
			}
		}

		/*cout << " Final uav_result before reallocation: \n";
		PrintKnapsackResult(final_result);*/

		// 将剩余带宽重新分配给 elastic 用户
		double final_remaining_bandwidth = capacity - final_result.totalWeight;
		//cout << "\nStep 4: Final remaining bandwidth = " << final_remaining_bandwidth << endl;

		if (final_remaining_bandwidth > 1e-9)
		{
			// 重新计算 lambda_opt（只考虑 elastic 用户）


			double new_lambda_opt = W_sum / (capacity - final_result.hardWeight + C_inv_sum);

			/*cout << "  Recalculated lambda_opt = " << new_lambda_opt << " (old = " << lambda_opt << ")" << endl;*/

			// 更新所有 elastic 用户的分配
			for (auto& user_id : final_result.allocatedList)
			{
				User user = sysModel.users[user_id];
				if (user.uType == ELASTIC_UTILITY)
				{
					double cap = sysModel.cap_list[uav_id][user_id];

					double old_bandwidth = final_result.allocatedBandwidth[user_id];
					double new_bandwidth = (user.weight / (new_lambda_opt * log(2))) - (1.0 / cap);

					if (new_bandwidth < 0)
						new_bandwidth = 0.0;

					if (new_bandwidth > old_bandwidth + 1e-10)
					{
						double old_value = final_result.allocatedValue[user_id];
						double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];

						double new_value = user.utility(new_bandwidth, cap, SNR_avg_dB);
						double delta_bw = new_bandwidth - old_bandwidth;
						double delta_value = new_value - old_value;

						// 更新分配
						final_result.allocatedBandwidth[user_id] = new_bandwidth;
						final_result.allocatedValue[user_id] = new_value;
						final_result.totalWeight += delta_bw;
						final_result.totalValue += delta_value;
						final_result.elasticWeight += delta_bw;
						final_result.elasticValue += delta_value;

						/*cout << "  Updated Elastic User " << user_id << ": "
							<< old_bandwidth << " -> " << new_bandwidth
							<< " (+" << delta_bw << ")" << endl;*/
					}
				}

			}
		}
		return final_result;
	}


	// 打印最终结果
	/*cout << "\nWater Filling Algorithm EXPResult for UAV ID " << uav_id << ":\n";
	PrintKnapsackResult(final_result, uav_id);*/


}

vector<KKT_parameters> BAProblem::compute_KKT_parameters(int uav_id, const vector<User>& unproc_users)
{
	vector<KKT_parameters> kkt_params_list;

	// 首先计算efficient_list
	for (auto& user : unproc_users)
	{
		int user_id = user.ID;
		// 初始化当前user_id的KKT_parameters
		KKT_parameters params;
		params.user_id = user_id;
		params.W_sum = 0.0;
		params.C_inv_sum = 0.0;
		params.B_elastic_sum = 0.0;
		params.B_hard_sum = 0.0;
		params.efficient = 0.0;

		if (user.uType == HARD_UTILITY)
		{
			double rMin = user.rMin;
			double B_th = sysModel.Bth_list[uav_id][user_id];
			double efficiency = (user.weight * log2(1 + rMin)) / B_th;
			params.efficient = efficiency;

			kkt_params_list.push_back(params);
		}
		else if (user.uType == ELASTIC_UTILITY)
		{

			// 根据效用函数定义，效用定义为： U(b) = w * log_2(1 + b * C)
			// 因此其边际效用为： U'(b) = w * C / (ln(2) * (1 + b * C))
			// 其中C表示信道容量，即 C = log2(1 + SNR)
			double C = sysModel.cap_list[uav_id][user_id]; // 用户j的信道容量

			// 这里计算的是用户的边际效用在资源为0时的值
			double efficiency = user.weight * C / (log(2) * (1 + 0 * C));
			params.efficient = efficiency;
			kkt_params_list.push_back(params);

			// 这里计算的是用户的边际效用在资源为无人机总带宽时的值
			double total_bandwidth = sysModel.uavs[uav_id].total_bandwidth;
			double min_efficiency = user.weight * C / (log(2) * (1 + total_bandwidth * C));
			params.efficient = min_efficiency;
			kkt_params_list.push_back(params);

		}
		else
		{
			//cout << "Error in BAProblem::compute_KKT_parameters: Unknown user type for user_id_J1 = " << user_id_J1 << '\n';
			continue;
		}

	}
	KKT_parameters params0;
	params0.user_id = sysModel.n1 + sysModel.n2 + 1; // 斜率为0的虚拟用户
	params0.efficient = 0.0;
	kkt_params_list.push_back(params0);

	KKT_parameters params_inf;
	params_inf.user_id = -1; // 斜率为无穷大的虚拟用户
	params_inf.efficient = std::numeric_limits<double>::infinity();
	kkt_params_list.push_back(params_inf);

	// 将kkt_params_list按照efficient从大到小排序
	sort(kkt_params_list.begin(), kkt_params_list.end(),
		[](const KKT_parameters& param_a, const KKT_parameters& param_b) {
			return param_a.efficient > param_b.efficient;
		});

	// 然后构建W_sum，C_inv_sum，B_elastic_sum和B_hard_sum
	// 前缀和的形式
	double pre_hard_bandwidth = 0;
	for (int i = 0; i < kkt_params_list.size(); i++)
	{
		int user_id = kkt_params_list[i].user_id;
		if (user_id == -1)
		{
			// 此时为斜率无穷大的虚拟用户，跳过
			// 初始的累积消耗就是0
			continue;
		}
		if (user_id == sysModel.n1 + sysModel.n2 + 1)
		{
			// 此时为斜率为0的虚拟用户，elastic累积消耗是无穷大
			kkt_params_list[i].B_elastic_sum = std::numeric_limits<double>::infinity();
			// hard累积消耗是所有hard用户的最小带宽需求之和
			double B_hard_total = 0.0;
			for (auto& user : unproc_users)
			{
				if (user.uType == HARD_UTILITY)
				{
					int user_id = user.ID;
					double B_th = sysModel.Bth_list[uav_id][user_id];
					B_hard_total += B_th;
				}
			}
			kkt_params_list[i].B_hard_sum = B_hard_total;
			continue;
		}

		User& user = sysModel.users[user_id];
		// 初始化当前user_id的KKT_parameters
		KKT_parameters& params = kkt_params_list[i];
		if (i > 0)
		{
			// 计算当前user_id之前所有用户的参数和
			KKT_parameters& prev_params = kkt_params_list[i - 1];
			params.W_sum = prev_params.W_sum;
			params.C_inv_sum = prev_params.C_inv_sum;
			params.B_elastic_sum = prev_params.B_elastic_sum;

			// 如果上一个是hard用户，则累积其最小带宽需求
			params.B_hard_sum = prev_params.B_hard_sum + pre_hard_bandwidth;
			pre_hard_bandwidth = 0;
		}
		if (user.uType == HARD_UTILITY)
		{
			// 遍历当当前用户时，斜率刚好为hard用户斜率，此时设定为不消耗体积
			// 只有当后续更小的斜率时，才会累积该hard用户的最小带宽需求
			pre_hard_bandwidth = sysModel.Bth_list[uav_id][user_id];
			params.B_elastic_sum = params.W_sum / params.efficient - params.C_inv_sum;
		}
		else if (user.uType == ELASTIC_UTILITY)
		{
			params.W_sum = params.W_sum + user.weight / log(2);
			double C = sysModel.cap_list[uav_id][user_id];
			if (C > 1e-10)
				params.C_inv_sum = params.C_inv_sum + 1.0 / C;
			params.B_elastic_sum = params.W_sum / params.efficient - params.C_inv_sum;
		}
	}


	return kkt_params_list;
}

KnapsackResult BAProblem::FPTAS_singleUAV_new(Uav uav, vector<User> unproc_users, double epsilon)
{
	int uav_id = uav.ID;
	double capacity = uav.total_bandwidth;

	// cout << "user num " << unproc_users.size() << endl;
	// 基准解
	KnapsackResult wf_result = WaterFillingAlgorithm_singleUAV_new(uav, unproc_users);
	double wf_utility = wf_result.totalValue;

	// 分类用户
	vector<User> unproc_hard_users;
	vector<User> unproc_elastic_users;
	for (auto& user : unproc_users) {
		if (user.uType == HARD_UTILITY)
			unproc_hard_users.push_back(user);
		else
			unproc_elastic_users.push_back(user);
	}

	if (unproc_hard_users.empty()) {
		return wf_result;
	}

	// 计算 Elastic KKT 参数
	vector<KKT_parameters> WF_Params_elastic =
		compute_KKT_parameters(uav_id, unproc_elastic_users);

	// 准备 DP 数据
	vector<int> user_indices;
	vector<double> weights;
	vector<double> values;

	for (auto& user : unproc_hard_users) {
		int user_id = user.ID;
		double cap = sysModel.cap_list[uav_id][user_id];
		double bandwidth = sysModel.Bth_list[uav_id][user_id];
		double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];

		double utility = user.utility(bandwidth, cap, SNR_avg_dB);

		weights.push_back(bandwidth);
		values.push_back(utility);
		user_indices.push_back(user_id);
	}

	int n1 = weights.size();

	// 缩放参数
	double delta = (epsilon * wf_utility) / n1;
	int scaled_V = (int)((2 * n1) / epsilon);

	vector<int> scaled_values(n1);
	for (int i = 0; i < n1; i++) {
		scaled_values[i] = (int)(values[i] / delta);
	}

	// DP
	constexpr double INF = std::numeric_limits<double>::infinity();
	vector<double> dp(scaled_V + 1, INF);
	vector<vector<char>> selected(n1, vector<char>(scaled_V + 1, 0));
	dp[0] = 0.0;

	int current_max_v = 0;	// 记录考虑前i-1个用户时，能产生的最大价值

	// 经典动态规划过程：
	for (int i = 0; i < n1; i++) {
		// sv表示当前用户缩放后的价值
		int sv = scaled_values[i];
		if (sv <= 0) continue;

		int upper_bound = std::min(scaled_V, current_max_v + sv);

		for (int v = upper_bound; v >= sv; v--) {
			// 只在有效价值区间遍历，此区间外无可行解

			if (dp[v - sv] != INF) {
				// 满足此条件说明在当前价值为v时，不考虑用户i（减去i的价值）存在用户组合使其价值刚好为 v - sv, 这使得此时加入用户 i 可行。
				double new_weight = dp[v - sv] + weights[i];
				if (new_weight <= capacity + 1e-9 && new_weight < dp[v] - 1e-9) {
					// 考虑当前用户不违反容量约束，且

					dp[v] = new_weight;
					selected[i][v] = 1;
					if (v > current_max_v)
						current_max_v = v;
				}
			}
		}
		// 输出动态规划表格dp
		/*cout << "仅考虑前 " << i << " 个用户的动态规划表格, 其价值为 " << sv <<"\n\t";
		for (int v = 0; v <= upper_bound; v++)
		{
			cout << dp[v] << ", ";
		}
		cout << endl;*/
	}
	//cout << "current_max_v = " << current_max_v << "dp[max_v] = " << dp[current_max_v] << endl;
	//// 只打印 selected[i][v] == 1 的位置
	//cout << "Selected 数组中值为1的位置:" << endl;
	//for (int i = 0; i < n1; i++) {
	//	cout << "物品 " << i << ": ";
	//	bool found = false;
	//	for (int v = 0; v <= scaled_V; v++) {
	//		if (selected[i][v]) {
	//			cout << v << " ";
	//			found = true;
	//		}
	//	}
	//	if (!found) cout << "(无)";
	//	cout << endl;
	//}

	// 枚举组合
	double max_utility = 0.0;
	int max_hard_v = 0;
	int max_elastic_utility_l = -1;
	double max_elastic_lambda = 0.0;

	for (int v = 0; v <= current_max_v; v++) {
		double hard_bandwidth_consume = dp[v];
		if (hard_bandwidth_consume == INF)
			continue;

		double elastic_cap = capacity - hard_bandwidth_consume;
		if (elastic_cap < -1e-9)
			continue;

		double elastic_utility = 0.0;
		int l_elastic_last = -1;
		double lambda_opt = 0.0;

		if (!WF_Params_elastic.empty() && elastic_cap > 1e-9) {
			auto it = std::lower_bound(WF_Params_elastic.begin(),
				WF_Params_elastic.end(),
				elastic_cap,
				[](const KKT_parameters& elem, double cap) {
					return elem.B_elastic_sum < cap;
				});

			if (it != WF_Params_elastic.end()) {
				l_elastic_last = std::distance(WF_Params_elastic.begin(), it);
				if (l_elastic_last > 0 &&
					WF_Params_elastic[l_elastic_last].B_elastic_sum > elastic_cap + 1e-9) {
					l_elastic_last--;
				}
			}
			else {
				l_elastic_last = WF_Params_elastic.size() - 1;
			}

			if (l_elastic_last < 0) l_elastic_last = 0;

			KKT_parameters& params_elastic = WF_Params_elastic[l_elastic_last];
			double W_sum = params_elastic.W_sum;
			double C_inv_sum = params_elastic.C_inv_sum;

			if (elastic_cap + C_inv_sum > 1e-9) {
				lambda_opt = W_sum / (elastic_cap + C_inv_sum);

				for (int i = 0; i <= l_elastic_last; i++) {
					int user_id = WF_Params_elastic[i].user_id;
					User& u = sysModel.users[user_id];
					double cap = sysModel.cap_list[uav_id][user_id];

					if (cap < 1e-6) continue;

					if (lambda_opt > 1e-9) {
						double bandwidth = (u.weight / (lambda_opt * log(2))) - (1.0 / cap);
						bandwidth = std::max(0.0, std::min(bandwidth, elastic_cap));
						double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];

						if (bandwidth > 1e-10) {
							elastic_utility += u.utility(bandwidth, cap, SNR_avg_dB);
						}
					}
				}
			}
		}

		double hard_utility = v * delta;
		double total_utility = hard_utility + elastic_utility;

		if (total_utility > max_utility) {
			max_utility = total_utility;
			max_hard_v = v;
			max_elastic_utility_l = l_elastic_last;
			max_elastic_lambda = lambda_opt;
		}
	}

	// 回溯
	KnapsackResult final_result;
	final_result.uav_id = uav_id;

	int current_v = max_hard_v;
	for (int i = n1 - 1; i >= 0 && current_v > 0; i--)
	{
		int sv = scaled_values[i];
		if (sv <= 0) continue;

		if (current_v >= sv && selected[i][current_v])
		{
			int user_id = user_indices[i];
			User user = sysModel.users[user_id];
			double bandwidth = sysModel.Bth_list[uav_id][user_id];
			double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];

			double value = unproc_hard_users[i].utility(
				bandwidth, sysModel.cap_list[uav_id][user_id], SNR_avg_dB);

			add_KnapsackResult(final_result, user, bandwidth, value);

			current_v -= sv;
		}
	}

	// Elastic 用户
	if (max_elastic_utility_l >= 0) {
		for (int j = 0; j <= max_elastic_utility_l && j < WF_Params_elastic.size(); j++) {
			int user_id = WF_Params_elastic[j].user_id;
			User& user = sysModel.users[user_id];
			double cap = sysModel.cap_list[uav_id][user_id];

			if (cap < 1e-6) continue;

			double bandwidth = (user.weight / (max_elastic_lambda * log(2))) - (1.0 / cap);
			bandwidth = std::max(0.0, bandwidth);
			double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];


			if (bandwidth > 1e-10) {
				double value = user.utility(bandwidth, cap, SNR_avg_dB);

				add_KnapsackResult(final_result, user, bandwidth, value);
			}
		}
	}

	// 二次 Elastic 优化（借鉴 WaterFilling）
	double final_remaining = capacity - final_result.totalWeight;
	if (final_remaining > 1e-9 && !unproc_elastic_users.empty()) {
		double new_W_sum = 0.0, new_C_inv_sum = 0.0;
		for (auto& user : unproc_elastic_users) {
			new_W_sum += user.weight / log(2);
			double cap = sysModel.cap_list[uav_id][user.ID];
			if (cap > 1e-10)
				new_C_inv_sum += 1.0 / cap;
		}

		double new_lambda = new_W_sum / (capacity - final_result.hardWeight + new_C_inv_sum);

		for (auto& user : unproc_elastic_users) {
			int user_id = user.ID;
			double cap = sysModel.cap_list[uav_id][user_id];
			if (cap < 1e-6) continue;

			double old_bandwidth = final_result.allocatedBandwidth[user_id];
			double new_bandwidth = (user.weight / (new_lambda * log(2))) - (1.0 / cap);
			new_bandwidth = std::max(0.0, new_bandwidth);

			if (new_bandwidth > old_bandwidth + 1e-10) {
				double old_value = final_result.allocatedValue[user_id];
				double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];

				double new_value = user.utility(new_bandwidth, cap, SNR_avg_dB);
				double delta_bw = new_bandwidth - old_bandwidth;
				double delta_value = new_value - old_value;

				final_result.allocatedBandwidth[user_id] = new_bandwidth;
				final_result.allocatedValue[user_id] = new_value;
				final_result.totalWeight += delta_bw;
				final_result.totalValue += delta_value;
				final_result.elasticWeight += delta_bw;
				final_result.elasticValue += delta_value;
			}
		}
	}

	// 回溯完成后
	//cout << "\n===== Backtracking Verification =====" << endl;

	// 计算回溯结果的 scaled value
	double backtracked_scaled_value = 0.0;
	for (auto user_id : final_result.allocatedList) {
		// 找到该用户在 user_indices 中的索引
		auto it = find(user_indices.begin(), user_indices.end(), user_id);
		if (it != user_indices.end()) {
			int idx = distance(user_indices.begin(), it);
			backtracked_scaled_value += scaled_values[idx];
		}
	}

	/*cout << "Expected scaled value: " << max_hard_v << endl;
	cout << "Backtracked scaled value: " << backtracked_scaled_value << endl;
	cout << "Expected hard utility: " << (max_hard_v * delta) << endl;
	cout << "Actual hard utility: " << final_result.hardValue << endl;

	if (abs(backtracked_scaled_value - max_hard_v) > 0.5) {
		cout << "❌ ERROR: Backtracking mismatch!" << endl;
		cout << "  Difference: " << abs(backtracked_scaled_value - max_hard_v) << endl;
	}

	if (abs(final_result.hardValue - (max_hard_v * delta)) > 1.0) {
		cout << "❌ ERROR: Hard utility mismatch!" << endl;
		cout << "  Expected: " << (max_hard_v * delta) << endl;
		cout << "  Actual: " << final_result.hardValue << endl;
	}*/

	//cout << "wf_result = " << wf_utility << ", fptas result = " << final_result.totalValue << endl;
	return final_result;
}


void BAProblem::print_KKT_parameters(const vector<KKT_parameters>& kkt_params)
{
	cout << "KKT Parameters:\n";
	int index = 0;
	for (const auto& params : kkt_params)
	{
		int user_id = params.user_id;
		cout << index++ << " User ID: " << user_id
			<< "\tW_sum: " << params.W_sum
			<< "\tC_inv_sum: " << params.C_inv_sum
			<< "\tB_elastic_sum: " << params.B_elastic_sum
			<< "\tB_hard_sum: " << params.B_hard_sum
			<< "\tefficient: " << params.efficient << '\n';
	}
}

KnapsackResult BAProblem::KktBasedElasticUtility(Uav& uav, vector<User>& unproc_users, vector<double>& uti_max)
{
	// 根据obsidian笔记"[[003 算法设计2 混合效用的单无人机资源分配问题#2 3 2 对于连续部分：KKT条件找最优解]]"
	// 当中推导出了最优拉格朗日乘子λ的表达式

	// 2025年12月10日10:28:24 备注
	// 该该算法是多无人机分配问题中的子问题. 当用户被分配过后，其效用函数变为边际效用函数，在本方法中使用边际效用函数. 边际效用函数是一个分段函数，不能直接用KKT条件求解.
	// 针对这一问题，新的求解方法Pegging算法实现在函数BAProblem::PeggingAlgorithm中.


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
			sum_S += u.weight / log2(u.rMin + 1);
			sum_C += 1 / cap_list[uav_id][user_id];
		}

		// 计算λ的值
		lambda = sum_S / (uav.elastic_bandwidth + sum_C);

		// 记录最小的带宽值，初始化为无穷大
		double min_bandwidth = numeric_limits<double>::max();
		int min_band_user = -1; // 记录最小带宽对应的用户ID

		// 

		for (auto entry : allocated_users)
		{
			int user_id = entry.first;
			User& u = allusers[user_id];

			double Sij = u.weight / log2(u.rMin + 1);
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
			//cout << "User " << min_user << " removed from allocation due to negative min_band." << '\n';
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
	int uav_id = uav.ID;
	double capacity = uav.elastic_bandwidth; // 无人机当前带宽容量
	int n = unproc_users.size();
	vector<vector<double>>& cap_list = sysModel.cap_list;




	KnapsackResult result;	// 算法的解

	// 计算边际效用函数的0点/断点
	map<int, double> bw0; // {user_id_J1, breakpoint}
	for (auto& user : unproc_users)
	{
		int user_id = user.ID;
		double cap = cap_list[uav_id][user_id];
		if (cap <= 0)
			continue; // 避免除以0
		double max_utility = uti_max[user_id];
		double weight = user.weight;
		// 计算边际效用函数的0点
		double bwi = (exp(max_utility / weight) - 1) / (cap);
		// 
		bw0[user_id] = bwi;
	}

	map<int, double> bw;	// 记录被分配带宽的{用户ID, 带宽}
	// 初始化bw，每个用户的带宽为0
	for (auto& user : unproc_users)
	{
		int user_id = user.ID;
		bw.insert({ user_id, 0 });
	}
	vector<int> I_F; // 集合I_F，第k轮，仍处于可行区间的集合；初始包含所有用户
	for (auto& user : unproc_users)
	{
		int user_id = user.ID;
		I_F.push_back(user_id);
	}

	int k = 0;	 // 迭代计数器，防止死循环
	vector<int> Peged_L;	// 已经被固定在左侧区间的用户集合
	vector<int> Peged_R;	// 已经被固定在右侧区间的用户集合
	while (k < n)
	{
		cout << "--- Pegging Algorithm Iteration " << k << " ---\n";
		// 输出第k轮的I_F
		cout << "I_F: ";
		for (auto user_id : I_F)
		{
			cout << user_id << " ";
		}
		cout << endl;

		// 初始化第k轮
		vector<int> I_L; // 集合I_L，第k轮，处于左侧区间的集合; 边际效用为负数
		vector<int> I_R; // 集合I_R，第k轮，处于右侧区间的集合; 达到最大效用
		double S_L = 0; // $S_B^k = \sum_{j\in I_B^k}(b_{ij}^0 - b_{ij}^k)$; 第k轮，用户$i$少用的资源；
		double S_R = 0; // $S_A^k = \sum_{j\in I_A^k}(b_{ij}^k - B_{\text{elastic}})$; 第k轮，用户$i$多用的资源；

		// 解决第k轮的KKT条件
		// 先计算几个关键参数
		double sum_weight = 0;	// 计算I_F中所有用户的权重之和
		double sum_inv_cap = 0; // 计算I_F中所有用户的信道容量倒数之和
		for (auto user_id : I_F)
		{
			User& u = sysModel.users[user_id];
			sum_weight += u.weight;
			sum_inv_cap += 1.0 / cap_list[uav_id][user_id];
		}
		// 计算分配的带宽bw
		for (auto user_id : I_F)
		{
			User& u = sysModel.users[user_id];

			double weight = u.weight;
			double Cij = 1.0 / cap_list[uav_id][user_id];
			double lambda = sum_weight / (capacity + sum_inv_cap);
			double bandwidth_ij = (weight / lambda - Cij);
			bw[user_id] = bandwidth_ij;
		}
		// 输出计算得到的bw
		cout << "Calculated bandwidths (before pegging):\n";
		for (auto entry : bw)
		{
			int user_id = entry.first;
			double bandwidth = entry.second;
			double bandwidth0 = bw0[user_id];
			cout << "User ID: " << user_id << "\tBandwidth: " << bandwidth << "\tBreakpoint bw0: " << bandwidth0 << '\n';
		}

		// 检查每个用户的带宽分配情况，更新I_L, I_R, S_L, S_R
		for (auto user_id : I_F)
		{
			double bandwidth_ij = bw[user_id];
			if (bandwidth_ij < bw0[user_id])
			{
				// 用户处于左侧区间
				I_L.push_back(user_id);
				S_L += (bw0[user_id] - bandwidth_ij); // b_ij^k = 0
			}
			else if (bandwidth_ij > capacity)
			{
				// 用户处于右侧区间
				I_R.push_back(user_id);
				S_R += (capacity - bandwidth_ij);
			}
		}
		// 输出I_L和I_R
		cout << "I_L: ";
		for (auto user_id : I_L)
		{
			cout << user_id << " ";
		}
		cout << endl;
		cout << "I_R: ";
		for (auto user_id : I_R)
		{
			cout << user_id << " ";
		}
		cout << endl;


		// 判断I_L和I_R是否都为空，都为空就结束迭代
		if (I_L.empty() && I_R.empty()) {
			cout << "终止迭代" << k << "：I_L和I_R均为空。\n";
			break; // 迭代结束
		}

		// 按照S_L和S_R的大小关系更新I_F
		if (S_L >= S_R)
		{
			// 更新I_F = I_F - I_L
			vector<int> new_I_F;
			for (auto user_id : I_F)
			{
				// 如果user_id不在I_L中，则保留
				if (find(I_L.begin(), I_L.end(), user_id) == I_L.end())
				{
					new_I_F.push_back(user_id);
				}
				else
				{
					Peged_L.push_back(user_id);
				}
			}
			I_F = new_I_F;
		}
		else
		{
			// 更新I_F = I_F - I_R
			vector<int> new_I_F;
			for (auto user_id : I_F)
			{
				// 如果user_id不在I_R中，则保留
				if (find(I_R.begin(), I_R.end(), user_id) == I_R.end())
				{
					new_I_F.push_back(user_id);
				}
				else
				{
					Peged_R.push_back(user_id);
				}
			}
			I_F = new_I_F;
		}

		k += 1;
	}
	// 输出bw的分配结果
	cout << "Final bandwidth allocation after Pegging Algorithm:\n";
	for (auto entry : bw)
	{
		int user_id = entry.first;
		double bandwidth = entry.second;
		cout << "User ID: " << user_id << "\tBandwidth: " << bandwidth << '\n';
	}


	// 根据Pegging算法的结果，确定最终的带宽分配bw
	for (auto user_id : Peged_L)
		bw[user_id] = 0;
	for (auto user_id : Peged_R)
		bw[user_id] = capacity;


	// 根据最终的bw分配结果构造KnapsackResult
	for (auto entry : bw)
	{
		int user_id = entry.first;
		double bandwidth = entry.second;
		if (bandwidth > 1e-10) // 只记录被分配带宽的用户
		{
			User& u = sysModel.users[user_id];
			double value = u.marginal_utility(bandwidth, cap_list[uav_id][user_id], uti_max[user_id]);
			result.allocatedList.push_back(user_id);
			result.allocatedBandwidth.insert({ user_id, bandwidth });
			result.allocatedValue.insert({ user_id, value });
			result.totalWeight += bandwidth;
			result.totalValue += value;
		}
	}

	return result;
}



pair<vector<KnapsackResult>, map<int, UserResult>> BAProblem::approposed_multiUAV_allocation(vector<Uav> uavs, vector<User> users, int used_single_alg, double parameter)
{
	const int M = sysModel.m;                    // UAV数量
	const int N = sysModel.n1 + sysModel.n2;     // 用户总数


	// ================= 2026年2月13日20:44:04验证徐老师讨论 ====================
	// 先分别计算每個





	// ==================== 初始化结果容器 ====================
	std::vector<KnapsackResult> uav_results(M);
	for (int k = 0; k < M; k++) {
		uav_results[k].uav_id = sysModel.uavs[k].ID;
		clean_KnapsackResult(uav_results[k]);
		uav_results[k].uav_id = sysModel.uavs[k].ID; // clean后重设
	}

	// 1. 预先计算最大的 UAV ID，用于初始化返回结果的大小，防止 ID 越界
	int max_uav_id = -1;
	for (const auto& u : uavs) {
		if (u.ID > max_uav_id) max_uav_id = u.ID;
	}

	int num_initial_uavs = uavs.size();
	int num_initial_users = users.size();


	// 主循环：进行 N 轮分配，每轮分配一个 UAV
	// 注意：循环次数应基于初始 UAV 数量，但需要在循环内检查 uavs 是否为空
	for (int k = 0; k < num_initial_uavs; k++)
	{
		if (uavs.empty()) break; // 安全检查

		KnapsackResult best_result;
		double best_utility_gain = -INFINITY; // 建议设为负无穷 -INFINITY 或更小的数
		int best_uav_id = -1;

		// 输出当前轮次信息：
		// cout << "\n=== Multi-UAV Allocation Round " << k + 1 << " ===" << endl;
		// 输出剩余未分配的 UAV 列表
		//cout << "Remaining UAVs: ";
		// for(auto uav : uavs)
		// 	cout << " " << uav.ID;
		// cout << endl;
		// 输出每个无人机剩余的可服务用户数，每行打印10个
		//cout << " UAV Serviceable Users:" << endl;
		for (auto& uav : uavs)
		{
			int uav_id = uav.ID;
			int un_user_num = 0;
			vector<User>& serviceable_users = sysModel.uav_serviceable_users_map[uav_id];
			un_user_num = serviceable_users.size();

			// cout << "  UAV " << uav_id << ": ";
			// cout << un_user_num << "users." << endl;
			// for (size_t i = 0; i < serviceable_users.size(); i++)
			// {
			// 	cout << serviceable_users[i].ID;
			// 	if (i != serviceable_users.size() - 1)
			// 		cout << ", ";
			// 	if ((i + 1) % 10 == 0 && i != serviceable_users.size() - 1)
			// 		cout << "\n           "; // 每10个用户换行
			// }
			// cout << endl;
		}

		// 遍历剩余的 UAV
		for (auto& uav : uavs)
		{
			int uav_id = uav.ID;
			// cout << "outer uav id " << uav_id << endl;
			KnapsackResult uav_result;

			// 获取该 UAV 的可服务用户列表（使用 map 安全查找）
			vector<User>& serviceable_users = sysModel.uav_serviceable_users_map[uav_id];

			// 如果没有可服务用户，跳过计算，防止算法内部除0或空指针
			if (serviceable_users.empty()) continue;

			if (used_single_alg == 1)
			{
				uav_result = WaterFillingAlgorithm_singleUAV_new(uav, serviceable_users, 1);
			}
			else if (used_single_alg == 2)
			{
				// cout << "uav " << uav_id << endl;
				uav_result = FPTAS_singleUAV_new(uav, serviceable_users, parameter);
				// cout << "out" << endl;
			}
			else
			{
				cout << "Error: Unknown single UAV allocation algorithm selected." << endl;
				continue;
			}

			double utility_gain = uav_result.totalValue;

			// 贪心选择：找增益最大的
			if (utility_gain > best_utility_gain)
			{
				best_utility_gain = utility_gain;
				best_result = uav_result;
				best_uav_id = uav_id;
			}

			// 输出第一轮的每个 UAV 的分配结果
			if (k == 0)
			{

				//PrintKnapsackResult(uav_result, uav_id, 500);

			}
		}

		// 3. 关键修复：检查是否找到了有效的 UAV
		if (best_uav_id == -1)
		{
			cout << "Warning: No valid UAV allocation found in round " << k << endl;
			// 如果这一轮没有任何 UAV 能产生有效分配（比如都找不到用户，或者收益都比 -1 还低）
			// 此时应该跳出循环，或者移除第一个 UAV 以防死循环，视具体业务逻辑而定
			// 这里选择 break，因为剩下 UAV 都无法分配了
			break;
		}

		// 保存结果（此时 all_results 已扩容，best_uav_id 也是有效的）
		uav_results[best_uav_id] = best_result;

		//cout << "*********The chosen uav is uav" << best_uav_id << ", the seveble user num is " << sysModel.uav_serviceable_users_map[best_uav_id].size() << ", the seved user num is " << best_result.allocatedList.size() << endl;

		// 更新系统状态

		// 1. 移除已分配的 UAV
		uavs.erase(std::remove_if(uavs.begin(), uavs.end(),
			[best_uav_id](const Uav& uav) { return uav.ID == best_uav_id; }),
			uavs.end());

		// 2. 移除已被该 UAV 占用的用户
		// 这里的逻辑需要遍历所有剩余 UAV 的待选列表

		if (!best_result.allocatedList.empty())
		{
			// 遍历所有剩余 UAV 的 ID (直接遍历 map 即可，不需要遍历 uavs vector 再查找 map)
			for (auto& entry : sysModel.uav_serviceable_users_map)
			{
				int current_uav_id = entry.first;
				// 跳过刚刚分配完的那个 ID (虽然 map 里还有，但在 uavs vector 里已经被删了，逻辑上不需要处理)
				if (current_uav_id == best_uav_id) continue;

				// 当前uav的剩余可行用户列表
				vector<User>& users_list = entry.second;

				// 从待选列表中移除已被 best_result 分配的用户
				users_list.erase(std::remove_if(users_list.begin(), users_list.end(),
					[&best_result](const User& user) {
						// 检查 user.ID 是否存在于 best_result.allocatedList 中
						for (int allocated_id : best_result.allocatedList) {
							if (user.ID == allocated_id) return true; // 需要移除
						}
						return false; // 不需要移除
					}),
					users_list.end());
			}
		}
	}



	// 
	// for (auto& u_result : uav_results)
	// {
	// 	int uav_id = u_result.uav_id;
	// 	Uav uav = sysModel.uavs[uav_id];
	// 	vector<User> unpropsed_users;
	// 	for (auto user_id : u_result.allocatedList)
	// 	{
	// 		unpropsed_users.push_back(sysModel.users[user_id]);
	// 	}
	// 	u_result = WaterFillingAlgorithm_singleUAV_new(uav, unpropsed_users);
	// }

	// 对结果进行启发式的改进

	std::map<int, UserResult> user_results = construct_user_results(uav_results);

	return { uav_results, user_results };
}

pair<vector<KnapsackResult>, map<int, UserResult>> BAProblem::approposed_multiUAV_allocation_new(vector<Uav> uavs, vector<User> users, int used_single_alg, double parameter)
{
	const int M = sysModel.m;                    // UAV数量
	const int N = sysModel.n1 + sysModel.n2;     // 用户总数
	// cout << "uav nums" << M << endl;

	// ================= 2026年2月13日20:44:04验证徐老师讨论 ====================
	// 先分别计算每個





	// ==================== 初始化结果容器 ====================
	std::vector<KnapsackResult> uav_results(M);
	for (int k = 0; k < M; k++) {
		uav_results[k].uav_id = sysModel.uavs[k].ID;
		clean_KnapsackResult(uav_results[k]);
		uav_results[k].uav_id = sysModel.uavs[k].ID; // clean后重设
	}

	// 1. 预先计算最大的 UAV ID，用于初始化返回结果的大小，防止 ID 越界
	int max_uav_id = -1;
	for (const auto& u : uavs) {
		if (u.ID > max_uav_id) max_uav_id = u.ID;
	}

	int num_initial_uavs = uavs.size();
	int num_initial_users = users.size();


	// 主循环：进行 N 轮分配，每轮分配一个 UAV
	// 注意：循环次数应基于初始 UAV 数量，但需要在循环内检查 uavs 是否为空

	KnapsackResult best_result;

	// 遍历每个UAV
	for (auto& uav : uavs)
	{
		int uav_id = uav.ID;
		// cout << "outer uav id " << uav_id << endl;
		KnapsackResult uav_result;

		// 获取该 UAV 的可服务用户列表（使用 map 安全查找）
		vector<User>& serviceable_users = sysModel.uav_serviceable_users_map[uav_id];

		// 如果没有可服务用户，跳过计算，防止算法内部除0或空指针
		if (serviceable_users.empty()) continue;

		if (used_single_alg == 1)
		{
			uav_result = WaterFillingAlgorithm_singleUAV_new(uav, serviceable_users, 1);
		}
		else if (used_single_alg == 2)
		{
			// cout << "uav " << uav_id << endl;
			uav_result = FPTAS_singleUAV_new(uav, serviceable_users, parameter);
			// cout << "out" << endl;
		}
		else
		{
			cout << "Error: Unknown single UAV allocation algorithm selected." << endl;
			continue;
		}
		uav_results[uav_id] = uav_result;
	}


	// 如果在每次循环中没有剔除被服务的用户， 那么用户可能会被重复服务。此处需要去重；
	for (int i = 0; i < sysModel.n1 + sysModel.n2; i++)
	{
		int last_uav = -1;
		int best_uav = -1;
		double best_value = 0;
		for (auto& u_result : uav_results)
		{
			int uav_id = u_result.uav_id;

			if (u_result.allocatedValue.count(i) > 0)
			{
				// 用户i存在于当前无人机的分配中
				if (u_result.allocatedValue[i] - best_value >= EPS)
				{
					if (best_uav != -1)
					{
						// 在best_uav中移除用户i
						auto& u_list = uav_results[best_uav].allocatedList;
						// 假设要删除 u_list 中值为 i 的元素
						auto it = std::find(u_list.begin(), u_list.end(), i);

						if (it != u_list.end()) {
							// 将目标元素与末尾元素交换位置
							std::iter_swap(it, u_list.end() - 1);

							// 直接移除末尾元素，不涉及后续元素的平移
							u_list.pop_back();
						}
					}

					best_value = u_result.allocatedValue[i];
					best_uav = uav_id;
				}
				else
				{
					// 在uav_id中移除用户i
					auto& u_list = uav_results[uav_id].allocatedList;
					// 假设要删除 u_list 中值为 i 的元素
					auto it = std::find(u_list.begin(), u_list.end(), i);

					if (it != u_list.end()) {
						// 将目标元素与末尾元素交换位置
						std::iter_swap(it, u_list.end() - 1);

						// 直接移除末尾元素，不涉及后续元素的平移
						u_list.pop_back();
					}
				}

				last_uav = uav_id;
			}
		}
	}

	// 根据新的匹配关系，重新解
	for (auto& u_result : uav_results)
	{
		int uav_id = u_result.uav_id;
		Uav uav = sysModel.uavs[uav_id];
		vector<User> unp_users;
		for (auto user_id : u_result.allocatedList)
		{
			User user = sysModel.users[user_id];
			unp_users.push_back(user);
		}
		if (unp_users.size() == 0)
		{
			continue;
		}
		clean_KnapsackResult(u_result);
		if (used_single_alg == 1)
		{
			u_result = WaterFillingAlgorithm_singleUAV_new(uav, unp_users, 1);
		}
		else if (used_single_alg == 2)
		{
			u_result = FPTAS_singleUAV_new(uav, unp_users, parameter);
		}
	}


	// 
	/*for (auto& u_result : uav_results)
	{
		int uav_id = u_result.uav_id;
		Uav uav = sysModel.uavs[uav_id];
		vector<User> unpropsed_users;
		for (auto user_id : u_result.allocatedList)
		{
			unpropsed_users.push_back(sysModel.users[user_id]);
		}
		u_result = WaterFillingAlgorithm_singleUAV_new(uav, unpropsed_users);
	}*/

	// 对结果进行启发式的改进

	std::map<int, UserResult> user_results = construct_user_results(uav_results);

	return { uav_results, user_results };
}


void BAProblem::PrintKnapsackResult(const KnapsackResult& result, int uav_id, int maxItemsToPrint, int indent)
{
	string sp(indent, ' '); // 缩进字符串
	string lineSep(50, '-');

	cout << sp << lineSep << endl;
	cout << sp << " [Knapsack EXPResult Summary of UAV] " << uav_id << endl;
	cout << sp << lineSep << endl;

	// 1. 打印统计摘要 (使用 fixed 和 setprecision 控制小数位)
	cout << fixed << setprecision(4);
	cout << sp << "Total Value  : " << setw(10) << result.totalValue
		<< " (Hard: " << result.hardValue << ", Elastic: " << result.elasticValue << ")" << endl;
	cout << sp << "Total Weight : " << setw(10) << result.totalWeight
		<< " (Hard: " << result.hardWeight << ", Elastic: " << result.elasticWeight << ")" << endl;
	cout << sp << "Items Count  : " << result.allocatedList.size() << endl;

	// 2. 打印物品详情列表
	if (!result.allocatedList.empty()) {
		cout << sp << lineSep << endl;
		cout << sp << " [Allocated Items Detail"
			<< (maxItemsToPrint != -1 && result.allocatedList.size() > maxItemsToPrint ? " (Partial)" : "")
			<< "] " << endl;

		// 表头
		cout << sp << left << setw(10) << "User ID"
			<< left << setw(20) << "Bandwidth"
			<< left << setw(20) << "Value"
			<< left << setw(20) << "Efficiency" << endl;

		int count = 0;
		for (int uid : result.allocatedList) {
			// 控制打印数量
			if (maxItemsToPrint != -1 && count >= maxItemsToPrint) {
				cout << sp << "... (" << (result.allocatedList.size() - count) << " more items hidden) ..." << endl;
				break;
			}

			// 安全获取 map 中的值 (防止 map 中缺失 key 导致崩溃，虽然理论上不应发生)
			double bw = 0.0, val = 0.0, eff = 0.0;
			if (result.allocatedBandwidth.count(uid)) bw = result.allocatedBandwidth.at(uid);
			if (result.allocatedValue.count(uid)) val = result.allocatedValue.at(uid);
			if (result.allocatedValue.count(uid))
			{
				User user = sysModel.users[uid];
				double cap = sysModel.cap_list[uav_id][uid];
				eff = user.utility_derivative(bw, cap);
			}

			cout << sp << left << setw(10) << uid
				<< left << setw(20) << bw
				<< left << setw(20) << val
				<< left << setw(20) << eff << endl;

			count++;
		}
	}
	else {
		cout << sp << " (No items allocated)" << endl;
	}
	cout << sp << lineSep << endl;
	// 恢复默认流格式
	cout.unsetf(ios::fixed);
	cout << endl;

}

void BAProblem::PrintKnapsackResultList(const vector<KnapsackResult>& results, int maxItemsPerResult)
{
	double total_value = 0.0;
	cout << "========================================" << endl;
	cout << "      Knapsack Results List (Size: " << results.size() << ")" << endl;
	cout << "========================================" << endl;

	for (size_t i = 0; i < results.size(); ++i) {
		cout << "EXPResult #" << i + 1 << ":" << endl;
		// 调用单个打印函数，增加缩进以便区分
		PrintKnapsackResult(results[i], i, maxItemsPerResult, 2);
		total_value += results[i].totalValue;
	}
	cout << "========================================" << endl;
	cout << "total value of all results: " << total_value << endl << setprecision(3);
}

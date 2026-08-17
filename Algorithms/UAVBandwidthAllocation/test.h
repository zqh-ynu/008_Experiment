#pragma once
#include "EntityDefinition.h"
#include "config.h"

// 本文件主要用于测试各个函数的功能


// 在主函数或其他地方调用
// ExperimentConfig config = load_experiment_config("config.json");


// 测试Point类
void test_Point() {
	cout << "Testing Point class..." << endl
		<< "Creating Point p1 at (1, 2, 3)..." << endl;
	Point p1(1, 2, 3);
	cout << "Creating Point p2 at (4, 5, 6)..." << endl;
	Point p2(4, 5, 6);
	cout << "Distance between p1 and p2: " << Point::cal_distance(p1, p2) << endl;

	cout << "Horizontal distance between p1 and p2: " << Point::cal_horizontal_distance(p1, p2) << endl;
	cout << "Point class test completed." << endl << endl;
}

// 测试User类
void test_User() {
	cout << "Testing User class..." << endl
		<< "Creating User u1 at (1, 2, 0) with hard utility..." << endl;
	User u1(1, HARD_UTILITY, 1, 1, 2, 0, 10, 5, 0.1);
	u1.print_user();
	cout << "Creating User u2 at (3, 4, 0) with elastic utility..." << endl;
	User u2(2, ELASTIC_UTILITY, 1, 3, 4, 0, 5, 2, 0.05);
	u2.print_user();
	cout << "Creating User u3 at (5, 6, 0) with half-elastic utility..." << endl;
	User u3(3, HALFSOFT_UTILITY, 1, 5, 6, 0, 8, 3, 0.08);
	u3.print_user();
	cout << "User class test completed." << endl << endl;
}

// 测试UAV类
void test_UAV() {
	cout << "Testing UAV class..." << endl
		<< "Creating UAV a1 at (0, 0, 300)..." << endl;
	Uav a1(1, 0, 0, uav_alt, 20);
	a1.print_UAV();
	cout << "Creating UAV a2 at (100, 100, 300)..." << endl;
	Uav a2(2, 100, 100, uav_alt, 20);
	a2.print_UAV();
	cout << "UAV class test completed." << endl << endl;
}

// 测试Channel类
void test_Channel() {
	cout << "Testing Channel class..." << endl
		<< "Creating UAV a1 at (0, 0, 300)..." << endl;
	Uav a1(1, 0, 0, uav_alt, 20);
	cout << "Creating User u1 at (100, 100, 0) with hard utility..." << endl;
	User u1(1, HARD_UTILITY, 1, 100, 100, 0, 10, 5, 0.1);
	cout << "Creating Channel between a1 and u1..." << endl;
	Channel ch(a1, u1);
	ch.print_all();
	cout << "Channel class test completed." << endl << endl;
}

// 测试SystemMD类
void test_SystemMD() {
	cout << "Testing SystemMD class..." << endl;
	vector<User> users;
	users.emplace_back(1, HARD_UTILITY, 1, 10, 10, 0, 10, 5, 0.1);
	users.emplace_back(2, ELASTIC_UTILITY, 1, 20, 20, 0, 5, 2, 0.05);
	users.emplace_back(3, HALFSOFT_UTILITY, 1, 30, 30, 0, 8, 3, 0.08);
	vector<Uav> uavs;
	uavs.emplace_back(1, 0, 0, uav_alt, 20);
	uavs.emplace_back(2, 100, 100, uav_alt, 20);
	cout << "Creating SystemMD instance..." << endl;
	SystemMd sysModel(users, uavs);
	sysModel.print_all_users();
	sysModel.print_all_uavs();
	sysModel.print_dis_list();
	sysModel.print_SNRa_list();
	sysModel.print_SNRt_list();
	sysModel.print_M_list();
	sysModel.print_cap_list();
	cout << "SystemMD class test completed." << endl << endl;
}

// 测试BAProblem::FPTAS_0_1_knapsack函数


// 测试BAProblem::KKT_based_elastic_utility函数
void test_KKT_based_elastic_utility() {
	cout << "Testing KKT_based_elastic_utility function..." << endl;
	vector<User> users;
	users.emplace_back(0, HARD_UTILITY, 1, 10, 10, 0, 10, 5, 0.1);
	users.emplace_back(1, ELASTIC_UTILITY, 1, 20, 20, 0, 5, 2, 0.05);
	users.emplace_back(2, ELASTIC_UTILITY, 1, 30, 30, 0, 8, 3, 0.08);
	vector<Uav> uavs;
	uavs.emplace_back(0, 0, 0, uav_alt, 20);
	uavs[0].elastic_bandwidth = 0.0001; // 假设为软效用用户分配15MHz带宽
	SystemMd sysModel(users, uavs);

	BAProblem ba(sysModel);

	vector<double> existed_utilitys = { 0.0, 0.0, 0.0 }; // 假设所有用户初始效用为0

	KnapsackResult result = ba.KktBasedElasticUtility(uavs[0], users, existed_utilitys);
	cout << "Allocated bandwidths: ";
	ba.PrintKnapsackResult(result, 0);
	cout << "\nTotal allocated bandwidth: " << result.totalWeight << " MHz" << endl;
	cout << "Total utility: " << result.totalValue << endl;
	cout << "KKT_based_elastic_utility function test completed." << endl << endl;
}


void print_sorted_users(vector<User>& users)
{
	// 将users中的用户，按照单位资源效用从大到小排序，并输出用户id及其单位资源效用
	vector<pair<int, double>> user_utilities; // pair<用户ID, 单位资源效用>
	for (auto& u : users)
	{
		double unit_utility = u.weight / u.rMin; // 简单起见，单位资源效用定义为权重/最小速率需求
		user_utilities.push_back({ u.ID, unit_utility });
	}
	sort(user_utilities.begin(), user_utilities.end(), [](const pair<int, double>& param_a, const pair<int, double>& param_b) {
		return param_a.second > param_b.second; // 按单位资源效用从大到小排序
		});
	cout << "Users sorted by unit resource utility (weight/rMin):" << endl;
	for (auto& p : user_utilities)
	{
		cout << "User ID: " << p.first << ", Unit Resource Utility: " << p.second << endl;
	}
	cout << endl;
	// 将users中的用户，按照需求的最小数据速率降序排序，并输出用户id及其最小数据速率
	vector<pair<int, double>> user_rMin; // pair<用户ID, 最小数据速率>
	for (auto& u : users)
	{
		user_rMin.push_back({ u.ID, u.rMin });
	}
	sort(user_rMin.begin(), user_rMin.end(), [](const pair<int, double>& param_a, const pair<int, double>& param_b) {
		return param_a.second > param_b.second; // 按最小数据速率从大到小排序
		});
	cout << "Users sorted by minimum data rate (rMin):" << endl;
	for (auto& p : user_rMin)
	{
		cout << "User ID: " << p.first << ", Minimum Data Rate (rMin): " << p.second << endl;
	}
	cout << endl;
	// 将users中的用户，按照权重降序排序，并输出用户id及其权重
	vector<pair<int, int>> user_weight; // pair<用户ID, 权重>
	for (auto& u : users)
	{
		user_weight.push_back({ u.ID, u.weight });
	}
	sort(user_weight.begin(), user_weight.end(), [](const pair<int, int>& param_a, const pair<int, int>& param_b) {
		return param_a.second > param_b.second; // 按权重从大到小排序
		});
	cout << "Users sorted by weight:" << endl;
	for (auto& p : user_weight)
	{
		cout << "User ID: " << p.first << ", Weight: " << p.second << endl;
	}
	cout << endl;

}

// 测试BAProblem::RP_based_subproblem_allocation()函数
// 包含10个硬效用用户和10个弹性效用用户
// 按照以下规则生成实例：
// 用户：
//	1. 所有用户的x坐标与y坐标随机在(-250, 250)之间
//	2. 所有用户的权重随机在(1, 5)之间，且是整数
//	3. 所有用户的需求的最小数据速率随机分布在(0.5, 10)之间
//	4. 所有用户的需求的最大中断概率随机分布在(10e-1, 10e-3)之间
void test_RP_based_subproblem_allocation() {
	cout << "Testing RP_based_subproblem_allocation_FPTAS function..." << endl;
	// 设置随机种子
	// srand((unsigned int)time(NULL));
	// 为了结果可复现，使用固定种子
	srand(42);

	vector<User> users;
	// 10个硬效用用户
	for (int i = 0; i < 10; i++) {
		double x = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
		double y = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
		int weight = rand() % 5 + 1; // (1, 5)
		double rMin = (static_cast<double>(rand()) / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
		double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
		users.emplace_back(i, HARD_UTILITY, weight, x, y, 0, rMin, pOut);
	}
	// print_sorted_users(users);

	// 10个弹性效用用户
	for (int i = 10; i < 20; i++) {
		double x = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
		double y = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
		int weight = rand() % 5 + 1; // (1, 5)
		double rMin = (static_cast<double>(rand()) / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
		double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
		users.emplace_back(i, ELASTIC_UTILITY, weight, x, y, 0, rMin, pOut);
	}
	vector<Uav> uavs;
	uavs.emplace_back(0, 0, 0, uav_alt, 20);
	SystemMd sysModel(users, uavs);
	BAProblem ba(sysModel);

	vector<double> existed_utilities(20, 0.0); // 假设所有用户初始效用为0

	double epsilon = 0.5;
	map<double, KnapsackResult> result_map = ba.RP_based_subproblem_allocation_experiment1(0, uavs[0].total_bandwidth, users, existed_utilities);

	// 输出result_map
	cout << "Results of RP_based_subproblem_allocation_FPTAS:" << endl;



	// 找到最大效用对应的比例
	double max_utility = 0;
	double best_ratio = 0;
	for (const auto& entry : result_map) {
		double ratio = entry.first;
		KnapsackResult result = entry.second;
		cout << "Ratio: " << ratio << endl;
		ba.PrintKnapsackResult(result, 0);
		cout << '\n';


		if (result.totalValue > max_utility) {
			max_utility = result.totalValue;
			best_ratio = ratio;
		}
	}
	cout << "Best Ratio: " << best_ratio << ", Max Utility: " << max_utility << endl;

	cout << "RP_based_subproblem_allocation_FPTAS function test completed." << endl << endl;

	// 按以下格式将结果写到文件
	// 第一行："ContinuousRatio, TotalUtility, ElasticUtility, HardUtility"
	// 往后的每一行：result_map.first, result_map.second.totalValue
	string fname = experimentDataPath + "\\testSearchMethod\\" + "RP_result_addRemain1.csv";
	ofstream outfile(fname);
	if (!outfile.is_open()) {
		cerr << "Error opening file for writing: " << fname << endl;
		return;
	}
	outfile << "ContinuousRatio, TotalUtility, ElasticUtility, HardUtility, TotalAllocatedBandwidth, ElasticAllocatedBandwidth, HardAllocatedBandwidth\n";
	for (const auto& entry : result_map) {
		double ratio = entry.first;
		KnapsackResult result = entry.second;
		outfile << ratio << ", " << result.totalValue << ", " << result.elasticValue << ", " << result.hardValue << ", "
			<< result.totalWeight << ", " << result.elasticWeight << ", " << result.hardWeight << "\n";
	}
	outfile.close();
}


// 测试BAProblem::RP_based_subproblem_allocation2()函数
// 包含10个硬效用用户和10个弹性效用用户
// 按照以下规则生成实例：
// 用户：
//	1. 所有用户的x坐标与y坐标随机在(-1000, 1000)之间
//	2. 所有用户的权重随机在(1, 5)之间，且是整数
//	3. 所有用户的需求的最小数据速率随机分布在(0.5, 10)之间
//	4. 所有用户的需求的最大中断概率随机分布在(10e-1, 10e-3)之间
void test_RP_based_subproblem_allocation2() {
	cout << "Testing RP_based_subproblem_allocation2 function..." << endl;
	// 设置随机种子
	// srand((unsigned int)time(NULL));
	// 为了结果可复现，使用固定种子
	srand(42);
	vector<User> users;
	int hard_user_num = 50;
	int elastic_user_num = 50;

	// 10个硬效用用户
	for (int i = 0; i < hard_user_num; i++) {
		//	1. 所有用户的x坐标与y坐标随机在(-1000, 1000)之间
		double x = (static_cast<double>(rand()) / RAND_MAX) * 2000 - 1000; // (-1000, 1000)
		double y = (static_cast<double>(rand()) / RAND_MAX) * 2000 - 1000; // (-1000, 1000)
		int weight = rand() % 5 + 1; // (1, 5)
		double rMin = (static_cast<double>(rand()) / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
		double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
		users.emplace_back(i, HARD_UTILITY, weight, x, y, 0, rMin, pOut);
	}
	// print_sorted_users(users);
	// 10个弹性效用用户
	for (int i = hard_user_num; i < hard_user_num + elastic_user_num; i++) {
		//	1. 所有用户的x坐标与y坐标随机在(-1000, 1000)之间
		double x = (static_cast<double>(rand()) / RAND_MAX) * 2000 - 1000; // (-1000, 1000)
		double y = (static_cast<double>(rand()) / RAND_MAX) * 2000 - 1000; // (-1000, 1000)
		int weight = rand() % 5 + 1; // (1, 5)
		double rMin = (static_cast<double>(rand()) / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
		double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
		users.emplace_back(i, ELASTIC_UTILITY, weight, x, y, 0, rMin, pOut);
	}
	vector<Uav> uavs;
	uavs.emplace_back(0, 0, 0, uav_alt, 20);
	SystemMd sysModel(users, uavs);
	sysModel.print_all_users();
	sysModel.print_dis_list();
	sysModel.print_M_list();
	sysModel.print_SNRa_list();
	sysModel.print_SNRt_list();
	sysModel.print_cap_list();

	BAProblem ba(sysModel);

	vector<double> existed_utilitys(hard_user_num + elastic_user_num, 0.0); // 假设所有用户初始效用为0

	double delta = 0.1;
	double epsilon = 0.5;
	map<double, KnapsackResult> result_map = ba.RP_based_subproblem_allocation_experiment2(0, uavs[0].total_bandwidth, users, existed_utilitys);
	// 输出result_map
	cout << "Results of RP_based_subproblem_allocation2:" << endl;
	// 找到最大效用对应的硬资源量
	double max_utility = 0;
	double best_hard_bandwidth = 0;
	for (auto& entry : result_map) {
		double hard_bandwidth = entry.first;
		KnapsackResult result = entry.second;
		ba.PrintKnapsackResult(result, 0);
		cout << '\n';
		if (result.totalValue > max_utility) {
			max_utility = result.totalValue;
			best_hard_bandwidth = hard_bandwidth;
		}
	}
	cout << "Best Hard Bandwidth: " << best_hard_bandwidth << ", Max Utility: " << max_utility << endl;
	cout << "RP_based_subproblem_allocation2 function test completed." << endl << endl;
	// 按以下格式将结果写到文件
	// 第一行："HardBandwidth, TotalUtility, ElasticUtility, HardUtility"
	// 往后的每一行：result_map.first, result_map.second.totalValue
	string fname = experimentDataPath + "\\testSearchMethod\\" + "RP_result2_addRemain1_range1000.csv";
	ofstream outfile(fname);
	if (!outfile.is_open()) {
		cerr << "Error opening file for writing: " << fname << endl;
		return;
	}
	outfile << "HardBandwidth, TotalUtility, ElasticUtility, HardUtility, TotalAllocatedBandwidth, ElasticAllocatedBandwidth, HardAllocatedBandwidth\n";
	for (auto& entry : result_map) {
		double hard_bandwidth = entry.first;
		KnapsackResult result = entry.second;
		outfile << hard_bandwidth << ", " << result.totalValue << ", " << result.elasticValue << ", " << result.hardValue << ", "
			<< result.totalWeight << ", " << result.elasticWeight << ", " << result.hardWeight << "\n";
	}
	outfile.close();
}

// 测试BAProblem::RP_based_subproblem_allocation()函数
// 包含10个硬效用用户和10个半软效用用户
// 按照以下规则生成实例：
// 用户：
//	1. 所有用户的x坐标与y坐标随机在(-250, 250)之间
//	2. 所有用户的权重随机在(1, 5)之间，且是整数
//	3. 所有用户的需求的最小数据速率随机分布在(0.5, 10)之间
//	4. 所有用户的需求的最大中断概率随机分布在(10e-1, 10e-3)之间
void test_RP_based_subproblem() {
	cout << "Testing RP_based_subproblem function..." << endl;
	// 设置随机种子
	// srand((unsigned int)time(NULL));
	// 为了结果可复现，使用固定种子
	srand(42);
	vector<User> users;
	// 10个硬效用用户
	for (int i = 0; i < 10; i++) {
		double x = ((double)rand() / RAND_MAX) * 500 - 250; // (-250, 250)
		double y = ((double)rand() / RAND_MAX) * 500 - 250; // (-250, 250)
		int weight = rand() % 5 + 1; // (1, 5)
		double rMin = ((double)rand() / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
		double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
		users.emplace_back(i, HARD_UTILITY, weight, x, y, 0, rMin, pOut);
	}
	// print_sorted_users(users);
	// 10个半软效用用户
	for (int i = 10; i < 20; i++) {
		double x = ((double)rand() / RAND_MAX) * 500 - 250; // (-250, 250)
		double y = ((double)rand() / RAND_MAX) * 500 - 250; // (-250, 250)
		int weight = rand() % 5 + 1; // (1, 5)
		double rMin = ((double)rand() / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
		double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
		users.emplace_back(i, HALFSOFT_UTILITY, weight, x, y, 0, rMin, pOut);
	}

	vector<double> existed_utilitys(20, 0.0); // 假设所有用户初始效用为0

	vector<Uav> uavs;
	uavs.emplace_back(0, 0, 0, uav_alt, 20);
	SystemMd sysModel(users, uavs);
	BAProblem ba(sysModel);
	double epsilon = 0.5;
	KnapsackResult result = ba.RP_based_subproblem_allocation_FPTAS(0, uavs[0].total_bandwidth, users, existed_utilitys);
	// 输出result
	cout << "Results of RP_based_subproblem_allocation_FPTAS:" << endl;
	ba.PrintKnapsackResult(result, 0);
	cout << '\n';
	cout << "RP_based_subproblem_allocation_FPTAS function test completed." << endl << endl;

}


// 测试std::pair< std::vector<KnapsackResult>, std::vector<std::pair<int,double>> > BAProblem::local_search_allocation
// 包含10个硬效用用户和10个半软效用用户
// 按照以下规则生成实例：
// 用户：
//	1. 所有用户的x坐标与y坐标随机在(-250, 250)之间
//	2. 所有用户的权重随机在(1, 5)之间，且是整数
//	3. 所有用户需求的最小数据速率随机分布在(0.5, 10)之间
//	4. 所有用户需求的最大中断概率随机分布在(10e-1, 10e-3)之间
// UAV：
//	1. 所有UAV的x坐标与y坐标随机在(-250, 250)之间
//	2. 所有UAV的z坐标固定为0
//	3. 所有UAV的带宽随机分布在(10, 20)之间，且是整数
void test_local_search_allocation() {
	cout << "Testing local_search_allocation function..." << endl;

	string exp_config_file = experimentDataPath + "testSearchMethod\\simulation_config_uavNum2.json";
	ExperimentConfig exp_config = load_experiment_config(exp_config_file);
	SystemMd sysModel = generate_instance(exp_config);



	int hard_user_num = sysModel.n1;
	int elastic_user_num = sysModel.n2;
	vector<User>& users = sysModel.users;
	vector<Uav>& uavs = sysModel.uavs;

	BAProblem ba(sysModel);


	auto LS_result = ba.local_search_allocation();
	vector<KnapsackResult>& knapsack_result_list = LS_result.first;	// 记录每个UAV服务的用户
	vector<UserResult>& allocation_result = LS_result.second; // 记录每个uesr被分配的带宽
	cout << "Results of local_search_allocation:" << endl;
	cout << "Total Utility: " << BAProblem::get_total_utility(knapsack_result_list) << endl;
	cout << "Allocated Users:\n";
	for (int user_id = 0; user_id < hard_user_num + elastic_user_num; user_id++) {
		int uav_id = allocation_result[user_id].uav_id;
		double bandwidth = allocation_result[user_id].allocated_bandwidth;
		double user_utility = allocation_result[user_id].utility;
		// 保留5位小数
		cout << fixed << setprecision(5);
		cout << "\tUser " << user_id << ": UAV " << uav_id << ", " << bandwidth << " MHz, Utility: " << user_utility << '\n';
	}
	cout << '\n';

	// 输出每个无人机服务的用户
	cout << "Allocated Users by UAV:\n";
	for (int uav_id = 0; uav_id < uavs.size(); uav_id++) {
		ba.PrintKnapsackResult(knapsack_result_list[uav_id], uav_id);
	}
	cout << "local_search_allocation function test completed." << endl << endl;



	// 验证 LS_result 中每个用户的效用是否正确
	// 通过将分配给用户的带宽代入其效用函数计算得到
	cout << "Verifying user utilities..." << endl;
	for (int uav_id = 0; uav_id < uavs.size(); uav_id++) {
		auto& kr = knapsack_result_list[uav_id];
		for (auto user_id : kr.allocatedList)
		{
			User& user = users[user_id];
			double bandwidth = kr.allocatedBandwidth[user_id];
			double alg_value = kr.allocatedValue[user_id];	// 算法计算的效用

			double channel_cap = sysModel.cap_list[uav_id][user_id]; // 信道容量
			double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];

			double cal_value = user.utility(bandwidth, channel_cap, SNR_avg_dB);

			if (abs(alg_value - cal_value) > 1e-5) {
				cout << "Mismatch for User " << user_id << ": alg_value = " << alg_value << ", cal_value = " << cal_value << endl;
			}
			else {
				cout << "Match for User " << user_id << ": utility = " << alg_value << endl;
			}
		}
	}

	// 输出所有与用户id=12的用户相关的信息，包括用户信息，所有UAV与该用户的信道容量，分配结果
	int test_user_id = 12;
	cout << "Details for User " << test_user_id << ":\n";
	users[test_user_id].print_user();
	for (int uav_id = 0; uav_id < uavs.size(); uav_id++) {
		double channel_cap = sysModel.cap_list[uav_id][test_user_id];
		cout << "UAV " << uav_id << " Channel Capacity: " << channel_cap << " Mbps\n";
	}
	// 查找该用户被分配的UAV及带宽，效用
	int assigned_uav_id = allocation_result[test_user_id].uav_id;
	double assigned_bandwidth = allocation_result[test_user_id].allocated_bandwidth;
	double assigned_utility = allocation_result[test_user_id].utility;
	cout << "Assigned UAV: " << assigned_uav_id << ", Bandwidth: " << assigned_bandwidth << " MHz, Utility: " << assigned_utility << endl;


	cout << "local_search_allocation verification completed." << endl << endl;

}


// 测试RP_based_subproblem_allocation_Greedy函数
void test_RP_based_subproblem_allocation_Greedy() {
	cout << "Testing RP_based_subproblem_allocation_Greedy function..." << endl;
	string exp_config_file = experimentDataPath + "testSearchMethod\\simulation_config_uavNum1.json";
	ExperimentConfig exp_config = load_experiment_config(exp_config_file);
	SystemMd sysModel = generate_instance(exp_config);


	int hard_user_num = sysModel.n1;
	int elastic_user_num = sysModel.n2;
	vector<User>& users = sysModel.users;
	vector<Uav>& uavs = sysModel.uavs;

	BAProblem ba(sysModel);
	// 存储每个用户的最大效用，初始化为0
	vector<double> uti_max(users.size(), 0.0);
	auto EXPResult = ba.RP_based_subproblem_allocation_Greedy(0, uavs[0].total_bandwidth, users, uti_max);
	cout << fixed << setprecision(5);
}


// 测试PeggingAlgorithm
void test_PeggingAlgorithm() {
	cout << "Testing PeggingAlgorithm..." << endl;

	string exp_config_file = experimentDataPath + "testSearchMethod\\simulation_config_uavNum1.json";
	ExperimentConfig exp_config = load_experiment_config(exp_config_file);
	SystemMd sysModel = generate_instance(exp_config);


	int hard_user_num = sysModel.n1;
	int elastic_user_num = sysModel.n2;
	vector<User>& users = sysModel.users;
	vector<Uav>& uavs = sysModel.uavs;
	BAProblem ba(sysModel);
	// 存储每个用户的最大效用，初始化为0
	vector<double> uti_max(users.size(), 0.0);
	// 构造elastic用户列表
	vector<User> elastic_users;
	for (User& user : users) {
		if (user.uType == ELASTIC_UTILITY) {
			elastic_users.push_back(user);
		}
	}

	uavs[0].elastic_bandwidth = uavs[0].total_bandwidth;
	KnapsackResult result = ba.PeggingAlgorithm(uavs[0], elastic_users, uti_max);
	// 输出结果
	cout << "Results of PeggingAlgorithm:" << endl;
	ba.PrintKnapsackResult(result, 0);

	cout << "PeggingAlgorithm test completed." << endl << endl;
}

// 测试函数 KnapsackResult BAProblem::WaterFillingAlgorithm_singleUAV(int uav_id, double capacity, vector<User>& unproc_users)
void test_WaterFillingAlgorithm() {
	cout << "Testing WaterFillingAlgorithm_singleUAV..." << endl;
	string exp_config_file = experimentDataPath + "testSearchMethod\\simulation_config_files\\simulation_config_uavNum1.json";
	string globle_config_file = algProjPath + "config\\def_config.json";

	ExperimentConfig exp_config = load_experiment_config(exp_config_file);
	load_global_channel_config(globle_config_file);
	SystemMd sysModel = generate_instance(exp_config);

	sysModel.print_SystemInfo();

	int hard_user_num = sysModel.n1;
	int elastic_user_num = sysModel.n2;
	vector<User>& users = sysModel.users;
	vector<Uav>& uavs = sysModel.uavs;
	BAProblem ba(sysModel);


	//KnapsackResult result = ba.WaterFillingAlgorithm_singleUAV(uavs[0], users);
	KnapsackResult result = ba.WaterFillingAlgorithm_singleUAV_new(uavs[0], users);
	// 输出结果
	cout << "Results of WaterFillingAlgorithm_singleUAV:" << endl;
	ba.PrintKnapsackResult(result, 0);
	cout << "WaterFillingAlgorithm_singleUAV test completed." << endl << endl;
}

// 测试KnapsackResult BAProblem::FPTAS_singleUAV(int uav_id, double capacity, vector<User>& unproc_users, double epsilon)
void test_FPTAS_singleUAV() {
	cout << "Testing FPTAS_singleUAV..." << endl;
	string exp_config_file = experimentDataPath + "testSearchMethod\\simulation_config_files\\simulation_config_uavNum1.json";
	string globle_config_file = algProjPath + "config\\def_config.json";

	ExperimentConfig exp_config = load_experiment_config(exp_config_file);
	load_global_channel_config(globle_config_file);
	SystemMd sysModel = generate_instance(exp_config);
	sysModel.print_SystemInfo();

	int hard_user_num = sysModel.n1;
	int elastic_user_num = sysModel.n2;
	vector<User>& users = sysModel.users;
	vector<Uav>& uavs = sysModel.uavs;
	BAProblem ba(sysModel);



	// 构造elastic用户列表
	vector<User> elastic_users;
	for (User& user : users) {
		if (user.uType == ELASTIC_UTILITY) {
			elastic_users.push_back(user);
		}
	}
	uavs[0].total_bandwidth = 2;
	uavs[0].elastic_bandwidth = uavs[0].total_bandwidth;
	KnapsackResult result = ba.FPTAS_singleUAV_new(uavs[0], users, 0.02);


	// 输出结果
	cout << "Results of FPTAS_singleUAV:" << endl;
	ba.PrintKnapsackResult(result, 0);
	cout << "FPTAS_singleUAV test completed." << endl << endl;
}

// 在 test.h 中添加

void test_SystemMd_from_file() {
	cout << "Testing SystemMd::SystemMd(string, string, string)..." << endl;

	// 1. 创建临时的 user.csv
	string user_csv = experimentDataPath + "data\\County_loc_Type_req\\simulated_data\\1_1000users_data_130822.csv";

	// 2. 创建临时的 uav.csv
	string uav_csv = experimentDataPath + "data\\County_loc_Type_req\\simulated_data\\1_10uavs_loc_130822.csv";

	// 3. 配置文件 (假设路径存在，或者使用已有的配置文件路径)
	// 这里为了测试运行，假设 load_global_channel_config 可以处理不存在的文件或者给一个虚拟路径
	// 实际运行时请指向真实的 config.json
	string config_file = algProjPath + "config\\def_config.json";

	// 4. 调用构造函数
	// 注意：这里的用户和无人机经纬度差异巨大（用户在浙江，无人机在河北/北京附近）
	// 坐标原点会是所有点中经纬度最小的。
	// User Min Lon: ~119.906, UAV Min Lon: ~117.485 -> 原点 Lon 117.485
	// User Min Lat: ~28.455, UAV Min Lat: ~40.419 -> 原点 Lat 28.455
	SystemMd sys(user_csv, uav_csv, config_file);

	// 5. 验证数据
	cout << "\n--- Verification ---" << endl;
	sys.print_SystemInfo(20);

	// 验证转换逻辑
	// 检查第一个 User (ID 0, 原 2380) 的 rMin 是否为 1.68/1000 = 0.00168 Mbps
	if (sys.users.size() > 0) {
		cout << "User 0 rMin (Expected 0.00168): " << sys.users[0].rMin << endl;
		if (abs(sys.users[0].rMin - 0.00168) < 1e-6) cout << "PASS: rMin conversion." << endl;
		else cout << "FAIL: rMin conversion." << endl;
	}

	// 验证 Hard 用户的中断概率
	// User 2 (ID 2, 原 2134) 是 Hard, pOut 应该是 0.1
	if (sys.users.size() > 2) {
		cout << "User 2 pOut (Expected 0.1): " << sys.users[2].pOut << endl;
		if (abs(sys.users[2].pOut - 0.1) < 1e-6) cout << "PASS: pOut reading." << endl;
		else cout << "FAIL: pOut reading." << endl;
	}

	// 验证 UAV 带宽
	if (sys.uavs.size() > 0) {
		cout << "UAV 0 Bandwidth (Expected 50): " << sys.uavs[0].total_bandwidth << endl;
	}

	// 6. 清理临时文件 (可选)
	// remove(user_csv.c_str());
	// remove(uav_csv.c_str());

	cout << "test_SystemMd_from_file completed." << endl << endl;
}

/// <summary>
/// 测试凸松弛和舍入算法
/// </summary>
void test_ConvexRelaxationAndRounding() {
	cout << "========================================" << endl;
	cout << "测试: ConvexRelaxationAndRounding算法" << endl;
	cout << "========================================" << endl;

	// 1. 创建系统模型
	// 根据您的项目配置文件路径修改
	string user_file = experimentDataPath + "data/variable_user_num/1000u_num/user_data/1_1000users_data_440515.csv";
	string uav_file = experimentDataPath + "data/variable_user_num/1000u_num/uav_data/1_10uavs_loc_440515.csv";
	string config_file = algProjPath + "config/def_config.json";

	SystemMd sysModel(user_file, uav_file, config_file);


	//sysModel.print_SystemInfo(20);



	cout << "\n系统信息:" << endl;
	cout << "  UAV数量: " << sysModel.m << endl;
	cout << "  硬用户数量: " << sysModel.n1 << endl;
	cout << "  弹性用户数量: " << sysModel.n2 << endl;
	cout << "  总用户数: " << (sysModel.n1 + sysModel.n2) << endl;

	/*Uav uav = sysModel.uavs[0];
	vector<Uav> test_uavs;
	test_uavs.push_back(uav);
	sysModel.uavs = test_uavs;
	sysModel.m = 1;*/
	// sysModel.print_SystemInfo(20);


	// 2. 创建带宽分配问题实例
	BAProblem problem(sysModel);

	// 3. 运行凸松弛和舍入算法
	cout << "\n开始运行凸松弛和舍入算法..." << endl;
	cout << "二分查找容差: 1e-4" << endl;

	auto start_time = chrono::high_resolution_clock::now();
	// auto result = problem.ConvexRelaxationAndRounding_multiUAV(1e-4);

	//auto result = problem.MatchingSQP_Allocation();
	// auto result = problem.MatchingGameAllocation();
	// auto result = problem.SADA_Allocation();
	auto result = problem.HungarianMatchingAllocation();
	// auto result = problem.approposed_multiUAV_allocation_new(sysModel.uavs, sysModel.users, 2);

	auto end_time = chrono::high_resolution_clock::now();
	auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

	cout << "\n算法运行完成，耗时: " << duration.count() << " ms" << endl;

	// 4. 获取结果
	vector<KnapsackResult>& uavResults = result.first;
	map<int, UserResult>& userResults = result.second;


	// 5. 检查是否找到可行解
	if (uavResults.empty()) {
		cout << "\n警告: 未找到可行解!" << endl;
		return;
	}

	// 6. 打印每个UAV的分配结果
	cout << "\n========================================" << endl;
	cout << "UAV分配结果" << endl;
	cout << "========================================" << endl;
	problem.PrintKnapsackResultList(uavResults, 500);


	// 7. 打印用户视角的结果
	/*cout << "\n========================================" << endl;
	cout << "用户分配结果" << endl;
	cout << "========================================" << endl;

	int served_hard_users = 0;
	int served_elastic_users = 0;

	for (int i = 0; i < userResults.size(); i++) {
		if (userResults[i].uav_id >= 0) {
			string type = (i < sysModel.n1) ? "硬用户" : "弹性用户";
			cout << "用户 " << i << " (" << type << "): "
				<< "连接UAV " << userResults[i].uav_id << ", "
				<< "带宽=" << fixed << setprecision(4) << userResults[i].allocated_bandwidth << " MHz, "
				<< "效用=" << setprecision(4) << userResults[i].utility << endl;

			if (i < sysModel.n1) {
				served_hard_users++;
			}
			else {
				served_elastic_users++;
			}
		}
	}*/

	// 8. 打印统计信息
	/*cout << "\n========================================" << endl;
	cout << "统计信息" << endl;
	cout << "========================================" << endl;
	cout << "系统总效用: " << fixed << setprecision(6) << total_system_utility << endl;
	cout << "  硬用户效用: " << setprecision(6) << total_hard_utility << endl;
	cout << "  弹性用户效用: " << setprecision(6) << total_elastic_utility << endl;
	cout << "总带宽使用: " << setprecision(4) << total_bandwidth_used << " MHz" << endl;
	cout << "总带宽容量: " << setprecision(4) << (sysModel.m * sysModel.uavs[0].total_bandwidth) << " MHz" << endl;
	cout << "带宽利用率: " << setprecision(2) << (total_bandwidth_used / (sysModel.m * sysModel.uavs[0].total_bandwidth) * 100) << "%" << endl;
	cout << "服务的硬用户数: " << served_hard_users << " / " << sysModel.n1 << endl;
	cout << "服务的弹性用户数: " << served_elastic_users << " / " << sysModel.n2 << endl;*/

	// 9. 验证约束满足情况
	cout << "\n========================================" << endl;
	cout << "约束验证" << endl;
	cout << "========================================" << endl;

	bool all_constraints_satisfied = true;

	// 约束1: 每个用户最多连接1个UAV
	for (int i = 0; i < userResults.size(); i++) {
		int connection_count = 0;
		for (int k = 0; k < uavResults.size(); k++) {
			if (find(uavResults[k].allocatedList.begin(),
				uavResults[k].allocatedList.end(), i) != uavResults[k].allocatedList.end()) {
				connection_count++;
			}
		}
		if (connection_count > 1) {
			cout << "  ✗ 用户 " << i << " 连接了 " << connection_count << " 个UAV (违反约束1)" << endl;
			all_constraints_satisfied = false;
		}
	}

	// 约束3: 每个UAV的总带宽不超过容量
	for (int k = 0; k < uavResults.size(); k++) {
		if (uavResults[k].totalWeight > sysModel.uavs[k].total_bandwidth + 1e-6) {
			cout << "  ✗ UAV " << k << " 分配带宽 " << uavResults[k].totalWeight
				<< " 超过容量 " << sysModel.uavs[k].total_bandwidth << " (违反约束3)" << endl;
			all_constraints_satisfied = false;
		}
	}

	// 约束4: 硬用户的最小速率保证
	for (int i = 0; i < sysModel.n1; i++) {
		if (userResults[i].uav_id >= 0) {
			int k = userResults[i].uav_id;
			double allocated_bw = userResults[i].allocated_bandwidth;
			double SNR_linear = pow(10.0, sysModel.SNRth_list[k][i] / 10.0);
			double achieved_rate = allocated_bw * log2(1.0 + SNR_linear);
			double required_rate = sysModel.users[i].rMin;

			if (achieved_rate < required_rate - 1e-6) {
				cout << "  ✗ 硬用户 " << i << " 实际速率 " << achieved_rate
					<< " 低于需求 " << required_rate << " (违反约束4)" << endl;
				all_constraints_satisfied = false;
			}
		}
	}

	// 验证elastic用户效用是否符合
	for (auto& u_result : userResults)
	{
		int user_id = u_result.first;
		int uav_id = u_result.second.uav_id;
		if (uav_id == -1)
			continue;
		double uti_result = u_result.second.utility;
		double band_result = u_result.second.allocated_bandwidth;
		double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_id];
		double cap = sysModel.cap_list[uav_id][user_id];
		double real_uti = sysModel.users[user_id].utility(band_result, cap, SNR_avg_dB);

		if (abs(real_uti - uti_result) > 1e-9)
		{
			cout << "user" << user_id << " has wrong utility: uti_result = " << uti_result << ", real_uti = " << real_uti << endl;
		}

	}

	if (all_constraints_satisfied) {
		cout << "  ✓ 所有约束均满足!" << endl;
	}

	cout << "\n========================================" << endl;
	cout << "测试完成" << endl;
	cout << "========================================" << endl;
}

// 测试vector<KnapsackResult> BAProblem::approposed_multiUAV_allocation(vector<Uav> uavs, vector<User> users, double used_single_alg, double parameter)
void test_approposed_multiUAV_allocation() {
	cout << "========================================" << endl;
	cout << "测试: Approposed Multi-UAV Allocation算法" << endl;
	cout << "========================================" << endl;
	// 1. 创建系统模型
	// 根据您的项目配置文件路径修改
	string user_file = experimentDataPath + "data\\County_loc_Type_req\\simulated_data\\50_1000users_data_510723.csv";
	string uav_file = experimentDataPath + "data\\County_loc_Type_req\\simulated_data\\50_10uavs_loc_510723.csv";
	string config_file = algProjPath + "config\\def_config.json";
	SystemMd sysModel(user_file, uav_file, config_file);
	sysModel.print_SystemInfo();

	// 将所有UAV的带宽设置为20 MHz，以便测试
	/*for (auto& uav : sysModel.uavs) {
		uav.total_bandwidth = 50.0;
	}*/
	sysModel.print_SystemInfo(20);
	// 2. 创建带宽分配问题实例
	BAProblem problem(sysModel);
	// 3. 运行 Approposed Multi-UAV Allocation 算法
	cout << "\n开始运行 Approposed Multi-UAV Allocation 算法..." << endl;
	auto start_time = chrono::high_resolution_clock::now();

	vector<Uav> test_uavs;
	test_uavs.push_back(sysModel.uavs[0]); // 仅测试第一个UAV

	auto results = problem.approposed_multiUAV_allocation(sysModel.uavs, sysModel.users, 2);
	auto uavResults = results.first;
	auto end_time = chrono::high_resolution_clock::now();
	auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
	cout << "\n算法运行完成，耗时: " << duration.count() << " ms" << endl;
	// 4. 获取结果
	// vector<KnapsackResult>& uavResults = result;
	// 5. 检查是否找到可行解
	if (uavResults.empty()) {
		cout << "\n警告: 未找到可行解!" << endl;
		return;
	}
	// 6. 打印uavResults，每个UAV的分配结果
	cout << "\n========================================" << endl;
	cout << "UAV分配结果" << endl;
	problem.PrintKnapsackResultList(uavResults, 10);

}

// 测试两种方法的多UAV分配效果对比
void test_multiUAV_allocation_comparison() {
	// 在文件夹E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\data\County_loc_Type_req\simulated_data下，有50组用户和UAV数据文件
	// 用户文件名格式：50_1000users_data_xxYYZZ.csv
	// UAV文件名格式：50_10uavs_loc_xxYYZZ.csv

	// 针对每一组数据，分别使用
	// approposed_multiUAV_allocation(sysModel.uavs, sysModel.users, 1); 其中 used_single_alg=1，表示使用WaterFillingAlgorithm_singleUAV_new
	// approposed_multiUAV_allocation(sysModel.uavs, sysModel.users, 2); 其中 used_single_alg=2，表示使用FPTAS_singleUAV_new

	// 用分别记录方法的平均效用

	//===========================
	//具体实现：

	// 1. 获取数据文件列表
	cout << "Testing multi-UAV allocation comparison..." << endl;
	string config_file = algProjPath + "config\\def_config.json";


	string dataFilePath = experimentDataPath + "data\\County_loc_Type_req\\simulated_data\\";
	vector<string> userFiles;
	vector<string> uavFiles;

	bool success = getMatchedFilePairs(
		dataFilePath,
		dataFilePath,
		"1000users_data",   // User 文件关键词
		"10uavs_loc",       // UAV 文件关键词
		50,                 // 只需要前 50 组
		userFiles,          // [输出]
		uavFiles            // [输出]
	);
	if (!success) {
		cout << "Failed to load data files." << endl;
		return;
	}
	cout << "Successfully loaded " << userFiles.size() << " pairs of data." << endl;
	// 打印验证
	if (!userFiles.empty()) {
		cout << "First Pair Check: " << endl;
		cout << "   User: " << fs::path(userFiles[0]).filename() << endl;
		cout << "   UAV : " << fs::path(uavFiles[0]).filename() << endl;
	}

	// 定义两个变量分别存储两种方法的总效用
	double total_utility_method1 = 0.0;
	double total_utility_method2 = 0.0;

	// 3. 循环处理每一组数据
	for (size_t k = 0; k < userFiles.size(); ++k) {
		cout << "Processing group " << (k + 1) << "..." << endl;

		// 此处加载数据到 sysModel
		SystemMd sysModel(userFiles[k], uavFiles[k], config_file);

		BAProblem problem1(sysModel);
		// 方法1: used_single_alg=1 (WaterFillingAlgorithm_singleUAV_new)
		auto results1 = problem1.approposed_multiUAV_allocation(sysModel.uavs, sysModel.users, 1);
		auto uavResults1 = results1.first;
		total_utility_method1 += BAProblem::get_total_utility(uavResults1);
		problem1.~BAProblem();

		// 方法2: used_single_alg=2 (FPTAS_singleUAV_new)
		BAProblem problem2(sysModel);
		auto results2 = problem2.approposed_multiUAV_allocation(sysModel.uavs, sysModel.users, 2);
		auto uavResults2 = results2.first;
		total_utility_method2 += BAProblem::get_total_utility(uavResults2);
		problem2.~BAProblem();
	}

	// 4. 计算平均效用
	double average_utility_method1 = total_utility_method1 / userFiles.size();
	double average_utility_method2 = total_utility_method2 / userFiles.size();

	// 5. 输出结果
	cout << "\n========================================" << endl;
	cout << "Multi-UAV Allocation Comparison Results" << endl;
	cout << "========================================" << endl;
	cout << "Method 1 (WaterFillingAlgorithm_singleUAV_new) Average Utility: " << fixed << setprecision(6) << average_utility_method1 << endl;
	cout << "Method 2 (FPTAS_singleUAV_new) Average Utility: " << fixed << setprecision(6) << average_utility_method2 << endl;


}
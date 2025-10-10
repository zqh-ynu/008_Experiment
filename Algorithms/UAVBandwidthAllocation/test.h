#pragma once
#include "EntityDefinition.h"

// 本文件主要用于测试各个类的功能




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
	cout << "Creating User u3 at (5, 6, 0) with half-soft utility..." << endl;
	User u3(3, HALFSOFT_UTILITY, 1, 5, 6, 0, 8, 3, 0.08);
	u3.print_user();
	cout << "User class test completed." << endl << endl;
}

// 测试UAV类
void test_UAV() {
	cout << "Testing UAV class..." << endl
		<< "Creating UAV a1 at (0, 0, 300)..." << endl;
	Uav a1(1, 0, 0, uav_H, 20);
	a1.print_UAV();
	cout << "Creating UAV a2 at (100, 100, 300)..." << endl;
	Uav a2(2, 100, 100, uav_H, 20);
	a2.print_UAV();
	cout << "UAV class test completed." << endl << endl;
}

// 测试Channel类
void test_Channel() {
	cout << "Testing Channel class..." << endl
		<< "Creating UAV a1 at (0, 0, 300)..." << endl;
	Uav a1(1, 0, 0, uav_H, 20);
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
	uavs.emplace_back(1, 0, 0, uav_H, 20);
	uavs.emplace_back(2, 100, 100, uav_H, 20);
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
	uavs.emplace_back(0, 0, 0, uav_H, 20);
	uavs[0].soft_bandwidth = 0.0001; // 假设为软效用用户分配15MHz带宽
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
	sort(user_utilities.begin(), user_utilities.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
		return a.second > b.second; // 按单位资源效用从大到小排序
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
	sort(user_rMin.begin(), user_rMin.end(), [](const pair<int, double>& a, const pair<int, double>& b) {
		return a.second > b.second; // 按最小数据速率从大到小排序
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
	sort(user_weight.begin(), user_weight.end(), [](const pair<int, int>& a, const pair<int, int>& b) {
		return a.second > b.second; // 按权重从大到小排序
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
	cout << "Testing RP_based_subproblem_allocation function..." << endl;
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
	uavs.emplace_back(0, 0, 0, uav_H, 20);
	SystemMd sysModel(users, uavs);
	BAProblem ba(sysModel);

	vector<double> existed_utilitys(20, 0.0); // 假设所有用户初始效用为0

	double epsilon = 0.5;
	map<double, KnapsackResult> result_map = ba.RP_based_subproblem_allocation_experiment1(0, uavs[0].total_bandwidth, users, existed_utilitys);

	// 输出result_map
	cout << "Results of RP_based_subproblem_allocation:" << endl;



	// 找到最大效用对应的比例
	double max_utility = 0;
	double best_ratio = 0;
	for (const auto& entry : result_map) {
		double ratio = entry.first;
		KnapsackResult result = entry.second;
		cout << "Ratio: " << ratio << endl;
		ba.PrintKnapsackResult(result);
		cout << '\n';


		if (result.totalValue > max_utility) {
			max_utility = result.totalValue;
			best_ratio = ratio;
		}
	}
	cout << "Best Ratio: " << best_ratio << ", Max Utility: " << max_utility << endl;

	cout << "RP_based_subproblem_allocation function test completed." << endl << endl;
	
	// 按以下格式将结果写到文件
	// 第一行："ContinuousRatio, TotalUtility, SoftUtility, HardUtility"
	// 往后的每一行：result_map.first, result_map.second.totalValue
	string fname = experimentDataPath + "\\testSearchMethod\\" + "RP_result_addRemain1.csv";
	ofstream outfile(fname);
	if (!outfile.is_open()) {
		cerr << "Error opening file for writing: " << fname << endl;
		return;
	}
	outfile << "ContinuousRatio, TotalUtility, SoftUtility, HardUtility, TotalAllocatedBandwidth, SoftAllocatedBandwidth, HardAllocatedBandwidth\n";
	for (const auto& entry : result_map) {
		double ratio = entry.first;
		KnapsackResult result = entry.second;
		outfile << ratio << ", " << result.totalValue << ", " << result.softValue << ", " << result.hardValue << ", "
			<< result.totalWeight << ", " << result.softWeight << ", " << result.hardWeight << "\n";
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
	uavs.emplace_back(0, 0, 0, uav_H, 20);
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
		ba.PrintKnapsackResult(result);
		cout << '\n';
		if (result.totalValue > max_utility) {
			max_utility = result.totalValue;
			best_hard_bandwidth = hard_bandwidth;
		}
	}
	cout << "Best Hard Bandwidth: " << best_hard_bandwidth << ", Max Utility: " << max_utility << endl;
	cout << "RP_based_subproblem_allocation2 function test completed." << endl << endl;
	// 按以下格式将结果写到文件
	// 第一行："HardBandwidth, TotalUtility, SoftUtility, HardUtility"
	// 往后的每一行：result_map.first, result_map.second.totalValue
	string fname = experimentDataPath + "\\testSearchMethod\\" + "RP_result2_addRemain1_range1000.csv";
	ofstream outfile(fname);
	if (!outfile.is_open()) {
		cerr << "Error opening file for writing: " << fname << endl;
		return;
	}
	outfile << "HardBandwidth, TotalUtility, SoftUtility, HardUtility, TotalAllocatedBandwidth, SoftAllocatedBandwidth, HardAllocatedBandwidth\n";
	for (auto& entry : result_map) {
		double hard_bandwidth = entry.first;
		KnapsackResult result = entry.second;
		outfile << hard_bandwidth << ", " << result.totalValue << ", " << result.softValue << ", " << result.hardValue << ", "
			<< result.totalWeight << ", " << result.softWeight << ", " << result.hardWeight << "\n";
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
	uavs.emplace_back(0, 0, 0, uav_H, 20);
	SystemMd sysModel(users, uavs);
	BAProblem ba(sysModel);
	double epsilon = 0.5;
	KnapsackResult result = ba.RP_based_subproblem_allocation(0, uavs[0].total_bandwidth, users, existed_utilitys);
	// 输出result
	cout << "Results of RP_based_subproblem_allocation:" << endl;
	ba.PrintKnapsackResult(result);
	cout << '\n';
	cout << "RP_based_subproblem_allocation function test completed." << endl << endl;
	
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
	// 设置固定随机种子
    srand(42);
	int soft_user_num = 10;
    int hard_user_num = 10;
    vector<User> users;
    for (int i = 0; i < hard_user_num; i++) {
        double x = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
        double y = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
        int weight = rand() % 5 + 1; // (1, 5)
        double rMin = (static_cast<double>(rand()) / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
        double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
        users.emplace_back(i, HARD_UTILITY, weight, x, y, 0, rMin, pOut);
    }
    for (int i = hard_user_num; i < hard_user_num + soft_user_num; i++) {
        double x = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
        double y = (static_cast<double>(rand()) / RAND_MAX) * 500 - 250; // (-250, 250)
        int weight = rand() % 5 + 1; // (1, 5)
        double rMin = (static_cast<double>(rand()) / RAND_MAX) * 9.5 + 0.5; // (0.5, 10)
        double pOut = pow(10, -(rand() % 3 + 1)); // (10e-1, 10e-3)
        users.emplace_back(i, ELASTIC_UTILITY, weight, x, y, 0, rMin, pOut);
    }
    
    vector<Uav> uavs;
    uavs.emplace_back(0, 125, 0, uav_H, 10);
    uavs.emplace_back(1, -125, 0, uav_H, 20);

    SystemMd sysModel(users, uavs);
	// 输出所有用户，所有UAV
	sysModel.print_all_users();
	sysModel.print_all_uavs();
	// 输出sysModel
    sysModel.print_M_list();
    sysModel.print_SNRa_list();
    sysModel.print_SNRt_list();
    sysModel.print_cap_list();
    sysModel.print_dis_list();

    BAProblem ba(sysModel);


    auto LS_result = ba.local_search_allocation();
	vector<KnapsackResult>& knapsack_result_list = LS_result.first;	// 记录每个UAV服务的用户
    vector<UserResult>& allocation_result = LS_result.second; // 记录每个uesr被分配的带宽
	cout << "Results of local_search_allocation:" << endl;
    cout << "Total Utility: " << BAProblem::get_total_utility(knapsack_result_list) << endl;
    cout << "Allocated Users:\n";
    for (int user_id = 0; user_id < hard_user_num + soft_user_num; user_id++) {
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
		BAProblem::PrintKnapsackResult(knapsack_result_list[uav_id], uav_id);
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

			double cal_value = user.utility(bandwidth, channel_cap);

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

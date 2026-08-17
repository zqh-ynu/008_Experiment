// B2022.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "EntityDefinition.h"
#include "config.h"

int main() {
	
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

		// 执行算法
	}
}


// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件




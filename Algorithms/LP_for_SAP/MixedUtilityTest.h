#pragma once
#include "EntityDefinition.h"



void test_alpha_parametrized_fractional_KP()
{
	int hardItemCount = 10; // 硬效用物品数量
	int softItemCount = 10; // 软效用物品数量
	int besteffertItemCount = 0; // 重量效用物品数量

	int binCount = 1;

	double a[] = { 1, 2, 1, 3, 2, 4, 1, 5, 3, 6 };
	double b[] = { 5, 10, 3, 9, 8, 16, 6, 30, 6, 12 };
	double w[] = { 2, 5, 7, 4, 9, 3, 5, 10, 13, 6 };


	double** thresholds = new double* [binCount];
	for (int i = 0; i < binCount; ++i) {
		thresholds[i] = new double[hardItemCount + softItemCount + besteffertItemCount];
	}
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < hardItemCount + softItemCount + besteffertItemCount; j++) {
			thresholds[i][j] = rand() % 10 + 1; // 随机设置物品最小资源需求
		}
	}

	vector<Bin> bins(binCount);
	for (int j = 0; j < binCount; j++) {
		Bin b;
		b.cap = 30; // 随机设置箱子容量
		b.load = 0; // 初始负载为0
		b.id = j; // 设置箱子ID
		bins[j] = b;
	}
	vector<Item> items(hardItemCount + softItemCount + besteffertItemCount);
	for(int j = 0; j< hardItemCount; j++)
	{
		Item item;
		item.id = j; // 设置物品ID
		// 随机设置物品权重
		item.weight = rand() % 15 + 1;
		item.type = HARD_UTILITY; // 设置物品类型
		items[j] = item;
	}
	for (int j = hardItemCount; j < hardItemCount + softItemCount; j++) {
		Item item;
		item.id = j; // 设置物品ID
		item.weight = w[j - hardItemCount]; // 随机设置物品权重
		item.type = (j < hardItemCount) ? HARD_UTILITY : (j < hardItemCount + softItemCount) ? SOFT_UTILITY : BESTEFFERT_UTILITY; // 设置物品类型
		items[j] = item;
	}

	

	KPMU kpmu(hardItemCount, softItemCount, besteffertItemCount, binCount);
	kpmu.init(thresholds, bins, items);

	kpmu.print_all();

	vector<double> X_S;
	vector<double> objectives;
	vector<double> alphas;
	for (double alpha = 0; alpha <= 20; alpha += 0.1) {
		double objective = kpmu.relexed_alpha_parametrized_fractional_KP(0, X_S, alpha);
		objectives.push_back(objective);
		alphas.push_back(alpha);
		// 输出X_S
		printf("X_S: ");
		for (double x : X_S) {
			printf("%.2f ", x);
		}
		printf("\n");
		// 输出alpha和目标函数值
		printf("Alpha: %.2f, Objective: %.2f\n\n", alpha, objective);
	}

	// 将结果输出到文件
	string filepath = experimentDataPath + "testAlphaParametrizedFractionalKP\\";
	string filename = filepath + "alpha_parametrized_fractional_KP_results.txt";
	ofstream resultFile(filename);
	if (!resultFile.is_open()) {
		cerr << "Error opening file for writing results." << endl;
		return;
	}
	// 写入标题行
	resultFile << "Alpha,Objective\n";
	// 写入每个alpha和对应的目标函数值
	for (size_t i = 0; i < alphas.size(); ++i) {
		resultFile << fixed << setprecision(2) << alphas[i] << "," << objectives[i] << "\n";
	}
	resultFile.close();

	for (int i = 0; i < binCount; ++i) {
		delete[] thresholds[i]; // 释放每一行的内存
	}
	delete[] thresholds; // 释放二维数组的内存

	

}
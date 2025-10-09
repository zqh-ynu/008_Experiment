#pragma once

// B2022.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include "EntityDefinition.h"



void test1() {
	int itemCount = 4; // 物品数量
	int binCount = 3; // 箱子数量
	Bin b0;
	Bin b1;
	Bin b2;
	b0.cap = 2;
	b1.cap = 3;
	b2.cap = 4;
	vector<Bin> bins = { b0, b1, b2 };
	double** itemWeights = new double* [binCount];
	double** profits = new double* [binCount];
	for (int i = 0; i < binCount; ++i) {
		itemWeights[i] = new double[itemCount];
		profits[i] = new double[itemCount];
	}
	itemWeights[0][0] = 1.0; itemWeights[0][1] = 2.0; itemWeights[0][2] = 2.0; itemWeights[0][3] = 1.0; // 物品0的权重
	itemWeights[1][0] = 1.0; itemWeights[1][1] = 3.0; itemWeights[1][2] = 3.0; itemWeights[1][3] = 2.0; // 物品1的权重
	itemWeights[2][0] = 1.0; itemWeights[2][1] = 3.0; itemWeights[2][2] = 4.0; itemWeights[2][3] = 3.0; // 物品2的权重

	profits[0][0] = 3.0; profits[0][1] = 1.0; profits[0][2] = 5.0; profits[0][3] = 25.0; // 物品0的利润
	profits[1][0] = 1.0; profits[1][1] = 1.0; profits[1][2] = 15.0; profits[1][3] = 15.0; // 物品1的利润
	profits[2][0] = 5.0; profits[2][1] = 1.0; profits[2][2] = 25.0; profits[2][3] = 5.0; // 物品2的利润   

	SAP sap(itemCount, binCount);
	sap.init(itemWeights, profits, bins);
	// sap.print_all();
	vector<int> selectedBins = sap.LP_based_approximation();

	
}


void test2()
{
    int itemCount = 50; // 物品数量
    int binCount = 10; // 箱子数量

	vector<Bin> bins(binCount);
	for (int i = 0; i < binCount; ++i) {
		Bin b;
		b.cap = rand() % 100 + 1; // 随机设置箱子容量
		b.load = 0; // 初始负载为0
		b.id = i; // 设置箱子ID
		bins[i] = b;
	}
	double** itemWeights = new double* [binCount];
	double** profits = new double* [binCount];
	for (int i = 0; i < binCount; ++i) {
		itemWeights[i] = new double[itemCount];
		profits[i] = new double[itemCount];
	}
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < itemCount; ++j) {
			itemWeights[i][j] = rand() % 50 + 1; // 随机设置物品权重
			profits[i][j] = rand() % 20 + 1; // 随机设置物品利润
		}
	}
	SAP sap(itemCount, binCount);
	sap.init(itemWeights, profits, bins);
	// sap.print_all();
	vector<int> selectedBins = sap.LP_based_approximation();
	for (int i = 0; i < selectedBins.size(); ++i) {
		cout << "Selected bin for item " << i << ": " << selectedBins[i] << endl;
	}

	// 计算最终的总价值和总重量
	double totalValue = 0.0;
	double totalWeight = 0.0;
	for (int i = 0; i < itemCount; ++i) {
		int binIndex = selectedBins[i];
		if (binIndex != -1) { // 如果物品被分配到某个箱子
			totalValue += profits[binIndex][i]; // 累加利润
			totalWeight += itemWeights[binIndex][i]; // 累加权重
		}
	}
	cout << "Total Value: " << totalValue << endl;
	cout << "Total Weight: " << totalWeight << endl;
	// 计算每个箱子的负载
	for (int i = 0; i < binCount; ++i) {
		double load = 0.0;
		for (int j = 0; j < itemCount; ++j) {
			if (selectedBins[j] == i) { // 如果物品 j 被分配到箱子 i
				load += itemWeights[i][j]; // 累加物品的权重
			}
		}
		bins[i].load = load; // 更新箱子的负载
	}
	for (int i = 0; i < binCount; ++i) {
		cout << "Bin " << i << ": Load = " << bins[i].load << ", Capacity = " << bins[i].cap << endl;
	}

	for (int i = 0; i < binCount; ++i) {
		delete[] itemWeights[i];
		delete[] profits[i];
	}
	
	
	cout << "Test completed successfully." << endl;
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

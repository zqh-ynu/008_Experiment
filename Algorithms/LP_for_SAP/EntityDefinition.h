#pragma once
#include "predefine.h"

typedef vector<int> S;	// 一个匹配集合，其中每一项是一个物品的ID
#define HARD_UTILITY 1 // 硬效用类型
#define SOFT_UTILITY 2 // 软效用类型
#define BESTEFFERT_UTILITY 3 // 重量效用类型
class Item;
class Bin;
class SAP;

struct KnapsackResult {
	vector<int> selectedItems; // 选中的物品ID列表
	double totalValue;         // 总价值
	double totalWeight;        // 总重量
};

// 根据item类型，有不同的效用函数
class Item
{
public:
	int id = 0; // 物品的ID
	int type = HARD_UTILITY; // 物品的类型
	int weight = 1; // 物品的重要性

	// 根据item类型，有不同的效用函数
	// y是分配的资源，threshold是最小资源阈值
	double utility(double y, double threshold);

	double hard_utility(double y, double threshold) const {
		if (y >= threshold) return weight;
		else return 0;
	}
	double soft_utility(double y, double threshold, double beta = 0.5, double p = 1, double q = 1) const {
		if (y * q < threshold) {
			return weight * beta * exp(p * (y * q - threshold)); // 软效用：使用sigmoid函数计算效用
		}
		else {
			return weight * (1 - (1 - beta) * exp((-p) * (y * q - threshold))); // 软效用：使用sigmoid函数计算效用
		}
	}
	/* * 计算软效用函数的导数的逆函数
	 * 该函数用于计算在给定alpha、threshold、beta、p和q的情况下，
	 * 使得soft_utility等于alpha的y（资源）值。
	 *
	 * @param alpha 软效用函数的导数的值
	 * @param threshold 最小资源阈值
	 * @param beta 软效用参数
	 * @param p 软效用参数
	 * @param q 软效用参数
	 * @return 返回使得soft_utility等于alpha的y值
	*/
	double invers_of_derivative_soft(double alpha, double threshold, double beta = 0.5, double p = 1, double q = 1) const;

	double soft_utility2(double y, double a = 0.5, double b = 1) const;
	double derivative_of_soft_utility2(double y, double a = 0.5, double b = 1) const;
	double inverses_of_derivative_soft2(double alpha, double a = 0.5, double b = 1) const;

	double besteffert_utility(double y, double threshold) const {
		return y; // 重量效用：仅重量
	}

	

	double invers_of_derivative_besteffert(double alpha, double threshold, double beta = 0.5, double p = 1, double q = 1) const;

	void print_item() {
		cout << "Item ID: " << id << ", Type: " << type << ", Weight: " << weight << endl;
	}
};

class Bin
{
public:
	int id = 0; // 箱子的ID
	int cap = 0; // 箱子的容量
	int load = 0; // 箱子的当前负载

	void print_bin() {
		cout << "Bin ID: " << id << ", Capacity: " << cap << ", Load: " << load << endl;
	}
};

class SAP
{
public:
	int itemCount = 0; // 物品的数量
	int binCount = 0; // 箱子的数量

	vector<Item> items; // 物品集合
	vector<Bin> bins; // 箱子集合

	double** itemWeights = nullptr; // 物品的权重矩阵
	double** profits = nullptr; // 物品的利润矩阵

	
	/* 构造函数，初始化物品数量和箱子数量
	 * @param itemCount_ 物品数量
	 * @param binCount_ 箱子数量
	 */
	SAP(int itemCount_, int binCount_);

	/*
	* 初始化SAP类的成员变量，包括物品权重、利润和箱子集合。
	* 同时为每个物品和箱子设置ID。
	*
	* @param itemWeights_ 物品权重矩阵
	* @param profits_ 物品利润矩阵
	* @param bins_ 箱子集合
	*/
	void init(double** itemWeights_, double** profits_, vector<Bin> bins_);

	/*
	 * 使用线性规划近似算法来解决分箱问题。
	 * 该方法通过迭代更新对偶变量和匹配集合，最终得到一个近似解。
	 *
	 * @return 返回一个整数向量，表示每个箱子选择的物品集合的ID
	 */
	vector<int> LP_based_approximation();

	/*
	 * 该对偶线性规划是SAP LP算法中的一部分
	 * 该方法构建了一个线性规划模型，在给定违反约束的集合SS的情况下，
	 * 更新对偶变量 q 和 lambda。
	 *
	 * @param SS 输入的集合，表示每个箱子可选的分配集合
	 * @param q_ 输出的对偶变量 q 的值
	 * @param lambda_ 输出的对偶变量 lambda 的值
	 */
	void dual_LP(vector<vector<S>>& SS, vector<double>& q_, vector<double>& lambda_);
	/*
	 *  根据对偶规划选择到的有价值分配collection SS，在原规划中只考虑这些分配
	 * 该方法构建了一个线性规划模型，求解原始问题的最优解。
	 *
	 * @param SS 输入的集合，表示每个箱子可选的分配集合
	 * @param X_S 输出的变量 X_S 的分数解
	 */
	void primal_LP(vector<vector<S>>& SS, vector<vector<double>>& X_S);
	/*
	 * 使用FPTAS（Fully Polynomial Time Approximation Scheme）算法解决0-1背包问题。
	 * 该方法通过缩放物品价值，使用动态规划求解近似解。
	 *
	 * @param weights 物品的重量向量
	 * @param values 物品的价值向量
	 * @param capacity 背包的容量
	 * @param epsilon 近似精度参数
	 * @return 返回一个 KnapsackResult 结构体，包含选中的物品ID、总价值和总重量
	 */
	KnapsackResult knapsackFPTAS(vector<double>& weights, vector<double>& values, double capacity, double epsilon);
	/*
	 * 随机舍入方法，将分数解 X_S 转换为整数解。
	 * 该方法通过概率X_S[i][j]随机选择每个箱子中的一个集合，将其舍入为1，其余舍入为0。
	 *
	 * @param X_S 输入的分数解矩阵
	 * @return 返回一个整数矩阵，表示每个箱子选择的集合
	 */
	vector<vector<int>> random_Rounding(vector<vector<double>> & X_S);
	/*
	 * 根据舍入后的结果，选择每个物品的归属箱子。
	 * 如果一个物品被多个箱子选择，则选择利润最大的箱子作为该物品的归属箱子。
	 *
	 * @param resultSS 输入的舍入后的结果集合
	 * @return 返回一个整数向量，表示每个物品选择的箱子ID
	 */
	vector<int> bin_selection(vector<vector<int>>& resultSS);

	void print_all();

	void print_itemWeights();

	void print_profits();
};

// Knapsack problem with mixed utility function
class KPMU {
public:
	int itemCount = 0; // 物品的数量
	int hardItemCount = 0; // 硬效用物品的数量
	int softItemCount = 0; // 软效用物品的数量
	int besteffertItemCount = 0; // 重量效用物品的数量
	int binCount = 0; // 箱子的数量

	vector<Item> items; // 物品集合
	vector<Bin> bins; // 箱子集合

	double** thresholds = nullptr; // 物品的最小资源需求

	KPMU(int itemCount_, int binCount_);
	KPMU(int hardItemCount_, int softItemCount_, int besteffertItemCount_, int binCount_);
	void init(double** thresholds_, vector<Bin> bins, vector<Item> items_);

	/*
	* 对binID指定的箱子的子问题进行线性规划松弛。
	* 根据KKT条件，最优解中软效用和尽力而为效用的导数相等。
	* 我们猜测效用倒数为alpha，根据alpha和效用函数的导数的逆函数，
	* 可以得到此时对应消耗的资源
	* 注意：该方法需要使用Cplex库来求解线性规划问题。
	* 
	* @param binID 箱子的ID
	* @param X_S 保存线性规划松弛后的分数解
	* @param alpha 软效用函数的导数的值
	* @return 返回松弛后的线性规划的目标函数值
	
	*/
	double relexed_alpha_parametrized_fractional_KP(int binID, vector<double>& X_S, double alpha = 1);

	void print_all();

	void print_thresholds();
};
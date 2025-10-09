#include "EntityDefinition.h"


/*
* 该文件定义了SAP类及其相关方法，用于解决分箱问题。
 * 包括初始化、线性规划近似算法、对偶线性规划和原始线性规划等方法。
 */ 

/*
* SAP类的构造函数，初始化物品数量和箱子数量，并为物品和箱子分配内存。
 * 同时初始化物品权重和利润矩阵。
 *
 * @param itemCount_ 物品数量
 * @param binCount_ 箱子数量
*/
SAP::SAP(int itemCount_, int binCount_) 
	: itemCount(itemCount_), binCount(binCount_)
{
	items.resize(itemCount);
	bins.resize(binCount);
	itemWeights = new double* [binCount];
	profits = new double* [binCount];
	for (int i = 0; i < binCount; ++i) {
		itemWeights[i] = new double[itemCount];
		profits[i] = new double[itemCount];
	}

	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < itemCount; ++j) {
			itemWeights[i][j] = 0.0; // 初始化权重为0
			profits[i][j] = 0.0; // 初始化利润为0
		}
	}
}


void SAP::init(double** itemWeights_, double** profits_, vector<Bin> bins_)
{
	itemWeights = itemWeights_;
	profits = profits_;
	bins = bins_;
	
	for (int j = 0; j < itemCount; ++j) {
		items[j].id = j; // 设置物品ID
	}
	for (int i = 0; i < binCount; ++i) {
		bins[i].id = i; // 设置箱子ID
	}
}

vector<int> SAP::LP_based_approximation()
{
	vector<vector<S>> SS(binCount);
	
	vector<double> q(binCount, 0.0); // 对偶变量 q
	vector<double> lambda(itemCount, 0.0); // 对偶变量 lambda
	int s = 1;	// 一个指示变量，=1时表示此次迭代有新的S加入了SS

	vector<vector<double>> profits_(binCount);
	for (int i = 0; i < binCount; ++i) {
		profits_[i].resize(itemCount, 0.0);
	}

	while (s == 1)
	{
		s = 0; // 重置指示变量
		for (int i = 0; i < binCount; ++i)
		{
			for (int j = 0; j < itemCount; ++j)
			{
				// profile_[i][j] = max(0, profile[i][j] - lambda[i])
				profits_[i][j] = max(0.0, profits[i][j] - lambda[j]);
			}

			// 调用 knapsackFPTAS 函数来获取箱子 i 的最优匹配集合 S*
			vector<double> weights(itemCount, 0.0);
			vector<double> values(itemCount, 0.0);
			for (int j = 0; j < itemCount; ++j) {
				weights[j] = itemWeights[i][j]; // 获取箱子 i 的物品权重
				values[j] = profits_[i][j]; // 获取箱子 i 的物品利润
			}
			double capacity = bins[i].cap; // 获取箱子 i 的容量			
			
			KnapsackResult kResult = knapsackFPTAS(weights, values, capacity, 0.1); // 获取箱子 i 的最优匹配集合 S*
			S S_star = kResult.selectedItems; // 获取最优匹配集合 S*

			double q_star = 0;
			for (auto j : S_star) {
				q_star += profits_[i][j]; // 计算集合 S* 的总利润
			}

			if (q_star > q[i]) {
				s = 1; // 有新的 S 加入了 SS
				SS[i].push_back(S_star); // 将 S* 添加到 SS[i] 中

				// 打印集合 S*
				//printf("S* for bin %d: ", i);
				/*for (auto item : S_star) {
					printf("%d ", item);
					}
				printf("\n");*/
			}
		}

		dual_LP(SS, q, lambda); // 求解对偶线性规划，更新对偶变量 q 和 lambda

		// 打印对偶变量
		/*for (int i = 0; i < binCount; ++i) {
			printf("q[%d] = %lf\n", i, q[i]);
		}
		for (int j = 0; j < itemCount; ++j) {
			printf("lambda[%d] = %lf\n", j, lambda[j]);
		}*/


	}

	
	// 输出SS
	/*for (int i = 0; i < binCount; ++i) {
		printf("SS[%d]: ", i);
		for (auto S : SS[i]) {
			printf("{ ");
			for (auto item : S) {
				printf("%d ", item);
			}
			printf("} ");
		}
		printf("\n");
	}*/

	vector<vector<double>> X_S(binCount);
	primal_LP(SS, X_S); // 求解原始线性规划，更新变量 X_S
	// 输出X_S，保留3位小数
	for (int i = 0; i < binCount; ++i) {
		printf("X_S[%d]: ", i);
		for (int j = 0; j < X_S[i].size(); ++j) {
			printf("%.3lf ", X_S[i][j]);
		}
		printf("\n");
	}

	vector<vector<int>> X_S_int = random_Rounding(X_S); // 随机舍入，得到最终的选择集合
	// 输出X_S_int
	for (int i = 0; i < binCount; ++i) {
		printf("X_S_int[%d]: ", i);
		for (int j = 0; j < X_S_int[i].size(); ++j) {
			printf("%d ", X_S_int[i][j]);
		}
		printf("\n");
	}

	// 将 X_S_int 转换为结果集合 resultSS
	vector<vector<int>> resultSS(binCount);
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < X_S_int[i].size(); ++j) {
			if (X_S_int[i][j] == 1) { // 如果 X_S[i][j] == 1，表示选择了集合 S_j
				resultSS[i] = (SS[i][j]); // 将 S_j 添加到结果 SS 中
			}
		}
	}

	// 打印舍入后的结果
	for (int i = 0; i < binCount; ++i) {
		cout << "Bin " << i << ": ";
		for (auto item : resultSS[i]) {
			cout << item << " ";
		}
		cout << endl;
	}

	// 选择每个物品的箱子，去除重复的箱子
	vector<int> selectedBins = bin_selection(resultSS); 

	return selectedBins;
}


void SAP::dual_LP(vector<vector<S>>& SS, vector<double>& q_, vector<double>& lambda_)
{
	cout << "-------------dual_LP---------------\n";
	IloEnv env;
	try {
		IloModel model(env);
		// 对偶变量q和lambda的定义
		IloNumVarArray q(env, binCount);
		IloNumVarArray lambda(env, itemCount);
		for (IloInt i = 0; i < binCount; i++)
		{
			q[i] = IloNumVar(env, 0.0, IloInfinity); // q >= 0
		}
		for (IloInt j = 0; j < itemCount; j++)
		{
			lambda[j] = IloNumVar(env, 0.0, IloInfinity); // lambda >= 0
		}

		// 约束条件
		// SS 是一个collection，长度为 binCount
		// SS[i] 是一个集合，包含了第 i 个箱子可选的分配集合
		for (IloInt i = 0; i < binCount; i++)
		{
			for (auto S : SS[i]) 
			{
				IloExpr constrain(env);
				constrain += q[i]; // 对应箱子 i 的对偶变量 q_i

				double f_S = 0.0; // 集合 S 的收益

				for (auto j : S)
				{
					constrain += lambda[j]; // 对应物品 j 的对偶变量 lambda_j
					f_S += profits[i][j]; // 累加集合 S 中物品的利润;
				}
				// 添加约束 q_i + sum(lambda_j) <= f_S
				model.add(constrain >= f_S);
			}
		}

		// 对偶目标函数 sum(q[i]) + sum(lambda[j])
		IloExpr obj(env);
		for (IloInt i = 0; i < binCount; i++)
		{
			obj += q[i]; 
		}
		for (IloInt j = 0; j < itemCount; j++)
		{
			obj += lambda[j];
		}
		model.add(IloMinimize(env, obj));

		
		
		IloCplex cplex(model);
		if (!cplex.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		/*double solution_value = cplex.getObjValue();
		printf("Solution value = %lf\n", solution_value);*/


		IloNumArray q_vals(env); 
		cplex.getValues(q_vals, q);	//  获取变量的值
		IloNumArray lambda_vals(env);
		cplex.getValues(lambda_vals, lambda); // 获取变量的值

		for (IloInt i = 0; i < binCount; i++)
		{
			q_[i] = q_vals[i]; // 将对偶变量 q 的值存储到 q_ 中
		}
		for (IloInt j = 0; j < itemCount; j++)
		{
			lambda_[j] = lambda_vals[j]; // 将对偶变量 lambda 的值存储到 lambda_ 中
		}
	}
	catch (IloException& e) { cerr << "Concert exception caught:" << e << endl; }
	catch (...) { cerr << "Unknuwn exception caught" << endl; }
	env.end();
}


void SAP::primal_LP(vector<vector<S>>& SS, vector<vector<double>>& X_S)
{
	cout << "-------------primal_LP---------------\n";
	IloEnv env;
	try {
		IloModel model(env);	
		
		// 原始变量 X_S 的定义
		IloNumVarArray2 X(env, binCount);
		for (IloInt i = 0; i < binCount; i++)
		{
			int n = SS[i].size(); // SS[i] 的大小
			X[i] = IloNumVarArray(env, n, 0.0, 1.0, ILOFLOAT); // X_S[i][j] ∈ [0, 1]
			model.add(X[i]); // 添加变量数组到模型中
		}


		// 约束条件1
		// SS 是一个collection，长度为 binCount
		// SS[i] 是一个集合，包含了第 i 个箱子可选的分配集合
		for (IloInt j = 0; j < itemCount; j++)
		{
			IloExpr constrain(env);
			for (IloInt i = 0; i < binCount; i++)
			{
				for (IloInt k = 0; k < SS[i].size(); k++)
				{
					if (find(SS[i][k].begin(), SS[i][k].end(), j) != SS[i][k].end()) {
						constrain += X[i][k]; // 如果物品 j 在集合 S 中，则添加对应的 X_S[i][k]
					}
				}
			}
			model.add(constrain <= 1); // 每个物品只能被分配到一个集合中
		}

		// 约束条件2
		// 每个箱子只能分配一个集合
		for (IloInt i = 0; i < binCount; i++)
		{
			IloExpr constrain(env);

			for (IloInt k = 0; k < SS[i].size(); k++)
			{
				
				constrain += X[i][k]; // 累加箱子 i 的所有集合
			}
			model.add(constrain <= 1); // 每个箱子只能分配一个集合
		}


		// 原始目标函数 sum(X_S[i][j] * f_S)
		IloExpr obj(env);
		for (IloInt i = 0; i < binCount; i++)
		{
			for (IloInt k = 0; k < SS[i].size(); k++)
			{
				double f_S = 0.0; // 集合 S 的收益
				for (auto j : SS[i][k])
				{
					f_S += profits[i][j]; // 累加集合 S 中物品的利润
				}
				obj += f_S * X[i][k]; // 添加到目标函数中
			}
		}	

		model.add(IloMaximize(env, obj));



		IloCplex cplex(model);
		if (!cplex.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}

		double solution_value = cplex.getObjValue();
		printf("Solution value = %lf\n", solution_value);


		//  获取变量的值
		X_S.resize(binCount);
		for (IloInt i = 0; i < binCount; i++)
		{
			int n = SS[i].size(); // SS[i] 的大小
			X_S[i].resize(n);
			for (IloInt k = 0; k < n; k++)
			{
				X_S[i][k] = cplex.getValue(X[i][k]); // 获取每个 X_S[i][k] 的值
			}
		}	

		
	}
	catch (IloException& e) { cerr << "Concert exception caught:" << e << endl; }
	catch (...) { cerr << "Unknuwn exception caught" << endl; }
	env.end();
}

 
KnapsackResult SAP:: knapsackFPTAS(vector<double>& weights, vector<double>& values, double capacity, double epsilon)
{
	int n = weights.size();
	KnapsackResult result;
	if (n == 0 || capacity <= 0) return result;

	// 1. 计算缩放因子K
	int v_max = *max_element(values.begin(), values.end());
	double K = max(1.0, epsilon * v_max / n); // 避免K=0

	// 2. 缩放价值并计算新价值总和
	vector<int> scaled_values(n);
	int scaled_V = 0;
	for (int i = 0; i < n; i++) {
		scaled_values[i] = floor(values[i] / K);
		scaled_V += scaled_values[i];
	}

	// 3. 动态规划求解：dp[v] = 达到价值v的最小重量
	vector<int> dp(scaled_V + 1, INT_MAX);
	vector<vector<bool>> selected(n + 1, vector<bool>(scaled_V + 1, false));
	dp[0] = 0;

	// 填充DP表并记录选择决策
	for (int i = 0; i < n; i++) {
		for (int v = scaled_V; v >= scaled_values[i]; v--) {
			if (dp[v - scaled_values[i]] != INT_MAX) {
				int new_weight = dp[v - scaled_values[i]] + weights[i];
				if (new_weight <= capacity && new_weight < dp[v]) {
					dp[v] = new_weight;
					selected[i][v] = true; // 标记选择当前物品
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
	for (int i = n - 1; i >= 0; i--) {
		if (selected[i][current_value]) {
			result.selectedItems.push_back(i); // 记录物品ID
			current_value -= scaled_values[i];
			result.totalWeight += weights[i];
		}
	}
	reverse(result.selectedItems.begin(), result.selectedItems.end()); // 反转列表使ID顺序与输入一致

	// 6. 计算实际总价值
	for (int id : result.selectedItems) {
		result.totalValue += values[id];
	}

	return result;
}


vector<vector<int>> SAP::random_Rounding(vector<vector<double>>& X_S)
{
	// 通过随机舍入的方式将X_S转换为整数解
	// 以X_S[i][j]的概率将X_S[i][j]舍入为1，否则舍入为0
	// 初始化随机数引擎（使用 Mersenne Twister 算法）
	std::random_device rd;  // 随机种子
	std::mt19937 gen(rd()); // 高性能随机数引擎
	static std::uniform_real_distribution<float> dist(0.0f, 1.0f); // [0,1) 均匀分布
	
	// 创建输出矩阵（尺寸与输入相同）
	vector<vector<int>> rounded_X_S(binCount);
	for (int i = 0; i < binCount; ++i) {
		int n = X_S[i].size(); // SS[i] 的大小
		rounded_X_S[i].resize(n, 0); // 初始化为0
		
		if (n == 0) continue; // 如果没有集合，跳过

		double random = dist(gen); // 生成一个[0,1)之间的随机数
		
		for (int j = 0; j < n; ++j) {
			random -= X_S[i][j]; // 减去当前集合的概率
			if (random <= 0) {
				rounded_X_S[i][j] = 1; // 如果随机数小于等于当前集合的概率，则舍入为1
				break; // 每行只选择一个集合
			}
		}
	}
	return rounded_X_S;
}

vector<int> SAP::bin_selection(vector<vector<int>>& resultSS)
{
	vector<int> selectedBins(itemCount, -1); // 初始化为-1，表示未选择
	for (int i = 0; i < binCount; i++)
	{
		for (auto j : resultSS[i])
		{
			if (selectedBins[j] == -1) {
				selectedBins[j] = i; // 选择第一个箱子作为物品的归属箱子
			}
			else {
				int selectedBin = selectedBins[j]; // 获取已选择的箱子
				// 如果已经选择了箱子，比较当前箱子的利润和已选择箱子的利润
				if (profits[i][j] > profits[selectedBin][j]) {
					selectedBins[j] = i; // 更新为更高利润的箱子
				}
			}
		}
	}
	
	return selectedBins;
}

void SAP::print_all()
{
	cout << "Item Count: " << itemCount << endl;
	cout << "Bin Count: " << binCount << endl;
	cout << "Items:" << endl;
	for (const auto& item : items) {
		cout << "Item ID: " << item.id << endl;
	}
	cout << "Bins:" << endl;
	for (const auto& bin : bins) {
		cout << "Bin ID: " << bin.id << ", Capacity: " << bin.cap << ", Load: " << bin.load << endl;
	}
	print_itemWeights();
	print_profits();
}

void SAP::print_itemWeights()
{
	cout << "Item Weights:" << endl;
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < itemCount; ++j) {
			cout << itemWeights[i][j] << " ";
		}
		cout << endl;
	}
}

void SAP::print_profits()
{
	cout << "Profits:" << endl;
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < itemCount; ++j) {
			cout << profits[i][j] << " ";
		}
		cout << endl;
	}
}

KPMU::KPMU(int itemCount_, int binCount_)
	: itemCount(itemCount_), binCount(binCount_)
{
	items.resize(itemCount);
	bins.resize(binCount);
	thresholds = new double* [binCount];
	for (int i = 0; i < binCount; ++i) {
		thresholds[i] = new double[itemCount];
	}
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < itemCount; ++j) {
			thresholds[i][j] = 0.0; // 初始化最小资源需求为0
		}
	}
}

KPMU::KPMU(int hardItemCount_, int softItemCount_, int besteffertItemCount_, int binCount_)
{
	hardItemCount = hardItemCount_;
	softItemCount = softItemCount_;
	besteffertItemCount = besteffertItemCount_;
	binCount = binCount_;
	itemCount = hardItemCount + softItemCount + besteffertItemCount;
	items.resize(itemCount);
	bins.resize(binCount);
	thresholds = new double* [binCount];
	for (int i = 0; i < binCount; ++i) {
		thresholds[i] = new double[itemCount];
	}
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < itemCount; ++j) {
			thresholds[i][j] = 0.0; // 初始化最小资源需求为0
		}
	}
}

void KPMU::init(double** thresholds_, vector<Bin> bins_, vector<Item> items_)
{
	thresholds = thresholds_;
	bins = bins_;
	items = items_;
}

double KPMU::relexed_alpha_parametrized_fractional_KP(int binID, vector<double>& X_S, double alpha)
{
	double a[] = { 1, 2, 1, 3, 2, 4, 1, 5, 3, 6 };
	double b[] = { 5, 10, 3, 9, 8, 16, 6, 30, 6, 12 };
	int i = binID;
	IloEnv env;
	try {
		IloModel model(env);
		IloNumVarArray x(env, itemCount, 0.0, 1.0, ILOFLOAT); // 定义变量 x[i] ∈ [0, 1]

		model.add(x);

		// 约束条件：sum(x[i] * weight[i]) <= capacity
		IloExpr constrain(env);
		for (IloInt j = 0; j < hardItemCount; j++)
		{
			constrain += x[j] * thresholds[i][j]; // 硬效用物品的质量
		}
		for (IloInt j = hardItemCount; j < hardItemCount + softItemCount; j++)
		{
			//double y = items[j].inverses_of_derivative_soft2(alpha, a[j - hardItemCount], b[j - hardItemCount]);
			double y = items[j].invers_of_derivative_soft(alpha, thresholds[i][j]);
			constrain += x[j] * y; // 软效用物品的质量
		}
		for (IloInt j = hardItemCount + softItemCount; j < itemCount; j++)
		{
			double y = items[j].invers_of_derivative_besteffert(alpha, thresholds[i][j]);
			constrain += x[j] * y; // 重量效用物品的质量
		}
		model.add(constrain <= bins[i].cap); // 添加约束：总质量不超过箱子容量
		constrain.end(); // 释放表达式资源

		// 目标函数：max sum(x[i] * utility(i, thresholds[i][j]))
		IloExpr obj(env);
		for (IloInt j = 0; j < hardItemCount; j++)
		{
			double u = items[j].utility(thresholds[i][j], thresholds[i][j]);
			// cout << "Hard utility for item " << j << ": " << u << endl;
			obj += x[j] * u; // 硬效用物品的效用
		}
		for (IloInt j = hardItemCount; j < hardItemCount + softItemCount; j++)
		{
			//double y = items[j].inverses_of_derivative_soft2(alpha, a[j - hardItemCount], b[j - hardItemCount]);
			//double u = items[j].soft_utility2(y, a[j - hardItemCount], b[j - hardItemCount]);
			double y = items[j].invers_of_derivative_soft(alpha, thresholds[i][j]);
			double u = items[j].soft_utility(y, thresholds[i][j]);
			// 输出资源y和对应的效用u
			cout << "Soft utility for item " << j << ": y = " << y << ", u = " << u << endl;

			obj += x[j] * u; // 软效用物品的效用
		}
		for (IloInt j = hardItemCount + softItemCount; j < itemCount; j++)
		{
			double y = items[j].invers_of_derivative_besteffert(alpha, thresholds[i][j]);
			obj += x[j] * items[j].utility(y, thresholds[i][j]); // 重量效用物品的效用
		}
		model.add(IloMaximize(env, obj)); // 添加目标函数

		// 取消输出求解信息
		env.setOut(env.getNullStream()); // 禁用输出流

		IloCplex cplex(model);
		if (!cplex.solve()) {
			env.error() << "Failed to optimize LP." << endl;
			throw(-1);
		}
		
		double solution_value = cplex.getObjValue();
		printf("Solution value = %lf\n", solution_value);

		IloNumArray x_vals(env);
		cplex.getValues(x_vals, x); // 获取变量的值
		X_S.resize(itemCount); // 确保 X_S 的大小
		for (IloInt i = 0; i < itemCount; i++)
		{
			X_S[i] = x_vals[i]; // 将变量值存储到 X_S 中
		}
		env.end(); // 结束环境，释放资源
		return solution_value; // 返回目标函数值

	}
	catch (IloException& e) { cerr << "Concert exception caught:" << e << endl; }
	catch (...) { cerr << "Unknuwn exception caught" << endl; }
	return 0.0;
}

void KPMU::print_all()
{
	cout << "Item Count: " << itemCount << endl;
	cout << "Bin Count: " << binCount << endl;
	cout << "Hard Item Count: " << hardItemCount << endl;
	cout << "Soft Item Count: " << softItemCount << endl;
	cout << "Best Effort Item Count: " << besteffertItemCount << endl;
	cout << "Items:" << endl;
	for (auto& item : items) {
		item.print_item();
	}
	cout << "Bins:" << endl;
	for (auto& bin : bins) {
		bin.print_bin();
	}
	print_thresholds();

	cout << endl;
}

void KPMU::print_thresholds()
{
	cout << "Thresholds:" << endl;
	for (int i = 0; i < binCount; ++i) {
		for (int j = 0; j < itemCount; ++j) {
			cout << thresholds[i][j] << " ";
		}
		cout << endl;
	}
}

double Item::utility(double y, double threshold)
{
	if (type == HARD_UTILITY) {
		return hard_utility(y, threshold); // 硬效用：如果y大于等于质量，则返回权重，否则返回0
	}
	else if (type == SOFT_UTILITY) {
		return soft_utility(y, threshold); // 软效用：使用sigmoid函数计算效用
	}
	else if (type == BESTEFFERT_UTILITY) {
		return y; // 类型2：仅重量
	}
	return 0.0; // 默认返回0
}

double Item::soft_utility2(double y, double a, double b) const
{
	return weight / (1 + exp(-a * y + b)); // 软效用：使用sigmoid函数计算效用
}

double Item::derivative_of_soft_utility2(double y, double a, double b) const
{
	double exp_part = exp(-a * y + b);
	return weight * a * exp_part / ((1 + exp_part) * (1 + exp_part)); // 软效用的导数
}

double Item::inverses_of_derivative_soft2(double alpha, double a, double b) const
{
	// 计算导数的反函数
	// alpha 是导数的值，求对应的 y

	if (alpha == 0)
		return 0;
	// 使用二分查找求解，y的值至少为 b/a
	double left = b / a; // y的下界
	double right = 15; // y的上界，假设不会超过1000
	double mid;
	double y = left; // 初始化y
	while (right - left > 1e-6) // 精度要求
	{
		mid = (left + right) / 2;
		double deriv = derivative_of_soft_utility2(mid, a, b);
		if (deriv > alpha)
			left = mid; // 导数值小于alpha，增大y
		else
			right = mid; // 导数值大于等于alpha，减小y
	}
	y = (left + right) / 2; // 近似解
	// 输出y和对应的导数值

	
	return y; // 返回近似解

}

double Item::invers_of_derivative_soft(double alpha, double threshold, double beta, double p, double q) const
{
	if (alpha == 0)
		return 0;
	// fenzi = ln( alpha / (weight * ( 1 - beta ) * p * q  ) )
	double fenzi = p * threshold - log(alpha / (weight * (1 - beta) * p * q));
	double fenmu = p * q; // 分母
	double y = fenzi / fenmu; // 计算y
	if(y * q < threshold)
		y = threshold / q; // 保证y不小于threshold / (p * q)
	// 输出调试信息
	/*cout << "Item ID: " << id << ", alpha: " << alpha << ", threshold: " << threshold
		<< ", beta: " << beta << ", p: " << p << ", q: " << q
		<< ", fenzi: " << fenzi << ", fenmu: " << fenmu 
		<< ", y: " << y <<  endl;*/

	return y; // 返回逆导数的值
}

double Item::invers_of_derivative_besteffert(double alpha, double threshold, double beta, double p, double q) const
{
	return 0.0;
}

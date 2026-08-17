#pragma once
#include "nonUniformArray.h"
#include "queue.h"
#include<vector>
#include<algorithm>
#include<cmath>

double verbose = false; // 调试值
const int L = 5000, W = 5000, h = 300;
//const int L = 900, W = 900, h = 100; // 调试场景
int numUsers = 1000;
//const int numUsers = 300; // 调试场景
const int numClusters = 2;
//const int numClusters = 5; // 调试场景
int numUsersPerCluster[numClusters];
double xCluster[numClusters], yCluster[numClusters];
bool isCovered[numClusters];
bool isExtraCovered[numClusters];

double R = 600, r = 500;
//double R = 300, r = 250; // 调试场景
//assert(h < r);
double userCommRadius = sqrt(r * r - h * h); // the user communication radius at altitude h
const int numUAVs = 30;
//const int numUAVs = 5; // 调试场景 
int CapUAV = 100; // the maximum number of users that a UAV can serve
const int delta = 250; // 300 meters
const int numHoverLocations = (L / delta) * (W / delta);
//const int numHoverLocations = (L / delta)*(W / delta) +numClusters + 1 * 4; //调试场景
double xHoverLoc[numHoverLocations], yHoverLoc[numHoverLocations]; // the candidate hovering locations at altitude h
int minHops[numHoverLocations][numHoverLocations];
bool adjMatrix[numHoverLocations][numHoverLocations];

bool isLocCoverUserMatrix[numHoverLocations][numClusters]; // 标记某用户群是否被某网格位置覆盖
int locCoverSpecCluster[numHoverLocations][numClusters]; // 记录某位置具体覆盖的所有用户群编号
double disMatrix[numClusters][numHoverLocations];

bool isToDeployUAV[numHoverLocations];
int nearestEdgeWeight[numHoverLocations];

int hoverLocations[numHoverLocations];
int bestHoverLocations[numHoverLocations];
int numLocCoverClusters[numHoverLocations]; // 记录每个网格覆盖多少个用户群
int numDeployedLocations;
int D;

double Pt = -6; // transmission power of UAV (dB)
double gt = 5; // gain of UAV (dB)
double PN = -105; // noise power (dB)
double pl_cons = 40.4; // = 20 log_10 (4pi f_c /c), dB
double pl_fadingLOS = 1; // fading loss of los link
double pl_fadingNLOS = 20; // fading loss of NLOS link
double Bw = 2.5; // MHz

double avgDataRates[numClusters][numHoverLocations];

int residualUsers[numClusters], residualUAVCap[numHoverLocations];
struct Edge {
	int userID;
	int locationID;
	double dataRate;
};
Edge allEdges[numClusters * numUAVs];
inline bool cmp(Edge& x, Edge& y) { return x.dataRate > y.dataRate; }

double distance2D(double x1, double y1, double x2, double y2)
{
	double dx = x1 - x2, dy = y1 - y2;
	return sqrt(dx * dx + dy * dy);
}

double distance3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
	double dx = x1 - x2, dy = y1 - y2, dz = z1 - z2;
	return sqrt(dx * dx + dy * dy + dz * dz);
}

inline double Uniform()
{
	//return (rand() % Mode) / (double)Mode;
	return double(rand()) / RAND_MAX;//rand返回整数值，此处强制转换为double类型
}

inline int calSegH()
{
	int segH;

	if ((userCommRadius <= R / 2) && (userCommRadius > 0))
	{
		segH = 2;
	}
	else if ((userCommRadius <= sqrt(2) * R / 2) && (userCommRadius > R / 2))
	{
		segH = 3;
	}
	else if ((userCommRadius <= R) && (userCommRadius > sqrt(2) * R / 2))
	{
		segH = 4;
	}
	else
	{
		return 0;
	}
	return segH;
}

class maxUAVCoverage {

public:
	void initialization();
public:
	int AlgKnapsack();
	int AlgKnapsackSlow();
	int AlgTreeDecomp();
	int AlgAssign();
	int ApproAlgShort();
	int ApproAlgLong();

	//double deepGreedyAlg(double &bestSumDis);
	//double greedyAlg(double& bestSumDis);
	//double greedyProfitLabelAlg(double& bestSumDis);
	double profitEachLocation[numHoverLocations];
	double AlgTreeDecomp_old();
	double AlgGreedy();
protected:
	void findShortestPath(int s);
	void findAllShortestPath();
	int extendByGreedy();
	int extendByGreedyModified(); // 调试
	double extendByGreedyProfitLabel();
protected:
	bool visited[1000];
	int parentSPT[1 + numClusters + numHoverLocations + 1];
	// int disFromSource[numHoverLocations];
	int allParentSPT[1 + numClusters + numHoverLocations + 1][1 + numClusters + numHoverLocations + 1];
	// int allDisFromSource[numHoverLocations][numHoverLocations];
public:
	void calAvgDataRates();
	int calMaxThroughput();
	int calDeployedCoverUsers(); // location全部选择完后，或，初始选择完后，进行计算
	// int calExtraCoverUsers(); // 临时增加的节点所额外覆盖的用户
	void calUsersCoveredPerLocation();
protected:
	void assignUserLocations();
	void assignHoverLocations();
	void findMinHopsAmongLocations();
	/*public:
		nonUniformArray<flowGraphEdge> flowGraph;
		nonUniformArray<residualGraphEdge> residualGraph;*/
};

int maxUAVCoverage::calMaxThroughput()
{
	//assert(numDeployedLocations <= numUAVs);
	int i, j;
	int locationID;
	Edge oneEdge;
	int numEdges = 0;
	//all edges
	for (i = 0; i < numClusters; ++i)
	{
		oneEdge.userID = i;
		for (j = 0; j < numDeployedLocations; ++j)
		{
			locationID = hoverLocations[j];
			//assert(locationID >= 0 && locationID < numHoverLocations);
			if (disMatrix[i][locationID] > r) continue;
			oneEdge.locationID = locationID;
			oneEdge.dataRate = avgDataRates[i][locationID];
			allEdges[numEdges] = oneEdge;
			numEdges++;
		}
	}
	if (0 == numEdges) return 0;
	sort(allEdges, allEdges + numEdges, cmp); // sort in decreasing order

	for (i = 0; i < numClusters; ++i)
		residualUsers[i] = numUsersPerCluster[i]; // 每群剩余未覆盖用户数量
	for (j = 0; j < numDeployedLocations; ++j)
		residualUAVCap[hoverLocations[j]] = CapUAV; // 每个无人机位置剩余承载量

	int userID;
	int k;
	int minCap;
	// double sumDataRates = 0;
	int sumCoveredUsers = 0;
	for (k = 0; k < numEdges; ++k)
	{
		// 对于某条边
		userID = allEdges[k].userID; // 连接的用户群ID
		locationID = allEdges[k].locationID; // 连接的位置ID
		minCap = residualUsers[userID]; // 该用户群的人数
		if (residualUAVCap[locationID] < minCap) // 该位置剩余量 小于 该用户群人数
			minCap = residualUAVCap[locationID];
		// sumDataRates += minCap * allEdges[k].dataRate; // 该位置剩余量 大于 该用户群人数，直接计算
		sumCoveredUsers += minCap;

		residualUsers[userID] -= minCap; // 该用户群剩余未覆盖人数
		residualUAVCap[locationID] -= minCap; // 无人机剩余承载量
	}
	return sumCoveredUsers;
}

void maxUAVCoverage::calAvgDataRates()
{
	int i, j;
	double eucliDistance;
	double horizonalDistance;
	double pl_dis;
	double pl_los, pl_nlos;
	double SNR_los, SNR_nlos;
	double data_los, data_nlos;
	double angle;
	double a = 9.611725, b = 0.158062;
	double e = 2.71828;
	double prob_LOS;
	double dataRate;
	for (i = 0; i < numClusters; ++i)
		for (j = 0; j < numHoverLocations; ++j)
		{
			horizonalDistance = distance2D(xCluster[i], yCluster[i], xHoverLoc[j], yHoverLoc[j]);
			eucliDistance = sqrt(horizonalDistance * horizonalDistance + double(h) * h);
			if (eucliDistance > r) { avgDataRates[i][j] = 0;		continue; }

			pl_dis = 20.0 * log10(eucliDistance);

			pl_los = pl_cons + pl_dis + pl_fadingLOS;
			SNR_los = pow(10.0, (Pt + gt - pl_los - PN) / 10.0);
			data_los = Bw * log2(1.0 + SNR_los);

			pl_nlos = pl_cons + pl_dis + pl_fadingNLOS;
			SNR_nlos = pow(10.0, (Pt + gt - pl_nlos - PN) / 10.0);
			data_nlos = Bw * log2(1.0 + SNR_nlos);

			angle = 180.0 * asin(h / eucliDistance) / 3.1415926; // elevation angle, in degree
			prob_LOS = 1.0 / (1.0 + a * pow(e, -b * (angle - a)));

			dataRate = prob_LOS * data_los + (1.0 - prob_LOS) * data_nlos;
			avgDataRates[i][j] = dataRate;
			//if(verbose) printf("user: (%.0lf, %.0lf), uav:(%.0lf, %.0lf), dis: %.0lf, dataLoS: %.3lf, dataNLoS: %.3lf, angle: %.2lf, probLOS: %.3lf, avgDataRate: %.2lf Mbps\n ", xCluster[i], yCluster[i], xHoverLoc[j], yHoverLoc[j], eucliDistance, data_los, data_nlos, angle, prob_LOS, dataRate);
		}
}

int maxUAVCoverage::AlgKnapsack()
{
	int i, j, k, l, m, z;
	int maxUsers = 0;
	int curUsers;
	int loop_time = 0;

	int u, v;
	int clusterID;

	int MinDistances[numHoverLocations];
	int allNearestNodes[numHoverLocations];

	int residualUsersCopy[numClusters];

	// 初始选1个点
	for (i = 0; i < numHoverLocations; ++i)
	{
		numDeployedLocations = 1;
		hoverLocations[0] = i;
		for (k = 0; k < numHoverLocations; ++k)	isToDeployUAV[k] = false;
		isToDeployUAV[i] = true;

		//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------					
		//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
		//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	

		int minDis;
		int nearestNode;
		// Find the shortest distances and paths to the nodes already in the subtree
		for (k = 0; k < numHoverLocations; ++k) // Vi中未添加节点
		{
			// printf("isToDeployUAV is %d\n", isToDeployUAV[halfDHops[test1]]); // 调试
			if (true == isToDeployUAV[k]) continue;

			MinDistances[k] = minHops[i][k];
			allNearestNodes[k] = i;
		}

		// 计算初始点所覆盖的用户数量
		int totalUsers = 0;
		totalUsers = calDeployedCoverUsers();
		// printf("initial total is %d\n", totalUsers);

		// extendByGreedy---->遍历各节点，比较增益，选出最优
		int bestNextLocation; // 最优位置
		int bestIncreasedUsers; // 最优增量
		double bestRatio;
		int curTestUsers; // 当前测试节点用户增量

		// 下标：代表该未添加节点
		// 存储的值：代表离该未添加节点最近的已添加节点

		int curMinCap; // 当前较小剩余量
		int curResidualUsers = 0; // 初始化临时变量，目的是测试节点过程中先不改变residualUsers[]数组中的值
		int curResidualUAVCap = 0; // 初始化临时变量，目的是测试节点过程中先不改变residualUAVCap[]数组中的值
		int nextNode;
		int testNode;

		while (numDeployedLocations < numUAVs)
		{
			bestRatio = -1;
			for (testNode = 0; testNode < numHoverLocations; ++testNode)
			{
				if (true == isToDeployUAV[testNode]) continue;
				//if (MinDistances[testNode] >= 2) continue;  // do not see the nodes that are far away
				if (numDeployedLocations + MinDistances[testNode] > numUAVs) continue;
				hoverLocations[numDeployedLocations] = testNode;
				numDeployedLocations++;
				nearestNode = allNearestNodes[testNode];
				v = testNode;
				while (nearestNode != allParentSPT[nearestNode][v]) // not the root of the tree
				{
					nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
					hoverLocations[numDeployedLocations] = nextNode;
					++numDeployedLocations;
					v = nextNode;
				}

				for (k = numDeployedLocations - MinDistances[testNode]; k < numDeployedLocations; ++k)
				{
					v = hoverLocations[k];
					for (l = 0; l < numLocCoverClusters[v]; ++l)
					{
						clusterID = locCoverSpecCluster[v][l];
						residualUsersCopy[clusterID] = residualUsers[clusterID]; // copy the # of uncovered user in each cluster
					}
				}

				curTestUsers = 0;
				for (k = numDeployedLocations - MinDistances[testNode]; k < numDeployedLocations; ++k)
				{
					v = hoverLocations[k];
					curResidualUAVCap = residualUAVCap[v];
					for (l = 0; l < numLocCoverClusters[v]; ++l)
					{
						clusterID = locCoverSpecCluster[v][l];
						curMinCap = residualUsersCopy[clusterID];
						if (curResidualUAVCap < curMinCap)
							curMinCap = curResidualUAVCap;
						residualUsersCopy[clusterID] -= curMinCap;
						curResidualUAVCap -= curMinCap;
						curTestUsers += curMinCap;
					}
				}

				if (1.0 * curTestUsers / (double)MinDistances[testNode] > bestRatio)
				{
					bestRatio = 1.0 * curTestUsers / (double)MinDistances[testNode];
					bestIncreasedUsers = curTestUsers;
					bestNextLocation = testNode;
				}

				numDeployedLocations -= MinDistances[testNode];
			} // 选出最优
			//printf("bestIncreasedUsers is %d and bestNextLocation is %d\n", bestIncreasedUsers, bestNextLocation); 

			// added the nodes on the shortest path
			hoverLocations[numDeployedLocations] = bestNextLocation;
			numDeployedLocations++;
			isToDeployUAV[bestNextLocation] = true;
			nearestNode = allNearestNodes[bestNextLocation];
			v = bestNextLocation;
			while (nearestNode != allParentSPT[nearestNode][v])
			{
				nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
				hoverLocations[numDeployedLocations] = nextNode;
				isToDeployUAV[nextNode] = true;
				++numDeployedLocations;
				v = nextNode;
			}

			curTestUsers = 0;
			for (k = numDeployedLocations - MinDistances[bestNextLocation]; k < numDeployedLocations; ++k)
			{
				v = hoverLocations[k];
				curResidualUAVCap = residualUAVCap[v];
				for (l = 0; l < numLocCoverClusters[v]; ++l)
				{
					clusterID = locCoverSpecCluster[v][l];
					curMinCap = residualUsers[clusterID];
					if (curResidualUAVCap < curMinCap)
						curMinCap = curResidualUAVCap;
					residualUsers[clusterID] -= curMinCap;
					curResidualUAVCap -= curMinCap;
					curTestUsers += curMinCap;
				}
			}
			totalUsers += curTestUsers;

			//update the shortest paths and distances
			for (l = 0; l < numHoverLocations; ++l)
			{
				if (true == isToDeployUAV[l]) continue;
				for (k = numDeployedLocations - MinDistances[bestNextLocation]; k < numDeployedLocations; ++k)
				{
					v = hoverLocations[k];

					if (minHops[l][v] < MinDistances[l])
					{
						MinDistances[l] = minHops[l][v];
						allNearestNodes[l] = v;
					}
				}
			}
		}

		// 所有节点添加完毕
		//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
		if (maxUsers < totalUsers)
			maxUsers = totalUsers;
		//}
	}
	return maxUsers;
}

int maxUAVCoverage::AlgKnapsackSlow()
{
	int i, j, k, l, m, z;
	int maxUsers = 0;
	int curUsers;
	int loop_time = 0;

	int u, v;
	int clusterID;

	int MinDistances[numHoverLocations];
	int allNearestNodes[numHoverLocations];

	int residualUsersCopy[numClusters];

	// 初始选2个点
	for (i = 0; i < numHoverLocations; ++i)
	{
		for (j = i + 1; j < numHoverLocations; ++j)
		{
			if (minHops[i][j] > numUAVs - 1) continue;
			for (k = 0; k < numHoverLocations; ++k)	isToDeployUAV[k] = false;

			numDeployedLocations = 1;
			hoverLocations[0] = j;
			isToDeployUAV[j] = true;

			v = j;
			while (-1 != allParentSPT[i][v])
			{
				u = allParentSPT[i][v];
				hoverLocations[numDeployedLocations] = u;
				++numDeployedLocations;
				isToDeployUAV[u] = true;
				v = u;
			}



			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------					
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
						//for (k = 0; k < numHoverLocations; ++k)
						//	MinDistances[k] = 1000000;

			int minDis;
			int nearestNode;
			// Find the shortest distances and paths to the nodes already in the subtree
			for (k = 0; k < numHoverLocations; ++k) // Vi中未添加节点
			{
				// printf("isToDeployUAV is %d\n", isToDeployUAV[halfDHops[test1]]); // 调试
				if (true == isToDeployUAV[k]) continue;
				minDis = 1000000;
				for (l = 0; l < numDeployedLocations; ++l) // 已添加节点
				{	// 对于Vi中每一个未添加节点，判断其与已添加的某个节点是否相邻
					v = hoverLocations[l];
					if (minHops[k][v] < minDis)
					{
						minDis = minHops[k][v];
						nearestNode = v;
					}
				}
				MinDistances[k] = minDis;
				allNearestNodes[k] = nearestNode;
			}

			// 计算初始点所覆盖的用户数量
			int totalUsers = 0;
			totalUsers = calDeployedCoverUsers();
			// printf("initial total is %d\n", totalUsers);

			// extendByGreedy---->遍历各节点，比较增益，选出最优
			int bestNextLocation; // 最优位置
			int bestIncreasedUsers; // 最优增量
			double bestRatio;
			int curTestUsers; // 当前测试节点用户增量

			// 下标：代表该未添加节点
			// 存储的值：代表离该未添加节点最近的已添加节点

			int curMinCap; // 当前较小剩余量
			int curResidualUsers = 0; // 初始化临时变量，目的是测试节点过程中先不改变residualUsers[]数组中的值
			int curResidualUAVCap = 0; // 初始化临时变量，目的是测试节点过程中先不改变residualUAVCap[]数组中的值
			int nextNode;
			int testNode;

			while (numDeployedLocations < numUAVs)
			{
				bestRatio = -1;
				for (testNode = 0; testNode < numHoverLocations; ++testNode)
				{
					if (true == isToDeployUAV[testNode]) continue;
					//if (MinDistances[testNode] >= 2) continue;  // do not see the nodes that are far away
					if (numDeployedLocations + MinDistances[testNode] > numUAVs) continue;
					hoverLocations[numDeployedLocations] = testNode;
					numDeployedLocations++;
					nearestNode = allNearestNodes[testNode];
					v = testNode;
					while (nearestNode != allParentSPT[nearestNode][v]) // not the root of the tree
					{
						nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
						hoverLocations[numDeployedLocations] = nextNode;
						++numDeployedLocations;
						v = nextNode;
					}

					for (k = numDeployedLocations - MinDistances[testNode]; k < numDeployedLocations; ++k)
					{
						v = hoverLocations[k];
						for (l = 0; l < numLocCoverClusters[v]; ++l)
						{
							clusterID = locCoverSpecCluster[v][l];
							residualUsersCopy[clusterID] = residualUsers[clusterID]; // copy the # of uncovered user in each cluster
						}
					}

					curTestUsers = 0;
					for (k = numDeployedLocations - MinDistances[testNode]; k < numDeployedLocations; ++k)
					{
						v = hoverLocations[k];
						curResidualUAVCap = residualUAVCap[v];
						for (l = 0; l < numLocCoverClusters[v]; ++l)
						{
							clusterID = locCoverSpecCluster[v][l];
							curMinCap = residualUsersCopy[clusterID];
							if (curResidualUAVCap < curMinCap)
								curMinCap = curResidualUAVCap;
							residualUsersCopy[clusterID] -= curMinCap;
							curResidualUAVCap -= curMinCap;
							curTestUsers += curMinCap;
						}
					}

					if (1.0 * curTestUsers / (double)MinDistances[testNode] > bestRatio)
					{
						bestRatio = 1.0 * curTestUsers / (double)MinDistances[testNode];
						bestIncreasedUsers = curTestUsers;
						bestNextLocation = testNode;
					}

					numDeployedLocations -= MinDistances[testNode];
				} // 选出最优
				//printf("bestIncreasedUsers is %d and bestNextLocation is %d\n", bestIncreasedUsers, bestNextLocation); 

				 // added the nodes on the shortest path
				hoverLocations[numDeployedLocations] = bestNextLocation;
				numDeployedLocations++;
				isToDeployUAV[bestNextLocation] = true;
				nearestNode = allNearestNodes[bestNextLocation];
				v = bestNextLocation;
				while (nearestNode != allParentSPT[nearestNode][v])
				{
					nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
					hoverLocations[numDeployedLocations] = nextNode;
					isToDeployUAV[nextNode] = true;
					++numDeployedLocations;
					v = nextNode;
				}

				curTestUsers = 0;
				for (k = numDeployedLocations - MinDistances[bestNextLocation]; k < numDeployedLocations; ++k)
				{
					v = hoverLocations[k];
					curResidualUAVCap = residualUAVCap[v];
					for (l = 0; l < numLocCoverClusters[v]; ++l)
					{
						clusterID = locCoverSpecCluster[v][l];
						curMinCap = residualUsers[clusterID];
						if (curResidualUAVCap < curMinCap)
							curMinCap = curResidualUAVCap;
						residualUsers[clusterID] -= curMinCap;
						curResidualUAVCap -= curMinCap;
						curTestUsers += curMinCap;
					}
				}
				totalUsers += curTestUsers;

				//update the shortest paths and distances
				for (l = 0; l < numHoverLocations; ++l)
				{
					if (true == isToDeployUAV[l]) continue;
					for (k = numDeployedLocations - MinDistances[bestNextLocation]; k < numDeployedLocations; ++k)
					{
						v = hoverLocations[k];

						if (minHops[l][v] < MinDistances[l])
						{
							MinDistances[l] = minHops[l][v];
							allNearestNodes[l] = v;
						}
					}
				}
			}

			// 所有节点添加完毕
			curUsers = totalUsers;
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
			if (maxUsers < curUsers)
				maxUsers = curUsers;
			//}

		}
	}

	return maxUsers;
}

int maxUAVCoverage::ApproAlgLong()
{
	int i, j, k, l, m, n;
	int maxUsers = 0;
	int curUsers;
	// int loop_time = 0;

	int minHopsA = 0, minHopsB = 0, minHopsC = 0, minSum = 0;
	int s1 = 0, t1 = 0, s2 = 0, t2 = 0;
	int u, v;
	int clusterID;

	int MinDistances[numHoverLocations];


	int bestHoveringLocations[numHoverLocations];
	int bestNumHoveringLocations;

	bool isLabelled[numHoverLocations];
	for (i = 0; i < numHoverLocations; ++i) isLabelled[i] = false;
	for (i = 0; i < numHoverLocations; ++i) profitEachLocation[i] = 0;

	int curIncreasedThroughput;
	int bestIncreasedThroughput;
	int bestNextLoc;
	int restUAVCap;
	int lessCap;

	// assign profit for each hovering location
	for (i = 0; i < numClusters; ++i)
		residualUsers[i] = numUsersPerCluster[i]; // 每群剩余未覆盖用户数量初始化
	for (j = 0; j < numHoverLocations; ++j)
		residualUAVCap[j] = CapUAV; // 每个无人机位置剩余承载量
	for (i = 0; i < numHoverLocations; ++i)
	{
		bestIncreasedThroughput = -1;
		for (j = 0; j < numHoverLocations; ++j)
		{
			if (true == isLabelled[j]) continue;

			curIncreasedThroughput = 0;
			for (l = 0; l < numLocCoverClusters[j]; ++l)
			{
				clusterID = locCoverSpecCluster[j][l];
				curIncreasedThroughput += residualUsers[clusterID];
			}

			if (curIncreasedThroughput > residualUAVCap[j])
				curIncreasedThroughput = residualUAVCap[j];

			if (curIncreasedThroughput > bestIncreasedThroughput)
			{
				bestIncreasedThroughput = curIncreasedThroughput;
				bestNextLoc = j;
			}
		}
		bestIncreasedThroughput = 0;
		restUAVCap = residualUAVCap[bestNextLoc];
		for (l = 0; l < numLocCoverClusters[bestNextLoc]; ++l)
		{
			clusterID = locCoverSpecCluster[bestNextLoc][l];
			lessCap = residualUsers[clusterID];
			if (lessCap > restUAVCap) lessCap = restUAVCap;
			residualUsers[clusterID] -= lessCap;
			restUAVCap -= lessCap;
			bestIncreasedThroughput += lessCap;
		}
		isLabelled[bestNextLoc] = true;
		profitEachLocation[bestNextLoc] = bestIncreasedThroughput;
		if (bestIncreasedThroughput == 0) break;
		//printf("%d th node, assign profit to node %d, profit: %d\n", i + 1, bestNextLoc, bestIncreaseFlow);
	}

	clock_t st = clock();
	// initial three nodes
	for (i = 0; i < numHoverLocations; ++i)
	{
		for (j = i + 1; j < numHoverLocations; ++j)
		{
			for (k = j + 1; k < numHoverLocations; ++k)
			{
				//loop_time++;
				minHopsA = minHops[i][j] + minHops[i][k];
				minHopsB = minHops[i][j] + minHops[j][k];
				minHopsC = minHops[j][k] + minHops[i][k];
				// keep t1 = t2
				if (minHopsA <= minHopsB) {
					minSum = minHopsA;
					s1 = j;
					t1 = i;
					s2 = k;
					t2 = i;
				}
				else {
					minSum = minHopsB;
					s1 = i;
					t1 = j;
					s2 = k;
					t2 = j;
				}
				if (minSum > minHopsC) {
					minSum = minHopsC;
					s1 = j;
					t1 = k;
					s2 = i;
					t2 = k;
				}
				if (minSum > numUAVs - 1) continue;

				for (l = 0; l < numHoverLocations; ++l)	isToDeployUAV[l] = false;

				numDeployedLocations = 1;
				hoverLocations[0] = s2;
				isToDeployUAV[s2] = true;
				v = s2;
				while (-1 != allParentSPT[t2][v])
				{
					u = allParentSPT[t2][v];
					hoverLocations[numDeployedLocations] = u;
					++numDeployedLocations;
					isToDeployUAV[u] = true;
					v = u;
				}
				hoverLocations[numDeployedLocations] = s1;
				numDeployedLocations++;
				isToDeployUAV[s1] = true;
				v = s1;
				while (t1 != allParentSPT[t1][v])
				{
					u = allParentSPT[t1][v]; // t1 = t2
					if (false == isToDeployUAV[u])
					{
						hoverLocations[numDeployedLocations] = u;
						++numDeployedLocations;
						isToDeployUAV[u] = true;
					}
					v = u;
				}

				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------					
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
							//for (l = 0; l < numHoverLocations; ++l)
								//MinDistances[l] = 1000000;

				int minDis;
				// Find the shortest distances and paths to the nodes already in the subtree
				for (m = 0; m < numHoverLocations; ++m) // Vi中未添加节点
				{
					// printf("isToDeployUAV is %d\n", isToDeployUAV[halfDHops[test1]]); // 调试
					if (true == isToDeployUAV[m]) continue;
					minDis = 1000000;
					for (l = 0; l < numDeployedLocations; ++l)
					{
						v = hoverLocations[l];
						if (minHops[m][v] < minDis)
							minDis = minHops[m][v];
					}
					MinDistances[m] = minDis;
					//allNearestNodes[m] = nearestNode;
				}

				// 计算初始点所覆盖的用户数量
				int totalUsers = 0;
				for (l = 0; l < numDeployedLocations; ++l)
					totalUsers += profitEachLocation[hoverLocations[l]];
				// printf("initial total is %d\n", totalUsers);

				int bestNextLocation; // 最优位置
				int bestIncreasedUsers; // 最优增量
				int curTestUsers; // 当前测试节点用户增量

				int testNode;
				while (numDeployedLocations < numUAVs)
				{
					bestIncreasedUsers = -1;
					for (testNode = 0; testNode < numHoverLocations; ++testNode)
					{
						if (true == isToDeployUAV[testNode]) continue;
						if (MinDistances[testNode] >= 2) continue;  // do not see the nodes that are far away

						curTestUsers = profitEachLocation[testNode];

						if (curTestUsers > bestIncreasedUsers)
						{
							bestIncreasedUsers = curTestUsers;
							bestNextLocation = testNode;
						}
						// no better hovering locations
						if (bestIncreasedUsers == residualUAVCap[testNode]) break;
					} // 选出最优

					// added the nodes on the shortest path
					hoverLocations[numDeployedLocations] = bestNextLocation;
					numDeployedLocations++;
					isToDeployUAV[bestNextLocation] = true;

					totalUsers += profitEachLocation[bestNextLocation];

					//update the shortest paths and distances
					for (l = 0; l < numHoverLocations; ++l)
					{
						if (true == isToDeployUAV[l]) continue;
						if (minHops[bestNextLocation][l] < MinDistances[l])
							MinDistances[l] = minHops[bestNextLocation][l];
					}
				}
				// 所有节点添加完毕
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
				if (maxUsers < totalUsers)
				{
					maxUsers = totalUsers;
					bestNumHoveringLocations = numDeployedLocations;
					for (l = 0; l < numDeployedLocations; ++l)
						bestHoveringLocations[l] = hoverLocations[l];
				}

				// if (loop_time % 50000 == 0) printf("loop time: %d, time: %.2lf seconds\n", loop_time, (clock() - st) / 1000.0);
			}
		}
	}

	numDeployedLocations = bestNumHoveringLocations;
	for (l = 0; l < numDeployedLocations; ++l)
		hoverLocations[l] = bestHoveringLocations[l];
	maxUsers = calDeployedCoverUsers();
	return maxUsers;
}

int maxUAVCoverage::AlgTreeDecomp()
{
	int i, j, k, l;
	int maxUsers = 0;
	int curUsers;
	int loop_time = 0;

	int u, v;
	int clusterID;

	int bestHoveringLocations[numHoverLocations];
	int bestNumHoveringLocations;

	int MinDistances[numHoverLocations];
	int allNearestNodes[numHoverLocations];

	bool isLabelled[numHoverLocations];
	for (i = 0; i < numHoverLocations; ++i) isLabelled[i] = false;
	for (i = 0; i < numHoverLocations; ++i) profitEachLocation[i] = 0;

	int curIncreasedThroughput;
	int bestIncreasedThroughput;
	int bestNextLoc;
	int restUAVCap;
	int lessCap;

	// assign profit for each hovering location
	for (i = 0; i < numClusters; ++i)
		residualUsers[i] = numUsersPerCluster[i]; // 每群剩余未覆盖用户数量初始化
	for (j = 0; j < numHoverLocations; ++j)
		residualUAVCap[j] = CapUAV; // 每个无人机位置剩余承载量
	for (i = 0; i < numHoverLocations; ++i)
	{
		bestIncreasedThroughput = -1;
		for (j = 0; j < numHoverLocations; ++j)
		{
			if (true == isLabelled[j]) continue;

			curIncreasedThroughput = 0;
			for (l = 0; l < numLocCoverClusters[j]; ++l)
			{
				clusterID = locCoverSpecCluster[j][l];
				curIncreasedThroughput += residualUsers[clusterID];
			}

			if (curIncreasedThroughput > residualUAVCap[j])
				curIncreasedThroughput = residualUAVCap[j];

			if (curIncreasedThroughput > bestIncreasedThroughput)
			{
				bestIncreasedThroughput = curIncreasedThroughput;
				bestNextLoc = j;
			}
		}
		bestIncreasedThroughput = 0;
		restUAVCap = residualUAVCap[bestNextLoc];
		for (l = 0; l < numLocCoverClusters[bestNextLoc]; ++l)
		{
			clusterID = locCoverSpecCluster[bestNextLoc][l];
			lessCap = residualUsers[clusterID];
			if (lessCap > restUAVCap) lessCap = restUAVCap;
			residualUsers[clusterID] -= lessCap;
			restUAVCap -= lessCap;
			bestIncreasedThroughput += lessCap;
		}
		isLabelled[bestNextLoc] = true;
		profitEachLocation[bestNextLoc] = bestIncreasedThroughput;
		if (bestIncreasedThroughput == 0) break;
		//printf("%d th node, assign profit to node %d, profit: %d\n", i + 1, bestNextLoc, bestIncreaseFlow);
	}
	// the operation of assigning profit finishes

	// 初始选2个点
	for (i = 0; i < numHoverLocations; ++i)
	{
		for (j = i + 1; j < numHoverLocations; ++j)
		{
			if (minHops[i][j] > numUAVs - 1) continue;
			for (k = 0; k < numHoverLocations; ++k)	isToDeployUAV[k] = false;

			numDeployedLocations = 1;
			hoverLocations[0] = j;
			isToDeployUAV[j] = true;

			v = j;
			while (-1 != allParentSPT[i][v])
			{
				u = allParentSPT[i][v];
				hoverLocations[numDeployedLocations] = u;
				++numDeployedLocations;
				isToDeployUAV[u] = true;
				v = u;
			}

			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------					
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
						//for (k = 0; k < numHoverLocations; ++k)
						//	MinDistances[k] = 1000000;

			int minDis;
			int nearestNode;
			// Find the shortest distances and paths to the nodes already in the subtree
			for (k = 0; k < numHoverLocations; ++k) // Vi中未添加节点
			{
				// printf("isToDeployUAV is %d\n", isToDeployUAV[halfDHops[test1]]); // 调试
				if (true == isToDeployUAV[k]) continue;
				minDis = 1000000;
				for (l = 0; l < numDeployedLocations; ++l) // 已添加节点
				{	// 对于Vi中每一个未添加节点，判断其与已添加的某个节点是否相邻
					v = hoverLocations[l];
					if (minHops[k][v] < minDis)
					{
						minDis = minHops[k][v];
						nearestNode = v;
					}
				}
				MinDistances[k] = minDis;
				allNearestNodes[k] = nearestNode;
			}

			// 计算初始点所覆盖的用户数量
			int totalUsers = 0;
			for (l = 0; l < numDeployedLocations; ++l)
				totalUsers += profitEachLocation[hoverLocations[l]];
			// printf("initial total is %d\n", totalUsers);

			// extendByGreedy---->遍历各节点，比较增益，选出最优
			int bestNextLocation; // 最优位置
			int bestIncreasedUsers; // 最优增量
			double bestRatio;
			int curTestUsers; // 当前测试节点用户增量

			// 下标：代表该未添加节点
			// 存储的值：代表离该未添加节点最近的已添加节点
			int nextNode;
			int testNode;

			while (numDeployedLocations < numUAVs)
			{
				bestRatio = -1;
				for (testNode = 0; testNode < numHoverLocations; ++testNode)
				{
					if (true == isToDeployUAV[testNode]) continue;
					//if (MinDistances[testNode] >= 2) continue;  // do not see the nodes that are far away
					if (numDeployedLocations + MinDistances[testNode] > numUAVs) continue;
					hoverLocations[numDeployedLocations] = testNode;
					numDeployedLocations++;
					nearestNode = allNearestNodes[testNode];
					v = testNode;
					curTestUsers = profitEachLocation[testNode];
					while (nearestNode != allParentSPT[nearestNode][v]) // not the root of the tree
					{
						nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
						hoverLocations[numDeployedLocations] = nextNode;
						++numDeployedLocations;
						v = nextNode;
						curTestUsers += profitEachLocation[nextNode];
					}

					if (1.0 * curTestUsers / (double)MinDistances[testNode] > bestRatio)
					{
						bestRatio = 1.0 * curTestUsers / (double)MinDistances[testNode];
						bestIncreasedUsers = curTestUsers;
						bestNextLocation = testNode;
					}

					numDeployedLocations -= MinDistances[testNode];
				} // 选出最优
				//printf("bestIncreasedUsers is %d and bestNextLocation is %d\n", bestIncreasedUsers, bestNextLocation); 

				 // added the nodes on the shortest path
				hoverLocations[numDeployedLocations] = bestNextLocation;
				numDeployedLocations++;
				isToDeployUAV[bestNextLocation] = true;
				nearestNode = allNearestNodes[bestNextLocation];
				v = bestNextLocation;
				curTestUsers = profitEachLocation[bestNextLocation];
				while (nearestNode != allParentSPT[nearestNode][v])
				{
					nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
					hoverLocations[numDeployedLocations] = nextNode;
					isToDeployUAV[nextNode] = true;
					++numDeployedLocations;
					v = nextNode;
					curTestUsers += profitEachLocation[nextNode];
				}
				totalUsers += curTestUsers;

				//update the shortest paths and distances
				for (l = 0; l < numHoverLocations; ++l)
				{
					if (true == isToDeployUAV[l]) continue;
					for (k = numDeployedLocations - MinDistances[bestNextLocation]; k < numDeployedLocations; ++k)
					{
						v = hoverLocations[k];

						if (minHops[l][v] < MinDistances[l])
						{
							MinDistances[l] = minHops[l][v];
							allNearestNodes[l] = v;
						}
					}
				}
			}

			// 所有节点添加完毕
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
			//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
			if (maxUsers < totalUsers)
			{
				maxUsers = totalUsers;
				bestNumHoveringLocations = numDeployedLocations;
				for (l = 0; l < numDeployedLocations; ++l)
					bestHoveringLocations[l] = hoverLocations[l];
			}
		}
	}

	for (i = 0; i < bestNumHoveringLocations; ++i)
		hoverLocations[i] = bestHoveringLocations[i];
	numDeployedLocations = bestNumHoveringLocations;
	maxUsers = calDeployedCoverUsers();

	return maxUsers;
}

int maxUAVCoverage::AlgAssign()
{
	int i, j, k, l;
	int maxUsers = 0;
	int curUsers;
	int loop_time = 0;

	int u, v;
	int clusterID;

	int bestHoveringLocations[numHoverLocations];
	int bestNumHoveringLocations;

	int MinDistances[numHoverLocations];
	int allNearestNodes[numHoverLocations];

	bool isLabelled[numHoverLocations];
	int curIncreasedThroughput;
	int bestIncreasedThroughput;
	int bestNextLoc;
	int restUAVCap;
	int lessCap;

	for (int firstNode = 0; firstNode < numHoverLocations; ++firstNode)
	{
		//if(firstNode %10 == 0)  printf("firstNode: %d\n", firstNode);

		for (i = 0; i < numHoverLocations; ++i) isLabelled[i] = false;
		for (i = 0; i < numHoverLocations; ++i) profitEachLocation[i] = 0;

		// assign profit for each hovering location
		for (i = 0; i < numClusters; ++i)
			residualUsers[i] = numUsersPerCluster[i]; // 每群剩余未覆盖用户数量初始化
		for (j = 0; j < numHoverLocations; ++j)
			residualUAVCap[j] = CapUAV; // 每个无人机位置剩余承载量
		for (i = 0; i < numHoverLocations; ++i)
		{
			bestIncreasedThroughput = -1;
			if (i == 0)  bestNextLoc = firstNode;
			else {
				for (j = 0; j < numHoverLocations; ++j)
				{
					if (true == isLabelled[j]) continue;

					curIncreasedThroughput = 0;
					for (l = 0; l < numLocCoverClusters[j]; ++l)
					{
						clusterID = locCoverSpecCluster[j][l];
						curIncreasedThroughput += residualUsers[clusterID];
					}

					if (curIncreasedThroughput > residualUAVCap[j])
						curIncreasedThroughput = residualUAVCap[j];

					if (curIncreasedThroughput > bestIncreasedThroughput)
					{
						bestIncreasedThroughput = curIncreasedThroughput;
						bestNextLoc = j;
					}
				}
			}
			bestIncreasedThroughput = 0;
			restUAVCap = residualUAVCap[bestNextLoc];
			for (l = 0; l < numLocCoverClusters[bestNextLoc]; ++l)
			{
				clusterID = locCoverSpecCluster[bestNextLoc][l];
				lessCap = residualUsers[clusterID];
				if (lessCap > restUAVCap) lessCap = restUAVCap;
				residualUsers[clusterID] -= lessCap;
				restUAVCap -= lessCap;
				bestIncreasedThroughput += lessCap;
			}
			isLabelled[bestNextLoc] = true;
			profitEachLocation[bestNextLoc] = bestIncreasedThroughput;
			if (bestIncreasedThroughput == 0) break;
			//printf("%d th node, assign profit to node %d, profit: %d\n", i + 1, bestNextLoc, bestIncreaseFlow);
		}
		// the operation of assigning profit finishes

		// 初始选2个点
		for (i = 0; i < numHoverLocations; ++i)
		{
			for (j = i + 1; j < numHoverLocations; ++j)
			{
				if (minHops[i][j] > numUAVs - 1) continue;
				for (k = 0; k < numHoverLocations; ++k)	isToDeployUAV[k] = false;

				numDeployedLocations = 1;
				hoverLocations[0] = j;
				isToDeployUAV[j] = true;

				v = j;
				while (-1 != allParentSPT[i][v])
				{
					u = allParentSPT[i][v];
					hoverLocations[numDeployedLocations] = u;
					++numDeployedLocations;
					isToDeployUAV[u] = true;
					v = u;
				}

				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------					
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
							//for (k = 0; k < numHoverLocations; ++k)
							//	MinDistances[k] = 1000000;

				int minDis;
				int nearestNode;
				// Find the shortest distances and paths to the nodes already in the subtree
				for (k = 0; k < numHoverLocations; ++k) // Vi中未添加节点
				{
					// printf("isToDeployUAV is %d\n", isToDeployUAV[halfDHops[test1]]); // 调试
					if (true == isToDeployUAV[k]) continue;
					minDis = 1000000;
					for (l = 0; l < numDeployedLocations; ++l) // 已添加节点
					{	// 对于Vi中每一个未添加节点，判断其与已添加的某个节点是否相邻
						v = hoverLocations[l];
						if (minHops[k][v] < minDis)
						{
							minDis = minHops[k][v];
							nearestNode = v;
						}
					}
					MinDistances[k] = minDis;
					allNearestNodes[k] = nearestNode;
				}

				// 计算初始点所覆盖的用户数量
				int totalUsers = 0;
				for (l = 0; l < numDeployedLocations; ++l)
					totalUsers += profitEachLocation[hoverLocations[l]];
				// printf("initial total is %d\n", totalUsers);

				// extendByGreedy---->遍历各节点，比较增益，选出最优
				int bestNextLocation; // 最优位置
				int bestIncreasedUsers; // 最优增量
				double bestRatio;
				int curTestUsers; // 当前测试节点用户增量

				// 下标：代表该未添加节点
				// 存储的值：代表离该未添加节点最近的已添加节点
				int nextNode;
				int testNode;

				while (numDeployedLocations < numUAVs)
				{
					bestRatio = -1;
					for (testNode = 0; testNode < numHoverLocations; ++testNode)
					{
						if (true == isToDeployUAV[testNode]) continue;
						//if (MinDistances[testNode] >= 2) continue;  // do not see the nodes that are far away
						if (numDeployedLocations + MinDistances[testNode] > numUAVs) continue;
						hoverLocations[numDeployedLocations] = testNode;
						numDeployedLocations++;
						nearestNode = allNearestNodes[testNode];
						v = testNode;
						curTestUsers = profitEachLocation[testNode];
						while (nearestNode != allParentSPT[nearestNode][v]) // not the root of the tree
						{
							nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
							hoverLocations[numDeployedLocations] = nextNode;
							++numDeployedLocations;
							v = nextNode;
							curTestUsers += profitEachLocation[nextNode];
						}

						if (1.0 * curTestUsers / (double)MinDistances[testNode] > bestRatio)
						{
							bestRatio = 1.0 * curTestUsers / (double)MinDistances[testNode];
							bestIncreasedUsers = curTestUsers;
							bestNextLocation = testNode;
						}

						numDeployedLocations -= MinDistances[testNode];
					} // 选出最优
					//printf("bestIncreasedUsers is %d and bestNextLocation is %d\n", bestIncreasedUsers, bestNextLocation); 

					 // added the nodes on the shortest path
					hoverLocations[numDeployedLocations] = bestNextLocation;
					numDeployedLocations++;
					isToDeployUAV[bestNextLocation] = true;
					nearestNode = allNearestNodes[bestNextLocation];
					v = bestNextLocation;
					curTestUsers = profitEachLocation[bestNextLocation];
					while (nearestNode != allParentSPT[nearestNode][v])
					{
						nextNode = allParentSPT[nearestNode][v];//将矩阵中第v节点所记录的其父节点赋值给u，此时u为v的父节点
						hoverLocations[numDeployedLocations] = nextNode;
						isToDeployUAV[nextNode] = true;
						++numDeployedLocations;
						v = nextNode;
						curTestUsers += profitEachLocation[nextNode];
					}
					totalUsers += curTestUsers;

					//update the shortest paths and distances
					for (l = 0; l < numHoverLocations; ++l)
					{
						if (true == isToDeployUAV[l]) continue;
						for (k = numDeployedLocations - MinDistances[bestNextLocation]; k < numDeployedLocations; ++k)
						{
							v = hoverLocations[k];

							if (minHops[l][v] < MinDistances[l])
							{
								MinDistances[l] = minHops[l][v];
								allNearestNodes[l] = v;
							}
						}
					}
				}

				// 所有节点添加完毕
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
				if (maxUsers < totalUsers)
				{
					maxUsers = totalUsers;
					bestNumHoveringLocations = numDeployedLocations;
					for (l = 0; l < numDeployedLocations; ++l)
						bestHoveringLocations[l] = hoverLocations[l];
				}
			}
		}
	}
	for (i = 0; i < bestNumHoveringLocations; ++i)
		hoverLocations[i] = bestHoveringLocations[i];
	numDeployedLocations = bestNumHoveringLocations;
	maxUsers = calDeployedCoverUsers();

	return maxUsers;
}



int maxUAVCoverage::ApproAlgShort()
{
	int i, j, k, l, m, n;
	int maxUsers = 0;
	int curUsers;
	// int loop_time = 0;

	int minHopsA = 0, minHopsB = 0, minHopsC = 0, minSum = 0;
	int s1 = 0, t1 = 0, s2 = 0, t2 = 0;
	int u, v;
	int clusterID;

	int MinDistances[numHoverLocations];
	cout << "numHoverLocation = " << numHoverLocations << endl;
	// clock_t st = clock();
	// initial three nodes
	for (i = 0; i < numHoverLocations; ++i)
	{
		cout << "i = " << i << endl;
		for (j = i + 1; j < numHoverLocations; ++j)
		{
			// cout << "i = " << i << ", j = " << j << endl;
			for (k = j + 1; k < numHoverLocations; ++k)
			{
				// loop_time++;
				minHopsA = minHops[i][j] + minHops[i][k];
				minHopsB = minHops[i][j] + minHops[j][k];
				minHopsC = minHops[j][k] + minHops[i][k];
				// keep t1 = t2
				if (minHopsA <= minHopsB) {
					minSum = minHopsA;
					s1 = j;
					t1 = i;
					s2 = k;
					t2 = i;
				}
				else {
					minSum = minHopsB;
					s1 = i;
					t1 = j;
					s2 = k;
					t2 = j;
				}
				if (minSum > minHopsC) {
					minSum = minHopsC;
					s1 = j;
					t1 = k;
					s2 = i;
					t2 = k;
				}
				if (minSum > numUAVs - 1) continue;

				for (l = 0; l < numHoverLocations; ++l)	isToDeployUAV[l] = false;

				numDeployedLocations = 1;
				hoverLocations[0] = s2;
				isToDeployUAV[s2] = true;
				v = s2;
				while (-1 != allParentSPT[t2][v])
				{
					u = allParentSPT[t2][v];
					hoverLocations[numDeployedLocations] = u;
					++numDeployedLocations;
					isToDeployUAV[u] = true;
					v = u;
				}
				hoverLocations[numDeployedLocations] = s1;
				numDeployedLocations++;
				isToDeployUAV[s1] = true;
				v = s1;
				while (t1 != allParentSPT[t1][v])
				{
					u = allParentSPT[t1][v]; // t1 = t2
					if (false == isToDeployUAV[u])
					{
						hoverLocations[numDeployedLocations] = u;
						++numDeployedLocations;
						isToDeployUAV[u] = true;
					}
					v = u;
				}

				/*printf("the deployed number is %d and nodes chosen is %d, %d, %d\n", numDeployedLocations, i, j, k);
				for (m = 0; m < numDeployedLocations; ++m)
				{
					printf("the initial nodes are %d\n", hoverLocations[m]);
				}*/

				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------					
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
							//for (l = 0; l < numHoverLocations; ++l)
								//MinDistances[l] = 1000000;

				int minDis;
				// Find the shortest distances and paths to the nodes already in the subtree
				for (m = 0; m < numHoverLocations; ++m) // Vi中未添加节点
				{
					//printf("isToDeployUAV is %d\n", isToDeployUAV[halfDHops[test1]]); // 调试
					if (true == isToDeployUAV[m]) continue;
					minDis = 1000000;
					for (l = 0; l < numDeployedLocations; ++l)
					{
						v = hoverLocations[l];
						if (minHops[m][v] < minDis)
							minDis = minHops[m][v];
					}
					MinDistances[m] = minDis;
					//allNearestNodes[m] = nearestNode;
				}

				// 计算初始点所覆盖的用户数量
				int totalUsers = 0;
				totalUsers = calDeployedCoverUsers();
				// printf("initial total is %d\n", totalUsers);

				int bestNextLocation; // 最优位置
				int bestIncreasedUsers; // 最优增量
				//double bestRatio;
				int curTestUsers; // 当前测试节点用户增量

				// 下标：代表该未添加节点
				// 存储的值：代表离该未添加节点最近的已添加节点

				int curMinCap; // 当前较小剩余量
				int curResidualUsers = 0; // 初始化临时变量，目的是测试节点过程中先不改变residualUsers[]数组中的值
				int curResidualUAVCap = 0; // 初始化临时变量，目的是测试节点过程中先不改变residualUAVCap[]数组中的值
				//int nextNode;
				int testNode;

				while (numDeployedLocations < numUAVs)
				{
					//cout << "numDeployedLocations = " << numDeployedLocations << ", numUAVs = " << numUAVs << endl;
					bestIncreasedUsers = -1;
					for (testNode = 0; testNode < numHoverLocations; ++testNode)
					{
						if (true == isToDeployUAV[testNode]) continue;
						if (MinDistances[testNode] >= 2) continue;  // do not see the nodes that are far away

						curTestUsers = 0;
						for (l = 0; l < numLocCoverClusters[testNode]; ++l)
							curTestUsers += residualUsers[locCoverSpecCluster[testNode][l]];
						if (curTestUsers > residualUAVCap[testNode])
							curTestUsers = residualUAVCap[testNode];

						if (curTestUsers > bestIncreasedUsers)
						{
							bestIncreasedUsers = curTestUsers;
							bestNextLocation = testNode;
						}
						// no better hovering locations
						if (bestIncreasedUsers == residualUAVCap[testNode]) break;
					} // 选出最优

					// added the nodes on the shortest path
					hoverLocations[numDeployedLocations] = bestNextLocation;
					numDeployedLocations++;
					isToDeployUAV[bestNextLocation] = true;

					curTestUsers = 0;
					curResidualUAVCap = residualUAVCap[bestNextLocation];
					for (l = 0; l < numLocCoverClusters[bestNextLocation]; ++l)
					{
						clusterID = locCoverSpecCluster[bestNextLocation][l];
						curMinCap = residualUsers[clusterID];
						if (curResidualUAVCap < curMinCap)
							curMinCap = curResidualUAVCap;
						residualUsers[clusterID] -= curMinCap;
						curResidualUAVCap -= curMinCap;
						curTestUsers += curMinCap;
					}
					totalUsers += curTestUsers;

					//update the shortest paths and distances
					for (l = 0; l < numHoverLocations; ++l)
					{
						if (true == isToDeployUAV[l]) continue;
						if (minHops[bestNextLocation][l] < MinDistances[l])
							MinDistances[l] = minHops[bestNextLocation][l];
					}
				}
				// 所有节点添加完毕
				curUsers = totalUsers;
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------						
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
				//-----------------------------------------------------------------extendByGreedy分割线----------------------------------------------------------------------------------------	
				if (maxUsers < curUsers)
					maxUsers = curUsers;
				//if (loop_time % 50000 == 0) printf("loop time: %d, time: %.2lf seconds\n", loop_time, (clock() - st) / 1000.0);
			}
		}
	}
	return maxUsers;
}




// baseline 1
double maxUAVCoverage::AlgGreedy()
{
	int i, j, k;

	// ===== 方法1: 注释掉简单覆盖计数 =====
	/*
	double profit;
	double dis;
	for (i = 0; i < numHoverLocations; ++i) {
		profit = 0;
		for (j = 0; j < numClusters; ++j) {
			dis = distance2D(xHoverLoc[i], yHoverLoc[i], xCluster[j], yCluster[j]);
			if (dis <= userCommRadius)
				profit += numUsersPerCluster[j];
		}
		profitEachLocation[i] = profit;
	}
	*/

	bool isLabelled[numHoverLocations];
	for (i = 0; i < numHoverLocations; ++i) isLabelled[i] = false;

	bool isCovered[numClusters];
	for (i = 0; i < numClusters; ++i) isCovered[i] = false;

	// ===== 方法2: 取消注释最大覆盖算法 =====
	int sumCovered = 0;
	int bestIncreaseCovered;
	int bestNextLoc;
	int curCovered;
	double dis;

	for (i = 0; i < numHoverLocations; ++i) {
		bestIncreaseCovered = -1;
		for (j = 0; j < numHoverLocations; ++j) {
			if (true == isLabelled[j]) continue;

			curCovered = 0;
			for (k = 0; k < numClusters; ++k) {
				if (true == isCovered[k]) continue;
				dis = distance2D(xHoverLoc[j], yHoverLoc[j], xCluster[k], yCluster[k]);
				if (dis <= userCommRadius)
					curCovered += numUsersPerCluster[k];
			}

			if (curCovered > bestIncreaseCovered) {
				bestIncreaseCovered = curCovered;
				bestNextLoc = j;
			}
		}
		isLabelled[bestNextLoc] = true;
		profitEachLocation[bestNextLoc] = bestIncreaseCovered;
		for (k = 0; k < numClusters; ++k) {
			if (true == isCovered[k]) continue;
			dis = distance2D(xHoverLoc[bestNextLoc], yHoverLoc[bestNextLoc], xCluster[k], yCluster[k]);
			if (dis <= userCommRadius) isCovered[k] = true;
		}

		sumCovered += bestIncreaseCovered;
		// printf("%d th node, assign profit to node %d, profit: %d\n", 
		//        i + 1, bestNextLoc, bestIncreaseCovered);
	}

	// ===== 后续代码保持不变，但修改返回值 =====
	int maxCovered = 0;  // ← 改为最大覆盖用户数
	for (i = 0; i < numHoverLocations; ++i) {
		numDeployedLocations = 1;
		hoverLocations[0] = i;
		for (j = 0; j < numHoverLocations; ++j) isToDeployUAV[j] = false;
		isToDeployUAV[i] = true;

		extendByGreedyProfitLabel();
		curCovered = calDeployedCoverUsers();  // ← 改为计算覆盖用户数
		if (maxCovered < curCovered) {
			maxCovered = curCovered;
		}
	}
	return maxCovered;  // ← 返回最大覆盖用户数
}



/*int maxUAVCoverage::extendByGreedy()
{
	int k, l, m, n;
	int totalThroughput = 0;

	int minDis;
	for (k = 0; k < numHoverLocations; ++k)
	{
		if (true == isToDeployUAV[k]) continue;
		minDis = 1000000;
		for (l = 0; l < numDeployedLocations; ++l)
			if (true == adjMatrix[k][hoverLocations[l]]) //adjMatrix在findMinHops中更新
			{	minDis = 1;
				break;
			}
		nearestEdgeWeight[k] = minDis;
	}
	totalThroughput = calDeployedCoverUsers();
	//printf("initial total is %d\n", totalThroughput);

	int newThroughput;
	int bestNextLocation = 0;
	int bestIncreasedThroughput;
	int test2;
	int curMinCap = 0;
	while (numDeployedLocations < numUAVs)
	{
		bestIncreasedThroughput = -1;
		for (m = 0; m < numHoverLocations; ++m)
		{
			if (true == isToDeployUAV[m]) continue;
			if (nearestEdgeWeight[m] >= 2) continue; // not adjacent

			hoverLocations[numDeployedLocations] = m;

			//printf("the current added location is %d\n", hoverLocations[numDeployedLocations]);


			numDeployedLocations++;
			newThroughput = 0;

			//printf("the newthroughput is recovered as %d\n", newThroughput);


			// 计算临时额外覆盖，UAV capacity有限制
			curResidualUAVCap = residualUAVCap[m];
			for (test2 = 0; test2 < numLocCoverClusters[m]; ++test2)
			{
				curResidualUsers = residualUsers[locCoverSpecCluster[test2][m]];

				//printf("residual UAVcap is %d and residual users is %d\n", curResidualUAVCap, curResidualUsers);


				if (0 == curResidualUsers) continue; // 用户已全被覆盖
				if (0 == curResidualUAVCap) break; // 该位置剩余承载量为0
				curMinCap = curResidualUsers;
				if (curResidualUsers > curResidualUAVCap)
				{
					curMinCap = curResidualUAVCap;
				}
				newThroughput += curMinCap; // 此处计算的为增量，因为residual数组记录的值，已覆盖过的用户已经被减掉

				curResidualUsers -= curMinCap;
				curResidualUAVCap -= curMinCap;
			}

			//printf("new is %d\n", newThroughput);

			if (newThroughput > bestIncreasedThroughput)
			{
				bestIncreasedThroughput = newThroughput;
				bestNextLocation = m;
			}
			//printf("the best increased is %d and best nest location is %d\n", bestIncreasedThroughput, bestNextLocation);
			--numDeployedLocations;
		}

		if (bestIncreasedThroughput == 0) break;

		// 选出最优，添加最优
		int residualBestIncreased = 0;
		int test7 = 0;
		int bestMinCap = 0;
		hoverLocations[numDeployedLocations] = bestNextLocation;
		numDeployedLocations++;
		totalThroughput += bestIncreasedThroughput;
		isToDeployUAV[bestNextLocation] = true;

		// 更新剩余数量
		residualBestIncreased = bestIncreasedThroughput;
		residualUAVCap[bestNextLocation] -= bestIncreasedThroughput;
		for (test7 = 0; test7 < numLocCoverClusters[bestNextLocation]; ++test7)
		{
			if (0 == residualUsers[locCoverSpecCluster[test7][bestNextLocation]]) continue;
			if (0 == residualBestIncreased) break;

			//printf("best increase is %d and residual users are %d\n", residualBestIncreased, residualUsers[locCoverSpecCluster[test7][bestNextLocation]]);
			bestMinCap = residualBestIncreased;
			if (residualBestIncreased > residualUsers[locCoverSpecCluster[test7][bestNextLocation]]) // 新增人数比该用户群人数多，则说明同时覆盖了多个群
			{
				bestMinCap = residualUsers[locCoverSpecCluster[test7][bestNextLocation]];
			}
			residualUsers[locCoverSpecCluster[test7][bestNextLocation]] -= bestMinCap;
			residualBestIncreased -= bestMinCap;
		}

		for (n = 0; n < numHoverLocations; ++n)
		{
			if (true == isToDeployUAV[n]) continue;
			if (true == adjMatrix[bestNextLocation][n])
				nearestEdgeWeight[n] = 1;
		}
	}
			// curThroughput = totalThroughput;
	return totalThroughput;
}*/

double maxUAVCoverage::extendByGreedyProfitLabel()
{
	int i, j;
	int u, v;
	int totalUser;

	int minDis;
	for (i = 0; i < numHoverLocations; ++i) // 相邻关系
	{
		if (true == isToDeployUAV[i]) continue;
		minDis = 1000000;
		for (j = 0; j < numDeployedLocations; ++j)
			if (true == adjMatrix[i][hoverLocations[j]])
				minDis = 1;
		nearestEdgeWeight[i] = minDis;
	}
	totalUser = 0;
	for (i = 0; i < numDeployedLocations; ++i) // 初始覆盖用户
		totalUser += profitEachLocation[hoverLocations[i]];

	int curIncreaseUser;
	int bestIncreaseUser;
	int bestNextLocation;
	while (numDeployedLocations < numUAVs)
	{
		bestIncreaseUser = -1;
		for (j = 0; j < numHoverLocations; ++j)
		{
			if (true == isToDeployUAV[j]) continue;
			if (nearestEdgeWeight[j] >= 2) continue; // not adjacent

			hoverLocations[numDeployedLocations] = j;
			numDeployedLocations++;

			curIncreaseUser = profitEachLocation[j];
			if (curIncreaseUser > bestIncreaseUser)
			{
				bestIncreaseUser = curIncreaseUser;
				bestNextLocation = j;
			}

			--numDeployedLocations;
		}
		if (-1 == bestIncreaseUser) break;

		hoverLocations[numDeployedLocations] = bestNextLocation;
		numDeployedLocations++;
		totalUser += bestIncreaseUser;

		isToDeployUAV[bestNextLocation] = true;
		for (j = 0; j < numHoverLocations; ++j)
		{
			if (true == isToDeployUAV[j]) continue;
			if (true == adjMatrix[bestNextLocation][j])
				nearestEdgeWeight[j] = 1;
		}
	}
	return totalUser;
}


void maxUAVCoverage::findShortestPath(int s)
{
	int sz = numHoverLocations;
	int i, j;

	for (i = 0; i < sz; ++i) visited[i] = false;
	for (i = 0; i < sz; ++i) parentSPT[i] = -1;
	visited[s] = true;
	queue_clear();
	queue_push(s);

	int u;
	while (queue_size() > 0)
	{
		u = queue_pop();
		for (j = 0; j < numHoverLocations; ++j)
		{
			if (false == adjMatrix[u][j]) continue;
			if (true == visited[j]) continue;

			queue_push(j);
			visited[j] = true;
			parentSPT[j] = u;
			//allParentSPT[s][j] = u;
		}
	}
}

void maxUAVCoverage::findAllShortestPath()
{
	int i, j;

	for (i = 0; i < numHoverLocations; ++i)
	{
		findShortestPath(i);
		for (j = 0; j < numHoverLocations; ++j)
		{
			allParentSPT[i][j] = parentSPT[j];
		}

	}

}

void maxUAVCoverage::initialization()
{
	assignUserLocations();
	assignHoverLocations();
	findMinHopsAmongLocations();
	findAllShortestPath();
	//calAvgDataRates();
	calUsersCoveredPerLocation();
}

void maxUAVCoverage::findMinHopsAmongLocations()
{

	bool visited[numHoverLocations];
	int minDis[numHoverLocations];
	int i, j;


	for (i = 0; i < numClusters; ++i)
		for (j = 0; j < numHoverLocations; ++j)
			disMatrix[i][j] = distance2D(xCluster[i], yCluster[i], xHoverLoc[j], yHoverLoc[j]);

	for (i = 0; i < numHoverLocations; ++i)
	{
		queue_clear();
		queue_push(i); // mininum hops from node i to other nodes, by applying BFS
		for (j = 0; j < numHoverLocations; ++j) visited[j] = false;
		visited[i] = true;
		for (j = 0; j < numHoverLocations; ++j) minDis[j] = 99999;
		minDis[i] = 0;

		int v;
		while (queue_size() > 0)
		{
			v = queue_pop();
			for (j = 0; j < numHoverLocations; ++j)
			{
				if (true == visited[j]) continue;
				if (distance2D(xHoverLoc[v], yHoverLoc[v], xHoverLoc[j], yHoverLoc[j]) <= R)
				{
					minDis[j] = minDis[v] + 1;
					visited[j] = true;
					queue_push(j);
				}
			}

		}
		for (j = 0; j < numHoverLocations; ++j)
			minHops[i][j] = minDis[j];
	}

	for (i = 0; i < numHoverLocations; ++i)
		for (j = 0; j < numHoverLocations; ++j)
			adjMatrix[i][j] = false;
	for (i = 0; i < numHoverLocations; ++i)
		for (j = i + 1; j < numHoverLocations; ++j)
			if (distance2D(xHoverLoc[i], yHoverLoc[i], xHoverLoc[j], yHoverLoc[j]) <= R)
				//两点距离小于R，则认为该两点相邻，与网格相邻无关
				//即不以网格相邻为标准进行判断，而是以一跳距离为标准，衡量是否相邻
				adjMatrix[i][j] = adjMatrix[j][i] = true;
}

void maxUAVCoverage::assignUserLocations()
{
	double con = (double)numUsers / (log2(numClusters) / log2(2.35));
	int i;
	int sumUsers = 0, curUsers;
	for (i = 1; i < numClusters; ++i)
	{
		curUsers = (int)ceil(con / i); // zipf distribution
		if (sumUsers + curUsers > numUsers)
			curUsers = numUsers - sumUsers;
		numUsersPerCluster[i - 1] = curUsers;
		sumUsers += curUsers;

	}
	numUsersPerCluster[numClusters - 1] = numUsers - sumUsers;

	xCluster[0] = 0; yCluster[0] = 0;
	xCluster[1] = L; yCluster[1] = W;
	xCluster[2] = 0; yCluster[2] = W;
	xCluster[3] = L; yCluster[3] = 0;
	for (i = 4; i < numClusters; ++i)
	{
		xCluster[i] = L * Uniform();
		yCluster[i] = W * Uniform();
	}

	/*if (verbose)
	{
		for (i = 0; i < numClusters; ++i)
			printf("cluster %d, users: %d, x: %.0lf,\ty: %.lf\n", i, numUsersPerCluster[i], xCluster[i], yCluster[i]);
	}*/
}

void maxUAVCoverage::assignHoverLocations()
{
	int rows = L / delta;
	int cols = W / delta;
	int i, j;
	int index = 0;
	for (i = 0; i < rows; ++i)
		for (j = 0; j < cols; ++j)
		{
			xHoverLoc[index] = delta / 2 + i * delta;
			//横坐标，每个网格以中间点表示，因此第一个位置为150，间隔300
			yHoverLoc[index] = delta / 2 + j * delta;
			index++;
		}

	for (i = 0; i < numHoverLocations; ++i)
		printf("location %d, x: %.lf\t, y: %.lf\n", i, xHoverLoc[i], yHoverLoc[i]);
}

void maxUAVCoverage::calUsersCoveredPerLocation()
{
	int i, j;


	for (i = 0; i < numHoverLocations; ++i)
	{
		numLocCoverClusters[i] = 0; // 计数器，维护每个位置实际覆盖了多少个cluster，目的是减少循环次数
		for (j = 0; j < numClusters; ++j)
		{
			if (disMatrix[j][i] <= userCommRadius) // 通信范围内的用户
			{
				locCoverSpecCluster[i][numLocCoverClusters[i]] = j; // 行表示个数，列表示位置，每列下的元素表示所覆盖的具体cluster编号
				++numLocCoverClusters[i];
			}
		}
	}
}

int maxUAVCoverage::calDeployedCoverUsers()
{
	int i, j;
	int lessCap;
	int sumUsersCovered = 0;

	for (i = 0; i < numClusters; ++i)
		residualUsers[i] = numUsersPerCluster[i]; // 每群剩余未覆盖用户数量初始化
	for (j = 0; j < numHoverLocations; ++j)
		residualUAVCap[j] = CapUAV; // 每个无人机位置剩余承载量
	// 初始化结束

	int clusterId = 0;
	int uavCap;
	for (i = 0; i < numDeployedLocations; ++i) // 实际位置为hoverLocation[i]，i为下标
	{
		uavCap = residualUAVCap[hoverLocations[i]];
		for (j = 0; j < numLocCoverClusters[hoverLocations[i]]; ++j) // 实际用户群为locCoverSpecCluster[j][hoverLocation[i]]，j为行下标
		{
			clusterId = locCoverSpecCluster[hoverLocations[i]][j];
			//if (0 == residualUsers[clusterId]) continue; // 该用户群所有用户均已被覆盖
			//if (0 == residualUAVCap[hoverLocations[i]]) break; // 该位置剩余承载力为0
			lessCap = residualUsers[clusterId];
			if (lessCap > uavCap) // 用户群剩余用户 超过 承载力
				lessCap = uavCap;
			sumUsersCovered += lessCap;
			uavCap -= lessCap;
			residualUsers[clusterId] -= lessCap;
			//residualUAVCap[hoverLocations[i]] -= lessCap;	
		}
	}
	return sumUsersCovered;
}

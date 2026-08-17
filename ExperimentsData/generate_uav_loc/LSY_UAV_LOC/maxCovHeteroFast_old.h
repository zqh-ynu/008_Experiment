#pragma once
#include "MyHeap.h"
#include "queue.h"
#include<vector>
#include<map>
#include<queue>
#include<algorithm>
#include<math.h>
#include<time.h>
#include<windows.h>
#include<fstream>
#include<iostream>
using namespace std;

//修改：
//1.最大流==>堆：效果显著
//2.steiner tree：削减组合数：基本不影响结果和速度

double verbose = false;
#define maxP 5
#define maxN 200
const int INF = 999999;
const int numOfSon = 4;

//局部变量变全局变量

ofstream resultFile;
char fileName[100];

const int L = 5000, W = 5000, h = 300; // the length and width of the monitor area, and the height (altitude) of UAVs
// variables about user distribution
const int numUsers = 1000; // number of ground users [500,3000]
const int numClusters = 40;
int numUsersPerCluster[numClusters];
int UserBase[numClusters];
double xCluster[numClusters], yCluster[numClusters];

double R = 600, r = 500; // the communication ranges between two UAVs and a ground user
// 600/sqrt(2)=424.264
//assert(h < r);
double userCommRadius = sqrt(r * r - h * h); // the user communication radius at altitude h

int Lmax, L_best;
int h_max;
vector<int> optimalP;
vector<int> Qh;
int* QhArr = NULL;
int* QhArrPlus = NULL;
int QhArrSZ = 0;
int QhArrPlusSZ = 0;

const int s = 2;
const int K = 20;
// vector<int> VStar;
int* VStar = NULL;
int VStarCount = 0;
int** keyGraph = NULL;
//int keyGraph[s][s];
int** keyTree = NULL;
//int keyTree[s][2];

// variables about UAV deployment
const int numUAVs = K; // number of UAVs [2,20]
int minCapUAV = 50; // the minimum number of users that a UAV can serve
int maxCapUAV = 50; // the maximum number of users that a UAV can serve
int avgCap = (minCapUAV + maxCapUAV) / 2;
int Cap[K * 2]; // Ck
int CapUAV = avgCap; // the maximum number of users that a UAV can serve

const int delta = 250; // 250 meters
//const int delta = 375; // 300 meters
const int numHoverLocations = (L / delta + 1) * (W / delta + 1) + numClusters + 3 * 4; // 为啥加后面两项？？
//const int numHoverLocations = (L / delta)*(W / delta) +numClusters + 3 * 4; // 为啥加后面两项？？
double xHoverLoc[numHoverLocations], yHoverLoc[numHoverLocations]; // the candidate hovering locations at altitude h
int minHops[numHoverLocations][numHoverLocations];
bool adjMatrix[numHoverLocations][numHoverLocations]; // Adjacency matrix UAV邻接
bool User2UAVLink[numClusters][numHoverLocations]; // 用户和每个候选点之间的连接性
double disMatrix[numClusters][numHoverLocations]; // the horizontal distance between UAV candidate points and user cluster center
bool candidateKeyMore[numHoverLocations];

bool isToDeployUAV[numHoverLocations];
int nearestEdgeWeight[numHoverLocations];
int inBestSol[numHoverLocations];
int inVr[numHoverLocations];
int inSol[numHoverLocations];

int degreeUAV[numHoverLocations];
int degreeUE[numUsers];
bool Ur[numHoverLocations]; // UAV never tried to be deleted
bool Uf[numHoverLocations]; // UAV cannot to be deleted
bool connectionUE2UAV[numUsers][numHoverLocations];
vector<int> toDeleteUAV;

int hoverLocations[numHoverLocations];
int numDeployedLocations;

int residualUsers[numClusters], residualUAVCap[numHoverLocations];

struct Edge
{
	int userClusterID;
	int locationID;
	int allocateCap; //min(用户簇剩余人数，UAV剩余可服务人数)

	bool operator<(const Edge& e)
	{
		return this->allocateCap < e.allocateCap;
	}
};
//Edge allEdges[numClusters * numUAVs];
Edge top;
int residualClusterUser[numClusters];
int residualLocCap[numHoverLocations];
MyHeap<Edge> myHeap(0, numOfSon);

struct E { int x, y, c, g; } bag[maxN * maxN];
int adjEdgeLen = 0, tou = 1, wei = 1;
int STcost[maxN + 1][1 << maxP];
int color[maxP];
//int key[maxP];
int vis[maxN], dl[maxN], hh[maxN];

class Path
{
public:
	int s;
	int t;

	Path(int v1, int v2)
	{
		s = v1;
		t = v2;
	}

	bool operator<(const Path& p) const
	{
		return (s < p.s) || (s == p.s && t < p.t);
	}
};
map<Path, vector<int>> minPath;

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
	return double(rand()) / RAND_MAX;
}

float avgFloat(float* arr, int len)
{
	float sum = 0;
	for (int i = 0; i < len; i++)
		sum = sum + arr[i];
	return sum / len;
}

float avgInt(int* arr, int len)
{
	float sum = 0;
	for (int i = 0; i < len; i++)
		sum = sum + arr[i];
	return (float)sum / len;
}

class maxUAVCoverage {
public:
	void initialization();
	void printInitial();
protected:
	void assignUserLocations();
	void assignHoverLocatoins();
	void findMinHopsAmongLocations();
	void assignUAVCap(int minCap, int maxCap);
	void adjustCk(float* floatCap, int maxCap, int minCap, int avg, int loop);
	void redefineCk(int* arr, int len, int minCap, int maxCap, int segements);

public:
	void calLmaxAndP();
	void calQList();
protected:
	int calG(int L, vector<int> p);
	void showLmaxAndP();
	int calQ(int h);

public:
	int Cnk(int n, int k);
	void comb(int n, int k);
protected:
	void printRes(bool* index, int n);
	bool hasDone(bool* index, int n, int k);

protected:
	void ins(int x, int y, int c);
	void bfs(int S);
	void constructEdgeMap();
	int SteinerTreeAccurate(int key[]);
	int SteinerTreeAppro(int key[]);
public:
	void calVsAndCutAccurate();
	void calVsAndCutAppro();

public:
	void prime(int** graph, int n, int** tree);
protected:
	bool allNode(int** tree, int n);

protected:
	int findMinHopInVStar(int v, int VStarBase);
	bool constrainedWithM2(int hop, int* hopLevel);

	bool constrainedWithM2Plus(int hop, int hmax, int* hopLevel);

public:
	int calMaxAssignHeapV1(int* UAVList, int len);
	int calMaxAssignHeapV2(int* UAVList, int len);
	int ShowCalMaxAssignHeapV1(int* UAVList, int len);
	int ShowCalMaxAssignHeapV2(int* UAVList, int len);
protected:
	void initializeResidualArr(int* UAVList, int len);
	void initializeHeapByInsert(int* UAVList, int len);
	void resetEdges(int UID, int LocID);

	bool visited[1000];
	int parentSPT[1 + numClusters + numHoverLocations + 1];
	int capParentSPT[1 + numClusters + numHoverLocations + 1];

public:
	int ApproAlg();

	void calP_L(int L, vector<int>& optimalP_L);

	int calQ(int h, vector<int> optimalP_L);

	void calQList(int L, int& newh_max, vector<int> optimalP_L, vector<int>& newQh);

	int ApproAlgPlus(int preStar);

	int ApproAlgD();

	int ApproAlgCombine();

	int profitEachLocation[numHoverLocations];

};

// user cluster center/UAV candinate point location+min hop in UAVs+user data rate
void maxUAVCoverage::initialization()
{
	assignUserLocations();
	assignHoverLocatoins();
	findMinHopsAmongLocations();
	assignUAVCap(minCapUAV, maxCapUAV);

	comb(numHoverLocations, s);
	//calVsAndCutAccurate();
	calVsAndCutAppro();

	int count = 0;
	for (int i = 0; i < numHoverLocations; i++)
	{
		for (int j = 0; j < numHoverLocations; j++)
			if (adjMatrix[i][j])
			{
				count++;
			}
	}
	return;
	//printInitial();
	// calAvgDataRates();
}

void maxUAVCoverage::printInitial()
{
	int i, j;
	sprintf_s(fileName, "./output/V2/%d-%d-%d-Node-Conn.txt", numUsers, numUAVs, s);
	resultFile.open(fileName, ios::app);

	resultFile << K << ',' << s << ',' << numClusters << ',' << numHoverLocations << '\n';

	resultFile << "UAV Cap" << '\n';
	for (i = 0; i < K; i++)
		resultFile << Cap[i] << '\n';

	resultFile.close();
	return;
}

// define the location of user cluster center and the number of users in each cluster
// !!!! adjust to fat-tailed distribution
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

	if (verbose)
	{
		for (i = 0; i < numClusters; ++i)
			printf("cluster %d, users: %d, x: %.0lf,\ty: %.lf\n", i, numUsersPerCluster[i], xCluster[i], yCluster[i]);
	}
}

// grid center, user cluster center,  4 points around the 3 user clusters with the most people
void maxUAVCoverage::assignHoverLocatoins()
{
	int rows = L / delta;
	int cols = W / delta;
	int i, j;
	int index = 0;
	for (i = 0; i <= rows; ++i)
	{
		for (j = 0; j <= cols; ++j)
		{
			xHoverLoc[index] = delta / 2 + i * delta;
			yHoverLoc[index] = delta / 2 + j * delta;
			index++;
		}
	}
	for (i = 0; i < numClusters; ++i, ++index)
	{
		xHoverLoc[index] = xCluster[i];
		yHoverLoc[index] = yCluster[i];
	}

	double x, y;
	for (i = 0; i < 3; ++i, index += 4)
	{
		//consider the 3 clusters with the most people
		x = xCluster[i] - 100; // the left nearby location
		if (x < 0) x = 0;
		y = yCluster[i];
		xHoverLoc[index] = x;
		yHoverLoc[index] = y;

		x = xCluster[i] + 100; // the right nearby location
		if (x > L) x = L;
		y = yCluster[i];
		xHoverLoc[index + 1] = x;
		yHoverLoc[index + 1] = y;

		x = xCluster[i];
		y = yCluster[i] - 100; // the bottom nearby location
		if (y < 0) y = 0;
		xHoverLoc[index + 2] = x;
		yHoverLoc[index + 2] = y;

		x = xCluster[i];
		y = yCluster[i] + 100; // the top nearby location
		if (y > W) y = W;
		xHoverLoc[index + 3] = x;
		yHoverLoc[index + 3] = y;
	}

	if (verbose)
	{
		for (i = 0; i < numHoverLocations; ++i)
			printf("location %d, x: %.lf\t, y: %.lf\n", i, xHoverLoc[i], yHoverLoc[i]);
	}
}

// cal UAV hops(BFS) and UAV adjacency
// 添加最短跳数路径存储
void maxUAVCoverage::findMinHopsAmongLocations()
{

	bool visited[numHoverLocations];
	int minDis[numHoverLocations];
	int i, j;

	for (i = 0; i < numClusters; ++i)
	{
		for (j = 0; j < numHoverLocations; ++j)
		{
			disMatrix[i][j] = distance2D(xCluster[i], yCluster[i], xHoverLoc[j], yHoverLoc[j]);
			if (disMatrix[i][j] <= userCommRadius) User2UAVLink[i][j] = true;
			else User2UAVLink[i][j] = false;
		}
	}

	// int maxll = 0;
	for (i = 0; i < numHoverLocations; ++i)
	{
		queue_clear();
		queue_push(i); // mininum hops from node i to other nodes, by applying BFS
		minPath[Path(i, i)].push_back(i);
		// Initialization: the non cur node is not visited and the distance is infinite
		for (j = 0; j < numHoverLocations; ++j) visited[j] = false;
		visited[i] = true;
		for (j = 0; j < numHoverLocations; ++j) minDis[j] = 99999;
		minDis[i] = 0;

		int v;
		vector<int> temp;
		while (queue_size() > 0)
		{
			v = queue_pop();
			for (j = 0; j < numHoverLocations; ++j)
			{
				if (true == visited[j]) continue;
				if (distance2D(xHoverLoc[v], yHoverLoc[v], xHoverLoc[j], yHoverLoc[j]) <= R)
				{
					if (minPath.find(Path(i, j)) == minPath.end())
					{
						temp = minPath[Path(i, v)];
						temp.push_back(j);
						minPath[Path(i, j)] = temp;
					}
					minDis[j] = minDis[v] + 1;
					visited[j] = true;
					queue_push(j);
				}
			}

		}
		for (j = 0; j < numHoverLocations; ++j)
			minHops[i][j] = minDis[j];
		/*if (verbose)
		{
			printf("minimum hops from node %d, to other nodes:----\n", i);
			for (j = 0; j < numHoverLocations; ++j)
				printf("(->%d, %d)\t", j, minHops[i][j]);
			printf("\n");
		}*/
	}

	for (i = 0; i < numHoverLocations; ++i)
		for (j = 0; j < numHoverLocations; ++j)
			adjMatrix[i][j] = false;
	for (i = 0; i < numHoverLocations; ++i)
		for (j = i + 1; j < numHoverLocations; ++j)
			if (distance2D(xHoverLoc[i], yHoverLoc[i], xHoverLoc[j], yHoverLoc[j]) <= R)
				adjMatrix[i][j] = adjMatrix[j][i] = true;
}

void maxUAVCoverage::assignUAVCap(int minCap, int maxCap)
{
	for (int i = K; i < 2 * K; i++) Cap[i] = 0;
	if (maxCap == minCap)
	{
		for (int i = 0; i < K; i++) Cap[i] = minCap;
		return;
	}

	int i;
	for (i = 0; i < K; i++)
		Cap[i] = rand() % (maxCap - minCap) + minCap;
	sort(Cap, Cap + K, greater<int>());

	float floatCap[K];
	for (i = 0; i < K; i++)
		floatCap[i] = (float)Cap[i];

	adjustCk(floatCap, maxCap, minCap, avgCap, 50);

	for (int i = 0; i < K; i++)
		Cap[i] = round(floatCap[i]);
	sort(Cap, Cap + K, greater<int>());
	// redefineCk(Cap, K, minCap, maxCap, (minCapUAV + maxCapUAV) / 10);
	redefineCk(Cap, K, minCap, maxCap, (maxCapUAV - minCapUAV) / 10);

	// cout<<"============Ck============"<<endl;
	// for (i = 0; i < K; i++) cout<<Cap[i]<<",";
	// cout<<"avg: "<<avgInt(Cap, K)<<endl;
}

void maxUAVCoverage::adjustCk(float* floatCap, int maxCap, int minCap, int avg, int loop)
{
	int i;
	int iter = 0;
	float delta = avg - avgFloat(floatCap, K);
	while (iter < loop && abs(delta) > 0.01)
	{
		iter++;
		int moreMaxIndex = -1;
		int lessMinIndex = -1;
		float sumMore = 0, sumLess = 0;
		for (i = 0; i < K; i++)
		{
			floatCap[i] = floatCap[i] + delta;
			if (floatCap[i] > maxCap)
			{
				moreMaxIndex = i;
				//sumMore = sumMore + maxCap - floatCap[i];
				sumMore = sumMore + floatCap[i] - maxCap;
				floatCap[i] = maxCap;
			}
			if (floatCap[i] < minCap)
			{
				if (lessMinIndex == -1) lessMinIndex = i;
				sumLess = sumLess + floatCap[i] - minCap;
				floatCap[i] = minCap;
			}
		}
		int res;
		float avgDelta;
		if (moreMaxIndex > -1)
		{
			res = K - moreMaxIndex - 1;
			avgDelta = sumMore / res;
			for (i = moreMaxIndex + 1; i < K; i++)
				floatCap[i] = floatCap[i] + avgDelta;
		}
		if (lessMinIndex > -1)
		{
			res = lessMinIndex;
			avgDelta = sumLess / res;
			for (i = 0; i < lessMinIndex; i++)
				floatCap[i] = floatCap[i] + avgDelta;
		}
		delta = avg - avgFloat(floatCap, K);
		sort(floatCap, floatCap + K, greater<float>());
	}
	for (i = 0; i < K; i++)
	{
		if (floatCap[i] > maxCap) floatCap[i] = maxCap;
		if (floatCap[i] < minCap) floatCap[i] = minCap;
	}
}

void maxUAVCoverage::redefineCk(int* arr, int len, int minCap, int maxCap, int segements)
{
	int delta = (maxCap - minCap) / segements;
	int* split = new int[segements + 1];
	for (int i = 0; i <= segements; i++)
		split[i] = minCap + delta * i;
	for (int i = 0; i < len; i++)
	{
		if (arr[i] == minCap || arr[i] == maxCap) continue;
		for (int j = 0; j < segements; j++)
		{
			if (arr[i] >= split[j] && arr[i] < split[j + 1])
				arr[i] = (split[j] + split[j + 1]) / 2;/* code */
		}
	}
	delete[] split;
	// seg = 5
	// 50  100  150  200  250  300
	// 50 75  125  175  225  275  300
}

// Algorithm 2
void maxUAVCoverage::calLmaxAndP()
{
	if (K < s)
	{
		Lmax = K;
		optimalP.clear();
		return;
	}
	optimalP.clear();
	vector<int> tempP, P_L;
	Lmax = s; // 2<=s<K
	int L_lb = s;
	int L_ub = K;
	int L, G, p, j, i;

	if (s == 1)
	{
		while (L_lb + 1 < L_ub)
		{
			L = floor(((double)L_lb + (double)L_ub) / 2);
			int G_L = 99999999;
			tempP.clear();
			tempP.push_back(floor((double)(L - s) / 2));
			tempP.push_back(ceil((double)(L - s) / 2));
			G = calG(L, tempP);
			if (G < G_L)
			{
				G_L = G;
				P_L = tempP;
			}
			if (G_L <= K)
			{
				L_lb = L;
				Lmax = L;
				optimalP = P_L;
			}
			else
			{
				L_ub = L;
			}
		}
		// showLmaxAndP();
		return;
	}

	while (L_lb + 1 < L_ub)
	{
		L = floor(((double)L_lb + (double)L_ub) / 2);
		int G_L = 99999999;
		for (p = 0; p <= L - s; p++)
		{
			for (j = 0; j <= s - 2; j++)
			{
				if ((s - 1) * p + j <= L - s)
				{
					tempP.clear();
					tempP.push_back(floor((double)(L - s - (s - 1) * p - j) / 2));
					for (i = 1; i < j + 1; i++) tempP.push_back(p + 1);
					for (i = j + 1; i < s; i++) tempP.push_back(p);
					tempP.push_back(ceil((double)(L - s - (s - 1) * p - j) / 2));
					G = calG(L, tempP);
					if (G < G_L)
					{
						G_L = G;
						P_L = tempP;
					}
				}
			}
		}
		if (G_L <= K)
		{
			L_lb = L;
			Lmax = L;
			optimalP = P_L;
		}
		else
		{
			L_ub = L;
		}
	}
	if (verbose) showLmaxAndP();
}

int maxUAVCoverage::calG(int L, vector<int> p)
{
	// vector p: p1, p2, ..., ps, ps+1
	int ans1 = p[0] * (p[0] + 1) / 2;
	int ans2 = p[p.size() - 1] * (p[p.size() - 1] + 1) / 2;
	int ans3 = 0;
	for (int i = 1; i < p.size() - 1; i++)
		ans3 = ans3 + ((p[i] * p[i] + 2 * p[i] + p[i] % 2) / 4);
	int ans4 = 0;
	for (int i = 1; i < p.size() - 1; i++)
		ans4 = ans4 + p[i];
	return s + ans1 + ans2 + ans3 + ans4;
}

void maxUAVCoverage::showLmaxAndP()
{
	cout << "--------------------Algorithm 2 Output--------------------" << endl;
	cout << "L: " << Lmax << ", s: " << s << endl;
	cout << "L-s: " << Lmax - s << endl;
	for (int i = 0; i < optimalP.size(); i++)
		cout << optimalP[i] << ",";
	cout << endl;
	cout << "--------------------Algorithm 2 Output End--------------------" << endl;
}

void maxUAVCoverage::calQList()
{
	if (optimalP.size() == 0)
	{
		h_max = 0;
		Qh.push_back(Lmax);
		QhArr = new int[1];
		QhArrSZ = 1;
		QhArr[0] = Lmax;
		return;
	}

	h_max = max(optimalP[0], optimalP[optimalP.size() - 1]);

	int temp;
	for (int i = 1; i < optimalP.size() - 1; i++)
	{
		temp = ceil((double)optimalP[i] / 2);
		// cout<<"h max: "<<h_max<<",temp: "<<temp<<endl;
		h_max = temp > h_max ? temp : h_max;
	}
	// cout<<"h max: "<<h_max<<",temp: "<<temp<<endl;

	Qh.push_back(Lmax);
	for (int i = 1; i <= h_max; i++)
		Qh.push_back(calQ(i));

	QhArrSZ = Qh.size();
	QhArr = new int[QhArrSZ];
	for (int i = 0; i < QhArrSZ; i++)
	{
		QhArr[i] = Qh[i];
	}

	if (verbose)
	{
		cout << "Qh:";
		for (int i = 0; i <= h_max; i++)
			cout << Qh[i] << ",";
		cout << endl;
	}
}

int maxUAVCoverage::calQ(int h)
{
	int ans1 = max(optimalP[0] - h + 1, 0);
	int ans2 = 0;
	for (int i = 1; i < optimalP.size() - 1; i++)
		ans2 = ans2 + max(optimalP[i] - 2 * h + 2, 0);
	int ans3 = max(optimalP[optimalP.size() - 1] - h + 1, 0);
	return ans1 + ans2 + ans3;
}

// 从n个里面选k个的组合
void maxUAVCoverage::comb(int n, int k)
{
	// VStar.clear();
	VStarCount = 0;
	int L = Cnk(n, k);
	VStar = new int[L * k];
	bool* index = new bool[n]();
	for (int i = 0; i < k; i++) index[i] = true;
	printRes(index, n);
	while (!hasDone(index, n, k))
	{
		for (int i = 0; i < n - 1; i++)
		{
			if (index[i] && !index[i + 1])
			{
				index[i] = false;
				index[i + 1] = true;

				int count = 0;
				for (int j = 0; j < i; j++)
				{
					if (index[j])
					{
						index[j] = false;
						index[count++] = true;
					}
				}
				printRes(index, n);
				break;
			}
		}
	}
	delete[] index;
	if (verbose)
	{
		cout << "--------------------VStar SET--------------------" << endl;
		// for (int i = 0; i < VStar.size(); i = i + k)
		// {
		// 	for (int j = 0; j < k; j++)
		// 	{
		// 		cout<<VStar[i + j]<<" ";
		// 	}
		// 	cout<<endl;
		// } 
		cout << VStarCount / k << endl;
	}

}

void maxUAVCoverage::printRes(bool* index, int n)
{
	for (int i = 0; i < n; i++)
	{
		if (index[i])
		{
			// VStar.push_back(i);
			VStar[VStarCount] = i;
			VStarCount++;
			// cout<<i<<' ';
		}
	}
	// cout<<endl;
}

bool maxUAVCoverage::hasDone(bool* index, int n, int k)
{
	for (int i = n - 1; i >= n - k; i--)
	{
		if (!index[i]) return false;
	}
	return true;
}

void maxUAVCoverage::ins(int x, int y, int c)
{
	adjEdgeLen++;
	bag[adjEdgeLen].x = x;
	bag[adjEdgeLen].y = y;
	bag[adjEdgeLen].c = c;
	bag[adjEdgeLen].g = hh[x];
	hh[x] = adjEdgeLen;
}

void maxUAVCoverage::bfs(int S)
{
	while (tou != wei)
	{
		int x = dl[tou];
		for (int i = hh[x]; i; i = bag[i].g)
		{
			int y = bag[i].y;
			if (STcost[x][S] + bag[i].c < STcost[y][S])
			{
				STcost[y][S] = STcost[x][S] + bag[i].c;
				if (!vis[y])
				{
					vis[y] = 1;
					dl[wei] = y;
					wei++;
					if (wei >= maxN)
					{
						wei = 1;
					}
				}
			}
		}
		vis[x] = 0;
		tou++;
		if (tou >= maxN)
		{
			tou = 1;
		}
	}
}

void maxUAVCoverage::constructEdgeMap()
{
	adjEdgeLen = 0;
	for (int i = 0; i < numHoverLocations; i++)
	{
		for (int j = 0; j < numHoverLocations; j++)
		{
			if (adjMatrix[i][j])
			{
				ins(i + 1, j + 1, 1);
			}
		}
	}
}

// 返回边长和
int maxUAVCoverage::SteinerTreeAccurate(int key[])
{
	for (int i = 0; i < maxN + 1; i++)
	{
		for (int j = 0; j < (1 << maxP); j++)
			STcost[i][j] = INF;
	}
	//memset(STcost, 200 / 3, sizeof(STcost));

	for (int i = 1; i <= s; i++) { //只以其中一个关键节点为树的权值为0
		STcost[key[i]][1 << (i - 1)] = 0;
	}

	const int ma = (1 << s) - 1;
	for (int S = 0; S <= ma; S++)
	{
		tou = wei = 1;
		for (int SS = (S - 1) & S; SS; SS = (SS - 1) & S)
		{
			for (int i = 1; i <= numHoverLocations; i++)
			{
				STcost[i][S] = min(STcost[i][S], STcost[i][SS] + STcost[i][S ^ SS]);
			}
		}
		for (int i = 1; i <= numHoverLocations; i++)
		{
			if (STcost[i][S] < INF)
			{
				vis[i] = 1;
				dl[wei++] = i;
			}
		}
		bfs(S);
	}
	int ans = INF;
	for (int i = 1; i <= numHoverLocations; i++)
	{
		ans = min(ans, STcost[i][ma]);
	}
	return ans;
}

// 近似比为2，返回节点数
inline int maxUAVCoverage::SteinerTreeAppro(int key[])
{
	int i, j;
	for (i = 0; i < numHoverLocations; i++) candidateKeyMore[i] = true;
	for (i = 0; i < s; i++) candidateKeyMore[key[i]] = false;

	keyGraph = new int* [s];
	for (i = 0; i < s; ++i)
		keyGraph[i] = new int[s];

	keyTree = new int* [s];
	for (i = 0; i < s; ++i)
		keyTree[i] = new int[2];

	// construct induced subgraph
	int hopsInVj;
	for (i = 0; i < s; i++)
	{
		for (j = i; j < s; j++)
		{
			hopsInVj = minHops[key[i]][key[j]];
			keyGraph[i][j] = hopsInVj;
			keyGraph[j][i] = hopsInVj;
		}
	}

	// MST and Add path nodes
	vector<int> path;
	prime(keyGraph, s, (int**)keyTree);
	int numKeyConn = s;

	for (i = 1; i < s; i++)
	{
		path = minPath[Path(key[i], key[keyTree[i][0]])];
		int pathSZ = path.size();
		for (j = 1; j < pathSZ - 1; j++)
		{
			int node = path[j];
			if (candidateKeyMore[node])
			{
				numKeyConn++;
				candidateKeyMore[node] = false;
			}
		}
	}

	for (i = 0; i < s; ++i)
	{
		delete[] keyTree[i];
		delete[] keyGraph[i];
	}
	delete[] keyTree;
	delete[] keyGraph;

	return 0;
}

void maxUAVCoverage::calVsAndCutAccurate()
{
	memset(vis, 0, sizeof(vis));
	memset(dl, 0, sizeof(dl));
	memset(hh, 0, sizeof(hh));

	int cnt = 0;
	constructEdgeMap();

	for (int i = 0; i < VStarCount; i = i + s)
	{
		int key[s + 1];
		key[0] = 0;
		for (int j = 1; j <= s; j++) key[j] = VStar[i + j - 1] + 1;
		int len = SteinerTreeAccurate(key);
		//if(len>0)cout << "len:" << len << endl;
		if (len >= K)
		{
			cnt++;
			VStar[i] = -1;
			//cout << "len:" << len << endl;
			//for (int j = 1; j <= s; j++) cout << key[j] << ',';
			//cout << endl;
		}
	}
}

void maxUAVCoverage::calVsAndCutAppro()
{
	int key[s];
	int STnum = 0;

	for (int i = 0; i < VStarCount; i = i + s)
	{
		for (int j = 0; j < s; j++)
		{
			key[j] = VStar[i + j];
		}
		STnum = SteinerTreeAppro(key);
		if (STnum > K)
		{
			VStar[i] = -1;
		}
	}
}

int maxUAVCoverage::Cnk(int n, int k)
{
	// Cn,k = Cn-1,k + Cn-1,k-1
	if (k > n) return 0;
	if (n == k || k == 0) return 1;
	if (k == 1) return n;

	int i, j;
	//int table[n][k + 1];
	int** table = NULL;
	table = new int* [n];
	for (i = 0; i < n; i++)
	{
		table[i] = new int[k + 1];
	}
	for (i = 0; i < n; i++) table[i][0] = 1;
	for (i = 1; i <= k; i++) table[i - 1][i] = 1;

	for (i = 1; i < n; i++)
	{
		for (j = 1; j <= k; j++)
		{
			table[i + j - 1][j] = table[i + j - 2][j] + table[i + j - 2][j - 1];
		}
		if (i + j - 1 == n) break;
	}

	int ans = table[n - 1][k];
	for (i = 0; i < n; i++)
	{
		delete[] table[i];
	}
	delete[] table;
	return ans;
}

int maxUAVCoverage::findMinHopInVStar(int v, int VStarBase)
{
	int minHopInVStar = 999999;
	for (int i = 0; i < s; i++)
		if (minHops[v][VStar[VStarBase + i]] < minHopInVStar)
			minHopInVStar = minHops[v][VStar[VStarBase + i]];
	return minHopInVStar;
}

bool maxUAVCoverage::constrainedWithM2(int hop, int* hopLevel)
{
	if (hop > h_max) return false;
	for (int i = 0; i <= hop; i++)
	{
		if (hopLevel[hop] >= QhArr[hop]) return false;
	}
	return true;
}

bool maxUAVCoverage::constrainedWithM2Plus(int hop, int newhmax, int* hopLevel)
{
	if (hop > newhmax) return false;
	for (int i = 0; i <= hop; i++)
	{
		if (hopLevel[hop] >= QhArrPlus[hop]) return false;
	}
	return true;
}

void maxUAVCoverage::prime(int** graph, int n, int** tree)
{
	// cout<<"==============MST================"<<endl;
	int i, j, minWeight, minU, minV;
	tree[0][0] = -1;
	tree[0][1] = 0;
	for (i = 1; i < n; i++)
	{
		tree[i][0] = -2;
		tree[i][1] = 999999;
	}

	while (!allNode((int**)tree, n))
	{
		minWeight = 99999999;
		for (i = 0; i < n; i++)
		{
			if (tree[i][0] == -2) continue; // visited nodes are remained
			//if (*((int*)tree + 2*i + 0) == -2) continue; // visited nodes are remained
			for (j = 0; j < n; j++)
			{
				//if (*((int*)tree + 2 * j + 0) == -2 && *((int*)graph + n * i + j) < minWeight)
				if (tree[j][0] == -2 && graph[i][j] < minWeight)
				{
					minWeight = graph[i][j];
					minU = i;
					minV = j;
				}
			}
		}
		//*((int*)tree + 2 * minV + 0) = minU;
		//*((int*)tree + 2 * minV + 1)  = minWeight;
		tree[minV][0] = minU;
		tree[minV][1] = minWeight;
		//cout<<"u:"<<minU<<",V:"<<minV<<",minW:"<<minWeight<<endl;
	}
	// return (int**)tree;
}

bool maxUAVCoverage::allNode(int** tree, int n)
{
	for (int i = 0; i < n; i++)
		if (tree[i][0] == -2) return false;
	//if (*((int*)tree + 2*i) == -2) return false;
	return true;
}

// 最大堆:每次pop后不更新相关边Cap，遇到再检测
inline int maxUAVCoverage::calMaxAssignHeapV1(int* UAVList, int len)
{
	if (len == 0) return 0;

	initializeResidualArr(UAVList, len);
	initializeHeapByInsert(UAVList, len);

	int preUsersServed = -1, nowUsersServed = 0;
	int UID, LocID, edgeFlow, minFlow;
	while (true)
	{
		preUsersServed = nowUsersServed;
		if (myHeap.numHeapNode == 0) return nowUsersServed;

		// 取出堆顶
		top = myHeap.popHeap();
		UID = top.userClusterID;
		LocID = top.locationID;
		edgeFlow = top.allocateCap;

		/*判断top元素是否合格，原因：
			假设User x和UAV y的一条边已经使用，
			则其剩余人数/Cap改变，但是在堆中没有变
		*/
		minFlow = min(residualClusterUser[UID], residualLocCap[LocID]);
		if (edgeFlow != minFlow)
		{
			top.allocateCap = minFlow;
			myHeap.insertHeap(top);
			continue;
		}

		//更新服务用户数
		nowUsersServed = nowUsersServed + edgeFlow;
		residualClusterUser[UID] = residualClusterUser[UID] - edgeFlow;
		residualLocCap[LocID] = residualLocCap[LocID] - edgeFlow;
		if (preUsersServed == nowUsersServed)
		{
			return nowUsersServed;
		}
	}

	return 0;
}

// 最大堆:每次pop后更新相关边Cap，重建堆
inline int maxUAVCoverage::calMaxAssignHeapV2(int* UAVList, int len)
{
	if (len == 0) return 0;

	initializeResidualArr(UAVList, len);
	initializeHeapByInsert(UAVList, len);

	if (len == 1) return myHeap.popHeap().allocateCap;

	int preUsersServed = -1, nowUsersServed = 0;
	int UID, LocID, edgeFlow, minFlow;
	while (true)
	{
		preUsersServed = nowUsersServed;
		if (myHeap.numHeapNode == 0) return nowUsersServed;

		// 取出堆顶
		top = myHeap.popHeap();
		UID = top.userClusterID;
		LocID = top.locationID;
		edgeFlow = top.allocateCap;

		//更新服务用户数
		nowUsersServed = nowUsersServed + edgeFlow;
		residualClusterUser[UID] = residualClusterUser[UID] - edgeFlow;
		residualLocCap[LocID] = residualLocCap[LocID] - edgeFlow;

		// 修改堆内相关边Cap,再重建堆
		//minFlow = min(residualClusterUser[UID], residualUAVCap[LocID]);
		resetEdges(UID, LocID);

		if (preUsersServed == nowUsersServed)
		{
			return nowUsersServed;
		}
	}

	return 0;
}

inline int maxUAVCoverage::ShowCalMaxAssignHeapV1(int* UAVList, int len)
{
	if (len == 0) return 0;

	initializeResidualArr(UAVList, len);
	initializeHeapByInsert(UAVList, len);

	int preUsersServed = -1, nowUsersServed = 0;
	int UID, LocID, edgeFlow, minFlow;
	sprintf_s(fileName, "./output/test/flow-n-%d-m-%d-s-%d-delta%d.txt", numUsers, numHoverLocations, s, delta);
	resultFile.open(fileName, ios::app);
	resultFile << "user:" << endl;
	for (int i = 0; i < numClusters; i++)
		resultFile << residualClusterUser[i] << ',';
	resultFile << endl;
	resultFile << "uav loc:" << endl;
	for (int i = 0; i < len; i++)
		resultFile << UAVList[i] << ',';
	resultFile << endl;
	resultFile << "uav cap:" << endl;
	for (int i = 0; i < len; i++)
		resultFile << Cap[i] << ',';
	resultFile << endl;

	while (true)
	{
		preUsersServed = nowUsersServed;
		if (myHeap.numHeapNode == 0) return nowUsersServed;

		// 取出堆顶
		top = myHeap.popHeap();
		UID = top.userClusterID;
		LocID = top.locationID;
		edgeFlow = top.allocateCap;

		/*判断top元素是否合格，原因：
			假设User x和UAV y的一条边已经使用，
			则其剩余人数/Cap改变，但是在堆中没有变
		*/
		minFlow = min(residualClusterUser[UID], residualLocCap[LocID]);
		if (edgeFlow != minFlow)
		{
			top.allocateCap = minFlow;
			myHeap.insertHeap(top);
			continue;
		}

		//更新服务用户数
		nowUsersServed = nowUsersServed + edgeFlow;
		residualClusterUser[UID] = residualClusterUser[UID] - edgeFlow;
		residualLocCap[LocID] = residualLocCap[LocID] - edgeFlow;
		resultFile << UID << ',' << LocID << ',' << edgeFlow << endl;

		if (preUsersServed == nowUsersServed)
		{
			resultFile.close();
			return nowUsersServed;
		}
	}

	return 0;
}

inline int maxUAVCoverage::ShowCalMaxAssignHeapV2(int* UAVList, int len)
{
	if (len == 0) return 0;

	initializeResidualArr(UAVList, len);
	initializeHeapByInsert(UAVList, len);

	if (len == 1) return myHeap.popHeap().allocateCap;

	int preUsersServed = -1, nowUsersServed = 0;
	int UID, LocID, edgeFlow, minFlow;

	sprintf_s(fileName, "./output/test/flow-n-%d-m-%d-s-%d-delta%d.txt", numUsers, numHoverLocations, s, delta);
	resultFile.open(fileName, ios::app);
	resultFile << "user:" << endl;
	for (int i = 0; i < numClusters; i++)
		resultFile << residualClusterUser[i] << ',';
	resultFile << endl;
	resultFile << "uav loc:" << endl;
	for (int i = 0; i < len; i++)
		resultFile << UAVList[i] << ',';
	resultFile << endl;
	resultFile << "uav cap:" << endl;
	for (int i = 0; i < len; i++)
		resultFile << Cap[i] << ',';
	resultFile << endl;

	while (true)
	{
		preUsersServed = nowUsersServed;
		if (myHeap.numHeapNode == 0)
		{
			resultFile.close();
			return nowUsersServed;
		}

		// 取出堆顶
		top = myHeap.popHeap();
		UID = top.userClusterID;
		LocID = top.locationID;
		edgeFlow = top.allocateCap;
		resultFile << UID << ',' << LocID << ',' << edgeFlow << endl;

		//更新服务用户数
		nowUsersServed = nowUsersServed + edgeFlow;
		residualClusterUser[UID] = residualClusterUser[UID] - edgeFlow;
		residualLocCap[LocID] = residualLocCap[LocID] - edgeFlow;

		// 修改堆内相关边Cap,再重建堆
		//minFlow = min(residualClusterUser[UID], residualUAVCap[LocID]);
		resetEdges(UID, LocID);

		if (preUsersServed == nowUsersServed)
		{
			resultFile.close();
			return nowUsersServed;
		}
	}

	return 0;
}

//初始化剩余用户和UAV cap
inline void maxUAVCoverage::initializeResidualArr(int* UAVList, int len)
{
	for (int i = 0; i < numClusters; i++)
	{
		residualClusterUser[i] = numUsersPerCluster[i];
	}
	for (int i = 0; i < len; i++)
	{
		residualLocCap[UAVList[i]] = Cap[i];
	}
	return;
}

//插入User和UAV相连边
inline void maxUAVCoverage::initializeHeapByInsert(int* UAVList, int len)
{
	myHeap.numHeapNode = 0;
	myHeap.sonNum = numOfSon;

	int temp;
	for (int i = 0; i < numClusters; i++)
	{
		for (int j = 0; j < len; j++)
		{
			temp = UAVList[j];
			if (User2UAVLink[i][temp])
			{
				struct Edge e = { i,temp,min(residualClusterUser[i],residualLocCap[temp]) };
				myHeap.insertHeap(e);
			}
		}
	}
	return;
}

// 修改堆内相关边Cap,再重建堆
inline void maxUAVCoverage::resetEdges(int UID, int LocID)
{
	int newCap;
	struct Edge e;
	for (int i = 0; i < myHeap.numHeapNode; i++)
	{
		e = myHeap.storeHeapNode[i];
		if (e.userClusterID == UID)
		{
			newCap = min(residualClusterUser[UID], residualLocCap[e.locationID]);
			myHeap.storeHeapNode[i].allocateCap = newCap;
			continue;
		}
		else if (e.locationID == LocID)
		{
			newCap = min(residualClusterUser[e.userClusterID], residualLocCap[LocID]);
			myHeap.storeHeapNode[i].allocateCap = newCap;
			continue;
		}
	}

	myHeap.recreateMaxHeap();

	//删除队尾空节点
	int sizeHeap = myHeap.numHeapNode;
	for (int i = sizeHeap - 1; i >= 0; i--)
	{
		if (myHeap.storeHeapNode[i].allocateCap == 0)
		{
			myHeap.numHeapNode--;
			if (myHeap.numHeapNode == 0) return;
		}
		else return;
	}
}

int maxUAVCoverage::ApproAlg()
{
	int i, j, k;
	int mostUserServed = 0;

	calLmaxAndP();
	calQList();
	//comb(numHoverLocations, s);
	// vector<int> Vj;
	int Vj[K];
	int VjCount = 0;
	for (int aa = 0; aa < K; aa++) Vj[aa] = -1;
	bool candidateV[numHoverLocations];
	//int hopLevel[h_max + 1];
	int* hopLevel = new int[h_max + 1];
	int hopOneLoc2Vstar[numHoverLocations];
	// vector<int> tryV;
	bool tryV[numHoverLocations];
	int nStar = 0;
	// vector<int> VjStar;
	int VjStar[K];
	int VjStarCount = 0;
	for (int aa = 0; aa < K; aa++) VjStar[aa] = -1;
	//int minHopsWithVj[Lmax][Lmax];
	//int tree[Lmax][2];
	int** minHopsWithVj = NULL;
	minHopsWithVj = new int* [Lmax];
	for (i = 0; i < Lmax; ++i)
		minHopsWithVj[i] = new int[Lmax];

	int** tree = NULL;
	tree = new int* [Lmax];
	for (i = 0; i < Lmax; ++i)
		tree[i] = new int[2];

	//cout << "Lmax:" << Lmax << endl;
	for (i = 0; i < VStarCount; i = i + s)
	{
		// if ((i/s)>50000) break;

		 //if ((i/s)%1000==0) 
			 //cout<<"iter:"<<i/s<<endl;

		// Vj.clear();
		if (VStar[i] == -1) continue;
		VjCount = 0;
		for (j = 0; j < numHoverLocations; j++) candidateV[j] = true;
		for (j = 0; j <= h_max; j++) hopLevel[j] = 0;
		int hops;
		int maxUserInVj;
		int* tempUAV = new int[Lmax];
		for (j = 0; j < numHoverLocations; j++)
			hopOneLoc2Vstar[j] = findMinHopInVStar(j, i);

		// select Vj
		for (j = 0; j < Lmax; j++)
		{
			if (verbose) cout << "========iter:" << i << "," << j << "=========" << endl;
			// tryV.clear();
			for (k = 0; k < numHoverLocations; k++) tryV[k] = false;
			for (k = 0; k < numHoverLocations; k++)
			{
				if (candidateV[k])
				{
					if (verbose) cout << "k:" << k << ",";
					hops = hopOneLoc2Vstar[k];
					if (constrainedWithM2(hops, hopLevel))
					{
						// tryV.push_back(k);
						tryV[k] = true;
					}
					if (verbose) cout << "hops:" << hops << ",M2:" << constrainedWithM2(hops, hopLevel) << endl;
				}
			}

			for (k = 0; k < j; k++) tempUAV[k] = Vj[k];
			maxUserInVj = -1;
			int maxNode = -1;

			int tryVCount = 0;
			for (k = 0; k < numHoverLocations; k++)
			{
				if (tryV[k])
				{
					tempUAV[j] = k;
					tryVCount++;
					int n = calMaxAssignHeapV1(tempUAV, j + 1);
					if (n > maxUserInVj)
					{
						maxUserInVj = n;
						maxNode = k;
					}
				}
			}

			// if (tryV.size() == 0) break;
			if (tryVCount == 0) break;

			assert(maxNode != -1);
			// Vj.push_back(maxNode);
			Vj[VjCount] = maxNode;
			VjCount++;
			candidateV[maxNode] = false;
			// hops = findMinHopInVStar(maxNode, i);
			hops = hopOneLoc2Vstar[maxNode];
			for (k = 0; k <= hops; k++)
			{
				hopLevel[k]++;
			}
		}

		if (verbose)
		{
			cout << "Vj:" << maxUserInVj << endl;
			// for (k = 0; k < Vj.size(); k++)
			for (k = 0; k < VjCount; k++)
			{
				cout << Vj[k] << ",";
			}
			cout << endl;
		}

		// construct induced subgraph
		int hopsInVj;
		for (j = 0; j < VjCount; j++)
		{
			for (k = j; k < VjCount; k++)
			{
				hopsInVj = minHops[Vj[j]][Vj[k]];
				minHopsWithVj[j][k] = hopsInVj;
				minHopsWithVj[k][j] = hopsInVj;
			}
		}

		// MST and Add path nodes
		vector<int> path;
		prime((int**)minHopsWithVj, VjCount, (int**)tree);
		int numUAVConn = VjCount;
		int preVjCount = VjCount;

		for (j = 1; j < preVjCount; j++)
		{
			path = minPath[Path(Vj[j], Vj[tree[j][0]])];
			int pathSZ = path.size();
			for (k = 1; k < pathSZ - 1; k++)
			{
				int node = path[k];
				if (candidateV[node])
				{
					numUAVConn++;
					candidateV[node] = false;
					// Vj.push_back(path[k]);
					if (numUAVConn <= K)
					{
						Vj[VjCount] = node;
						VjCount++;
					}
				}
			}
		}
		//if (numUAVConn > K) VjCount = preVjCount;
		if (numUAVConn > K) continue;

		int sizeVj = VjCount;
		if (numUAVConn <= K)
		{
			int numUserInVj = calMaxAssignHeapV1(Vj, sizeVj);
			int addFlow = numUserInVj;
			int preAddFlow = 0;
			int addV;
			int tempFlow;
			bool flagAdd = false;

			bool connectedArr[numHoverLocations];
			for (j = 0; j < numHoverLocations; j++) connectedArr[j] = false;
			for (j = 0; j < numHoverLocations; j++)
			{
				for (k = 0; k < VjCount; k++)
				{
					if (adjMatrix[j][Vj[k]])
					{
						connectedArr[j] = true;
						break;
					}
				}
			}

			while (sizeVj < K)
			{
				flagAdd = false;
				for (j = 0; j < numHoverLocations; j++)
				{
					if (!candidateV[j]) continue;

					if (!connectedArr[j] && adjMatrix[j][Vj[sizeVj - 1]])
						connectedArr[j] = true;
					if (connectedArr[j])
					{
						Vj[sizeVj] = j;
						tempFlow = calMaxAssignHeapV1(Vj, sizeVj + 1);
						if (tempFlow > addFlow)
						{
							flagAdd = true;
							addFlow = tempFlow;
							addV = j;
						}
					}
				}
				if (addFlow == preAddFlow) break;
				if (!flagAdd) break;
				Vj[sizeVj] = addV;
				sizeVj++;
				VjCount++;
				candidateV[addV] = false;
				preAddFlow = addFlow;
			}

			if (addFlow > nStar)
			{
				nStar = addFlow;
				VjStarCount = VjCount;
				for (j = 0; j < VjStarCount; j++)
					VjStar[j] = Vj[j];

				if (verbose)
				{
					cout << nStar << ":";
					for (int ii = 0; ii < VjStarCount; ii++)
						cout << VjStar[ii] << ",";
					cout << endl;
				}
			}
		}
	}

	for (i = 0; i < Lmax; ++i)
	{
		delete[] tree[i];
		delete[] minHopsWithVj[i];
	}
	delete[] tree;
	delete[] minHopsWithVj;

	/*
	int finalUAV[K];
	for (i = 0; i < numHoverLocations; i++) candidateV[i] = true;
	for (i = 0; i < VjStarCount; i++)
	{
		finalUAV[i] = VjStar[i];
		candidateV[VjStar[i]] = false;
	}
	int finalUAVIndex = i;

	ShowCalMaxAssignHeapV1(finalUAV, VjStarCount);*/

	//cout << "not add adj:" << endl;
	//for (int aa = 0; aa < finalUAVIndex; aa++)
	//{
	//	for (int bb = 0; bb < finalUAVIndex; bb++)
	//	{
	//		cout << adjMatrix[finalUAV[aa]][finalUAV[bb]] << ',';
	//	}
	//	cout << endl;
	//}


	//sprintf_s(fileName, "./output_plus/node/%d-%d-%d-Node-Conn.txt", numUsers, numUAVs, s);
	//resultFile.open(fileName, ios::app);
	//resultFile << "finalUAVLoc:ApproAlg" << '\n';
	//for (int ii = 0; ii < finalUAVIndex; ii++)
	//	resultFile << finalUAV[ii] << ',';
	//resultFile << '\n';
	//for (int ii = 0; ii < finalUAVIndex; ii++)
	//{
	//	for (int jj = 0; jj < finalUAVIndex; jj++)
	//	{
	//		resultFile << adjMatrix[finalUAV[ii]][finalUAV[jj]] << '\t';
	//	}
	//	resultFile << '\n';
	//}
	//resultFile.close();

	/*
	calMaxAssignHeapV1(finalUAV, finalUAVIndex);// print

	resultFile << "UAV -> t" << '\n';
	for (int ii = numClusters + 1; ii <= numClusters + finalUAVIndex; ii++)
	{
		flowGraphEdge* p = &flowGraph.access(ii, 0);
		for (int jj = 0; jj < sizeEachRowFlowGraph[ii]; ++jj)
		{
			//if (p[jj].flow == 0) continue;
			resultFile << ii << '\t' << p[jj].v << '\t' << p[jj].flow << '\n';
		}
	}

	//resultFile << "flow graph: User -> UAV" << '\n';
	////int sz = sizeEachRowFlowGraph.size();
	//for (int ii = 1; ii <= numClusters; ++ii)
	//{
	//	flowGraphEdge* p = &flowGraph.access(ii, 0);
	//	for (int jj = 0; jj < sizeEachRowFlowGraph[ii]; ++jj)
	//	{
	//		if (p[jj].flow == 0) continue;
	//		resultFile << ii << ',' << p[jj].v << ',' << p[jj].flow << '\n';
	//	}
	//}

	resultFile.close();*/

	return nStar;
}

void maxUAVCoverage::calP_L(int fixL, vector<int>& optimalP_L)
{
	optimalP.clear();
	vector<int> tempP, P_L;
	Lmax = s; // 2<=s<K
	int L_lb = s;
	int L_ub = K;
	int G, p, j, i;

	if (s == 1)
	{
		int G_L = 99999999;
		tempP.clear();
		tempP.push_back(floor((double)(fixL - s) / 2));
		tempP.push_back(ceil((double)(fixL - s) / 2));
		G = calG(fixL, tempP);
		if (G < G_L)
		{
			G_L = G;
			P_L = tempP;
		}
		optimalP_L = P_L;
		return;
	}

	int G_L = 99999999;
	for (p = 0; p <= fixL - s; p++)
	{
		for (j = 0; j <= s - 2; j++)
		{
			if ((s - 1) * p + j <= fixL - s)
			{
				tempP.clear();
				tempP.push_back(floor((double)(fixL - s - (s - 1) * p - j) / 2));
				for (i = 1; i < j + 1; i++) tempP.push_back(p + 1);
				for (i = j + 1; i < s; i++) tempP.push_back(p);
				tempP.push_back(ceil((double)(fixL - s - (s - 1) * p - j) / 2));
				G = calG(fixL, tempP);
				if (G < G_L)
				{
					G_L = G;
					P_L = tempP;
				}
			}
		}
	}
	optimalP_L = P_L;
	return;
}

int maxUAVCoverage::calQ(int h, vector<int> optimalP_L)
{
	int ans1 = max(optimalP_L[0] - h + 1, 0);
	int ans2 = 0;
	for (int i = 1; i < optimalP_L.size() - 1; i++)
		ans2 = ans2 + max(optimalP_L[i] - 2 * h + 2, 0);
	int ans3 = max(optimalP_L[optimalP_L.size() - 1] - h + 1, 0);
	return ans1 + ans2 + ans3;
}

void maxUAVCoverage::calQList(int L, int& newh_max, vector<int> optimalP_L, vector<int>& newQh)
{
	newQh.clear();
	if (optimalP_L.size() == 0)
	{
		newh_max = 0;
		newQh.push_back(L);
		QhArrPlus = new int[1];
		QhArrPlusSZ = 1;
		QhArrPlus[0] = L;
		return;
	}

	newh_max = max(optimalP_L[0], optimalP_L[optimalP_L.size() - 1]);

	int temp;
	for (int i = 1; i < optimalP_L.size() - 1; i++)
	{
		temp = ceil((double)optimalP_L[i] / 2);
		// cout<<"h max: "<<newh_max<<",temp: "<<temp<<endl;
		newh_max = temp > newh_max ? temp : newh_max;
	}
	// cout<<"h max: "<<newh_max<<",temp: "<<temp<<endl;

	newQh.push_back(L);
	for (int i = 1; i <= newh_max; i++)
		newQh.push_back(calQ(i, optimalP_L));

	QhArrPlusSZ = newQh.size();
	QhArrPlus = new int[QhArrPlusSZ];
	for (int i = 0; i < QhArrPlusSZ; i++)
	{
		QhArrPlus[i] = newQh[i];
	}

	if (verbose)
	{
		cout << "newQh:";
		for (int i = 0; i <= newh_max; i++)
			cout << newQh[i] << ",";
		cout << endl;
	}
}

int maxUAVCoverage::ApproAlgPlus(int preStar)
{
	bool candidateV[numHoverLocations];
	int VjStar[K * 2];
	int Vj[100];
	int VjStarCount = 0;
	int VjCount = 0;
	int nStar = 0;
	int L_low = Lmax;
	int L_up = K + 1;
	int L_plus;
	L_best = 0;
	int finalUAV[K];
	int finalUAVCount = 0;
	int hopLevel[K + 1];
	vector<int> newP_L, newQh;
	int newh_max;
	while (L_low + 1 < L_up)
	{
		QhArrPlus = NULL;
		QhArrPlusSZ = 0;
		L_plus = floor(((double)L_low + (double)L_up) / 2);
		//cout << "L_low:" << L_low << ",L_up:" << L_up << ",L_plu:" << L_plus << endl;

		newP_L.clear();
		newQh.clear();
		calP_L(L_plus, newP_L);
		calQList(L_plus, newh_max, newP_L, newQh);

		int i, j, k;
		int mostUserServed = 0;

		VjCount = 0;
		for (int aa = 0; aa < 2 * K; aa++) Vj[aa] = -1;

		int hopOneLoc2Vstar[numHoverLocations];
		bool tryV[numHoverLocations];
		nStar = 0;
		VjStarCount = 0;
		for (int aa = 0; aa < 2 * K; aa++) VjStar[aa] = -1;

		int** minHopsWithVj = NULL;
		minHopsWithVj = new int* [L_plus];
		for (i = 0; i < L_plus; ++i)
			minHopsWithVj[i] = new int[L_plus];

		int** tree = NULL;
		tree = new int* [L_plus];
		for (i = 0; i < L_plus; ++i)
			tree[i] = new int[2];

		for (i = 0; i < VStarCount; i = i + s)
		{
			if (VStar[i] == -1) continue;

			VjCount = 0;
			for (j = 0; j < numHoverLocations; j++) candidateV[j] = true;
			for (j = 0; j <= newh_max; j++) hopLevel[j] = 0;
			int hops;
			int maxUserInVj;
			int* tempUAV = new int[L_plus];
			for (j = 0; j < numHoverLocations; j++)
				hopOneLoc2Vstar[j] = findMinHopInVStar(j, i);

			// select Vj
			for (j = 0; j < L_plus; j++)
			{
				if (verbose) cout << "========iter:" << i << "," << j << "=========" << endl;
				// tryV.clear();
				for (k = 0; k < numHoverLocations; k++) tryV[k] = false;
				for (k = 0; k < numHoverLocations; k++)
				{
					if (candidateV[k])
					{
						if (verbose) cout << "k:" << k << ",";
						hops = hopOneLoc2Vstar[k];
						if (constrainedWithM2Plus(hops, newh_max, hopLevel))
						{
							tryV[k] = true;
						}
						if (verbose) cout << "hops:" << hops << ",M2:" << constrainedWithM2Plus(hops, newh_max, hopLevel) << endl;
					}
				}

				for (k = 0; k < j; k++) tempUAV[k] = Vj[k];
				maxUserInVj = -1;
				int maxNode = -1;

				int tryVCount = 0;
				for (k = 0; k < numHoverLocations; k++)
				{
					if (tryV[k])
					{
						tempUAV[j] = k;
						tryVCount++;
						int n = calMaxAssignHeapV1(tempUAV, j + 1);
						if (n > maxUserInVj)
						{
							maxUserInVj = n;
							maxNode = k;
						}
					}
				}

				if (tryVCount == 0) break;

				assert(maxNode != -1);
				Vj[VjCount] = maxNode;
				VjCount++;
				candidateV[maxNode] = false;
				hops = hopOneLoc2Vstar[maxNode];
				for (k = 0; k <= hops; k++)
				{
					hopLevel[k]++;
				}
			}

			if (verbose)
			{
				cout << "Vj:" << maxUserInVj << endl;
				for (k = 0; k < VjCount; k++)
				{
					cout << Vj[k] << ",";
				}
				cout << endl;
			}

			// construct induced subgraph
			int hopsInVj;
			for (j = 0; j < VjCount; j++)
			{
				for (k = j; k < VjCount; k++)
				{
					hopsInVj = minHops[Vj[j]][Vj[k]];
					minHopsWithVj[j][k] = hopsInVj;
					minHopsWithVj[k][j] = hopsInVj;
				}
			}

			// MST and Add path nodes
			vector<int> path;
			prime((int**)minHopsWithVj, VjCount, (int**)tree);
			int numUAVConn = VjCount;
			int preVjCount = VjCount;

			for (j = 1; j < preVjCount; j++)
			{
				path = minPath[Path(Vj[j], Vj[tree[j][0]])];
				int pathSZ = path.size();
				for (k = 1; k < pathSZ - 1; k++)
				{
					int node = path[k];
					if (candidateV[node])
					{
						numUAVConn++;
						candidateV[node] = false;
						Vj[VjCount] = node;
						VjCount++;
					}
				}
			}

			int sizeVj = VjCount;
			//if (numUAVConn <= K)
			{
				int numUserInVj = calMaxAssignHeapV1(Vj, sizeVj);

				int addFlow = numUserInVj;
				int preAddFlow = 0;
				int addV;
				int tempFlow;
				bool flagAdd = false;

				bool connectedArr[numHoverLocations];
				for (j = 0; j < numHoverLocations; j++) connectedArr[j] = false;
				for (j = 0; j < numHoverLocations; j++)
				{
					for (k = 0; k < VjCount; k++)
					{
						if (adjMatrix[j][Vj[k]])
						{
							connectedArr[j] = true;
							break;
						}
					}
				}

				while (sizeVj < K)
				{
					flagAdd = false;
					for (j = 0; j < numHoverLocations; j++)
					{
						if (!candidateV[j]) continue;

						if (!connectedArr[j] && adjMatrix[j][Vj[sizeVj - 1]])
							connectedArr[j] = true;
						if (connectedArr[j])
						{
							Vj[sizeVj] = j;
							tempFlow = calMaxAssignHeapV1(Vj, sizeVj + 1);
							if (tempFlow > addFlow)
							{
								flagAdd = true;
								addFlow = tempFlow;
								addV = j;
							}
						}
					}

					if (addFlow == preAddFlow) break;
					if (!flagAdd) break;
					Vj[sizeVj] = addV;
					sizeVj++;
					VjCount++;
					candidateV[addV] = false;
					preAddFlow = addFlow;
				}

				if (addFlow > nStar)
				{
					nStar = addFlow;
					VjStarCount = VjCount;
					for (j = 0; j < VjStarCount; j++)
						VjStar[j] = Vj[j];

					if (verbose)
					{
						cout << nStar << ":";
						for (int ii = 0; ii < VjStarCount; ii++)
							cout << VjStar[ii] << ",";
						cout << endl;
					}
				}
			}
		}

		for (i = 0; i < L_plus; ++i)
		{
			delete[] tree[i];
			delete[] minHopsWithVj[i];
		}
		delete[] tree;
		delete[] minHopsWithVj;

		if (VjStarCount <= K)
		{
			L_low = L_plus;

			if (nStar > preStar)
			{
				finalUAVCount = VjStarCount;
				for (int ii = 0; ii < VjStarCount; ii++)
					finalUAV[ii] = VjStar[ii];

				preStar = nStar;
				L_best = L_plus;
			}
		}
		else
		{
			L_up = L_plus;
		}
	}
	return preStar;
}

int maxUAVCoverage::ApproAlgD()
{
	if (K < 3) return 0;

	int i, j, k;
	int mostUserServed = 0;

	QhArrSZ = 3;
	QhArr = new int[3];
	Lmax = K > 5 ? 5 : K;
	h_max = 2;
	QhArr[0] = ceil((double)(K - 1) / 2) + 1;
	QhArr[1] = ceil((double)(K - 1) / 2);
	QhArr[2] = ceil((double)(K - 1) / 2) - 1;

	// vector<int> Vj;
	int Vj[K];
	int VjCount = 0;
	for (int aa = 0; aa < K; aa++) Vj[aa] = -1;

	bool candidateV[numHoverLocations];
	//int hopLevel[h_max + 1];
	int* hopLevel = new int[h_max + 1];
	int hopOneLoc2Vstar[numHoverLocations];
	// vector<int> tryV;
	bool tryV[numHoverLocations];
	int nStar = 0;
	// vector<int> VjStar;
	int VjStar[K];
	int VjStarCount = 0;
	for (int aa = 0; aa < K; aa++) VjStar[aa] = -1;

	int** minHopsWithVj = NULL;
	minHopsWithVj = new int* [Lmax];
	for (i = 0; i < Lmax; ++i)
		minHopsWithVj[i] = new int[Lmax];

	int** tree = NULL;
	tree = new int* [Lmax];
	for (i = 0; i < Lmax; ++i)
		tree[i] = new int[2];

	for (i = 0; i < numHoverLocations; i++)
	{
		VjCount = 0;
		for (j = 0; j < numHoverLocations; j++) candidateV[j] = true;
		for (j = 0; j <= h_max; j++) hopLevel[j] = 0;
		int hops;
		int maxUserInVj;
		int* tempUAV = new int[Lmax];
		for (j = 0; j < numHoverLocations; j++)
			hopOneLoc2Vstar[j] = minHops[j][i];

		// select Vj
		for (j = 0; j < Lmax; j++)
		{
			if (verbose) cout << "========iter:" << i << "," << j << "=========" << endl;
			// tryV.clear();
			for (k = 0; k < numHoverLocations; k++) tryV[k] = false;
			for (k = 0; k < numHoverLocations; k++)
			{
				if (candidateV[k])
				{
					if (verbose) cout << "k:" << k << ",";
					hops = hopOneLoc2Vstar[k];
					if (constrainedWithM2(hops, hopLevel))
					{
						// tryV.push_back(k);
						tryV[k] = true;
					}
					if (verbose) cout << "hops:" << hops << ",M2:" << constrainedWithM2(hops, hopLevel) << endl;
				}
			}

			for (k = 0; k < j; k++) tempUAV[k] = Vj[k];
			maxUserInVj = -1;
			int maxNode = -1;
			int tryVCount = 0;
			for (k = 0; k < numHoverLocations; k++)
			{
				if (tryV[k])
				{
					tempUAV[j] = k;
					tryVCount++;
					int n = calMaxAssignHeapV1(tempUAV, j + 1);
					if (n > maxUserInVj)
					{
						maxUserInVj = n;
						maxNode = k;
					}
				}
			}

			// if (tryV.size() == 0) break;
			if (tryVCount == 0) break;

			assert(maxNode != -1);
			// Vj.push_back(maxNode);
			Vj[VjCount] = maxNode;
			VjCount++;
			candidateV[maxNode] = false;
			// hops = findMinHopInVStar(maxNode, i);
			hops = hopOneLoc2Vstar[maxNode];
			for (k = 0; k <= hops; k++)
			{
				hopLevel[k]++;
			}
		}

		if (verbose)
		{
			cout << "Vj:" << maxUserInVj << endl;
			for (k = 0; k < VjCount; k++)
			{
				cout << Vj[k] << ",";
			}
			cout << endl;
		}

		// construct induced subgraph
		int hopsInVj;
		for (j = 0; j < VjCount; j++)
		{
			for (k = j; k < VjCount; k++)
			{
				hopsInVj = minHops[Vj[j]][Vj[k]];
				minHopsWithVj[j][k] = hopsInVj;
				minHopsWithVj[k][j] = hopsInVj;
			}
		}

		// MST and Add path nodes
		vector<int> path;
		prime((int**)minHopsWithVj, Lmax, (int**)tree);
		int numUAVConn = VjCount;
		int preVjCount = VjCount;

		for (j = 1; j < preVjCount; j++)
		{
			path = minPath[Path(Vj[j], Vj[tree[j][0]])];
			int pathSZ = path.size();
			for (k = 1; k < pathSZ - 1; k++)
			{
				int node = path[k];
				if (candidateV[node])
				{
					numUAVConn++;
					candidateV[node] = false;
					// Vj.push_back(path[k]);
					if (numUAVConn <= K)
					{
						Vj[VjCount] = node;
						VjCount++;
					}
				}
			}
		}
		if (numUAVConn > K) continue;
		int sizeVj = VjCount;

		if (numUAVConn <= K)
		{
			int numUserInVj = calMaxAssignHeapV1(Vj, sizeVj);

			int addFlow = numUserInVj;
			int preAddFlow = 0;
			int addV;
			int tempFlow;
			bool flagAdd = false;

			bool connectedArr[numHoverLocations];
			for (j = 0; j < numHoverLocations; j++) connectedArr[j] = false;
			for (j = 0; j < numHoverLocations; j++)
			{
				for (k = 0; k < VjCount; k++)
				{
					if (adjMatrix[j][Vj[k]])
					{
						connectedArr[j] = true;
						break;
					}
				}
			}

			while (sizeVj < K)
			{
				flagAdd = false;
				for (j = 0; j < numHoverLocations; j++)
				{
					if (!candidateV[j]) continue;

					if (!connectedArr[j] && adjMatrix[j][Vj[sizeVj - 1]])
						connectedArr[j] = true;
					if (connectedArr[j])
					{
						Vj[sizeVj] = j;
						tempFlow = calMaxAssignHeapV1(Vj, sizeVj + 1);
						if (tempFlow > addFlow)
						{
							flagAdd = true;
							addFlow = tempFlow;
							addV = j;
						}
					}
				}

				if (addFlow == preAddFlow) break;
				if (!flagAdd) break;
				Vj[sizeVj] = addV;
				sizeVj++;
				VjCount++;
				candidateV[addV] = false;
				preAddFlow = addFlow;
			}

			if (addFlow > nStar)
			{
				nStar = addFlow;
				VjStarCount = VjCount;
				for (j = 0; j < VjStarCount; j++)
					VjStar[j] = Vj[j];

				if (verbose)
				{
					cout << nStar << ":";
					for (int ii = 0; ii < VjStarCount; ii++)
						cout << VjStar[ii] << ",";
					cout << endl;
				}
			}
		}
	}

	for (i = 0; i < Lmax; ++i)
	{
		delete[] tree[i];
		delete[] minHopsWithVj[i];
	}
	delete[] tree;
	delete[] minHopsWithVj;


	int finalUAV[K];
	for (i = 0; i < numHoverLocations; i++) candidateV[i] = true;
	for (i = 0; i < VjStarCount; i++)
	{
		finalUAV[i] = VjStar[i];
		candidateV[VjStar[i]] = false;
	}
	int finalUAVIndex = i;

	//ShowCalMaxAssignHeapV2(finalUAV, VjStarCount);

	return nStar;
}

int maxUAVCoverage::ApproAlgCombine()
{
	clock_t st1, ft1, st2, ft2, st3, ft3;
	double timeAppro = 0;
	double timeApproD = 0;
	double timeApproPlus = 0;

	st1 = clock();
	int numApproAlg = ApproAlg();
	ft1 = clock();
	//cout << "Appro end" << endl;
	int preLmax = Lmax;

	st3 = clock();
	int numApproAlgPlus = ApproAlgPlus(numApproAlg);
	ft3 = clock();
	//cout << "ApproP end" << endl;

	st2 = clock();
	int numApproAlgD = ApproAlgD();
	ft2 = clock();
	//cout << "ApproD end" << endl;

	timeAppro = (ft1 - st1) / (1000.0);
	timeApproD = (ft2 - st2) / (1000.0);
	timeApproPlus = (ft3 - st3) / (1000.0);

	sprintf_s(fileName, "./output/test0/time-n-%d-m-%d-s-%d-delta%d.txt", numUsers, numHoverLocations, s, delta);
	resultFile.open(fileName, ios::app);
	resultFile << s << ',' << numUAVs << ',' << timeAppro << ',' << timeApproD << ',' << timeApproPlus << '\n';
	resultFile.close();

	sprintf_s(fileName, "./output/test0/num-n-%d-m-%d-s-%d-delta%d.txt", numUsers, numHoverLocations, s, delta);
	resultFile.open(fileName, ios::app);
	resultFile << s << ',' << numUAVs << ',' << numApproAlg << ',' << numApproAlgD << ',' << numApproAlgPlus << ',' << preLmax << ',' << L_best << '\n';
	resultFile.close();
	return numApproAlgPlus;
	//return numApproAlg;
	//return numApproAlgD;
}#pragma once

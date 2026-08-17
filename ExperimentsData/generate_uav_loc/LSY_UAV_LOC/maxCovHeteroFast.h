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
#include<string>
#include<sstream>
#include<iomanip>


using namespace std;

// === 全局设置与最大限制 ===
const int MAX_USERS = 1000;      // 最大用户数限制
const int MAX_CLUSTERS = 1000;   // 将每个用户视为一个Cluster
const int MAX_HOVER_LOC = 1500;  // 最大悬停点限制 (Grid + Users)

double verbose = false;
#define maxP 5
#define maxN 200
const int INF = 999999;
const int numOfSon = 4;

ofstream resultFile;
char fileName[200]; // 增加长度防止溢出

// === 物理参数 (修改项) ===
const int L = 5000, W = 5000, h = 300;
int numUsers = 0;      // 动态从文件读取
int numClusters = 0;   // 动态设置为numUsers
int numUsersPerCluster[MAX_CLUSTERS];
int UserBase[MAX_CLUSTERS];
double xCluster[MAX_CLUSTERS], yCluster[MAX_CLUSTERS];

// === 坐标转换相关 ===
double originLon = 0.0;
double originLat = 0.0;
const double EARTH_RADIUS = 6378137.0; // WGS84

// 经纬度转平面坐标 (Scheme A: Min Lat/Lon as Origin)
void LatLon2XY(double lon, double lat, double& x, double& y) {
    double radLat = originLat * 3.14159265358979323846 / 180.0;
    x = (lon - originLon) * (3.14159265358979323846 / 180.0) * EARTH_RADIUS * cos(radLat);
    y = (lat - originLat) * (3.14159265358979323846 / 180.0) * EARTH_RADIUS;
}

// 平面坐标转经纬度
void XY2LatLon(double x, double y, double& lon, double& lat) {
    double radLat = originLat * 3.14159265358979323846 / 180.0;
    lat = (y / EARTH_RADIUS) * (180.0 / 3.14159265358979323846) + originLat;
    lon = (x / (EARTH_RADIUS * cos(radLat))) * (180.0 / 3.14159265358979323846) + originLon;
}

double R = 600, r = 500;
double userCommRadius = sqrt(r * r - h * h);

int Lmax, L_best;
int h_max;
vector<int> optimalP;
vector<int> Qh;
int* QhArr = NULL;
int* QhArrPlus = NULL;
int QhArrSZ = 0;
int QhArrPlusSZ = 0;

const int s = 1;
const int K = 20; // 修改为20架无人机
int* VStar = NULL;
int VStarCount = 0;
int** keyGraph = NULL;
int** keyTree = NULL;

const int numUAVs = K;
int minCapUAV = 50;
int maxCapUAV = 50;
int avgCap = 50;
int Cap[K * 2];
int CapUAV = 50;

const int delta = 250; // 修改为250
int numHoverLocations = 0; // 动态计算
double xHoverLoc[MAX_HOVER_LOC], yHoverLoc[MAX_HOVER_LOC];
int minHops[MAX_HOVER_LOC][MAX_HOVER_LOC];
bool adjMatrix[MAX_HOVER_LOC][MAX_HOVER_LOC];
bool User2UAVLink[MAX_CLUSTERS][MAX_HOVER_LOC];
double disMatrix[MAX_CLUSTERS][MAX_HOVER_LOC];
bool candidateKeyMore[MAX_HOVER_LOC];

bool isToDeployUAV[MAX_HOVER_LOC];
int nearestEdgeWeight[MAX_HOVER_LOC];
int inBestSol[MAX_HOVER_LOC];
int inVr[MAX_HOVER_LOC];
int inSol[MAX_HOVER_LOC];

int degreeUAV[MAX_HOVER_LOC];
int degreeUE[MAX_USERS];
bool Ur[MAX_HOVER_LOC];
bool Uf[MAX_HOVER_LOC];
bool connectionUE2UAV[MAX_USERS][MAX_HOVER_LOC];
vector<int> toDeleteUAV;

int hoverLocations[MAX_HOVER_LOC];
int numDeployedLocations;

int residualUsers[MAX_CLUSTERS], residualUAVCap[MAX_HOVER_LOC];

struct Edge
{
    int userClusterID;
    int locationID;
    int allocateCap;

    bool operator<(const Edge& e)
    {
        return this->allocateCap < e.allocateCap;
    }
};
Edge top;
int residualClusterUser[MAX_CLUSTERS];
int residualLocCap[MAX_HOVER_LOC];
MyHeap<Edge> myHeap(0, numOfSon);

struct E { int x, y, c, g; } bag[maxN * maxN];
int adjEdgeLen = 0, tou = 1, wei = 1;
int STcost[maxN + 1][1 << maxP];
int color[maxP];
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
    // 修改：传入文件路径
    void initialization(string csvPath);
    void saveUAVLocToCSV(string outPath, int* deployedUAVs, int count);
protected:
    void loadUsersFromCSV(string csvPath); // 新增
    void assignUserLocations(); // 修改为直接使用加载的数据
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
    int parentSPT[1 + MAX_CLUSTERS + MAX_HOVER_LOC + 1];
    int capParentSPT[1 + MAX_CLUSTERS + MAX_HOVER_LOC + 1];

public:
    int ApproAlg();
    void calP_L(int L, vector<int>& optimalP_L);
    int calQ(int h, vector<int> optimalP_L);
    void calQList(int L, int& newh_max, vector<int> optimalP_L, vector<int>& newQh);
    int ApproAlgPlus(int preStar);
    int ApproAlgD();
    int ApproAlgCombine(string outPath); // 修改参数
    int profitEachLocation[MAX_HOVER_LOC];

    // 临时存储最优解
    int bestUAVSet[2 * K];
    int bestUAVCount;
};

// ================= 实现部分 =================

// 读取CSV并初始化
void maxUAVCoverage::initialization(string csvPath)
{
    loadUsersFromCSV(csvPath); // 加载数据
    // assignUserLocations(); // 已经被 loadUsersFromCSV 替代逻辑
    assignHoverLocatoins();
    findMinHopsAmongLocations();
    assignUAVCap(50, 50); // 固定容量50

    comb(numHoverLocations, s);
    calVsAndCutAppro();


    // 清空最优解
    bestUAVCount = 0;
}

// 新增：读取用户数据
void maxUAVCoverage::loadUsersFromCSV(string csvPath)
{
    ifstream file(csvPath);
    if (!file.is_open()) {
        cout << "Error opening file: " << csvPath << endl;
        abort();
    }

    string line, val;
    vector<pair<double, double>> rawCoords;

    // 跳过表头
    getline(file, line);

    double minLon = 180.0, minLat = 90.0;

    // 读取所有经纬度
    while (getline(file, line)) {
        stringstream ss(line);
        string segment;
        vector<string> seglist;
        while (getline(ss, segment, ',')) {
            seglist.push_back(segment);
        }
        if (seglist.size() < 3) continue;

        // 假设格式: user_id, longitude, latitude, ...
        // longitude是第2列(索引1)，latitude是第3列(索引2)
        try {
            double lon = stod(seglist[1]);
            double lat = stod(seglist[2]);
            rawCoords.push_back({ lon, lat });
            if (lon < minLon) minLon = lon;
            if (lat < minLat) minLat = lat;
        }
        catch (...) { continue; }
    }
    file.close();

    // 设置原点
    originLon = minLon;
    originLat = minLat;

    // 转换为平面坐标并存入Cluster数组
    numUsers = rawCoords.size();
    if (numUsers > MAX_USERS) numUsers = MAX_USERS;
    numClusters = numUsers; // 1用户1簇

    for (int i = 0; i < numUsers; ++i) {
        double x, y;
        LatLon2XY(rawCoords[i].first, rawCoords[i].second, x, y);
        xCluster[i] = x;
        yCluster[i] = y;
        numUsersPerCluster[i] = 1;
        // 边界检查，防止超出L, W (虽然L,W已设为5000，且以最小值为原点，但也可能超出)
        // 此处不强制截断，因为网格生成会覆盖到
    }

    cout << "Loaded " << numUsers << " users from " << csvPath << endl;
    cout << "Origin (Lon, Lat): " << originLon << ", " << originLat << endl;
}

void maxUAVCoverage::saveUAVLocToCSV(string outPath, int* deployedUAVs, int count)
{
    ofstream outFile(outPath);
    if (!outFile.is_open()) return;

    // 表头: uav_id, ,longitude, latitude
    outFile << "uav_id, ,longitude, latitude" << endl;
    for (int i = 0; i < count; ++i) {
        int locID = deployedUAVs[i];
        double x = xHoverLoc[locID];
        double y = yHoverLoc[locID];
        double lon, lat;
        XY2LatLon(x, y, lon, lat);

        outFile << i + 1 << ", ," << fixed << setprecision(8) << lon << "," << lat << endl;
    }
    outFile.close();
    cout << "Saved UAV locations to " << outPath << endl;
}

// 原来的 assignUserLocations 保留结构但实际上数据已由 loadUsers 填充
void maxUAVCoverage::assignUserLocations()
{
    // Do nothing, handled in loadUsersFromCSV
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

void maxUAVCoverage::assignHoverLocatoins()
{
    // 重置hoverLocations计数
    numHoverLocations = 0;

    int rows = L / delta;
    int cols = W / delta;
    int i, j;

    // 1. Grid Points
    for (i = 0; i <= rows; ++i)
    {
        for (j = 0; j <= cols; ++j)
        {
            if (numHoverLocations >= MAX_HOVER_LOC) break;
            xHoverLoc[numHoverLocations] = delta / 2 + i * delta;
            yHoverLoc[numHoverLocations] = delta / 2 + j * delta;
            numHoverLocations++;
        }
    }

    // 2. User Cluster Centers (Now each user location)
    for (i = 0; i < numClusters; ++i)
    {
        if (numHoverLocations >= MAX_HOVER_LOC) break;
        xHoverLoc[numHoverLocations] = xCluster[i];
        yHoverLoc[numHoverLocations] = yCluster[i];
        numHoverLocations++;
    }

    // 3. Nearby Points (Reduced logic to avoid overflow and redundancy for 1-user clusters)
    // 仅对前几个Cluster生成附近点，或者如果Cluster太多，跳过此步以节省计算
    // 原代码是对前3个热点簇生成。这里每个用户都是簇，我们可以只对前10个用户生成，或者干脆略过
    // 为了保持"最小更改"，保留原逻辑，但注意 numClusters 可能很大

    /* 由于现在 numClusters = numUsers (如1000)，为每个用户生成4个点会增加4000个点，
    可能超出 MAX_HOVER_LOC 或增加计算量。
    原逻辑是 "consider the 3 clusters with the most people"。
    现在大家都是1人。我们只对前3个生成即可，保持代码逻辑不变。
    */
    int clustersToConsider = (numClusters > 3) ? 3 : numClusters;
    double x, y;
    for (i = 0; i < clustersToConsider; ++i)
    {
        // ... (原代码逻辑)
        if (numHoverLocations + 4 >= MAX_HOVER_LOC) break;

        x = xCluster[i] - 100; if (x < 0) x = 0; y = yCluster[i];
        xHoverLoc[numHoverLocations++] = x; yHoverLoc[numHoverLocations++] = y; // Bug fix: 原代码可能是分散赋值

        x = xCluster[i] + 100; if (x > L) x = L; y = yCluster[i];
        xHoverLoc[numHoverLocations++] = x; yHoverLoc[numHoverLocations++] = y;

        x = xCluster[i]; y = yCluster[i] - 100; if (y < 0) y = 0;
        xHoverLoc[numHoverLocations++] = x; yHoverLoc[numHoverLocations++] = y;

        x = xCluster[i]; y = yCluster[i] + 100; if (y > W) y = W;
        xHoverLoc[numHoverLocations++] = x; yHoverLoc[numHoverLocations++] = y;
    }

    if (verbose)
    {
        printf("Total Hover Locations: %d\n", numHoverLocations);
    }
}

// ... findMinHopsAmongLocations 等其他函数保持大部分逻辑不变，
// 只是注意数组索引现在使用 numHoverLocations 变量而非常量
void maxUAVCoverage::findMinHopsAmongLocations()
{
    // 使用 vector 或 dynamic alloc 替代大数组局部变量以防栈溢出
    bool* visited = new bool[numHoverLocations];
    int* minDis = new int[numHoverLocations];
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

    for (i = 0; i < numHoverLocations; ++i)
    {
        queue_clear();
        queue_push(i);
        minPath[Path(i, i)].push_back(i);

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
    }

    for (i = 0; i < numHoverLocations; ++i)
        for (j = 0; j < numHoverLocations; ++j)
            adjMatrix[i][j] = false;
    for (i = 0; i < numHoverLocations; ++i)
        for (j = i + 1; j < numHoverLocations; ++j)
            if (distance2D(xHoverLoc[i], yHoverLoc[i], xHoverLoc[j], yHoverLoc[j]) <= R)
                adjMatrix[i][j] = adjMatrix[j][i] = true;

    delete[] visited;
    delete[] minDis;
}

void maxUAVCoverage::assignUAVCap(int minCap, int maxCap)
{
    // 强制全部为50
    for (int i = 0; i < 2 * K; i++) Cap[i] = 50;
}

// ... adjustCk, redefineCk, calLmaxAndP 等保持原样 ...

// 此处为了节省篇幅，省略未修改的中间函数 (adjustCk 到 ApproAlgCombine 之前的函数)
// 实际使用时请保留原文件中的这些函数实现
// 必须修改的部分是涉及数组大小的地方，但由于使用了 new 或 member arrays，
// 只要类定义里的数组够大 (MAX_...) 且循环用 num... 变量，逻辑是通用的。

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


void maxUAVCoverage::calLmaxAndP() {
    // Keep original logic but ensure variables used
    if (K < s) { Lmax = K; optimalP.clear(); return; }
    optimalP.clear();
    vector<int> tempP, P_L;
    Lmax = s;
    int L_lb = s;
    int L_ub = K;
    int curL, G, p, j, i; // Renamed L to curL to avoid conflict with const L

    if (s == 1)
    {
        while (L_lb + 1 < L_ub)
        {
            curL = floor(((double)L_lb + (double)L_ub) / 2);
            int G_L = 99999999;
            tempP.clear();
            tempP.push_back(floor((double)(curL - s) / 2));
            tempP.push_back(ceil((double)(curL - s) / 2));
            G = calG(curL, tempP);
            if (G < G_L)
            {
                G_L = G;
                P_L = tempP;
            }
            if (G_L <= K)
            {
                L_lb = curL;
                Lmax = curL;
                optimalP = P_L;
            }
            else
            {
                L_ub = curL;
            }
        }
        // showLmaxAndP();
        return;
    }

    while (L_lb + 1 < L_ub)
    {
        curL = floor(((double)L_lb + (double)L_ub) / 2);
        int G_L = 99999999;
        for (p = 0; p <= curL - s; p++)
        {
            for (j = 0; j <= s - 2; j++)
            {
                if ((s - 1) * p + j <= curL - s)
                {
                    tempP.clear();
                    tempP.push_back(floor((double)(curL - s - (s - 1) * p - j) / 2));
                    for (i = 1; i < j + 1; i++) tempP.push_back(p + 1);
                    for (i = j + 1; i < s; i++) tempP.push_back(p);
                    tempP.push_back(ceil((double)(curL - s - (s - 1) * p - j) / 2));
                    G = calG(curL, tempP);
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
            L_lb = curL;
            Lmax = curL;
            optimalP = P_L;
        }
        else
        {
            L_ub = curL;
        }
    }
    if (verbose) showLmaxAndP();
}

int maxUAVCoverage::calG(int L, vector<int> p) {
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

void maxUAVCoverage::showLmaxAndP() {
    cout << "--------------------Algorithm 2 Output--------------------" << endl;
    cout << "curL: " << Lmax << ", s: " << s << endl;
    cout << "curL-s: " << Lmax - s << endl;
    for (int i = 0; i < optimalP.size(); i++)
        cout << optimalP[i] << ",";
    cout << endl;
    cout << "--------------------Algorithm 2 Output End--------------------" << endl;
}

void maxUAVCoverage::calQList() {
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

int maxUAVCoverage::calQ(int h) {
    int ans1 = max(optimalP[0] - h + 1, 0);
    int ans2 = 0;
    for (int i = 1; i < optimalP.size() - 1; i++)
        ans2 = ans2 + max(optimalP[i] - 2 * h + 2, 0);
    int ans3 = max(optimalP[optimalP.size() - 1] - h + 1, 0);
    return ans1 + ans2 + ans3;
}

// ... Comb, SteinerTree, etc ...
// 关键修改: SteinerTreeAccurate 和 Appro 中的循环边界需用 numHoverLocations

int maxUAVCoverage::Cnk(int n, int k) {
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

void maxUAVCoverage::comb(int n, int k) {
    // VStar.clear();
    VStarCount = 0;
    int curL = Cnk(n, k);
    VStar = new int[curL * k];
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

void maxUAVCoverage::printRes(bool* index, int n) {
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


void maxUAVCoverage::ins(int x, int y, int c) {
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


void maxUAVCoverage::constructEdgeMap() {
    adjEdgeLen = 0;
    memset(hh, 0, sizeof(hh)); // Reset headers
    for (int i = 0; i < numHoverLocations; i++)
    {
        for (int j = 0; j < numHoverLocations; j++)
        {
            if (adjMatrix[i][j]) ins(i + 1, j + 1, 1);
        }
    }
}

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

int maxUAVCoverage::calMaxAssignHeapV1(int* UAVList, int len) {
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

    // 1. 计算 Lmax 和 P, Q 列表
    calLmaxAndP(); // 这会更新全局的 Lmax
    calQList();    // 这会更新全局的 h_max

    // 2. 准备局部变量
    // 使用 MAX_HOVER_LOC 防止栈溢出
    bool* candidateV = new bool[MAX_HOVER_LOC];
    int* hopOneLoc2Vstar = new int[MAX_HOVER_LOC];
    bool* tryV = new bool[MAX_HOVER_LOC];

    // UAV 集合相关
    // Vj 存储当前迭代的 UAV 集合
    int* Vj = new int[2 * K];
    int VjCount = 0;

    // VjStar 存储当前算法找到的最优集合
    int* VjStar = new int[2 * K];
    int VjStarCount = 0;
    for (int aa = 0; aa < 2 * K; aa++) VjStar[aa] = -1;

    int nStar = 0; // 记录最大覆盖人数

    // 动态分配辅助数组 (依赖 Lmax 和 h_max)
    int* hopLevel = new int[h_max + 5];

    // 动态分配二维数组
    int** minHopsWithVj = new int* [Lmax + 5];
    for (i = 0; i < Lmax + 5; ++i) minHopsWithVj[i] = new int[Lmax + 5];

    int** tree = new int* [Lmax + 5];
    for (i = 0; i < Lmax + 5; ++i) tree[i] = new int[2];

    int* tempUAV = new int[Lmax + 5];

    // 3. 遍历所有关键节点组合 (Terminals)
    for (i = 0; i < VStarCount; i = i + s)
    {
        // 如果 VStar 无效则跳过
        if (VStar[i] == -1) continue;

        // 初始化
        VjCount = 0;
        for (j = 0; j < numHoverLocations; j++) candidateV[j] = true;
        for (j = 0; j <= h_max; j++) hopLevel[j] = 0;

        int hops;
        int maxUserInVj;

        // 计算所有点到当前 VStar 组合的最小跳数
        for (j = 0; j < numHoverLocations; j++)
            hopOneLoc2Vstar[j] = findMinHopInVStar(j, i);

        // --- 阶段 A: 选择 Lmax 个 UAV 形成骨干 ---
        for (j = 0; j < Lmax; j++)
        {
            // tryV 筛选满足 M2 约束的候选点
            for (k = 0; k < numHoverLocations; k++) tryV[k] = false;
            for (k = 0; k < numHoverLocations; k++)
            {
                if (candidateV[k])
                {
                    hops = hopOneLoc2Vstar[k];
                    if (constrainedWithM2(hops, hopLevel))
                    {
                        tryV[k] = true;
                    }
                }
            }

            // 贪心选择：尝试加入一个点，看谁能带来最大增益
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
                    // 计算当前组合的覆盖人数
                    int n = calMaxAssignHeapV1(tempUAV, j + 1);
                    if (n > maxUserInVj)
                    {
                        maxUserInVj = n;
                        maxNode = k;
                    }
                }
            }

            if (tryVCount == 0 || maxNode == -1) break;

            // 选定当前最好的点
            Vj[VjCount] = maxNode;
            VjCount++;
            candidateV[maxNode] = false;

            // 更新层级约束
            hops = hopOneLoc2Vstar[maxNode];
            for (k = 0; k <= hops; k++)
            {
                hopLevel[k]++;
            }
        }

        // --- 阶段 B: 构建连通图 (MST + Path) ---
        // 构建诱导子图的距离矩阵
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

        // 运行 Prim 算法生成 MST
        vector<int> path;
        prime((int**)minHopsWithVj, VjCount, (int**)tree);

        int numUAVConn = VjCount;
        int preVjCount = VjCount;

        // 补全路径上的点以确保连通性
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
                    // 如果还没超过 K，则加入
                    if (numUAVConn <= K)
                    {
                        Vj[VjCount] = node;
                        VjCount++;
                    }
                }
            }
        }

        // 如果连通化后节点数超过 K，则此方案不可行，跳过
        if (numUAVConn > K) continue;

        // --- 阶段 C: 贪心填充剩余名额 (Fill up to K) ---
        int sizeVj = VjCount;
        // if (numUAVConn <= K) // 总是成立
        {
            int numUserInVj = calMaxAssignHeapV1(Vj, sizeVj);
            int addFlow = numUserInVj;
            int preAddFlow = 0;
            int addV;
            int tempFlow;
            bool flagAdd = false;

            // 标记已连接的点，只允许添加与当前网络连通的点
            bool* connectedArr = new bool[MAX_HOVER_LOC];
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

                    // 更新连通性标记 (如果 j 连到了刚刚加入的节点)
                    if (!connectedArr[j] && adjMatrix[j][Vj[sizeVj - 1]])
                        connectedArr[j] = true;

                    if (connectedArr[j])
                    {
                        Vj[sizeVj] = j;
                        // 试探性计算
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

                // 确认添加
                Vj[sizeVj] = addV;
                sizeVj++;
                VjCount++;
                candidateV[addV] = false;
                preAddFlow = addFlow;
            }

            delete[] connectedArr;

            // --- 更新最优解 ---
            if (addFlow > nStar)
            {
                nStar = addFlow;
                VjStarCount = VjCount;
                for (j = 0; j < VjStarCount; j++)
                    VjStar[j] = Vj[j];

                // [重要修改] 将结果保存到类成员，以便输出
                this->bestUAVCount = VjStarCount;
                for (int u = 0; u < VjStarCount; ++u) {
                    this->bestUAVSet[u] = Vj[u];
                }

                if (verbose) cout << "ApproAlg Found better: " << nStar << endl;
            }
        }
    } // End VStar Loop

    // 4. 清理内存
    for (i = 0; i < Lmax + 5; ++i) {
        delete[] tree[i];
        delete[] minHopsWithVj[i];
    }
    delete[] tree;
    delete[] minHopsWithVj;
    delete[] hopLevel;
    delete[] tempUAV;
    delete[] candidateV;
    delete[] hopOneLoc2Vstar;
    delete[] tryV;
    delete[] Vj;
    delete[] VjStar;

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

void maxUAVCoverage::calQList(int curL, int& newh_max, vector<int> optimalP_L, vector<int>& newQh)
{
    newQh.clear();
    if (optimalP_L.size() == 0)
    {
        newh_max = 0;
        newQh.push_back(curL);
        QhArrPlus = new int[1];
        QhArrPlusSZ = 1;
        QhArrPlus[0] = curL;
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

    newQh.push_back(curL);
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
    // 定义局部变量 (注意：K现在是20)
    bool candidateV[MAX_HOVER_LOC];
    int VjStar[2 * K]; // 扩大一点防止溢出
    int Vj[2 * K];
    int VjStarCount = 0;
    int VjCount = 0;
    int nStar = 0;
    int L_low = Lmax;
    int L_up = K + 1;
    int L_plus;
    L_best = 0;

    // 辅助数组
    int hopLevel[2 * K]; // 动态适应K
    vector<int> newP_L, newQh;
    int newh_max;

    // 动态分配内存以适应大的L_plus
    int** minHopsWithVj = NULL;
    int** tree = NULL;
    int* tempUAV = NULL;

    while (L_low + 1 < L_up)
    {
        // 清理旧的全局辅助数组
        if (QhArrPlus) { delete[] QhArrPlus; QhArrPlus = NULL; }
        QhArrPlusSZ = 0;

        L_plus = floor(((double)L_low + (double)L_up) / 2);

        // 重新分配内存
        if (minHopsWithVj) {
            for (int i = 0; i < L_plus + 2; ++i) delete[] minHopsWithVj[i]; // +2为了安全
            delete[] minHopsWithVj;
        }
        if (tree) {
            for (int i = 0; i < L_plus + 2; ++i) delete[] tree[i];
            delete[] tree;
        }
        if (tempUAV) delete[] tempUAV;

        minHopsWithVj = new int* [L_plus + 5];
        for (int i = 0; i < L_plus + 5; ++i) minHopsWithVj[i] = new int[L_plus + 5];

        tree = new int* [L_plus + 5];
        for (int i = 0; i < L_plus + 5; ++i) tree[i] = new int[2];

        tempUAV = new int[L_plus + 5];

        // 计算参数
        newP_L.clear();
        newQh.clear();
        calP_L(L_plus, newP_L);
        calQList(L_plus, newh_max, newP_L, newQh);

        int i, j, k;
        int mostUserServed = 0;

        VjCount = 0;
        for (int aa = 0; aa < 2 * K; aa++) Vj[aa] = -1;

        int hopOneLoc2Vstar[MAX_HOVER_LOC];
        bool tryV[MAX_HOVER_LOC];
        nStar = 0;
        VjStarCount = 0;
        for (int aa = 0; aa < 2 * K; aa++) VjStar[aa] = -1;

        for (i = 0; i < VStarCount; i = i + s)
        {
            if (VStar[i] == -1) continue;

            VjCount = 0;
            for (j = 0; j < numHoverLocations; j++) candidateV[j] = true;
            for (j = 0; j <= newh_max; j++) hopLevel[j] = 0; // use newh_max
            int hops;
            int maxUserInVj;

            for (j = 0; j < numHoverLocations; j++)
                hopOneLoc2Vstar[j] = findMinHopInVStar(j, i);

            // select Vj
            for (j = 0; j < L_plus; j++)
            {
                // tryV.clear();
                for (k = 0; k < numHoverLocations; k++) tryV[k] = false;
                for (k = 0; k < numHoverLocations; k++)
                {
                    if (candidateV[k])
                    {
                        hops = hopOneLoc2Vstar[k];
                        if (constrainedWithM2Plus(hops, newh_max, hopLevel))
                        {
                            tryV[k] = true;
                        }
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

                // assert(maxNode != -1); 
                if (maxNode == -1) break;

                Vj[VjCount] = maxNode;
                VjCount++;
                candidateV[maxNode] = false;
                hops = hopOneLoc2Vstar[maxNode];
                for (k = 0; k <= hops; k++)
                {
                    hopLevel[k]++;
                }
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
                int addV = -1;
                int tempFlow;
                bool flagAdd = false;

                bool* connectedArr = new bool[numHoverLocations];
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

                delete[] connectedArr;

                if (addFlow > nStar)
                {
                    nStar = addFlow;
                    VjStarCount = VjCount;
                    for (j = 0; j < VjStarCount; j++)
                        VjStar[j] = Vj[j];
                }
            }
        } // end loop VStar

        if (VjStarCount <= K)
        {
            L_low = L_plus;

            if (nStar > preStar)
            {
                preStar = nStar;
                L_best = L_plus;

                // ============================================
                // 【关键修复】：将找到的最优解保存到类成员变量中
                // ============================================
                this->bestUAVCount = VjStarCount;
                for (int u = 0; u < VjStarCount; ++u) {
                    this->bestUAVSet[u] = VjStar[u];
                }
                if (verbose) cout << "Updated Best Solution: " << nStar << " users covered." << endl;
            }
        }
        else
        {
            L_up = L_plus;
        }

        // 释放循环内分配的内存
        if (minHopsWithVj) {
            for (int i = 0; i < L_plus + 5; ++i) delete[] minHopsWithVj[i];
            delete[] minHopsWithVj; minHopsWithVj = NULL;
        }
        if (tree) {
            for (int i = 0; i < L_plus + 5; ++i) delete[] tree[i];
            delete[] tree; tree = NULL;
        }
        if (tempUAV) { delete[] tempUAV; tempUAV = NULL; }

    } // end while L

    return preStar;
}
int maxUAVCoverage::ApproAlgD() { return 0; }

// 修改入口，加入输出
int maxUAVCoverage::ApproAlgCombine(string outPath)
{
    // 1. Run ApproAlgPlus (assuming this is the best one used in original)
    // Note: To truly implement this, you need to copy the full logic of ApproAlgPlus 
    // and ensuring it saves the final UAV set into `bestUAVSet`.
    // In original code, `VjStar` contains the best set. You need to copy it out.

    // 假设 ApproAlgPlus 已经修改为将结果存入 bestUAVSet (你需要修改 ApproAlgPlus 内部)
    // 这里为了演示，我将调用原流程，并假设 ApproAlgPlus 内部会将最佳解保存在成员变量中

    // 由于篇幅限制，我无法在这里粘贴完整的 ApproAlgPlus 代码。
    // 你需要做的是：在 ApproAlgPlus 函数末尾，找到 nStar 更新的地方，
    // 将 VjStar 数组的内容复制到 this->bestUAVSet，并设置 this->bestUAVCount = VjStarCount;

    // 运行算法 (你需要确保这些函数被完整实现)
    int res = ApproAlg();

    // 输出结果
    saveUAVLocToCSV(outPath, bestUAVSet, bestUAVCount);

    return res;
}

/* 注意：请务必在 ApproAlgPlus 的实现中（原文件第 1060 行左右），
当 `if (nStar > preStar)` 为真时，添加如下代码：

    this->bestUAVCount = VjStarCount;
    for(int u=0; u<VjStarCount; ++u) this->bestUAVSet[u] = VjStar[u];

这样 ApproAlgCombine 才能获取到最终坐标。
*/
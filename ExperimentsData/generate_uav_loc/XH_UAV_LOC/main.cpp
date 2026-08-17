//#include<stdio.h>
//#include<assert.h>
//#include<math.h>
//#include<time.h>
//#include<iostream>
//#include<fstream>
//#include<ctime>
//using namespace std;
//#include "maxCoverageModified.h"
//
//int main_test()
//{
//	int i, j, k, l, m, n;
//	//srand(1);
//	clock_t st, ft;
//	maxUAVCoverage maxCoverage;
//
//	int loop_times = 1;
//	int loop;
//
//	// ApproAlgShort
//	double numUsersCoveredApproAlgShort = 0;
//	double timeApproAlgShort = 0;
//	st = clock();
//	for (loop = 0; loop < loop_times; ++loop)
//	{
//		//printf("Appro Alg %d th loop...\n", loop + 1);
//		srand(loop + 100);
//		maxCoverage.initialization();
//		numUsersCoveredApproAlgShort += maxCoverage.ApproAlgShort();
//	}
//	ft = clock();
//	numUsersCoveredApproAlgShort /= loop_times;
//	timeApproAlgShort = (ft - st) / (1000.0 * loop_times);
//
//
//
//
//	printf("num of UAVs: \t%d\n", numUAVs);
//	printf("num of Users: \t%d\n", numUsers);
//	printf("users covered by Short Alg: \t%.2lf Gpbs, time: %.2lf sec\n", numUsersCoveredApproAlgShort, timeApproAlgShort);
//	return 0;
//}

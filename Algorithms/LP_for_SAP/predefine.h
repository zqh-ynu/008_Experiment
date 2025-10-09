#pragma once
using namespace std;
#include<string>
#include<vector>
#include<map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <limits.h>
#include <cstdlib>
#include <iomanip>
//#include <ilcplex/ilocplex.h>
//#include <ilconcert/ilomodel.h>
#include <ctime>
#include <algorithm>
#include <random>
#include <filesystem>
#include <assert.h>
#include <climits> // 

#include <ilcplex/ilocplex.h>
#include <stdio.h>
using namespace std;
typedef IloArray <IloNumVarArray> IloNumVarArray2;

#define _USE_MATH_DEFINES // 该语句的作用是，启用数学库中的常量定义，如M_PI等
#include <cmath>



// 当前算法项目的绝对路径地址: E:\Research\My paper\2_Papers\008\008_Experiment\Algorithms\LP_for_SAP
const static string algProjPath = "E:\\Research\\My paper\\2_Papers\\008\\008_Experiment\\Algorithms\\LP_for_SAP\\";
// 当前论文项目的实验数据文件夹路径：E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\

const static string experimentDataPath = "E:\\Research\\My paper\\2_Papers\\008\\008_Experiment\\ExperimentsData\\";

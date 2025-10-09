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

#include <ctime>
#include <algorithm>
#include <random>
#include <filesystem>
#include <assert.h>
#include <climits> // 

#include <ilcplex/ilocplex.h>
#include <stdio.h>
#include <functional>

// 配置boost环境
// 在函数 Channel::cal_SNR_th() 中使用了boost库中的toms748_solve函数
// 配置方法：与cplex类似，下载boost库，然后配置环境变量
// 注意在属性页中cplex和boost均在Release下配置
#include <boost/math/tools/toms748_solve.hpp>
// 该文件中包含了gamma函数族
#include <boost/math/special_functions/gamma.hpp>
#include <boost/cstdint.hpp>


using namespace std;
typedef IloArray <IloNumVarArray> IloNumVarArray2;

#define _USE_MATH_DEFINES // 该语句的作用是，启用数学库中的常量定义，如M_PI等
#include <cmath>
#define HARD_UTILITY 1 // 硬效用类型
#define ELASTIC_UTILITY 2 // 交互式弹性效用类型/ TCP效用
#define HALFSOFT_UTILITY 3 // 右半边软效用
#define INFINITE 1e9 // 无穷大
#define EPS 1e-6 // 浮点数比较时的误差范围
#define BANDWIDTH 20 // 每个无人机的总带宽 20MHz


const double uav_H = 300;			// 无人机高度 300m
const double f = 2.4e9;             // 信号频段，1.9GHz 2.4GHz
const double P_N = -105;            // 阴影噪声 -105 dBm
const double eta_LoS = 1;           // 视距损失相关系数 1 dB
const double eta_NLoS = 20;         // 非视距损失相关系数 20 dB
const double c = 3e8;               // 光速 3*10^8 m/s
const double a = 9.611725;          // 视距概率系数  9.611725 27.23
const double b = 0.158062;          // 视距概率系数 0.158062 0.08
const double pi = 3.1415926535;		
const double R_c = 0.5;             // 最小频谱效率 0.5 bit/s/Hz
const double alpha = 3;             // 路径损耗指数 3
const double g_BS = 10;            // ibs的天线增益 dB
const double g_UAV = 5;           // uav的天线增益 dB




// 当前算法项目的绝对路径地址: E:\Research\My paper\2_Papers\008\008_Experiment\Algorithms\LP_for_SAP
const static string algProjPath = "E:\\Research\\My paper\\2_Papers\\008\\008_Experiment\\Algorithms\\UAVBandwidthAllocation\\";
// 当前论文项目的实验数据文件夹路径：E:\Research\My paper\2_Papers\008\008_Experiment\ExperimentsData\

const static string experimentDataPath = "E:\\Research\\My paper\\2_Papers\\008\\008_Experiment\\ExperimentsData\\";

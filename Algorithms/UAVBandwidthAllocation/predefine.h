#pragma once
// 本文件集中定义共享常量、第三方依赖、本机项目路径及信道配置。
using namespace std;
#include<string>
#include<vector>
#include<map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <cmath>
#include <limits.h>
#include <cstdlib>
#include <iomanip>// 用于格式化输出
#include <ctime>
#include <algorithm>
#include <random>
#include <filesystem>
#include <assert.h>
#include <climits> // 

#include <stdio.h>
#include <functional>
#include <nlohmann/json.hpp> // 引入json库，方便读写json格式的文件
using json = nlohmann::json;	// json类型重命名为json
namespace fs = std::filesystem;

// 配置boost环境
// 在函数 Channel::cal_SNR_th() 中使用了boost库中的toms748_solve函数
// 配置方法：与cplex类似，下载boost库，然后配置环境变量
// 注意在属性页中cplex和boost均在Release下配置
#include <boost/math/tools/toms748_solve.hpp>
// 该文件中包含了gamma函数族
#include <boost/math/special_functions/gamma.hpp>
#include <boost/cstdint.hpp>

#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include <IpSmartPtr.hpp>


using namespace std;

#define _USE_MATH_DEFINES // 该语句的作用是，启用数学库中的常量定义，如M_PI等
#include <cmath>
#define HARD_UTILITY 1 // 硬效用类型
#define ELASTIC_UTILITY 2 // 交互式弹性效用类型/ TCP效用
#define HALFSOFT_UTILITY 3 // 右半边软效用
#define INFINITE 1e9 // 无穷大
#define EPS 1e-9 // 浮点数比较时的误差范围
#define BANDWIDTH 20 // 每个无人机的总带宽 20MHz
constexpr double INF = std::numeric_limits<double>::infinity();

const double pi = 3.1415926535;
// 地球半径 (米)
const double EARTH_RADIUS = 6371000.0;

const int unit_para = 1000; // 单位转换参数，等于1000时，单位为kbps，等于1时，单位为Mbps

// 环境物理量
inline double freq_hz;
inline double speed_light;
inline double path_loss_exp;

// 几何
inline double uav_alt;
inline double uav_trans_power;

// 信道模型
inline double noise_dbm;        // 功率谱密度 dbm/hz
inline double los_loss_db;
inline double nlos_loss_db;
inline double param_a;
inline double param_b;
inline int max_coverage_distance; // 最大通信距离，单位米

// 硬件
inline double gain_bs_db;
inline double gain_uav_db;

// 约束
inline double min_se_threshold; // spectral efficiency




// 当前算法项目的绝对路径地址: E:\Research\My_paper\2_Papers\008\008_Experiment\Algorithms\UAVBandwidthAllocation
const static string algProjPath = "E:\\Research\\My_paper\\2_Papers\\008\\008_Experiment\\Algorithms\\UAVBandwidthAllocation\\";


// 当前论文项目的实验数据文件夹路径：E:\Research\My_paper\2_Papers\008\008_Experiment\ExperimentsData\\

const static string experimentDataPath = "E:\\Research\\My_paper\\2_Papers\\008\\008_Experiment\\ExperimentsData\\";


// 读取函数示例
inline void load_global_channel_config(string config_file) {

    // 1. 打开文件
    std::ifstream config_ifs(config_file);
    if (!config_ifs.is_open()) {
        cerr << "Error opening config file: " << config_file << endl;
    }

    // 2. 解析JSON内容
    json data;
    try {
        config_ifs >> data;
    }
    catch (const json::parse_error& e) {
        cerr << "JSON parse error in config file: " << e.what() << endl;
    }


    // --- 读取环境参数 (Environmental Constants) ---
    // 使用 .at() 方法比 [] 更安全，如果 key 不存在会抛出异常
    try {
        auto& env = data.at("environmental_constants");

        los_loss_db = env.at("los_loss_db").get<int>();
        nlos_loss_db = env.at("nlos_loss_db").get<int>(
        );
        noise_dbm = env.at("noise_dbm").get<int>();
        param_a = env.at("param_a").get<double>();
        param_b = env.at("param_b").get<double>();
        speed_light = env.at("speed_light").get<long long>(); // 光速数值较大，建议用 long long
		max_coverage_distance = env.at("max_coverage_distance").get<int>();

       /* std::cout << "=== 环境参数 ===" << std::endl;
        std::cout << "LoS 损耗: " << los_loss_db << " dB" << std::endl;
        std::cout << "NLoS 损耗: " << nlos_loss_db << " dB" << std::endl;
        std::cout << "参数 A: " << param_a << std::endl;
        std::cout << "参数 B: " << param_b << std::endl;
        std::cout << "光速: " << speed_light << " m/s" << std::endl;
		std::cout << "最大通信距离: " << max_coverage_distance << " m" << std::endl;*/

    }
    catch (json::out_of_range& e) {
        std::cerr << "缺少环境参数字段: " << e.what() << std::endl;
    }

    // --- 读取无人机参数 (UAV Constants) ---
    try {
        auto& uav = data.at("uav_constants");

        uav_alt = uav.at("uav_alt").get<int>();
        freq_hz = uav.at("freq_ghz").get<double>() * 1e9;
        uav_trans_power = uav.at("trans_power").get<int>();
        gain_uav_db = uav.at("gain_uav_db").get<int>();

        /*std::cout << "\n=== 无人机参数 ===" << std::endl;
        std::cout << "高度: " << uav_alt << " m" << std::endl;
        std::cout << "频率: " << freq_hz << " GHz" << std::endl;
        std::cout << "传输功率: " << uav_trans_power << " W" << std::endl;
        std::cout << "增益: " << gain_uav_db << " dB" << std::endl;*/

    }
    catch (json::out_of_range& e) {
        std::cerr << "缺少无人机参数字段: " << e.what() << std::endl;
    }
}

/**
 * @brief 获取文件夹下按ID配对的文件路径列表
 * * @param dirPath       数据文件夹路径
 * @param patternA      第一类文件的关键词 (例如 "1000users_data")
 * @param patternB      第二类文件的关键词 (例如 "10uavs_loc")
 * @param maxCount      最大ID数量 (例如 50，则查找 1~50 的文件)
 * @param outFilesA     [输出] 第一类文件完整路径列表 (按ID顺序)
 * @param outFilesB     [输出] 第二类文件完整路径列表 (按ID顺序)
 * @return true         成功找到至少一对文件
 * @return false        目录不存在或未找到匹配文件
 */
inline bool getMatchedFilePairs(const string& userDirPath, // 修改：拆分为用户路径
    const string& uavDirPath,                             // 修改：和无人机路径
    const string& patternA,
    const string& patternB,
    int maxCount,
    vector<string>& outFilesA,
    vector<string>& outFilesB)
{
    // 依然保留您原来的子文件夹拼接逻辑
    string user_full_path = userDirPath + "user_data/";
    string uav_full_path = uavDirPath + "uav_data/";
    outFilesA.clear();
    outFilesB.clear();

    if (!fs::exists(user_full_path) && !fs::exists(uav_full_path)) {
        cerr << "[Error] Directory not found." << endl;
        return false;
    }

    map<int, string> mapA, mapB;

    // 1. 扫描用户数据 (使用 user_full_path)
    for (const auto& entry : fs::directory_iterator(user_full_path)) {
        if (!entry.is_regular_file()) continue;
        string filename = entry.path().filename().string();
        if (filename.find(patternA) == string::npos) continue; // 确保包含关键词
        size_t underscorePos = filename.find('_');
        if (underscorePos != string::npos) {
            mapA[stoi(filename.substr(0, underscorePos))] = entry.path().string();
        }
    }

    // 2. 扫描无人机数据 (使用 uav_full_path)
    for (const auto& entry : fs::directory_iterator(uav_full_path)) {
        if (!entry.is_regular_file()) continue;
        string filename = entry.path().filename().string();
        if (filename.find(patternB) == string::npos) continue; // 确保包含关键词
        size_t underscorePos = filename.find('_');
        if (underscorePos != string::npos) {
            mapB[stoi(filename.substr(0, underscorePos))] = entry.path().string();
        }
    }

    // 3. 配对逻辑保持不变
    for (int i = 1; i <= maxCount; i++) {
        if (mapA.count(i) && mapB.count(i)) {
            outFilesA.push_back(mapA[i]);
            outFilesB.push_back(mapB[i]);
        }
    }
    return !outFilesA.empty();
}
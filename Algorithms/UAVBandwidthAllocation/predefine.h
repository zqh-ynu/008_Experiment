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
#include <stdexcept>
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


/// Load a complete channel configuration; missing, invalid or unreadable input throws before allocation.
inline void load_global_channel_config(string config_file) {
    std::ifstream input(config_file);
    if (!input) throw std::runtime_error("Cannot open channel configuration: " + config_file);
    json data;
    input >> data;
    const auto& env = data.at("environmental_constants");
    const auto& uav = data.at("uav_constants");
    const double los = env.at("los_loss_db").get<double>();
    const double nlos = env.at("nlos_loss_db").get<double>();
    const double noise = env.at("noise_dbm").get<double>();
    const double a = env.at("param_a").get<double>();
    const double b = env.at("param_b").get<double>();
    const double light = env.at("speed_light").get<double>();
    const int coverage = env.at("max_coverage_distance").get<int>();
    const double altitude = uav.at("uav_alt").get<double>();
    const double frequency = uav.at("freq_ghz").get<double>() * 1e9;
    const double power = uav.at("trans_power").get<double>();
    const double gain = uav.at("gain_uav_db").get<double>();
    for (double value : {los, nlos, noise, a, b, light, altitude, frequency, power, gain})
        if (!std::isfinite(value)) throw std::invalid_argument("Non-finite channel configuration");
    if (a <= 0 || b <= 0 || light <= 0 || coverage <= 0 ||
        altitude <= 0 || frequency <= 0 || power <= 0)
        throw std::invalid_argument("Invalid physical channel configuration");
    los_loss_db = los; nlos_loss_db = nlos; noise_dbm = noise;
    param_a = a; param_b = b; speed_light = light; max_coverage_distance = coverage;
    uav_alt = altitude; freq_hz = frequency; uav_trans_power = power; gain_uav_db = gain;
}

/// Extract a strictly positive numeric instance prefix from <id>_<description>.csv.
inline int allocation_instance_id(const string& path) {
    const string name = fs::path(path).filename().string();
    const auto end = name.find('_');
    if (end == string::npos || end == 0 ||
        name.substr(0, end).find_first_not_of("0123456789") != string::npos)
        throw std::invalid_argument("Invalid instance filename: " + name);
    const unsigned long long id = std::stoull(name.substr(0, end));
    if (id == 0 || id > INT_MAX) throw std::invalid_argument("Instance ID out of range: " + name);
    return static_cast<int>(id);
}

/// Scan readable files, reject duplicate IDs, and return up to maxCount matched pairs in numeric ID order.
/// Missing directories and malformed inputs throw; false means no matched pairs were found.
inline bool getMatchedFilePairs(const string& userDirPath,
    const string& uavDirPath, const string& patternA, const string& patternB,
    int maxCount, vector<string>& outFilesA, vector<string>& outFilesB) {
    outFilesA.clear();
    outFilesB.clear();
    if (maxCount <= 0) throw std::invalid_argument("Pair limit must be positive");
    const fs::path user_path = fs::path(userDirPath) / "user_data";
    const fs::path uav_path = fs::path(uavDirPath) / "uav_data";
    if (!fs::is_directory(user_path) || !fs::is_directory(uav_path))
        throw std::runtime_error("Missing user_data or uav_data directory");
    // Collect the whole directory before selecting a prefix so duplicate IDs cannot be hidden.
    auto collect = [](const fs::path& directory, const string& pattern) {
        map<int, string> files;
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (!entry.is_regular_file() || entry.path().extension() != ".csv") continue;
            if (entry.path().filename().string().find(pattern) == string::npos) continue;
            const string path = entry.path().string();
            const int id = allocation_instance_id(path);
            if (!files.emplace(id, path).second)
                throw std::runtime_error("Duplicate instance ID " + std::to_string(id) + " in " + directory.string());
            std::ifstream check(path, std::ios::binary);
            if (!check) throw std::runtime_error("Unreadable instance file: " + path);
        }
        return files;
    };
    const auto users = collect(user_path, patternA);
    const auto uavs = collect(uav_path, patternB);
    for (const auto& entry : users) {
        const auto found = uavs.find(entry.first);
        if (found == uavs.end()) continue;
        outFilesA.push_back(entry.second);
        outFilesB.push_back(found->second);
        if (outFilesA.size() == static_cast<size_t>(maxCount)) break;
    }
    return !outFilesA.empty();
}

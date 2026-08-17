#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

// 根据操作系统选择不同的头文件
#ifdef _WIN32
    // Windows特定头文件
#include <direct.h>
#include <io.h>
#define stat _stat
#define S_IFDIR _S_IFDIR
#else
    // Linux/Unix特定头文件
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif

// 定义M_PI常量(如果编译器未定义)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 地球半径(米)
const double EARTH_RADIUS = 6378137.0;

// 将角度转换为弧度
inline double toRadians(double degree) {
    return degree * M_PI / 180.0;
}

// 将弧度转换为角度
inline double toDegrees(double radian) {
    return radian * 180.0 / M_PI;
}

// 经纬度转笛卡尔坐标(平面投影)
// 使用墨卡托投影简化版本
// centerLon, centerLat: 参考中心点的经纬度
// lon, lat: 要转换的点的经纬度
// x, y: 输出的笛卡尔坐标(米)
inline void lonLatToCartesian(double centerLon, double centerLat,
    double lon, double lat,
    double& x, double& y) {
    double dLon = toRadians(lon - centerLon);
    double dLat = toRadians(lat - centerLat);

    double centerLatRad = toRadians(centerLat);

    // 使用等距圆柱投影(简化的平面近似)
    x = EARTH_RADIUS * dLon * std::cos(centerLatRad);
    y = EARTH_RADIUS * dLat;
}

// 笛卡尔坐标转经纬度
// centerLon, centerLat: 参考中心点的经纬度
// x, y: 笛卡尔坐标(米)
// lon, lat: 输出的经纬度
inline void cartesianToLonLat(double centerLon, double centerLat,
    double x, double y,
    double& lon, double& lat) {
    double centerLatRad = toRadians(centerLat);

    double dLon = x / (EARTH_RADIUS * std::cos(centerLatRad));
    double dLat = y / EARTH_RADIUS;

    lon = centerLon + toDegrees(dLon);
    lat = centerLat + toDegrees(dLat);
}

// 结构体:存储用户数据
struct UserData {
    int user_id;
    double longitude;
    double latitude;
    std::string user_type;
    double user_weight;
    double user_requirement_1;
    double user_requirement_2;
    std::string app_category;
};

// 结构体:存储UAV数据
struct UAVData {
    int uav_id;
    double longitude;
    double latitude;
    double bandwidth;
};

// 从CSV文件读取用户数据
// 返回值: 读取的用户数据列表
// 同时返回中心经纬度用于坐标转换
inline bool readUsersFromCSV(const std::string& filename,
    std::vector<UserData>& users,
    double& centerLon, double& centerLat) {
    std::ifstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    users.clear();
    std::string line;

    // 跳过表头
    if (!std::getline(file, line)) {
        std::cerr << "文件为空: " << filename << std::endl;
        return false;
    }

    // 首先读取所有用户数据
    double sumLon = 0.0, sumLat = 0.0;
    int count = 0;

    while (std::getline(file, line)) {
        // 跳过空行
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string token;
        UserData user;

        try {
            // 解析CSV行
            std::getline(ss, token, ',');
            user.user_id = std::stoi(token);

            std::getline(ss, token, ',');
            user.longitude = std::stod(token);

            std::getline(ss, token, ',');
            user.latitude = std::stod(token);

            std::getline(ss, token, ',');
            user.user_type = token;

            std::getline(ss, token, ',');
            user.user_weight = std::stod(token);

            std::getline(ss, token, ',');
            user.user_requirement_1 = std::stod(token);

            std::getline(ss, token, ',');
            user.user_requirement_2 = std::stod(token);

            std::getline(ss, token, ',');
            user.app_category = token;

            users.push_back(user);
            sumLon += user.longitude;
            sumLat += user.latitude;
            count++;
        }
        catch (const std::exception& e) {
            std::cerr << "解析行失败: " << line << std::endl;
            std::cerr << "错误: " << e.what() << std::endl;
            continue;
        }
    }

    file.close();

    if (count == 0) {
        std::cerr << "未读取到有效用户数据" << std::endl;
        return false;
    }

    // 计算中心点(所有用户位置的平均值)
    centerLon = sumLon / count;
    centerLat = sumLat / count;

    std::cout << "成功读取 " << count << " 个用户数据" << std::endl;
    std::cout << "参考中心点: (" << centerLon << ", " << centerLat << ")" << std::endl;

    return true;
}

// 将UAV坐标写入CSV文件
inline bool writeUAVsToCSV(const std::string& filename,
    const std::vector<UAVData>& uavs) {
    std::ofstream file(filename.c_str());
    if (!file.is_open()) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return false;
    }

    // 写入表头
    file << "uav_id,longitude,latitude,bandwidth\n";

    // 写入每个UAV的数据
    for (size_t i = 0; i < uavs.size(); ++i) {
        const UAVData& uav = uavs[i];
        file << uav.uav_id << ","
            << uav.longitude << ","
            << uav.latitude << ","
            << uav.bandwidth << "\n";
    }

    file.close();
    std::cout << "成功写入 " << uavs.size() << " 个UAV数据到文件: " << filename << std::endl;

    return true;
}

// 从文件名中提取信息
// 文件名格式: 1_1000users_data_340406.csv
inline bool parseFilename(const std::string& filename,
    int& instanceId,
    int& numUsers,
    std::string& uniqueId) {
    size_t pos1 = filename.find('_');
    if (pos1 == std::string::npos) return false;

    size_t pos2 = filename.find("users_data_", pos1 + 1);
    if (pos2 == std::string::npos) return false;

    size_t pos3 = filename.find(".csv");
    if (pos3 == std::string::npos) return false;

    try {
        instanceId = std::stoi(filename.substr(0, pos1));
        numUsers = std::stoi(filename.substr(pos1 + 1, pos2 - pos1 - 1));
        uniqueId = filename.substr(pos2 + 11, pos3 - pos2 - 11);
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "解析文件名失败: " << filename << std::endl;
        return false;
    }
}

// 生成UAV文件名
// 格式: 1_30uavs_data_340406.csv
inline std::string generateUAVFilename(int instanceId, int numUAVs, const std::string& uniqueId) {
    std::stringstream ss;
    ss << instanceId << "_" << numUAVs << "uavs_data_" << uniqueId << ".csv";
    return ss.str();
}

// 获取目录中所有CSV文件
inline std::vector<std::string> getCSVFilesInDirectory(const std::string& directory) {
    std::vector<std::string> files;

#ifdef _WIN32
    // Windows实现
    std::string searchPath = directory;
    if (!searchPath.empty() && searchPath[searchPath.length() - 1] != '\\' && searchPath[searchPath.length() - 1] != '/') {
        searchPath += "\\";
    }
    searchPath += "*.csv";

    struct _finddata_t fileInfo;
    intptr_t handle = _findfirst(searchPath.c_str(), &fileInfo);

    if (handle == -1) {
        std::cerr << "无法打开目录: " << directory << std::endl;
        return files;
    }

    do {
        std::string filename = fileInfo.name;
        // 只处理CSV文件,且文件名包含"users_data"
        if (filename.find("users_data") != std::string::npos) {
            files.push_back(filename);
        }
    } while (_findnext(handle, &fileInfo) == 0);

    _findclose(handle);
#else
    // Linux/Unix实现
    DIR* dir = opendir(directory.c_str());

    if (dir == NULL) {
        std::cerr << "无法打开目录: " << directory << std::endl;
        return files;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        std::string filename = entry->d_name;

        // 只处理CSV文件,且文件名包含"users_data"
        if (filename.length() > 4 &&
            filename.substr(filename.length() - 4) == ".csv" &&
            filename.find("users_data") != std::string::npos) {
            files.push_back(filename);
        }
    }

    closedir(dir);
#endif

    std::cout << "在目录 " << directory << " 中找到 " << files.size() << " 个用户数据文件" << std::endl;

    return files;
}

// 确保目录存在,如果不存在则创建
inline bool ensureDirectoryExists(const std::string& directory) {
    struct stat info;

    if (stat(directory.c_str(), &info) != 0) {
        // 目录不存在,尝试创建
#ifdef _WIN32
        if (_mkdir(directory.c_str()) != 0) {
#else
        if (mkdir(directory.c_str(), 0755) != 0) {
#endif
            std::cerr << "无法创建目录: " << directory << std::endl;
            return false;
        }
        std::cout << "创建目录: " << directory << std::endl;
        }
    else if (!(info.st_mode & S_IFDIR)) {
        std::cerr << "路径存在但不是目录: " << directory << std::endl;
        return false;
    }

    return true;
    }
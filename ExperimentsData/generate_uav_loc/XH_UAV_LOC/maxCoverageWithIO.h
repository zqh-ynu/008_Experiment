#pragma once
#include "maxCoverageModified.h"
#include "fileIO.h"
#include <iomanip>  // for std::setw, std::setprecision

// 扩展maxUAVCoverage类,添加文件读写功能
class maxUAVCoverageWithIO : public maxUAVCoverage {
private:
    // 参考原点：最小经纬度作为(0,0)点
    double minLongitude;
    double minLatitude;

public:
    // 从CSV文件读取用户位置并初始化
    // 直接使用用户实际坐标，以最小经纬度为原点
    bool loadUsersFromFile(const std::string& filename);

    // 自适应生成候选位置（基于实际用户分布）
    void assignHoverLocationsAdaptive();

    // 将部署的UAV坐标输出到CSV文件
    // bandwidth: 每个UAV的带宽(默认50)
    bool saveUAVsToFile(const std::string& filename, double bandwidth = 50.0);

    // 使用文件初始化(替代原来的initialization)
    void initializationFromFile(const std::string& userFile);

    // 获取参考原点经纬度
    void getReferenceOrigin(double& lon, double& lat) const {
        lon = minLongitude;
        lat = minLatitude;
    }

    // 重写AlgGreedy，修复保存最优解的问题
    int AlgGreedyFixed();
};

// 从CSV文件读取用户位置并初始化
bool maxUAVCoverageWithIO::loadUsersFromFile(const std::string& filename) {
    std::vector<UserData> users;
    double centerLon, centerLat;  // 临时变量，用于readUsersFromCSV

    // 读取用户数据
    if (!readUsersFromCSV(filename, users, centerLon, centerLat)) {
        std::cerr << "读取用户数据失败" << std::endl;
        return false;
    }

    int actualNumUsers = users.size();
    std::cout << "读取到 " << actualNumUsers << " 个用户" << std::endl;

    // ===== 步骤1: 找到最小经纬度作为参考原点(0,0) =====
    minLongitude = 1e9;
    minLatitude = 1e9;
    double maxLon = -1e9;
    double maxLat = -1e9;

    for (size_t i = 0; i < users.size(); ++i) {
        minLongitude = std::min(minLongitude, users[i].longitude);
        minLatitude = std::min(minLatitude, users[i].latitude);
        maxLon = std::max(maxLon, users[i].longitude);
        maxLat = std::max(maxLat, users[i].latitude);
    }

    std::cout << "\n参考原点（最小经纬度）:" << std::endl;
    std::cout << "  经度: " << minLongitude << std::endl;
    std::cout << "  纬度: " << minLatitude << std::endl;
    std::cout << "\n用户经纬度范围:" << std::endl;
    std::cout << "  经度: [" << minLongitude << ", " << maxLon << "]" << std::endl;
    std::cout << "  纬度: [" << minLatitude << ", " << maxLat << "]" << std::endl;

    // ===== 步骤2: 以最小经纬度为原点，转换所有用户坐标 =====
    std::vector<double> userXCoords;
    std::vector<double> userYCoords;

    double maxX = 0, maxY = 0;

    for (size_t i = 0; i < users.size(); ++i) {
        double x, y;
        lonLatToCartesian(minLongitude, minLatitude,
            users[i].longitude, users[i].latitude,
            x, y);

        userXCoords.push_back(x);
        userYCoords.push_back(y);

        maxX = std::max(maxX, x);
        maxY = std::max(maxY, y);
    }

    std::cout << "\n笛卡尔坐标范围（以最小经纬度为原点）:" << std::endl;
    std::cout << "  X: [0, " << maxX << "] 米 (宽度: " << maxX / 1000.0 << " 公里)" << std::endl;
    std::cout << "  Y: [0, " << maxY << "] 米 (高度: " << maxY / 1000.0 << " 公里)" << std::endl;

    // ===== 步骤3: 直接使用用户坐标，每个用户作为一个cluster =====
    for (int i = 0; i < actualNumUsers && i < numClusters; ++i) {
        xCluster[i] = userXCoords[i];
        yCluster[i] = userYCoords[i];
        numUsersPerCluster[i] = 1;  // 每个用户单独一个cluster
    }

    // 如果用户数少于numClusters，剩余cluster设为无用户
    for (int i = actualNumUsers; i < numClusters; ++i) {
        xCluster[i] = 0;
        yCluster[i] = 0;
        numUsersPerCluster[i] = 0;
    }

    std::cout << "\n用户位置加载完成（直接使用实际坐标）" << std::endl;
    std::cout << "有效用户数: " << std::min(actualNumUsers, numClusters) << std::endl;

    return true;
}

// 自适应生成候选位置（基于实际用户分布）
void maxUAVCoverageWithIO::assignHoverLocationsAdaptive() {
    int index = 0;

    // 计算用户分布的边界
    double maxX = 0, maxY = 0;
    for (int i = 0; i < numClusters; ++i) {
        if (numUsersPerCluster[i] > 0) {
            maxX = std::max(maxX, xCluster[i]);
            maxY = std::max(maxY, yCluster[i]);
        }
    }
    maxX = L;
    maxY = W;
    // 添加边距
    double marginX = maxX * 0.15;
    double marginY = maxY * 0.15;
    maxX += marginX;
    maxY += marginY;

    // 候选位置间距（自适应）
    double spacing = std::min(maxX, maxY) / 15.0;
    spacing = std::max(spacing, 200.0);  // 最小200米
    spacing = std::min(spacing, 500.0);  // 最大500米

    std::cout << "\n生成候选位置网格..." << std::endl;
    std::cout << "  场景范围: X=[0, " << maxX << "] Y=[0, " << maxY << "]" << std::endl;
    std::cout << "  网格间距: " << spacing << " 米" << std::endl;

    // 计算网格数量
    int gridX = (int)ceil(maxX / spacing) + 1;
    int gridY = (int)ceil(maxY / spacing) + 1;

    std::cout << "  网格尺寸: " << gridX << " × " << gridY << std::endl;

    // 生成均匀网格
    for (int i = 0; i < gridX && index < numHoverLocations; ++i) {
        for (int j = 0; j < gridY && index < numHoverLocations; ++j) {
            xHoverLoc[index] = i * spacing;
            yHoverLoc[index] = j * spacing;
            index++;
        }
    }

    int gridCount = index;

    // 在用户位置添加候选位置
    for (int i = 0; i < numClusters && index < numHoverLocations; ++i) {
        if (numUsersPerCluster[i] > 0) {
            xHoverLoc[index] = xCluster[i];
            yHoverLoc[index] = yCluster[i];
            index++;
        }
    }

    std::cout << "  网格候选位置: " << gridCount << " 个" << std::endl;
    std::cout << "  用户位置候选: " << (index - gridCount) << " 个" << std::endl;
    std::cout << "  候选位置总数: " << index << " 个" << std::endl;
}

// 将部署的UAV坐标输出到CSV文件
// 将部署的UAV坐标输出到CSV文件
bool maxUAVCoverageWithIO::saveUAVsToFile(const std::string& filename, double bandwidth) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "保存UAV部署信息" << std::endl;
    std::cout << "========================================" << std::endl;

    // 显示部署的UAV数量
    std::cout << "部署的UAV数量: " << numDeployedLocations << std::endl;

    if (numDeployedLocations == 0) {
        std::cerr << "警告: 没有部署任何UAV!" << std::endl;
        return false;
    }

    std::cout << "\nUAV部署位置详情:" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << "UAV_ID | LocationID | X(米) | Y(米) | 经度 | 纬度" << std::endl;
    std::cout << "-------------------------------------------" << std::endl;

    std::vector<UAVData> uavs;

    // 遍历所有部署的UAV
    for (int i = 0; i < numDeployedLocations; ++i) {
        // 获取该UAV的位置ID（在候选位置数组中的索引）
        int locationId = hoverLocations[i];

        // 获取笛卡尔坐标
        double x = xHoverLoc[locationId];
        double y = yHoverLoc[locationId];

        UAVData uav;
        uav.uav_id = i + 1;  // UAV ID从1开始
        uav.bandwidth = bandwidth;

        // 笛卡尔坐标直接转换为经纬度（参考原点是最小经纬度）
        cartesianToLonLat(minLongitude, minLatitude, x, y,
            uav.longitude, uav.latitude);

        // 显示详细信息
        std::cout << std::setw(6) << uav.uav_id << " | "
            << std::setw(10) << locationId << " | "
            << std::setw(9) << std::fixed << std::setprecision(2) << x << " | "
            << std::setw(9) << std::fixed << std::setprecision(2) << y << " | "
            << std::setw(10) << std::fixed << std::setprecision(6) << uav.longitude << " | "
            << std::setw(10) << std::fixed << std::setprecision(6) << uav.latitude
            << std::endl;

        uavs.push_back(uav);
    }

    std::cout << "-------------------------------------------" << std::endl;

    // 统计信息
    double minLon = 1e9, maxLon = -1e9, minLat = 1e9, maxLat = -1e9;
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;

    for (int i = 0; i < numDeployedLocations; ++i) {
        int locationId = hoverLocations[i];
        double x = xHoverLoc[locationId];
        double y = yHoverLoc[locationId];

        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
    }

    for (size_t i = 0; i < uavs.size(); ++i) {
        minLon = std::min(minLon, uavs[i].longitude);
        maxLon = std::max(maxLon, uavs[i].longitude);
        minLat = std::min(minLat, uavs[i].latitude);
        maxLat = std::max(maxLat, uavs[i].latitude);
    }

    std::cout << "\nUAV分布统计:" << std::endl;
    std::cout << "  笛卡尔坐标范围:" << std::endl;
    std::cout << "    X: [" << minX << ", " << maxX << "] 米 (跨度: " << (maxX - minX) << " 米)" << std::endl;
    std::cout << "    Y: [" << minY << ", " << maxY << "] 米 (跨度: " << (maxY - minY) << " 米)" << std::endl;
    std::cout << "  经纬度范围:" << std::endl;
    std::cout << "    经度: [" << std::fixed << std::setprecision(6) << minLon << ", " << maxLon << "]" << std::endl;
    std::cout << "    纬度: [" << minLat << ", " << maxLat << "]" << std::endl;

    // 检查是否排成直线（通过标准差判断）
    double sumLon = 0, sumLat = 0, sumX = 0, sumY = 0;
    for (size_t i = 0; i < uavs.size(); ++i) {
        sumLon += uavs[i].longitude;
        sumLat += uavs[i].latitude;
    }
    for (int i = 0; i < numDeployedLocations; ++i) {
        int locationId = hoverLocations[i];
        sumX += xHoverLoc[locationId];
        sumY += yHoverLoc[locationId];
    }

    double avgLon = sumLon / uavs.size();
    double avgLat = sumLat / uavs.size();
    double avgX = sumX / numDeployedLocations;
    double avgY = sumY / numDeployedLocations;

    double varLon = 0, varLat = 0, varX = 0, varY = 0;
    for (size_t i = 0; i < uavs.size(); ++i) {
        varLon += (uavs[i].longitude - avgLon) * (uavs[i].longitude - avgLon);
        varLat += (uavs[i].latitude - avgLat) * (uavs[i].latitude - avgLat);
    }
    for (int i = 0; i < numDeployedLocations; ++i) {
        int locationId = hoverLocations[i];
        varX += (xHoverLoc[locationId] - avgX) * (xHoverLoc[locationId] - avgX);
        varY += (yHoverLoc[locationId] - avgY) * (yHoverLoc[locationId] - avgY);
    }

    double stdLon = std::sqrt(varLon / uavs.size());
    double stdLat = std::sqrt(varLat / uavs.size());
    double stdX = std::sqrt(varX / numDeployedLocations);
    double stdY = std::sqrt(varY / numDeployedLocations);

    std::cout << "\n  标准差（用于检测是否排成直线）:" << std::endl;
    std::cout << "    经度标准差: " << std::scientific << std::setprecision(6) << stdLon << std::endl;
    std::cout << "    纬度标准差: " << stdLat << std::endl;
    std::cout << "    X标准差: " << std::fixed << std::setprecision(2) << stdX << " 米" << std::endl;
    std::cout << "    Y标准差: " << stdY << " 米" << std::endl;

    // 判断是否排成直线
    if (stdLon < 0.001 || stdLat < 0.001 || stdX < 50 || stdY < 50) {
        std::cout << "\n  ⚠️  警告: UAV可能排成直线!" << std::endl;
        std::cout << "      建议使用 AlgGreedyFixed() 或 ApproAlgShort()" << std::endl;
    }
    else {
        std::cout << "\n  ✓ UAV分布正常（非直线）" << std::endl;
    }

    // 写入文件
    std::cout << "\n正在写入文件: " << filename << std::endl;
    bool success = writeUAVsToCSV(filename, uavs);

    if (success) {
        std::cout << "✓ 成功保存 " << uavs.size() << " 个UAV坐标" << std::endl;
    }
    else {
        std::cerr << "✗ 保存失败!" << std::endl;
    }

    std::cout << "========================================" << std::endl;

    return success;
}

// 使用文件初始化
void maxUAVCoverageWithIO::initializationFromFile(const std::string& userFile) {
    std::cout << "========================================" << std::endl;
    std::cout << "从文件初始化（直接使用用户实际坐标）" << std::endl;
    std::cout << "========================================" << std::endl;

    // 从文件读取用户位置
    if (!loadUsersFromFile(userFile)) {
        std::cerr << "从文件初始化失败!" << std::endl;
        return;
    }

    // 生成自适应候选位置（而不是使用原来的assignHoverLocatoins）
    assignHoverLocations();

    // 其余初始化步骤与原来相同
    findMinHopsAmongLocations();
    findAllShortestPath();
    calUsersCoveredPerLocation();

    std::cout << "\n初始化完成!" << std::endl;
    std::cout << "========================================" << std::endl;
}

inline int maxUAVCoverageWithIO::AlgGreedyFixed()
{
    int i, j, k;
    double profit;
    double dis;

    // 为每个悬停位置分配利润
    for (i = 0; i < numHoverLocations; ++i) {
        profit = 0;
        for (j = 0; j < numClusters; ++j) {
            dis = distance2D(xHoverLoc[i], yHoverLoc[i], xCluster[j], yCluster[j]);
            if (dis <= userCommRadius)
                profit += numUsersPerCluster[j];
        }
        profitEachLocation[i] = profit;
    }

    bool isLabelled[numHoverLocations];
    for (i = 0; i < numHoverLocations; ++i) isLabelled[i] = false;

    bool isCovered[numClusters];
    for (i = 0; i < numClusters; ++i) isCovered[i] = false;

    // 保存最优解的变量
    int bestHoverLocations[numHoverLocations];
    int bestNumDeployedLocations = 0;

    int maxCovered = 0;
    int curCovered;

    for (i = 0; i < numHoverLocations; ++i) {
        numDeployedLocations = 1;
        hoverLocations[0] = i;
        for (j = 0; j < numHoverLocations; ++j) isToDeployUAV[j] = false;
        isToDeployUAV[i] = true;

        extendByGreedyProfitLabel();
        curCovered = calDeployedCoverUsers();

        if (maxCovered < curCovered) {
            maxCovered = curCovered;
            bestNumDeployedLocations = numDeployedLocations;
            for (j = 0; j < numDeployedLocations; ++j) {
                bestHoverLocations[j] = hoverLocations[j];
            }
        }
    }

    // 恢复最优解
    numDeployedLocations = bestNumDeployedLocations;
    for (i = 0; i < bestNumDeployedLocations; ++i) {
        hoverLocations[i] = bestHoverLocations[i];
    }

    return maxCovered;
}

// 批量处理函数:处理目录中的所有实例文件
inline void processBatchInstances(const std::string& inputDir,
    const std::string& outputDir,
    int numUAVsToDeployy = 30) {
    // 确保输出目录存在
    if (!ensureDirectoryExists(outputDir)) {
        std::cerr << "无法创建或访问输出目录" << std::endl;
        return;
    }

    // 获取输入目录中的所有CSV文件
    std::vector<std::string> files = getCSVFilesInDirectory(inputDir);

    if (files.empty()) {
        std::cout << "未找到任何用户数据文件" << std::endl;
        return;
    }

    std::cout << "开始批量处理 " << files.size() << " 个实例文件..." << std::endl;
    std::cout << "============================================" << std::endl;

    int successCount = 0;
    int failCount = 0;

    for (size_t i = 0; i < files.size(); ++i) {
        std::string userFile = files[i];
        std::cout << "\n[" << (i + 1) << "/" << files.size() << "] 处理文件: " << userFile << std::endl;

        // 解析文件名
        int instanceId, numUsers;
        std::string uniqueId;

        if (!parseFilename(userFile, instanceId, numUsers, uniqueId)) {
            std::cerr << "解析文件名失败,跳过此文件" << std::endl;
            failCount++;
            continue;
        }

        std::cout << "实例ID: " << instanceId
            << ", 用户数: " << numUsers
            << ", 唯一ID: " << uniqueId << std::endl;

        try {
            // 创建maxUAVCoverage对象
            maxUAVCoverageWithIO maxCoverage;

            // 从文件初始化
            std::string inputPath = inputDir + "/" + userFile;
            maxCoverage.initializationFromFile(inputPath);

            // 运行算法(这里使用ApproAlgShort作为示例)
            std::cout << "运行算法..." << std::endl;
            int numUsersCovered = maxCoverage.ApproAlgShort();
            std::cout << "覆盖用户数: " << numUsersCovered << std::endl;

            // 生成输出文件名
            std::string uavFilename = generateUAVFilename(instanceId, numUAVsToDeployy, uniqueId);
            std::string outputPath = outputDir + "/" + uavFilename;

            // 保存UAV坐标
            if (maxCoverage.saveUAVsToFile(outputPath, 50.0)) {
                std::cout << "成功处理并保存到: " << outputPath << std::endl;
                successCount++;
            }
            else {
                std::cerr << "保存UAV坐标失败" << std::endl;
                failCount++;
            }
        }
        catch (const std::exception& e) {
            std::cerr << "处理文件时发生错误: " << e.what() << std::endl;
            failCount++;
        }
    }

    std::cout << "\n============================================" << std::endl;
    std::cout << "批量处理完成!" << std::endl;
    std::cout << "成功: " << successCount << " 个文件" << std::endl;
    std::cout << "失败: " << failCount << " 个文件" << std::endl;
}
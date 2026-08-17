#include <stdio.h>
#include <iostream>
#include <string>
#include "maxCoverageWithIO.h"

using namespace std;

void printUsage() {
    cout << "使用方法:" << endl;
    cout << "1. 单文件模式:" << endl;
    cout << "   ./program single <用户数据文件> <UAV输出文件> [UAV数量]" << endl;
    cout << "   示例: ./program single input/1_1000users_data_340406.csv output/1_30uavs_data_340406.csv 30" << endl;
    cout << endl;
    cout << "2. 批量处理模式:" << endl;
    cout << "   ./program batch <输入目录> <输出目录> [UAV数量]" << endl;
    cout << "   示例: ./program batch input_data/ output_data/ 30" << endl;
    cout << endl;
}

// 单文件处理模式
void processSingleFile(const string& inputFile, const string& outputFile, int numUAVsLocal = 30) {
    cout << "========================================" << endl;
    cout << "单文件处理模式" << endl;
    cout << "========================================" << endl;
    cout << "输入文件: " << inputFile << endl;
    cout << "输出文件: " << outputFile << endl;
    cout << "UAV数量: " << numUAVsLocal << endl;
    cout << endl;

    try {
        // 创建对象
        maxUAVCoverageWithIO maxCoverage;

        // 从文件初始化
        cout << "正在从文件加载用户数据..." << endl;
        maxCoverage.initializationFromFile(inputFile);

        // 运行算法
        cout << "\n正在运行算法..." << endl;
        clock_t start = clock();
        int numUsersCovered = maxCoverage.AlgGreedyFixed();
        clock_t end = clock();
        double timeUsed = (end - start) / 1000.0;

        cout << "\n算法运行完成!" << endl;
        cout << "覆盖用户数: " << numUsersCovered << endl;
        cout << "运行时间: " << timeUsed << " 秒" << endl;

        // 保存UAV坐标
        cout << "\n正在保存UAV坐标..." << endl;
        if (maxCoverage.saveUAVsToFile(outputFile, 40.0)) {
            cout << "UAV坐标已成功保存到: " << outputFile << endl;
        }
        else {
            cerr << "保存UAV坐标失败!" << endl;
        }

        cout << "\n处理完成!" << endl;
        cout << "========================================" << endl;
    }
    catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
    }
}

string input_dir = "E:\\Research\\My paper\\2_Papers\\008\\008_Experiment\\ExperimentsData\\data\\variable_user_num\\";
string output_dir = "E:\\Research\\My paper\\2_Papers\\008\\008_Experiment\\ExperimentsData\\data\\variable_user_num\\";

void test_processSingleFile()
{
    string input_File = input_dir + "1000u_num\\user_data\\1_1000users_data_340406.csv";
    string output_File = output_dir + "1000u_num\\uav_data\\1_30uavs_data_340406.csv";
    int numUAVsLocal = 30;
    processSingleFile(input_File, output_File, numUAVsLocal);

}

int main() {
    cout << "UAV最大覆盖问题求解器(带文件I/O)" << endl;
    cout << "========================================" << endl;
    cout << endl;


    test_processSingleFile();
    //else if (mode == "batch") {
    //    // 批量处理模式
    //    if (argc < 4) {
    //        cerr << "参数不足!" << endl;
    //        printUsage();
    //        return 1;
    //    }

    //    string inputDir = argv[2];
    //    string outputDir = argv[3];
    //    int numUAVsLocal = (argc >= 5) ? atoi(argv[4]) : 30;

    //    cout << "批量处理模式" << endl;
    //    cout << "输入目录: " << inputDir << endl;
    //    cout << "输出目录: " << outputDir << endl;
    //    cout << "UAV数量: " << numUAVsLocal << endl;
    //    cout << endl;

    //    processBatchInstances(inputDir, outputDir, numUAVsLocal);
    //}
    //else {
    //    cerr << "未知模式: " << mode << endl;
    //    printUsage();
    //    return 1;
    //}

    return 0;
}
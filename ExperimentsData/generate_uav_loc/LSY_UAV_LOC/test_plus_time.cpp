#include<stdio.h>
#include<assert.h>
#include<math.h>
#include<time.h>
#include<iostream>
#include<fstream>
#include<string>
#include<windows.h> // 用于遍历文件
#include"maxCovHeteroFast.h"

using namespace std;

// 注意：如果路径包含中文字符，在ANSI模式下可能需要系统区域设置支持，建议路径尽量使用英文
string DataDir = "E:\\Research\\My paper\\2_Papers\\008\\008_Experiment\\ExperimentsData\\data\\County_loc_Type_req\\simulated_data\\";

int main()
{
    maxUAVCoverage maxCoverage;

    // 查找目录下所有csv文件
    string searchPath = DataDir + "*.csv";

    // 【修改点1】使用 WIN32_FIND_DATAA (ANSI版本结构体)
    WIN32_FIND_DATAA ffd;

    // 【修改点2】使用 FindFirstFileA (ANSI版本函数)
    HANDLE hFind = FindFirstFileA(searchPath.c_str(), &ffd);

    if (hFind == INVALID_HANDLE_VALUE) {
        cout << "No csv files found in " << DataDir << endl;
        return 0;
    }

    do {
        // 【修改点3】ffd.cFileName 现在是 char 数组，可以直接赋值给 string
        string inputFile = ffd.cFileName;

        // 排除已经生成的输出文件 (包含 "uavs_loc") 以免死循环或重复处理
        if (inputFile.find("uavs_loc") != string::npos) continue;

        cout << "Processing: " << inputFile << "..." << endl;

        // 生成输出文件名
        string outputFile = inputFile;
        string target = "users_data";
        string replacement = "20uavs_loc";

        size_t pos = outputFile.find(target);
        if (pos != string::npos) {
            // 寻找前一个下划线
            size_t prev_underscore = outputFile.rfind('_', pos - 1);
            if (prev_underscore != string::npos) {
                // 替换 "1000users_data" -> "20uavs_loc"
                outputFile.replace(prev_underscore + 1, pos + target.length() - (prev_underscore + 1), replacement);
            }
            else {
                outputFile.replace(pos, target.length(), replacement);
            }
        }
        else {
            outputFile = "out_" + inputFile;
        }

        string fullInputPath = DataDir + inputFile;
        string fullOutputPath = DataDir + outputFile;

        // 初始化并运行
        srand(1);
        maxCoverage.initialization(fullInputPath);

        maxCoverage.ApproAlgCombine(fullOutputPath);

        cout << "Finished " << inputFile << " -> " << outputFile << endl << endl;

        // 【修改点4】使用 FindNextFileA
    } while (FindNextFileA(hFind, &ffd) != 0);

    FindClose(hFind);
    return 0;
}
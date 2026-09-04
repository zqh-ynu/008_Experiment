#pragma once
// 本文件负责实验调度、断点结果持久化以及按算法版本隔离导出结果。
#include "EntityDefinition.h"
#include "config.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <chrono>
#include <stdexcept>

namespace fs = std::filesystem;

// 本文件主要记录实验相关内容
struct EXPResult {
	double duration = 0;         // 运行时间(ms)

	int total_num = 0;           // 服务总人数
	int hard_num = 0;            // 服务hard人数
	int elastic_num = 0;         // 服务elastic人数

	double total_utility = 0;    // 累计效用
	double hard_utility = 0;     // hard用户贡献的效用
	double elastic_utility = 0;  // elastic用户贡献的效用 (修正拼写: utitily -> utility)

	double hard_bandwidth = 0;   // hard用户消耗的总带宽
	double elastic_bandwidth = 0;// elastic用户消耗的总带宽

	double hard_throughput = 0;  // hard用户总吞吐量
	double elastic_throughput = 0;  // elastic用户总吞吐量
	double total_throughput = 0; // 总吞吐量
};

// ======================== CSV 列头定义 ========================
const string CSV_HEADER = "duration,total_num,hard_num,elastic_num,"
"total_utility,hard_utility,elastic_utility,"
"hard_bandwidth,elastic_bandwidth,"
"hard_throughput,elastic_throughput,total_throughput";

// ======================== 全局配置 ========================
vector<string> method_name_list = {
	"ApproBetter", "ApproFast", "AlgDRL",
	"AlgMatching", "AlgHardFirst", "AlgSADA"
};

// 所有 ToN 运行都写入独立版本目录，避免覆盖、追加或混用历史 MASS 结果。
const string TON_RESULT_VERSION_DIR = "ToN_marginal_eps0p1/";

// 核心数据结构：Key 是实验条件，Value 是各个算法对应的平均结果
map<string, vector<EXPResult>> experiment_data;

// ======================== EXPResult 序列化/反序列化 ========================

/** @brief 将一个 EXPResult 写为一行 CSV（不含换行） */
string expResultToCSVLine(const EXPResult& r) {
	ostringstream oss;
	oss << r.duration << ","
		<< r.total_num << "," << r.hard_num << "," << r.elastic_num << ","
		<< r.total_utility << "," << r.hard_utility << "," << r.elastic_utility << ","
		<< r.hard_bandwidth << "," << r.elastic_bandwidth << ","
		<< r.hard_throughput << "," << r.elastic_throughput << "," << r.total_throughput;
	return oss.str();
}

/** @brief 从一行 CSV 解析出一个 EXPResult */
bool parseCSVLineToEXPResult(const string& line, EXPResult& r) {
	istringstream iss(line);
	string token;
	try {
		getline(iss, token, ','); r.duration = stod(token);
		getline(iss, token, ','); r.total_num = stoi(token);
		getline(iss, token, ','); r.hard_num = stoi(token);
		getline(iss, token, ','); r.elastic_num = stoi(token);
		getline(iss, token, ','); r.total_utility = stod(token);
		getline(iss, token, ','); r.hard_utility = stod(token);
		getline(iss, token, ','); r.elastic_utility = stod(token);
		getline(iss, token, ','); r.hard_bandwidth = stod(token);
		getline(iss, token, ','); r.elastic_bandwidth = stod(token);
		getline(iss, token, ','); r.hard_throughput = stod(token);
		getline(iss, token, ','); r.elastic_throughput = stod(token);
		getline(iss, token, ','); r.total_throughput = stod(token);
		return true;
	}
	catch (...) {
		return false;
	}
}

// ======================== 中间结果持久化 ========================

/**
 * @brief 确保中间结果目录和CSV文件存在（含表头）
 * @param dir 目录路径，如 EXP_result_path/1000/
 */
void ensureIntermediateDir(const string& dir) {
	fs::create_directories(dir);
	for (const auto& method : method_name_list) {
		string filepath = dir + method + ".csv";
		if (!fs::exists(filepath)) {
			ofstream f(filepath);
			f << CSV_HEADER << "\n";
			f.close();
		}
	}
}

/**
 * @brief 读取某个中间CSV已有的数据行数（不含表头），用于断点续跑
 * @return 已完成的实例数
 */
int countCompletedRows(const string& filepath) {
	if (!fs::exists(filepath)) return 0;
	ifstream file(filepath);
	string line;
	int count = 0;
	bool header_skipped = false;
	while (getline(file, line)) {
		if (!header_skipped) { header_skipped = true; continue; }
		if (!line.empty()) count++;
	}
	return count;
}

/**
 * @brief 核对六种方法的 CSV 行数，并返回一致的已完成实例数。
 * @throws std::runtime_error 当六个方法文件的数据行数不一致时立即停止续跑。
 */
int getCompletedCount(const string& dir) {
	// 先完整收集各方法行数，异常时可以把每个文件的实际状态一并报告给用户。
	vector<pair<string, int>> row_counts;
	row_counts.reserve(method_name_list.size());
	for (const string& method : method_name_list) {
		string filepath = dir + method + ".csv";
		row_counts.push_back({ method, countCompletedRows(filepath) });
	}

	int expected_count = row_counts.empty() ? 0 : row_counts.front().second;
	bool consistent = true;
	for (const auto& entry : row_counts) {
		if (entry.second != expected_count) {
			consistent = false;
			break;
		}
	}
	if (!consistent) {
		cerr << "ERROR: checkpoint CSV row counts are inconsistent in " << dir << "\n";
		for (const auto& entry : row_counts)
			cerr << "  " << entry.first << ".csv: " << entry.second << " data rows\n";
		throw runtime_error(
			"Checkpoint stopped: method CSV row counts differ; no files were modified");
	}
	return expected_count;
}

/**
 * @brief 向中间CSV追加一行结果（一个实例的数据）
 * @param dir       目录路径
 * @param method_idx 方法索引
 * @param result    本次实例的结果
 */
void appendResult(const string& dir, int method_idx, const EXPResult& result) {
	string filepath = dir + method_name_list[method_idx] + ".csv";
	ofstream file(filepath, ios::app);
	if (!file.is_open()) {
		cerr << "Error: Could not open file for appending: " << filepath << endl;
		return;
	}
	file << expResultToCSVLine(result) << "\n";
	file.close();
}

/**
 * @brief 从中间CSV读取所有实例结果
 * @param dir 目录路径
 * @param method_idx 方法索引
 * @return 所有实例的EXPResult列表
 */
vector<EXPResult> loadIntermediateResults(const string& dir, int method_idx) {
	vector<EXPResult> results;
	string filepath = dir + method_name_list[method_idx] + ".csv";
	if (!fs::exists(filepath)) return results;

	ifstream file(filepath);
	string line;
	bool header_skipped = false;
	while (getline(file, line)) {
		if (!header_skipped) { header_skipped = true; continue; }
		if (line.empty()) continue;
		EXPResult r;
		if (parseCSVLineToEXPResult(line, r)) {
			results.push_back(r);
		}
	}
	return results;
}

// ======================== 计算单实例结果 ========================

/**
 * @brief 从单次算法运行结果计算EXPResult（不累加到全局，返回独立结果）
 */
EXPResult compute_single_EXPResult(SystemMd& sysModel, vector<KnapsackResult>& uav_results, double duration_ms) {
	EXPResult result;
	result.duration = duration_ms;

	for (auto& kr : uav_results) {
		int uav_id = kr.uav_id;

		result.total_num += kr.allocatedList.size();
		result.total_utility += kr.totalValue;
		result.hard_utility += kr.hardValue;
		result.elastic_utility += kr.elasticValue;
		result.hard_bandwidth += kr.hardWeight;
		result.elastic_bandwidth += kr.elasticWeight;

		for (auto user_id : kr.allocatedList) {
			// 使用 .at() 并捕获 std::out_of_range，这样 vector/map 的越界会抛出并被捕获，便于记录调试信息
			try {
				// 可能越界的访问：sysModel.users[user_id]
				User& user = sysModel.users.at(user_id);

				// 可能越界的访问：kr.allocatedBandwidth[user_id]
				// 假设 allocatedBandwidth 支持 at (vector/map/unordered_map 都有 at)
				double bandwidth = kr.allocatedBandwidth.at(user_id);

				// 可能越界的访问：sysModel.cap_list[uav_id][user_id]
				double cap = sysModel.cap_list.at(uav_id).at(user_id);

				if (user.uType == HARD_UTILITY) {
					result.hard_num++;
					result.hard_throughput += user.rMin;
					result.total_throughput += user.rMin;
				}
				else {
					result.elastic_num++;
					double throughput = bandwidth * cap;
					result.elastic_throughput += throughput;
					result.total_throughput += throughput;
				}
			}
			catch (const std::out_of_range& e) {
				// 详细记录上下文，帮助定位是哪个容器/索引越界
				cerr << "ERROR: out_of_range in compute_single_EXPResult: uav_id=" << uav_id
					<< ", user_id=" << user_id << "\n";
				cerr << "  exception: " << e.what() << "\n";

				// 打印相关容器尺寸与一些样本值，便于快速诊断
				cerr << "  sysModel.users.size() = " << sysModel.users.size() << "\n";
				cerr << "  sysModel.cap_list.size() = " << sysModel.cap_list.size() << "\n";
				if (uav_id >= 0 && uav_id < static_cast<int>(sysModel.cap_list.size())) {
					cerr << "  sysModel.cap_list[" << uav_id << "].size() = "
						<< sysModel.cap_list.at(uav_id).size() << "\n";
				}
				cerr << "  kr.allocatedList.size() = " << kr.allocatedList.size() << "\n";
				cerr << "  kr.allocatedBandwidth.size() = " << kr.allocatedBandwidth.size() << "\n";

				// 打印 allocatedList 前若干项，帮助判断 user_id 是否为意外的大数或非索引值
				cerr << "  allocatedList (first up to 10 entries): ";
				for (size_t i = 0; i < kr.allocatedList.size() && i < 10; ++i) {
					cerr << kr.allocatedList[i] << " ";
				}
				cerr << "\n";

				// 为了继续处理其它正确的分配，跳过当前出错的 user
				continue;
			}
			catch (const std::exception& e) {
				// 捕获其它可能的异常并记录
				cerr << "ERROR: exception in compute_single_EXPResult: uav_id=" << uav_id
					<< ", user_id=" << user_id << ", what=" << e.what() << "\n";
				continue;
			}
		}
	}
	return result;
}

// ======================== 汇总逻辑 ========================

/**
 * @brief 从中间CSV汇总计算平均结果，填充到 experiment_data
 * @param condition_key 实验条件key
 * @param intermediate_dir 中间结果目录
 */
void summarizeFromIntermediate(const string& condition_key, const string& intermediate_dir) {
	experiment_data[condition_key] = vector<EXPResult>(method_name_list.size());

	for (size_t m = 0; m < method_name_list.size(); ++m) {
		vector<EXPResult> instances = loadIntermediateResults(intermediate_dir, m);
		if (instances.empty()) continue;

		EXPResult& avg = experiment_data[condition_key][m];
		int count = instances.size();

		for (const auto& inst : instances) {
			avg.duration += inst.duration;
			avg.total_num += inst.total_num;
			avg.hard_num += inst.hard_num;
			avg.elastic_num += inst.elastic_num;
			avg.total_utility += inst.total_utility;
			avg.hard_utility += inst.hard_utility;
			avg.elastic_utility += inst.elastic_utility;
			avg.hard_bandwidth += inst.hard_bandwidth;
			avg.elastic_bandwidth += inst.elastic_bandwidth;
			avg.hard_throughput += inst.hard_throughput;
			avg.elastic_throughput += inst.elastic_throughput;
			avg.total_throughput += inst.total_throughput;
		}

		avg.duration /= count;
		avg.total_num /= count;
		avg.hard_num /= count;
		avg.elastic_num /= count;
		avg.total_utility /= count;
		avg.hard_utility /= count;
		avg.elastic_utility /= count;
		avg.hard_bandwidth /= count;
		avg.elastic_bandwidth /= count;
		avg.hard_throughput /= count;
		avg.elastic_throughput /= count;
		avg.total_throughput /= count;
	}
}

// ======================== 最终汇总CSV导出 ========================

/**
 * @brief 导出特定指标到 CSV（与原有接口兼容）
 */
void exportToCSV(const string& filename, vector<string>& first_key_lists,
	double EXPResult::* metric_ptr_double = nullptr,
	int EXPResult::* metric_ptr_int = nullptr) {
	ofstream file(filename);
	if (!file.is_open()) {
		cerr << "Error: Could not open file " << filename << endl;
		return;
	}

	// 写入表头
	file << "User_Scale";
	for (const auto& method : method_name_list) {
		file << "," << method;
	}
	file << "\n";

	for (const auto& first_key : first_key_lists) {
		if (experiment_data.find(first_key) == experiment_data.end()) continue;

		file << first_key;
		const vector<EXPResult>& results = experiment_data[first_key];

		for (size_t i = 0; i < method_name_list.size(); ++i) {
			file << ",";
			if (i < results.size()) {
				if (metric_ptr_double) file << results[i].*metric_ptr_double;
				else if (metric_ptr_int) file << results[i].*metric_ptr_int;
			}
			else {
				file << "0";
			}
		}
		file << "\n";
	}

	file.close();
	cout << "Successfully exported: " << filename << endl;
}

// ======================== 核心执行逻辑（支持断点续跑 + 逐实例持久化） ========================

/**
 * @brief 对一个实验条件运行所有实例，支持断点续跑
 * @param intermediate_dir 中间结果目录（如 EXP_result_path/1000/）
 * @param userFiles        用户数据文件列表
 * @param uavFiles         无人机数据文件列表
 * @param config_file      配置文件路径
 * @param total_count      总实例数
 */
void run_Instance_with_checkpoint(const string& intermediate_dir,
	vector<string>& userFiles,
	vector<string>& uavFiles,
	const string& config_file,
	int total_count, double total_bandwidth = 40) {
	// 必须先检查六个文件、再创建缺失文件：若 checkpoint 不完整，先原样报告磁盘证据，
	// 不通过自动补文件、删行或截断来掩盖不一致状态。
	int completed = getCompletedCount(intermediate_dir);
	ensureIntermediateDir(intermediate_dir);
	if (completed >= total_count) {
		cout << "  All " << total_count << " instances already completed, skipping." << endl;
		return;
	}
	if (completed > 0) {
		cout << "  Resuming from instance " << (completed + 1)
			<< " (" << completed << " already completed)" << endl;
	}

	total_bandwidth *= unit_para; // 带宽默认单位是MHz，unit_para = 1时为MHz，为1000时为KHz

	// 从断点处开始执行
	for (int k = completed; k < total_count; ++k) {
		cout << " Processing instance " << (k + 1) << "/" << total_count << "..." << endl;

		SystemMd sysModel(userFiles[k], uavFiles[k], config_file);
		for (auto& uav : sysModel.uavs)
		{
			uav.total_bandwidth = total_bandwidth;
		}
		// ---------- 方法0: ApproBetter -> ToN Algorithm 2，显式使用 epsilon=0.1 ----------
		{
			cout << "\t[" << method_name_list[0] << "]..." << endl;
			BAProblem problem(sysModel);
			auto t0 = chrono::high_resolution_clock::now();
			auto results = problem.Appro_multiUAV_ToN(
				sysModel.uavs, sysModel.users, 2, 0.1);
			auto t1 = chrono::high_resolution_clock::now();
			double ms = chrono::duration_cast<chrono::milliseconds>(t1 - t0).count();
			auto uavResults = results.first;
			EXPResult r = compute_single_EXPResult(problem.sysModel, uavResults, ms);
			appendResult(intermediate_dir, 0, r);
		}

		// ---------- 方法1: ApproFast -> ToN Algorithm 1；epsilon 仅用于统一接口校验 ----------
		{
			cout << "\t[" << method_name_list[1] << "]..." << endl;
			BAProblem problem(sysModel);
			auto t0 = chrono::high_resolution_clock::now();
			auto results = problem.Appro_multiUAV_ToN(
				sysModel.uavs, sysModel.users, 1, 0.1);
			auto t1 = chrono::high_resolution_clock::now();
			double ms = chrono::duration_cast<chrono::milliseconds>(t1 - t0).count();
			auto uavResults = results.first;
			EXPResult r = compute_single_EXPResult(problem.sysModel, uavResults, ms);
			appendResult(intermediate_dir, 1, r);
		}

		// ---------- 方法2: relaxRoundAlg ----------
		{
			cout << "\t[" << method_name_list[2] << "]..." << endl;
			BAProblem problem(sysModel);
			auto t0 = chrono::high_resolution_clock::now();
			auto results = problem.ConvexRelaxationAndRounding_multiUAV();
			auto t1 = chrono::high_resolution_clock::now();
			double ms = chrono::duration_cast<chrono::milliseconds>(t1 - t0).count();
			auto uavResults = results.first;
			EXPResult r = compute_single_EXPResult(problem.sysModel, uavResults, ms);
			appendResult(intermediate_dir, 2, r);
		}

		// ---------- 方法3: matchSQPAlg ----------
		{
			cout << "\t[" << method_name_list[3] << "]..." << endl;
			BAProblem problem(sysModel);
			auto t0 = chrono::high_resolution_clock::now();
			auto results = problem.MatchingSQP_Allocation();
			auto t1 = chrono::high_resolution_clock::now();
			double ms = chrono::duration_cast<chrono::milliseconds>(t1 - t0).count();
			auto uavResults = results.first;
			EXPResult r = compute_single_EXPResult(problem.sysModel, uavResults, ms);
			appendResult(intermediate_dir, 3, r);
		}

		// ---------- 方法4: HungarianMatchingAllocation ----------
		{
			cout << "\t[" << method_name_list[4] << "]..." << endl;
			BAProblem problem(sysModel);
			auto t0 = chrono::high_resolution_clock::now();
			auto results = problem.HungarianMatchingAllocation();
			auto t1 = chrono::high_resolution_clock::now();
			double ms = chrono::duration_cast<chrono::milliseconds>(t1 - t0).count();
			auto uavResults = results.first;
			EXPResult r = compute_single_EXPResult(problem.sysModel, uavResults, ms);
			appendResult(intermediate_dir, 4, r);
		}

		// ---------- 方法5: iterApproAlg ----------
		{
			cout << "\t[" << method_name_list[5] << "]..." << endl;
			BAProblem problem(sysModel);
			auto t0 = chrono::high_resolution_clock::now();
			auto results = problem.SADA_Allocation();
			auto t1 = chrono::high_resolution_clock::now();
			double ms = chrono::duration_cast<chrono::milliseconds>(t1 - t0).count();
			auto uavResults = results.first;
			EXPResult r = compute_single_EXPResult(problem.sysModel, uavResults, ms);
			appendResult(intermediate_dir, 5, r);
		}

		cout << "  Instance " << (k + 1) << " done and saved." << endl;
	}
}

// ======================== 导出所有汇总CSV ========================

void exportAllSummaryCSV(const string& summary_dir, vector<string>& condition_keys) {
	fs::create_directories(summary_dir);

	exportToCSV(summary_dir + "Run_time_ms.csv", condition_keys, &EXPResult::duration);

	exportToCSV(summary_dir + "Total_Num.csv", condition_keys, nullptr, &EXPResult::total_num);
	exportToCSV(summary_dir + "Hard_Num.csv", condition_keys, nullptr, &EXPResult::hard_num);
	exportToCSV(summary_dir + "Elastic_Num.csv", condition_keys, nullptr, &EXPResult::elastic_num);

	exportToCSV(summary_dir + "Total_Utility.csv", condition_keys, &EXPResult::total_utility);
	exportToCSV(summary_dir + "Hard_Utility.csv", condition_keys, &EXPResult::hard_utility);
	exportToCSV(summary_dir + "Elastic_Utility.csv", condition_keys, &EXPResult::elastic_utility);

	exportToCSV(summary_dir + "Hard_Bandwidth.csv", condition_keys, &EXPResult::hard_bandwidth);
	exportToCSV(summary_dir + "Elastic_Bandwidth.csv", condition_keys, &EXPResult::elastic_bandwidth);

	exportToCSV(summary_dir + "Hard_Throughput.csv", condition_keys, &EXPResult::hard_throughput);
	exportToCSV(summary_dir + "Elastic_Throughput.csv", condition_keys, &EXPResult::elastic_throughput);
	exportToCSV(summary_dir + "Total_Throughput.csv", condition_keys, &EXPResult::total_throughput);
}

// ======================== 实验一（重构版） ========================

void exp1_different_user_number() {
	cout << "========================================" << endl;
	cout << "EXP1: The impact of different user numbers on algorithm performance." << endl;
	cout << "========================================" << endl;

	// 1. 路径配置
	string config_file = experimentDataPath + "ExperimentsResults/EXP1_user_num/def_config.json";
	string dataSetPath = experimentDataPath + "data/variable_user_num/";
	string EXP_result_path = experimentDataPath +
		"ExperimentsResults/EXP1_user_num/" + TON_RESULT_VERSION_DIR;

	vector<string> user_num_dirs = { "1000", "2000", "3000", "4000", "5000" };
	string uav_num = "10";

	// 2. 逐条件执行（支持断点续跑）
	for (auto& u_num_str : user_num_dirs) {
		cout << "\n--- Condition: user_num = " << u_num_str << " ---" << endl;

		string dataFilePath = dataSetPath + u_num_str + "u_num/";
		string intermediate_dir = EXP_result_path + u_num_str + "/";

		vector<string> userFiles;
		vector<string> uavFiles;

		bool success = getMatchedFilePairs(
			dataFilePath,
			dataFilePath,
			u_num_str + "users_data",
			uav_num + "uavs_loc",
			50,
			userFiles,
			uavFiles
		);
		if (!success) {
			cout << "Failed to load data files for condition " << u_num_str << endl;
			continue;
		}
		cout << "Loaded " << userFiles.size() << " pairs of data files." << endl;

		//int count = userFiles.size();
		int count = 10;
		run_Instance_with_checkpoint(intermediate_dir, userFiles, uavFiles, config_file, count);
	}

	// 3. 汇总：从中间CSV读取 -> 计算平均 -> 填充 experiment_data
	cout << "\n--- Summarizing results ---" << endl;
	for (auto& u_num_str : user_num_dirs) {
		string intermediate_dir = EXP_result_path + u_num_str + "/";
		summarizeFromIntermediate(u_num_str, intermediate_dir);
	}

	// 4. 导出最终汇总CSV
	string summary_dir = EXP_result_path + "summary/";
	exportAllSummaryCSV(summary_dir, user_num_dirs);

	cout << "\nEXP1 completed." << endl;
}

void exp2_different_uav_number() {
	cout << "========================================" << endl;
	cout << "EXP2: The impact of different UAV numbers." << endl;
	cout << "========================================" << endl;

	string config_file = experimentDataPath + "ExperimentsResults/EXP2_uav_num/def_config.json";
	// 用户数据路径：固定在 3000u_num 文件夹
	string fixedUserPath = experimentDataPath + "data/variable_user_num/3000u_num/";
	// 无人机数据根路径
	string uavBaseSetPath = experimentDataPath + "data/variable_uav_num/";
	string EXP_result_path = experimentDataPath +
		"ExperimentsResults/EXP2_uav_num/" + TON_RESULT_VERSION_DIR;

	vector<string> uav_num_dirs = { "5", "10", "15", "20" };
	string user_pattern = "3000users_data";

	for (auto& uav_num_str : uav_num_dirs) {
		cout << "\n--- Condition: uav_num = " << uav_num_str << " ---" << endl;

		string uavDataPath = uavBaseSetPath + uav_num_str + "/";
		string intermediate_dir = EXP_result_path + uav_num_str + "/";

		vector<string> userFiles, uavFiles;

		// 调用修改后的原函数：传入不同的用户目录和无人机目录
		bool success = getMatchedFilePairs(
			fixedUserPath,
			uavDataPath,
			user_pattern,
			uav_num_str + "uavs_loc",
			50,
			userFiles,
			uavFiles
		);

		if (!success) {
			cout << "Failed to load data for UAV count: " << uav_num_str << endl;
			continue;
		}
		//int count = userFiles.size();
		int count = 10;
		// 运行实验
		run_Instance_with_checkpoint(intermediate_dir, userFiles, uavFiles, config_file, count);
	}

	// 汇总与导出（逻辑与 EXP1 一致）
	for (auto& uav_num_str : uav_num_dirs) {
		summarizeFromIntermediate(uav_num_str, EXP_result_path + uav_num_str + "/");
	}
	exportAllSummaryCSV(EXP_result_path + "summary/", uav_num_dirs);
}


void exp3_different_hard_user_ratio() {
	cout << "========================================" << endl;
	cout << "EXP2: The impact of different ratio of hard users." << endl;
	cout << "========================================" << endl;

	string config_file = experimentDataPath + "ExperimentsResults/EXP3_hard_user_ratio/def_config.json";


	// 数据根路径
	string dataSetPath = experimentDataPath + "data/variable_hard_user_ratio/";
	string EXP_result_path = experimentDataPath +
		"ExperimentsResults/EXP3_hard_user_ratio/" + TON_RESULT_VERSION_DIR;

	vector<string> hard_ratio_dirs = { "0", "2", "4", "6", "8", "10"};
	string uav_num = "10";
	string user_pattern = "3000users_data";
	string uav_pattern = "10uavs_loc";


	for (auto& hard_ratio : hard_ratio_dirs) {
		cout << "\n--- Condition: hard_ratio = " << hard_ratio << " ---" << endl;

		string DataPath = dataSetPath + hard_ratio + "/";
		string intermediate_dir = EXP_result_path + hard_ratio + "/";

		vector<string> userFiles, uavFiles;

		// 调用修改后的原函数：传入不同的用户目录和无人机目录
		bool success = getMatchedFilePairs(
			DataPath,
			DataPath,
			user_pattern,
			uav_pattern,
			30,
			userFiles,
			uavFiles
		);

		if (!success) {
			cout << "Failed to load data for hard ratio: " << hard_ratio << endl;
			continue;
		}
		//int count = userFiles.size();
		int count = 10;
		// 运行实验
		run_Instance_with_checkpoint(intermediate_dir, userFiles, uavFiles, config_file, count);
	}

	// 汇总与导出（逻辑与 EXP1 一致）
	for (auto& hard_ratio : hard_ratio_dirs) {
		summarizeFromIntermediate(hard_ratio, EXP_result_path + hard_ratio + "/");
	}
	exportAllSummaryCSV(EXP_result_path + "summary/", hard_ratio_dirs);
}


void exp4_different_total_bandwidth() {
	cout << "========================================" << endl;
	cout << "EXP2: The impact of different total bandwidth of each uav." << endl;
	cout << "========================================" << endl;

	string config_file = experimentDataPath + "ExperimentsResults/EXP4_bandwidth/def_config.json";


	// 数据根路径
	string dataSetPath = experimentDataPath + "data/variable_user_num/";
	string EXP_result_path = experimentDataPath +
		"ExperimentsResults/EXP4_bandwidth/" + TON_RESULT_VERSION_DIR;

	vector<int> bandwidth_dirs_int = { 10, 20, 30, 40, 50 };
	vector<string> bandwidth_dirs_str = { "10", "20", "30", "40", "50" };


	string user_pattern = "3000users_data";
	string uav_pattern = "10uavs_loc";


	string DataPath = dataSetPath + "3000u_num/";

	vector<string> userFiles, uavFiles;

	// 调用修改后的原函数：传入不同的用户目录和无人机目录
	bool success = getMatchedFilePairs(
		DataPath,
		DataPath,
		user_pattern,
		uav_pattern,
		30,
		userFiles,
		uavFiles
	);

	if (!success) {
		cout << "Failed to load data " << endl;
	}
	//int count = userFiles.size();
	int count = 10;

	for (auto& bandwidth : bandwidth_dirs_int) {
		cout << "\n--- Condition: hard_ratio = " << to_string(bandwidth) << " ---" << endl;

		string intermediate_dir = EXP_result_path + to_string(bandwidth) + "/";
		// 运行实验
		run_Instance_with_checkpoint(intermediate_dir, userFiles, uavFiles, config_file, count, bandwidth);
	}

	// 汇总与导出（逻辑与 EXP1 一致）
	for (auto& bandwidth : bandwidth_dirs_int) {
		summarizeFromIntermediate(to_string(bandwidth), EXP_result_path + to_string(bandwidth) + "/");
	}


	exportAllSummaryCSV(EXP_result_path + "summary/", bandwidth_dirs_str);
}
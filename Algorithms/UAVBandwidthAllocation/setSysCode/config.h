#pragma once
#include "predefine.h"


//-------------------------------------------------------
// 1. 通用的 Range 结构体
template <typename T>
struct Range {
    T min;
    T max;
};

// 2. 扁平化的总配置结构体
struct ExperimentConfig {
    // --- 拓扑类 (Topology) ---
    int num_uavs;
    int num_elastic_users;
    int num_hard_qos_users;
    Range<double> area_range; // 用户分布区域 (min_coordinate ~ max_coordinate)

    // --- 业务类 (Traffic & QoS) ---
    Range<double> rate_mbps_range; // 最小速率要求范围
    Range<int> outage_exp_range;   // 中断概率指数范围 (10^-x)
    Range<int> weight_range;       // 效用权重范围
};

ExperimentConfig load_experiment_config(const string& json_file_path) {
    ifstream ifs(json_file_path);
    if (!ifs.is_open()) {
        cerr << "Error: Cannot open config file: " << json_file_path << endl;
        exit(1);
    }

    json j;
    try {
        ifs >> j;
    }
    catch (json::parse_error& e) {
        cerr << "JSON Parse Error: " << e.what() << endl;
        exit(1);
    }

    ExperimentConfig cfg;

    // --- 1. 读取拓扑设置 (手动解包嵌套的 JSON) ---
    auto& topo = j["topology_settings"];
    cfg.num_uavs = topo["num_uavs"].get<int>();
    cfg.num_elastic_users = topo["user_counts"]["num_elastic_users"].get<int>();
    cfg.num_hard_qos_users = topo["user_counts"]["num_hard_qos_users"].get<int>();

    // 读取区域范围
    cfg.area_range.min = topo["deployment_area"]["min_coordinate_m"].get<double>();
    cfg.area_range.max = topo["deployment_area"]["max_coordinate_m"].get<double>();

    // --- 2. 读取流量设置 ---
    auto& traffic = j["traffic_generation_params"];
    auto& hard_qos = traffic["hard_qos_requirements"];

    // 读取速率范围
    cfg.rate_mbps_range.min = hard_qos["min_required_rate_range_mbps"]["min"].get<double>();
    cfg.rate_mbps_range.max = hard_qos["min_required_rate_range_mbps"]["max"].get<double>();

    // 读取中断概率指数范围 (注意 JSON key 是 min_exponent)
    cfg.outage_exp_range.min = hard_qos["outage_probability_exponent_range"]["min_exponent"].get<int>();
    cfg.outage_exp_range.max = hard_qos["outage_probability_exponent_range"]["max_exponent"].get<int>();

    // 读取权重范围
    cfg.weight_range.min = traffic["utility_weight_range"]["min"].get<int>();
    cfg.weight_range.max = traffic["utility_weight_range"]["max"].get<int>();

    return cfg;
}



// 根据配置信息随机生成系统实例
SystemMd generate_instance(const ExperimentConfig& cfg) {
    std::mt19937 gen(42);

    // 直接使用 cfg.area_range，不用再 cfg.topology.area...
    std::uniform_real_distribution<double> dist_pos(cfg.area_range.min, cfg.area_range.max);
    std::uniform_int_distribution<int> dist_weight(cfg.weight_range.min, cfg.weight_range.max);
    std::uniform_real_distribution<double> dist_rate(cfg.rate_mbps_range.min, cfg.rate_mbps_range.max);
    std::uniform_int_distribution<int> dist_pout_exp(cfg.outage_exp_range.min, cfg.outage_exp_range.max);

    vector<User> users;
    // 使用扁平变量，代码更短
    int total_users = cfg.num_hard_qos_users + cfg.num_elastic_users;

    for (int i = 0; i < total_users; ++i) {
        // 判断类型
        int type = (i < cfg.num_hard_qos_users) ? HARD_UTILITY : ELASTIC_UTILITY;

        double x = dist_pos(gen);
        double y = dist_pos(gen);
        int weight = dist_weight(gen);
        double rMin = dist_rate(gen);
        double pOut = std::pow(10, -dist_pout_exp(gen));

        users.emplace_back(i, type, weight, x, y, 0.0, rMin, pOut);
    }

    // 生成Uav实例
    vector<Uav> uavs;
    if (cfg.num_uavs == 1)
    {
        uavs.emplace_back(0, 0, 0, uav_alt, 20);
    }
    else if (cfg.num_uavs == 2)
    {
        uavs.emplace_back(0, 125, 0, uav_alt, 20);
        uavs.emplace_back(1, -125, 0, uav_alt, 20);
    }
    else
    {
        for (int i = 0; i < cfg.num_uavs; ++i) {
            // 简单地将 UAV 均匀分布在区域内
            double x = dist_pos(gen);
            double y = dist_pos(gen);
            uavs.emplace_back(i, x, y, uav_alt, 20); // 假设每个 UAV 带宽为 20 MHz
        }
    }

    return SystemMd(users, uavs);
}
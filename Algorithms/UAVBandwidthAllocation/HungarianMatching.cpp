/// @file HungarianMatching.cpp
/// @brief 基于匈牙利算法（KM算法）的子信道匹配资源分配算法
/// 
/// 算法流程：
///   Phase 1: 匈牙利匹配 — 确定用户-UAV关联关系
///   Phase 2: 直接使用匹配结果，每个用户分配一个子信道带宽
///
/// KM算法实现参考: https://cp-algorithms.com/graph/hungarian-algorithm.html

#include "EntityDefinition.h"
#include <chrono>
#include <set>


std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HungarianMatchingAllocation()
{
    const int K = sysModel.m;
    const int n_users = sysModel.n1 + sysModel.n2;
    const double B_sub = sysModel.users[0].BSub * unit_para;

    // ================================================================
    // Step 1: 计算每个 UAV 的子信道数
    // ================================================================
    vector<int> num_subchannels(K);
    int total_subchannels = 0;
    for (int k = 0; k < K; k++) {
        int sub_band_para = sysModel.uavs[k].total_bandwidth / (20 * unit_para);
        num_subchannels[k] = sub_band_para * 100;
        total_subchannels += num_subchannels[k];
    }

    // ================================================================
    // Step 2: 构建子信道 -> UAV 映射
    // ================================================================
    vector<int> subchannel_to_uav(total_subchannels);
    {
        int idx = 0;
        for (int k = 0; k < K; k++) {
            for (int c = 0; c < num_subchannels[k]; c++) {
                subchannel_to_uav[idx++] = k;
            }
        }
    }

    // ================================================================
    // Step 3: 确定方阵维度
    // ================================================================
    int nn = std::max(n_users, total_subchannels);

    // ================================================================
    // Step 3.5: 构建 UAV -> 可服务用户ID集合（用于快速查找）
    // ================================================================
    vector<std::set<int>> uav_serviceable_set(K);
    for (int k = 0; k < K; k++) {
        int uav_real_id = sysModel.uavs[k].ID;
        if (sysModel.uav_serviceable_users_map.count(uav_real_id)) {
            for (auto& u : sysModel.uav_serviceable_users_map[uav_real_id]) {
                uav_serviceable_set[k].insert(u.ID);
            }
        }
    }

    // ================================================================
    // Step 4: 构建 cost 矩阵 (1-indexed, 取负求最大权)
    // ================================================================
    const long long mat_size = (long long)(nn + 1) * (nn + 1);
    vector<double> cost_mat(mat_size, 0.0);
    auto COST = [&](int i, int j) -> double& {
        return cost_mat[(long long)i * (nn + 1) + j];
        };

    for (int i = 1; i <= n_users; i++) {
        int user_idx = i - 1;
        const User& user = sysModel.users[user_idx];

        for (int j = 1; j <= total_subchannels; j++) {
            int sub_idx = j - 1;
            int uav_id = subchannel_to_uav[sub_idx];
            double utility = 0.0;

            // 检查该用户是否在该UAV的可服务列表中，不在则边权为0
            if (uav_serviceable_set[uav_id].find(user.ID) == uav_serviceable_set[uav_id].end()) {
                COST(i, j) = 0.0;
                continue;
            }

            if (user.uType == HARD_UTILITY) {
                double cap = sysModel.cap_list[uav_id][user_idx];
                double achievable_rate = B_sub * cap;
                if (achievable_rate >= user.rMin - EPS) {
                    double SNR_avg_dB = sysModel.SNRave_list[uav_id][user_idx];
                    utility = user.hard_utility(B_sub, cap, SNR_avg_dB);
                }
            }
            else if (user.uType == ELASTIC_UTILITY) {
                double cap = sysModel.cap_list[uav_id][user_idx];
                utility = user.elastic_utility(B_sub, cap);
            }

            COST(i, j) = -utility;
        }
    }

    // ================================================================
    // Step 5: cp-algorithms O(n^3) 匈牙利算法
    // ================================================================
    auto t_start = std::chrono::high_resolution_clock::now();

    vector<double> u_pot(nn + 1, 0.0);
    vector<double> v_pot(nn + 1, 0.0);
    vector<int>    p_match(nn + 1, 0);
    vector<int>    way_arr(nn + 1, 0);

    for (int i = 1; i <= nn; ++i) {
        p_match[0] = i;
        int j0 = 0;
        vector<double> minv(nn + 1, 1e18);
        vector<bool>   used(nn + 1, false);
        fill(way_arr.begin(), way_arr.end(), 0);

        do {
            used[j0] = true;
            int i0 = p_match[j0];
            int j1 = -1;
            double delta = 1e18;

            for (int j = 1; j <= nn; ++j) {
                if (!used[j]) {
                    double cur = COST(i0, j) - u_pot[i0] - v_pot[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way_arr[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            for (int j = 0; j <= nn; ++j) {
                if (used[j]) {
                    u_pot[p_match[j]] += delta;
                    v_pot[j] -= delta;
                }
                else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p_match[j0] != 0);

        do {
            int j1 = way_arr[j0];
            p_match[j0] = p_match[j1];
            j0 = j1;
        } while (j0);
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_sec = std::chrono::duration<double>(t_end - t_start).count();

    // 提取匹配: ans[i] = 行i匹配到的列号 (1-indexed)
    vector<int> ans(nn + 1, 0);
    for (int j = 1; j <= nn; ++j) {
        if (p_match[j] >= 1 && p_match[j] <= nn) {
            ans[p_match[j]] = j;
        }
    }

    // ================================================================
    // Step 6: 从匹配结果提取每个UAV的用户关联关系
    // ================================================================

    // 每个UAV关联的Hard用户和Elastic用户列表 (存user_idx, 即0-indexed ID)
    vector<vector<int>> uav_hard_users(K);
    vector<vector<int>> uav_elastic_users(K);

    for (int i = 1; i <= n_users; i++) {
        int user_idx = i - 1;
        int matched_col = ans[i];

        if (matched_col >= 1 && matched_col <= total_subchannels) {
            int uav_id = subchannel_to_uav[matched_col - 1];
            double utility = -COST(i, matched_col);

            if (utility > EPS) {
                if (sysModel.users[user_idx].uType == HARD_UTILITY) {
                    uav_hard_users[uav_id].push_back(user_idx);
                }
                else {
                    uav_elastic_users[uav_id].push_back(user_idx);
                }
            }
        }
    }

    // ================================================================
    // Step 7（简化）: 直接使用匹配结果，每个用户分配 B_sub 带宽
    // ================================================================

    vector<KnapsackResult> uav_results(K);
    for (int k = 0; k < K; k++) {
        uav_results[k].uav_id = k;
    }

    for (int k = 0; k < K; k++) {
        // Hard用户
        for (int user_idx : uav_hard_users[k]) {
            User& user = sysModel.users[user_idx];
            double cap = sysModel.cap_list[k][user_idx];
            double SNR_avg_dB = sysModel.SNRave_list[k][user_idx];
            double utility = user.hard_utility(B_sub, cap, SNR_avg_dB);
            add_KnapsackResult(uav_results[k], user, B_sub, utility);
        }

        // Elastic用户
        for (int user_idx : uav_elastic_users[k]) {
            User& user = sysModel.users[user_idx];
            double cap = sysModel.cap_list[k][user_idx];
            double utility = user.elastic_utility(B_sub, cap);
            add_KnapsackResult(uav_results[k], user, B_sub, utility);
        }
    }

    // 构建 UserResult
    std::map<int, UserResult> user_results = construct_user_results(uav_results);

    return { uav_results, user_results };
}

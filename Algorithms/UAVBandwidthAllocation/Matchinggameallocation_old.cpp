#include "EntityDefinition.h"

/**
 * @brief 基于匹配博弈的三阶段联合用户关联与带宽分配算法
 *
 * 算法流程：
 *   第一阶段：Hard (RT) 用户关联与保底资源分配 (DA matching)
 *   第二阶段：Elastic (BE) 用户关联 (DA matching)
 *   第三阶段：Elastic (BE) 带宽按Utility比例分配
 *
 * @return pair<vector<KnapsackResult>, map<int, UserResult>>
 *   first:  每个UAV的分配结果列表，索引对应uavs顺序
 *   second: 每个用户的分配结果，key为user_id
 */
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::MatchingGameAllocation()
{
    const int M = sysModel.m;                    // UAV数量
    const int N = sysModel.n1 + sysModel.n2;     // 用户总数
    const int N_hard = sysModel.n1;              // Hard用户数量
    const int N_elastic = sysModel.n2;           // Elastic用户数量

    // ==================== 初始化结果容器 ====================
    std::vector<KnapsackResult> uav_results(M);
    for (int k = 0; k < M; k++) {
        uav_results[k].uav_id = sysModel.uavs[k].ID;
        clean_KnapsackResult(uav_results[k]);
        uav_results[k].uav_id = sysModel.uavs[k].ID; // clean后重设
    }
    std::map<int, UserResult> user_results;
    for (auto& user : sysModel.users) {
        UserResult user_result;
        user_result.uav_id = -1;
        user_result.allocated_bandwidth = 0;
        user_result.utility = 0;
        user_results[user.ID] = user_result;
    }

    // ==================== 分离Hard / Elastic用户 ====================
    // 从 uav_serviceable_users_map 中分离出每个UAV可服务的Hard和Elastic用户
    // 全局Hard用户列表和Elastic用户列表
    std::vector<User> all_hard_users, all_elastic_users;
    for (auto& user : sysModel.users) {
        if (user.uType == HARD_UTILITY)
            all_hard_users.push_back(user);
        else
            all_elastic_users.push_back(user);
    }

    // 每个UAV可服务的Hard / Elastic用户ID集合
    std::map<int, std::vector<int>> uav_serviceable_hard;   // uav_idx -> [user_id, ...]
    std::map<int, std::vector<int>> uav_serviceable_elastic; // uav_idx -> [user_id, ...]
    for (int k = 0; k < M; k++) {
        int uav_id = sysModel.uavs[k].ID;
        if (sysModel.uav_serviceable_users_map.count(uav_id)) {
            for (auto& u : sysModel.uav_serviceable_users_map[uav_id]) {
                if (u.uType == HARD_UTILITY)
                    uav_serviceable_hard[k].push_back(u.ID);
                else
                    uav_serviceable_elastic[k].push_back(u.ID);
            }
        }
    }

    // user_id -> 在sysModel.users中的索引（用于快速查表）
    std::map<int, int> uid_to_idx;
    for (int i = 0; i < N; i++) {
        uid_to_idx[sysModel.users[i].ID] = i;
    }

    // uav_id -> 在sysModel.uavs中的索引
    std::map<int, int> uav_id_to_idx;
    for (int k = 0; k < M; k++) {
        uav_id_to_idx[sysModel.uavs[k].ID] = k;
    }

    // ==================== 辅助lambda ====================
    // 获取 uav_idx=k 与 user_id=uid 之间的平均SNR (dB)
    auto get_SNRave_dB = [&](int k, int uid) -> double {
        int j = uid_to_idx[uid];
        return sysModel.SNRave_list[k][j];
        };

    // 获取 uav_idx=k 与 user_id=uid 之间的平均SNR (线性值)
    auto get_SNRave_linear = [&](int k, int uid) -> double {
        double snr_db = get_SNRave_dB(k, uid);
        return pow(10.0, snr_db / 10.0);
        };

    // 获取 Hard用户 uid 在 uav_idx=k 上的最小带宽需求
    auto get_Bth = [&](int k, int uid) -> double {
        int j = uid_to_idx[uid];
        if (j < N_hard)
            return sysModel.Bth_list[k][j];
        return INFINITY; // 非hard用户不应调用此函数
        };

    // 获取 Hard用户 uid 被 uav_idx=k 服务时的效用值
    auto get_hard_utility = [&](int k, int uid) -> double {
        int j = uid_to_idx[uid];
        double cap = sysModel.cap_list[k][j];
        double b_min = get_Bth(k, uid);
        double SNR_avg_dB = sysModel.SNRave_list[k][j];
        return sysModel.users[j].hard_utility(b_min, cap, SNR_avg_dB);
        };

    // UAV对Hard用户的带宽效率偏好值: utility / bandwidth_needed
    auto get_hard_efficiency = [&](int k, int uid) -> double {
        double uti = get_hard_utility(k, uid);
        double bth = get_Bth(k, uid);
        if (bth <= EPS || bth == INFINITY) return 0.0;
        return uti / bth;
        };

    // UAV对Elastic用户的偏好Utility值: w_j * log2(1 + log2(1 + SNR_linear))
    auto get_elastic_orientation = [&](int k, int uid) -> double {
        int j = uid_to_idx[uid];
        double w_j = sysModel.users[j].weight;
        double snr_linear = get_SNRave_linear(k, uid);
        return w_j * log2(1.0 + log2(1.0 + snr_linear));
        // return log2(1.0 + log2(1.0 + snr_linear));
        };

    // ====================================================================
    //                  第一阶段：Hard用户DA匹配
    // ====================================================================

    // --- 构建Hard用户偏好列表：按平均SNR降序排列可服务的UAV ---
    // preference_hard[uid] = [uav_idx sorted by SNR desc]
    std::map<int, std::vector<int>> preference_hard;
    for (auto& u : all_hard_users) {
        int uid = u.ID;
        std::vector<std::pair<double, int>> snr_uav_pairs;
        for (int k = 0; k < M; k++) {
            // 检查该UAV是否可服务该用户
            auto& slist = uav_serviceable_hard[k];
            if (std::find(slist.begin(), slist.end(), uid) != slist.end()) {
                // 还需要检查Bth是否有限（即该UAV能否满足该Hard用户）
                double bth = get_Bth(k, uid);
                if (bth < INFINITY && bth > 0) {
                    snr_uav_pairs.push_back({ get_SNRave_dB(k, uid), k });
                }
            }
        }
        // 按SNR降序排列
        std::sort(snr_uav_pairs.begin(), snr_uav_pairs.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        for (auto& p : snr_uav_pairs) {
            preference_hard[uid].push_back(p.second);
        }
    }

    // --- DA匹配过程 ---
    // 每个UAV当前暂时接受的Hard用户集合
    std::map<int, std::vector<int>> uav_accepted_hard; // uav_idx -> [user_id, ...]
    // 每个UAV为Hard用户已占用的带宽
    std::map<int, double> uav_hard_bw_used; // uav_idx -> total hard bandwidth
    for (int k = 0; k < M; k++) {
        uav_hard_bw_used[k] = 0.0;
    }

    // 每个Hard用户的偏好列表指针（下一个要申请的UAV在偏好列表中的位置）
    std::map<int, int> hard_pref_ptr;
    // 每个Hard用户的匹配状态
    std::map<int, bool> hard_matched;
    for (auto& u : all_hard_users) {
        hard_pref_ptr[u.ID] = 0;
        hard_matched[u.ID] = false;
    }

    bool has_proposal = true;
    while (has_proposal) {
        has_proposal = false;

        // Step A: 所有未匹配的Hard用户发起申请
        // 收集每个UAV收到的新申请者
        std::map<int, std::vector<int>> new_proposals; // uav_idx -> [uid, ...]
        for (auto& u : all_hard_users) {
            int uid = u.ID;
            if (hard_matched[uid]) continue;
            if (hard_pref_ptr[uid] >= (int)preference_hard[uid].size()) continue;

            int target_uav = preference_hard[uid][hard_pref_ptr[uid]];
            new_proposals[target_uav].push_back(uid);
            has_proposal = true;
        }

        if (!has_proposal) break;

        // Step B: 每个UAV评估申请者
        for (int k = 0; k < M; k++) {
            double B_UAV = sysModel.uavs[k].total_bandwidth;

            // 合并已接受的用户和新申请者
            std::vector<int> candidates = uav_accepted_hard[k];
            if (new_proposals.count(k)) {
                for (int uid : new_proposals[k]) {
                    candidates.push_back(uid);
                }
            }
            if (candidates.empty()) continue;

            // 按带宽效率降序排列
            std::sort(candidates.begin(), candidates.end(),
                [&](int a, int b) {
                    return get_hard_efficiency(k, a) > get_hard_efficiency(k, b);
                });

            // 准入控制：按优先级逐个接纳
            std::vector<int> accepted;
            double bw_sum = 0.0;
            for (int uid : candidates) {
                double bth = get_Bth(k, uid);
                if (bw_sum + bth <= B_UAV + EPS) {
                    accepted.push_back(uid);
                    bw_sum += bth;
                }
                else {
                    // 拒绝该用户及后续所有用户
                    break;
                }
            }

            // 确定被拒绝的用户集合
            std::set<int> accepted_set(accepted.begin(), accepted.end());
            // 更新UAV的接受列表
            uav_accepted_hard[k] = accepted;
            uav_hard_bw_used[k] = bw_sum;

            // 更新所有candidate的状态
            for (int uid : candidates) {
                if (accepted_set.count(uid)) {
                    hard_matched[uid] = true;
                }
                else {
                    // 被拒绝
                    hard_matched[uid] = false;
                    // 将偏好指针后移（从偏好列表中移除该UAV）
                    hard_pref_ptr[uid]++;
                }
            }
        }
    }

    // --- 第一阶段结果写入 ---
    // 记录每个UAV的剩余带宽
    std::vector<double> B_remain(M, 0.0);
    for (int k = 0; k < M; k++) {
        double B_UAV = sysModel.uavs[k].total_bandwidth;
        B_remain[k] = B_UAV - uav_hard_bw_used[k];
        if (B_remain[k] < EPS) B_remain[k] = 0.0;

        // 将Hard用户加入KnapsackResult
        for (int uid : uav_accepted_hard[k]) {
            int j = uid_to_idx[uid];
            double bth = get_Bth(k, uid);
            double cap = sysModel.cap_list[k][j];
            double SNR_avg_dB = sysModel.SNRave_list[k][j];
            double uti = sysModel.users[j].hard_utility(bth, cap, SNR_avg_dB);

            add_KnapsackResult(uav_results[k], sysModel.users[j], bth, uti);

            // 写入UserResult
            user_results[uid].uav_id = sysModel.uavs[k].ID;
            user_results[uid].allocated_bandwidth = bth;
            user_results[uid].utility = uti;
        }
        cout << "uav " << k << ": Hard users allocated bandwidth = " << uav_hard_bw_used[k]
			<< ", remaining bandwidth = " << B_remain[k] << endl;
    }

    // ====================================================================
    //              第二阶段：Elastic用户DA匹配
    // ====================================================================

    // --- 构建Elastic用户偏好列表：按平均SNR降序排列可服务且有剩余带宽的UAV ---
    std::map<int, std::vector<int>> preference_elastic;
    for (auto& u : all_elastic_users) {
        int uid = u.ID;
        std::vector<std::pair<double, int>> snr_uav_pairs;
        for (int k = 0; k < M; k++) {
            if (B_remain[k] <= EPS) continue; // 无剩余带宽的UAV不参与
            auto& slist = uav_serviceable_elastic[k];
            if (std::find(slist.begin(), slist.end(), uid) != slist.end()) {
                snr_uav_pairs.push_back({ get_SNRave_dB(k, uid), k });
            }
        }
        std::sort(snr_uav_pairs.begin(), snr_uav_pairs.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
        for (auto& p : snr_uav_pairs) {
            preference_elastic[uid].push_back(p.second);
        }
    }

    // --- 构建UAV对Elastic用户的偏好排序 ---
    // 预计算UAV对每个可服务Elastic用户的Utility值
    // （在DA过程中UAV按此值排序来决定接受/拒绝）
    // uav_elastic_pref[k] = [(utility, uid), ...] 降序
    std::map<int, std::vector<std::pair<double, int>>> uav_elastic_pref;
    for (int k = 0; k < M; k++) {
        if (B_remain[k] <= EPS) continue;
        for (int uid : uav_serviceable_elastic[k]) {
            double uti = get_elastic_orientation(k, uid);
            uav_elastic_pref[k].push_back({ uti, uid });
        }
        std::sort(uav_elastic_pref[k].begin(), uav_elastic_pref[k].end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });
    }


    std::map<int, std::vector<int>> uav_accepted_elastic; // uav_idx -> [uid, ...]
    std::map<int, bool> elastic_matched;
    std::map<int, int> elastic_pref_ptr;
    for (auto& u : all_elastic_users) {
        elastic_matched[u.ID] = false;
        elastic_pref_ptr[u.ID] = 0;
    }

    // Step 2.1: 未匹配的Elastic用户向偏好列表第一的UAV发起申请
    std::map<int, std::vector<int>> new_proposals;
    for (auto& u : all_elastic_users) {
        int uid = u.ID;
        if (elastic_matched[uid]) continue;
        if (elastic_pref_ptr[uid] >= (int)preference_elastic[uid].size()) continue;

        int target_uav = preference_elastic[uid][elastic_pref_ptr[uid]];
        new_proposals[target_uav].push_back(uid);
        has_proposal = true;
    }


    // Step 2.2: 每个UAV评估
    for (int k = 0; k < M; k++) {
        if (!new_proposals.count(k) && uav_accepted_elastic[k].empty()) continue;

        // 合并已接受和新申请者
        std::vector<int> candidates = uav_accepted_elastic[k];
        if (new_proposals.count(k)) {
            for (int uid : new_proposals[k]) {
                candidates.push_back(uid);
            }
        }

        // UAV接受所有申请者（弹性用户，带宽可任意细分）
        uav_accepted_elastic[k] = candidates;
        for (int uid : candidates) {
            elastic_matched[uid] = true;
        }
        // 不拒绝任何人 → 所有用户在此轮被接受，无需后续轮次
    }

    // ====================================================================
    //         第三阶段：Elastic用户带宽按Utility比例分配
    // ====================================================================

    for (int k = 0; k < M; k++) {
        if (B_remain[k] <= EPS) continue;
        if (uav_accepted_elastic[k].empty()) continue;
        cout << "remain bandwidth of uav " << k << " is " << B_remain[k] << endl;
        // 计算该UAV下所有Elastic用户的聚合Utility
        double total_orientation = 0.0;
        for (int uid : uav_accepted_elastic[k]) {
            total_orientation += get_elastic_orientation(k, uid);
        }

        if (total_orientation <= EPS) continue;

        // 按比例分配带宽
        for (int uid : uav_accepted_elastic[k]) {
            int j = uid_to_idx[uid];
            double orientation = get_elastic_orientation(k, uid);
            // double bw = B_remain[k] * (orientation / total_orientation);
            double bw = B_remain[k] * (1.0 / uav_accepted_elastic[k].size());
            
            // 计算实际效用
            double cap = sysModel.cap_list[k][j];
            double uti = sysModel.users[j].elastic_utility(bw, cap);

            add_KnapsackResult(uav_results[k], sysModel.users[j], bw, uti);

            // 写入UserResult
            user_results[uid].uav_id = sysModel.uavs[k].ID;
            user_results[uid].allocated_bandwidth = bw;
            user_results[uid].utility = uti;
        }
    }

    return { uav_results, user_results };
}
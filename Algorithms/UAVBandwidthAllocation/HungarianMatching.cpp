/// @file HungarianMatching.cpp
/// @brief 提供默认八候选、可显式缩减预算的单槽矩形 Hungarian HardFirst 及独立历史 Hungarian/KM 算法。
/// 正式 AlgHardFirst 使用最低等级视图，先冻结 hard 的一槽合同，再分配剩余槽给 elastic；不实现 NOMA。
#include "EntityDefinition.h"
#include <chrono>
#include <set>
#include <cstdint>
#include <numeric>
#include <random>

/// Count complete slots from total/effective bandwidth in the same internal units.
/// A slot includes 10% guard overhead; invalid inputs or int count overflow throw.
int hard_first_slot_count(double total_bandwidth, double effective_bandwidth) {
    if (!std::isfinite(total_bandwidth) || total_bandwidth < 0 ||
        !std::isfinite(effective_bandwidth) || effective_bandwidth <= 0)
        throw std::invalid_argument("Invalid hard-first bandwidth");
    const double slot = effective_bandwidth * (10.0 / 9.0);
    if (!std::isfinite(slot) || slot <= 0)
        throw std::overflow_error("Hard-first slot width overflow");
    // Only absorb machine-roundoff at an integer, never the allocation QoS tolerance.
    const double ratio = total_bandwidth / slot;
    const double nearest = std::round(ratio);
    const double corrected = std::abs(ratio - nearest) <=
        8 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(ratio))
        ? nearest : ratio;
    const double count = std::floor(corrected);
    if (!std::isfinite(count) || count > INT_MAX)
        throw std::overflow_error("Too many hard-first slots");
    return static_cast<int>(count);
}

/// Return the smallest positive slot count meeting common hard QoS, or zero if
/// even max_slots cannot serve the user. Width/rate/capacity use internal units.
int hard_first_min_slots(int max_slots, double width, double capacity, double minimum_rate) {
    if (max_slots < 0 || !std::isfinite(width) || width <= 0 ||
        !std::isfinite(capacity) || capacity < 0 ||
        !std::isfinite(minimum_rate) || minimum_rate < 0)
        throw std::invalid_argument("Invalid hard-first demand");
    if (max_slots == 0 || capacity == 0) return 0;
    if (!std::isfinite(max_slots * width) ||
        !std::isfinite(max_slots * width * capacity))
        throw std::overflow_error("Hard-first rate overflow");
    if (!hard_qos_satisfied(max_slots * width, capacity, minimum_rate)) return 0;
    int low = 1, high = max_slots;
    while (low < high) {
        const int middle = low + (high - low) / 2;
        if (hard_qos_satisfied(middle * width, capacity, minimum_rate)) high = middle;
        else low = middle + 1;
    }
    return low;
}

namespace {
/// Read the common positive effective width (MHz converted to internal units).
/// Empty user sets have no width requirement and return the default 180-kHz width.
double hard_first_width(const SystemMd& model) {
    if (model.users.empty()) return 0.18 * unit_para;
    const double width = model.users.front().BSub * unit_para;
    if (!std::isfinite(width) || width <= 0)
        throw std::invalid_argument("Invalid hard-first effective width");
    for (const auto& user : model.users)
        if (!std::isfinite(user.BSub) || user.BSub <= 0 ||
            !allocation_near(user.BSub * unit_para, width))
            throw std::invalid_argument("Hard-first requires a common effective width");
    return width;
}

/// Check both the declared service map and physical coverage; malformed IDs throw.
std::vector<std::vector<bool>> hard_first_links(const SystemMd& model) {
    std::vector<std::vector<bool>> links(model.m, std::vector<bool>(model.users.size(), false));
    for (const auto& entry : model.uav_serviceable_users_map) {
        if (entry.first < 0 || entry.first >= model.m)
            throw std::invalid_argument("Invalid hard-first serviceable UAV");
        for (const auto& user : entry.second) {
            if (user.ID < 0 || user.ID >= static_cast<int>(model.users.size()))
                throw std::invalid_argument("Invalid hard-first serviceable user");
            const int k = entry.first, j = user.ID;
            links[k][j] = model.dis_list.at(k).at(j) <= max_coverage_distance &&
                model.cap_list.at(k).at(j) > 0;
        }
    }
    return links;
}

} // namespace

/// 复用求解器的统一槽宽与保护开销计算总槽数；model只读，返回非负int，超界时拒绝截断。
int hard_first_physical_slot_count(const SystemMd& model) {
    const double width = hard_first_width(model);
    int total = 0;
    for (const auto& uav : model.uavs) {
        const int count = hard_first_slot_count(uav.total_bandwidth, width);
        if (count > INT_MAX - total) throw std::overflow_error("Too many total hard-first slots");
        total += count;
    }
    return total;
}

/// 校验 HardFirst 的最低等级有效带宽、单槽 hard 占用、覆盖和唯一关联。
/// model 应为最低等级求解视图；hard 的槽内余量保留占用，不计入报告带宽。
void validate_hard_first_allocation(const SystemMd& model,
    const std::vector<KnapsackResult>& results) {
    const double width = hard_first_width(model);
    if (results.size() != model.uavs.size())
        throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first result dimension");
    const auto links = hard_first_links(model);
    std::vector<bool> seen_user(model.users.size(), false), seen_uav(model.uavs.size(), false);
    for (const auto& result : results) {
        const int k = result.uav_id;
        if (k < 0 || k >= model.m || seen_uav[k])
            throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first UAV ID");
        seen_uav[k] = true;
        if (result.allocatedList.size() != result.allocatedBandwidth.size())
            throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first result entries disagree");
        const int budget = hard_first_slot_count(model.uavs[k].total_bandwidth, width);
        int used = 0;
        for (int j : result.allocatedList) {
            if (j < 0 || j >= static_cast<int>(model.users.size()) || seen_user[j] ||
                result.allocatedBandwidth.count(j) == 0 || !links[k][j])
                throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first invalid user association");
            seen_user[j] = true;
            const double bw = result.allocatedBandwidth.at(j);
            if (!std::isfinite(bw) || bw <= 0)
                throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first invalid bandwidth");
            const double cap = model.cap_list.at(k).at(j);
            int count = 0;
            if (model.users[j].uType == HARD_UTILITY) {
                const double rate = model.users[j].rMin;
                const double threshold = rate / cap;
                count = hard_first_min_slots(budget, width, cap, rate);
                if (count != 1 || !std::isfinite(threshold) || threshold <= 0 ||
                    !allocation_near(bw, threshold) || !hard_qos_satisfied(bw, cap, rate))
                    throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first hard user requires more than one slot or misses QoS");
            } else {
                const double slots = std::round(bw / width);
                if (!std::isfinite(slots) || slots < 1 || slots > INT_MAX ||
                    std::abs(bw / width - slots) >
                        16 * std::numeric_limits<double>::epsilon() * std::max(1.0, slots))
                    throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first noninteger elastic allocation");
                count = static_cast<int>(slots);
            }
            if (count > budget - used)
                throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first guard-inclusive budget exceeded");
            used += count;
        }
        const double occupied = used * (width * (10.0 / 9.0));
        if (!std::isfinite(occupied) || occupied > model.uavs[k].total_bandwidth +
            allocation_tolerance(occupied, model.uavs[k].total_bandwidth))
            throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first occupied bandwidth exceeded");
    }
}

namespace {
/// 物理槽只可能空闲、被单槽 hard 合同锁定，或暂分给 elastic 用户。
enum class SlotPhase { Vacant, FrozenHard, Elastic };
struct PhysicalSlot { int uav, local, owner = -1; SlotPhase phase = SlotPhase::Vacant; };

/// 记录单个候选的矩形规模、增广次数和三个阶段耗时；候选外层另行累计，不以计时决定分配。
struct HardFirstCounters {
    uint64_t hard_hungarian_rows = 0, hard_hungarian_columns = 0, hard_hungarian_augmentations = 0;
    uint64_t hard_eligible_edges = 0, hard_matches = 0;
    double hard_matching_ms = 0, hard_local_search_ms = 0, elastic_ms = 0;
    uint64_t elastic_proposals = 0, elastic_replacements = 0, elastic_rounds = 0;
    uint64_t elastic_proposal_bound = 0;
    // 每轮完整评估邻域，至多接受一个操作；停止原因与改进次数分别记录。
    uint64_t local_search_rounds = 0, local_search_improvements = 0;
    uint64_t local_search_admissions = 0, local_search_replacements = 0;
    uint64_t local_search_migrations = 0, local_search_swaps = 0;
    std::string local_search_stop_reason = "no_improving_move";
    /// 将工作量及毫秒计时序列化为 JSON；返回诊断对象，耗时字段不用于确定性比较。
    json to_json() const {
        return {{"hard_hungarian_rows", hard_hungarian_rows}, {"hard_hungarian_columns", hard_hungarian_columns},
            {"hard_hungarian_augmentations", hard_hungarian_augmentations},
            {"hard_matching_ms", hard_matching_ms}, {"hard_local_search_ms", hard_local_search_ms},
            {"elastic_ms", elastic_ms},
            {"hard_eligible_edges", hard_eligible_edges}, {"hard_matches", hard_matches},
            {"elastic_proposals", elastic_proposals},
            {"elastic_replacements", elastic_replacements}, {"elastic_rounds", elastic_rounds},
            {"elastic_proposal_bound", elastic_proposal_bound},
            {"local_search_max_rounds", HARD_FIRST_LOCAL_SEARCH_MAX_ROUNDS},
            {"local_search_rounds", local_search_rounds}, {"local_search_improvements", local_search_improvements},
            {"local_search_admissions", local_search_admissions}, {"local_search_replacements", local_search_replacements},
            {"local_search_migrations", local_search_migrations}, {"local_search_swaps", local_search_swaps},
            {"local_search_stop_reason", local_search_stop_reason}};
    }
};

/// 单候选结果包含分配、工作量、稳定用户ID顺序的效用及完整 hard 槽物理归属；空闲或elastic槽记为-1。
using HardFirstAllocation = std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>;
struct HardFirstCandidate {
    HardFirstAllocation allocation;
    HardFirstCounters counters;
    std::vector<int> hard_slot_owners;
    double hard_utility = 0, total_utility = 0;
};

/// 跟踪一个独立候选的单槽 hard 合同及真实槽归属；hard 搜索结束后才允许 elastic 匹配。
/// 每个用户最终只关联一架 UAV，hard 的槽内余量始终保持占用。
class HardFirstAllocator {
    const SystemMd& model;
    double width;
    int n, m;
    std::vector<PhysicalSlot> slots;
    std::vector<std::set<int>> held;
    std::vector<bool> frozen;
    std::vector<double> hard_value;
    std::vector<std::vector<bool>> links;
    HardFirstCounters counters;

    /// 返回每架 UAV 按槽 ID 排序的空闲槽，供合同落槽及 elastic 提案使用。
    std::vector<std::vector<int>> vacant_lists() const {
        std::vector<std::vector<int>> lists(m);
        for (int s = 0; s < static_cast<int>(slots.size()); ++s)
            if (slots[s].owner == -1) lists[slots[s].uav].push_back(s);
        return lists;
    }

    /// 为 elastic 用户建立按单槽速率递减的 UAV 列表；零权重用户不提案。
    std::vector<std::vector<int>> elastic_preferences(const std::vector<std::vector<int>>& available) {
        std::vector<std::vector<int>> prefs(n);
        for (int j = model.n1; j < n; ++j) {
            if (model.users[j].weight == 0) continue;
            for (int k = 0; k < m; ++k) {
                if (!links[k][j] || available[k].empty()) continue;
                prefs[j].push_back(k);
                counters.elastic_proposal_bound += available[k].size();
            }
            // 相同有效槽宽下，可靠容量越高，单槽速率越高。
            std::sort(prefs[j].begin(), prefs[j].end(), [&](int a, int b) {
                if (model.cap_list[a][j] != model.cap_list[b][j])
                    return model.cap_list[a][j] > model.cap_list[b][j];
                return a < b;
            });
        }
        return prefs;
    }

    /// 同步提交 elastic 槽位决定；已冻结的 hard 槽不可被覆盖。
    void commit_elastic(const std::vector<std::pair<int, int>>& decisions) {
        for (const auto& decision : decisions) {
            const auto& slot = slots[decision.first];
            if (slot.phase == SlotPhase::FrozenHard || decision.second < model.n1)
                throw std::logic_error("Elastic decision attempted to replace hard allocation");
            const int old = slot.owner;
            if (old == decision.second) continue;
            if (old >= 0) {
                if (old < model.n1) throw std::logic_error("Hard allocation in elastic decision");
                held[old].erase(decision.first);
                ++counters.elastic_replacements;
            }
        }
        for (const auto& decision : decisions) {
            auto& slot = slots[decision.first];
            slot.owner = decision.second;
            slot.phase = SlotPhase::Elastic;
            held[decision.second].insert(decision.first);
        }
    }

    /// 对 H 个真实 hard 用户与 max(H,S) 列运行矩形 Hungarian 最大权匹配，S 为物理槽数。
    /// candidate_id=0保留原顺序，其余候选用固定种子重排行/槽组；只增广真实行并冻结有效正权真实槽。
    void match_hard_single_slots(int candidate_id) {
        const int rows = model.n1;
        const int physical_slots = static_cast<int>(slots.size());
        const int columns = std::max(rows, physical_slots);
        counters.hard_hungarian_rows = static_cast<uint64_t>(rows);
        counters.hard_hungarian_columns = static_cast<uint64_t>(columns);
        if (rows == 0 || physical_slots == 0) return;
        if (columns == INT_MAX) throw std::overflow_error("Hard Hungarian dimension overflow");
        const size_t row_count = static_cast<size_t>(rows) + 1;
        const size_t stride = static_cast<size_t>(columns) + 1;
        if (row_count > std::vector<double>().max_size() / stride)
            throw std::overflow_error("Hard Hungarian matrix overflow");
        std::vector<double> cost(row_count * stride, 0.0);
        // 索引从1开始；候选0的行和真实槽列与原实现完全相同，排列只影响并列解的遍历顺序。
        std::vector<int> row_users(rows), uav_order(m), column_slots;
        std::iota(row_users.begin(), row_users.end(), 0);
        std::iota(uav_order.begin(), uav_order.end(), 0);
        if (candidate_id > 0) {
            std::mt19937 engine(HARD_FIRST_CANDIDATE_BASE_SEED + static_cast<uint32_t>(candidate_id));
            std::shuffle(row_users.begin(), row_users.end(), engine);
            std::shuffle(uav_order.begin(), uav_order.end(), engine);
        }
        std::vector<std::vector<int>> by_uav(m);
        for (int s = 0; s < physical_slots; ++s) by_uav[slots[s].uav].push_back(s);
        column_slots.reserve(physical_slots);
        for (int k : uav_order)
            column_slots.insert(column_slots.end(), by_uav[k].begin(), by_uav[k].end());
        // 虚拟列仍在所有真实槽之后；各UAV内部保持本地槽升序。
        auto COST = [&](int row, int column) -> double& {
            return cost[static_cast<size_t>(row) * stride + column];
        };
        for (int row = 1; row <= rows; ++row) {
            const int user = row_users[row - 1];
            if (hard_value[user] <= 0) continue;
            for (int column = 1; column <= physical_slots; ++column) {
                const int k = slots[column_slots[column - 1]].uav;
                if (!links[k][user] || hard_first_min_slots(1, width,
                    model.cap_list[k][user], model.users[user].rMin) != 1) continue;
                COST(row, column) = -hard_value[user];
                ++counters.hard_eligible_edges;
            }
        }
        if (counters.hard_eligible_edges == 0) return;

        const double infinity = std::numeric_limits<double>::infinity();
        std::vector<double> u(rows + 1, 0.0), v(columns + 1, 0.0);
        std::vector<int> p(columns + 1, 0), way(columns + 1, 0);
        // 仅真实行需要增广；零权位置使任意可行部分匹配可扩展为覆盖 H 行的指派。
        for (int row = 1; row <= rows; ++row) {
            p[0] = row;
            int column = 0;
            std::vector<double> minimum(columns + 1, infinity);
            std::vector<bool> used(columns + 1, false);
            std::fill(way.begin(), way.end(), 0);
            do {
                used[column] = true;
                const int current_row = p[column];
                int next_column = -1;
                double delta = infinity;
                for (int candidate = 1; candidate <= columns; ++candidate) {
                    if (used[candidate]) continue;
                    const double reduced = COST(current_row, candidate) - u[current_row] - v[candidate];
                    if (reduced < minimum[candidate]) {
                        minimum[candidate] = reduced;
                        way[candidate] = column;
                    }
                    if (minimum[candidate] < delta) {
                        delta = minimum[candidate];
                        next_column = candidate;
                    }
                }
                if (next_column < 0 || !std::isfinite(delta))
                    throw std::logic_error("Hard Hungarian found no finite augmenting path");
                for (int candidate = 0; candidate <= columns; ++candidate) {
                    if (used[candidate]) {
                        u[p[candidate]] += delta;
                        v[candidate] -= delta;
                    } else {
                        minimum[candidate] -= delta;
                    }
                }
                column = next_column;
            } while (p[column] != 0);
            do {
                const int previous = way[column];
                p[column] = p[previous];
                column = previous;
            } while (column != 0);
            ++counters.hard_hungarian_augmentations;
        }
        // 零权占位和虚拟列不构成服务或预留；未冻结的真实物理槽仍可供 elastic 使用。
        for (int column = 1; column <= physical_slots; ++column) {
            const int row = p[column];
            if (row < 1 || row > model.n1 || COST(row, column) >= 0) continue;
            const int user = row_users[row - 1], slot = column_slots[column - 1];
            if (!held[user].empty() || slots[slot].owner != -1)
                throw std::logic_error("Hard Hungarian association mismatch");
            slots[slot].owner = user;
            slots[slot].phase = SlotPhase::FrozenHard;
            held[user].insert(slot);
            frozen[user] = true;
            ++counters.hard_matches;
        }
    }

    /// 在 hard 单槽匹配后搜索最佳接纳和关联改进；多槽门槛边始终不可选。
    /// 最多HARD_FIRST_LOCAL_SEARCH_MAX_ROUNDS轮；每轮只提交一次改进，无有效改进立即返回。
    void improve_hard_allocation() {
        const int hard_count = model.n1;
        if (hard_count == 0) {
            counters.local_search_stop_reason = "no_hard_users";
            return;
        }
        for (int j = hard_count; j < n; ++j)
            if (!held[j].empty()) throw std::logic_error("Hard local search must precede elastic allocation");

        // need[k][j]=0 表示不可覆盖或单槽无法达到当前求解视图的最低等级。
        std::vector<int> budgets(m), used(m, 0), owner(hard_count, -1);
        std::vector<std::vector<int>> need(m, std::vector<int>(hard_count, 0));
        for (int k = 0; k < m; ++k) {
            budgets[k] = hard_first_slot_count(model.uavs[k].total_bandwidth, width);
            for (int j = 0; j < hard_count; ++j)
                if (budgets[k] > 0 && links[k][j]) need[k][j] = hard_first_min_slots(1, width,
                    model.cap_list[k][j], model.users[j].rMin);
        }
        // 把已匹配的 hard 单槽合同投影到紧凑归属数组。
        for (int j = 0; j < hard_count; ++j) {
            if (held[j].empty()) continue;
            const int k = slots[*held[j].begin()].uav;
            if (!frozen[j] || need[k][j] != 1 || held[j].size() != 1)
                throw std::logic_error("Hard local search received a non-single-slot contract");
            for (int s : held[j])
                if (slots[s].owner != j || slots[s].uav != k)
                    throw std::logic_error("Hard local search received inconsistent slot ownership");
            if (need[k][j] > budgets[k] - used[k]) throw std::logic_error("Hard local search input exceeds budget");
            owner[j] = k;
            used[k] += need[k][j];
        }

        enum class MoveKind { Admit, Replace, Relocate, Swap };
        struct Move {
            bool valid = false;
            MoveKind kind = MoveKind::Admit;
            int first = -1, second = -1, target = -1;
            double utility_gain = 0;
            int saved_slots = 0; // 正值减少资源，负值增加资源；仅在效用增量相同时比较。
        };
        uint64_t accepted = 0;
        counters.local_search_stop_reason = "iteration_limit";
        for (int round = 0; round < HARD_FIRST_LOCAL_SEARCH_MAX_ROUNDS; ++round) {
            ++counters.local_search_rounds;
            Move best;
            // 不使用容差接受效用下降；同一轮状态冻结，相同评分保持先枚举的操作。
            auto consider = [&](MoveKind kind, int first, int second, int target, double gain, int saving) {
                if (!std::isfinite(gain)) throw std::overflow_error("Hard local search utility gain overflow");
                if (!(gain > 0 || (gain == 0 && saving > 0))) return;
                if (!best.valid || gain > best.utility_gain ||
                    (gain == best.utility_gain && saving > best.saved_slots))
                    best = {true, kind, first, second, target, gain, saving};
            };

            // 1. 新增接纳：未服务用户按ID升序，再按目标UAV ID升序。
            for (int j = 0; j < hard_count; ++j) {
                if (owner[j] >= 0) continue;
                for (int k = 0; k < m; ++k)
                    if (need[k][j] > 0 && need[k][j] <= budgets[k] - used[k])
                        consider(MoveKind::Admit, j, -1, k, hard_value[j], -need[k][j]);
            }
            // 2. 同UAV替换：先枚举移除用户，再枚举新用户；检查先释放再接纳后的容量。
            for (int a = 0; a < hard_count; ++a) {
                const int k = owner[a];
                if (k < 0) continue;
                for (int b = 0; b < hard_count; ++b)
                    if (owner[b] < 0 && need[k][b] > 0 && need[k][b] <= budgets[k] - used[k] + need[k][a])
                        consider(MoveKind::Replace, a, b, k, hard_value[b] - hard_value[a], need[k][a] - need[k][b]);
            }
            // 3. 迁移：保留服务用户和效用，只有减少槽占用才可能被接受。
            for (int j = 0; j < hard_count; ++j) {
                const int old = owner[j];
                if (old < 0) continue;
                for (int k = 0; k < m; ++k)
                    if (k != old && need[k][j] > 0 && need[k][j] <= budgets[k] - used[k])
                        consider(MoveKind::Relocate, j, -1, k, 0.0, need[old][j] - need[k][j]);
            }
            // 4. 交换：仅枚举a<b，不同UAV同时释放旧槽后，两侧都必须放得下新单槽合同。
            for (int a = 0; a < hard_count; ++a) {
                const int ka = owner[a];
                if (ka < 0) continue;
                for (int b = a + 1; b < hard_count; ++b) {
                    const int kb = owner[b];
                    if (kb < 0 || ka == kb || need[ka][b] == 0 || need[kb][a] == 0) continue;
                    if (need[ka][b] > budgets[ka] - used[ka] + need[ka][a] ||
                        need[kb][a] > budgets[kb] - used[kb] + need[kb][b]) continue;
                    consider(MoveKind::Swap, a, b, -1, 0.0,
                        (need[ka][a] - need[ka][b]) + (need[kb][b] - need[kb][a]));
                }
            }
            if (!best.valid) {
                counters.local_search_stop_reason = "no_improving_move";
                break;
            }

            // 提交唯一获选操作，仅更新涉及的归属和UAV槽计数；本轮不再评估其他操作。
            const int a = best.first, b = best.second, target = best.target;
            switch (best.kind) {
            case MoveKind::Admit:
                owner[a] = target; used[target] += need[target][a];
                ++counters.local_search_admissions;
                break;
            case MoveKind::Replace:
                used[target] = used[target] - need[target][a] + need[target][b];
                owner[a] = -1; owner[b] = target;
                ++counters.local_search_replacements;
                break;
            case MoveKind::Relocate:
                used[owner[a]] -= need[owner[a]][a];
                used[target] += need[target][a]; owner[a] = target;
                ++counters.local_search_migrations;
                break;
            case MoveKind::Swap: {
                const int ka = owner[a], kb = owner[b];
                used[ka] = used[ka] - need[ka][a] + need[ka][b];
                used[kb] = used[kb] - need[kb][b] + need[kb][a];
                owner[a] = kb; owner[b] = ka;
                ++counters.local_search_swaps;
                break;
            }
            }
            for (int k = 0; k < m; ++k)
                if (used[k] < 0 || used[k] > budgets[k]) throw std::logic_error("Hard local search budget invariant failed");
            ++accepted; ++counters.local_search_improvements;
        }
        if (accepted == 0) return; // 无改进时保留槽ID和冻结状态，不影响后续elastic的申请顺序。

        // 仅在搜索结束时重建一次真实槽位；按用户ID与槽ID升序分配单槽合同。
        std::vector<std::vector<int>> by_uav(m);
        for (int s = 0; s < static_cast<int>(slots.size()); ++s) {
            by_uav[slots[s].uav].push_back(s);
            slots[s].owner = -1; slots[s].phase = SlotPhase::Vacant;
        }
        for (int j = 0; j < hard_count; ++j) { held[j].clear(); frozen[j] = false; }
        std::vector<int> cursor(m, 0);
        for (int j = 0; j < hard_count; ++j) {
            const int k = owner[j];
            if (k < 0) continue;
            for (int q = 0; q < need[k][j]; ++q) {
                const int s = by_uav[k].at(cursor[k]++);
                slots[s].owner = j; slots[s].phase = SlotPhase::FrozenHard;
                held[j].insert(s);
            }
            frozen[j] = true;
        }
        for (int k = 0; k < m; ++k)
            if (cursor[k] != used[k]) throw std::logic_error("Hard local search slot reconstruction mismatch");
    }

    /// Exact elastic utility gain using the frozen round-start count, excluding
    /// the contested slot for its incumbent. Zero-weight users never apply.
    double elastic_score(int j, int s, const std::vector<int>& counts) const {
        const int base = counts[j] - (slots[s].owner == j ? 1 : 0);
        if (base < 0) throw std::logic_error("DA incumbent count underflow");
        const double alpha = width * model.cap_list[slots[s].uav][j];
        const double rate = base * alpha;
        if (!std::isfinite(alpha) || !std::isfinite(rate))
            throw std::overflow_error("DA elastic rate overflow");
        const double gain = model.users[j].weight * std::log1p(alpha / (1 + rate)) / std::log(2.0);
        if (!std::isfinite(gain)) throw std::overflow_error("DA elastic score overflow");
        return gain;
    }

    /// 在 hard 冻结后的剩余槽上按真实 elastic 边际效用匹配；已持槽用户不跨 UAV。
    /// 每个用户-槽组合最多申请一次，返回时 hard 占用保持不变。
    void elastic_pass() {
        const auto available = vacant_lists();
        const auto prefs = elastic_preferences(available);
        std::vector<size_t> uav_cursor(n, 0), slot_cursor(n, 0);
        for (;;) {
            std::vector<bool> switched(n, false);
            bool moved = false;
            for (int j = model.n1; j < n; ++j) {
                if (uav_cursor[j] == prefs[j].size()) continue;
                const int k = prefs[j][uav_cursor[j]];
                if (slot_cursor[j] == available[k].size() && held[j].empty()) {
                    ++uav_cursor[j]; slot_cursor[j] = 0; switched[j] = true; moved = true;
                }
            }
            std::map<int, std::vector<int>> incoming;
            std::vector<int> snapshot(n);
            for (int j = model.n1; j < n; ++j) {
                snapshot[j] = static_cast<int>(held[j].size());
                if (switched[j] || uav_cursor[j] == prefs[j].size()) continue;
                const int k = prefs[j][uav_cursor[j]];
                if (slot_cursor[j] == available[k].size()) continue;
                incoming[available[k][slot_cursor[j]++]].push_back(j);
                ++counters.elastic_proposals;
            }
            if (incoming.empty() && !moved) break;
            ++counters.elastic_rounds;
            std::vector<std::pair<int, int>> decisions;
            for (const auto& requests : incoming) {
                const int s = requests.first;
                int winner = slots[s].owner;
                double best = winner < 0 ? 0 : elastic_score(winner, s, snapshot);
                for (int j : requests.second) {
                    const double score = elastic_score(j, s, snapshot);
                    if (score > best || (score == best && score > 0 && (winner < 0 || j < winner))) {
                        best = score; winner = j;
                    }
                }
                if (winner >= 0) decisions.push_back({s, winner});
            }
            commit_elastic(decisions);
        }
    }

public:
    /// 构造有限槽位并校验模型维度；hard 门槛取传入模型的 rMin。
    explicit HardFirstAllocator(const SystemMd& input) : model(input), width(hard_first_width(input)),
        n(static_cast<int>(input.users.size())), m(input.m), held(n), frozen(n, false),
        hard_value(input.n1 >= 0 ? input.n1 : 0) {
        if (m < 0 || m != static_cast<int>(model.uavs.size()) || model.n1 < 0 || model.n2 < 0 ||
            static_cast<int64_t>(model.n1) + model.n2 != n ||
            model.cap_list.size() != static_cast<size_t>(m) || model.dis_list.size() != static_cast<size_t>(m) ||
            model.SNRave_list.size() != static_cast<size_t>(m))
            throw std::invalid_argument("Invalid HardFirst dimensions");
        std::vector<int> budgets(m);
        int total = 0;
        for (int k = 0; k < m; ++k) {
            if (model.uavs[k].ID != k || model.cap_list[k].size() != static_cast<size_t>(n) ||
                model.dis_list[k].size() != static_cast<size_t>(n) || model.SNRave_list[k].size() != static_cast<size_t>(n))
                throw std::invalid_argument("Invalid HardFirst UAV/matrix");
            budgets[k] = hard_first_slot_count(model.uavs[k].total_bandwidth, width);
            if (budgets[k] > INT_MAX - total) throw std::overflow_error("HardFirst total slot overflow");
            total += budgets[k];
            for (int j = 0; j < n; ++j)
                if (!std::isfinite(model.cap_list[k][j]) || model.cap_list[k][j] < 0 ||
                    !std::isfinite(model.dis_list[k][j]) || model.dis_list[k][j] < 0)
                    throw std::invalid_argument("Invalid HardFirst link");
        }
        for (int j = 0; j < n; ++j) {
            const auto& user = model.users[j];
            if (user.ID != j || user.uType != (j < model.n1 ? HARD_UTILITY : ELASTIC_UTILITY) ||
                !std::isfinite(user.weight) || user.weight < 0 || !std::isfinite(user.rMin) || user.rMin < 0)
                throw std::invalid_argument("Invalid HardFirst user");
            if (j < model.n1) {
                hard_value[j] = user.weight * std::log2(1 + user.rMin);
                if (!std::isfinite(hard_value[j])) throw std::overflow_error("HardFirst hard utility overflow");
            }
        }
        links = hard_first_links(model);
        // No resource allocation is required for an empty user population.
        if (n == 0) return;
        slots.reserve(total);
        for (int k = 0; k < m; ++k)
            for (int s = 0; s < budgets[k]; ++s) slots.push_back({k, s});
    }

    /// 依次执行矩形 Hungarian、有界 hard 搜索和剩余槽 elastic 匹配，并分别测量真实毫秒耗时。
    /// candidate_id指定固定排列；返回独立候选的分配、效用、hard槽归属及计时，不发布诊断事件。
    HardFirstCandidate run(int candidate_id) {
        const auto matching_start = std::chrono::steady_clock::now();
        match_hard_single_slots(candidate_id);
        const auto search_start = std::chrono::steady_clock::now();
        improve_hard_allocation();
        const auto elastic_start = std::chrono::steady_clock::now();
        elastic_pass();
        const auto elastic_end = std::chrono::steady_clock::now();
        // 三个子区间不含构造器和结果构造；外层 duration_ms 继续测量完整求解调用。
        counters.hard_matching_ms = std::chrono::duration<double, std::milli>(search_start - matching_start).count();
        counters.hard_local_search_ms = std::chrono::duration<double, std::milli>(elastic_start - search_start).count();
        counters.elastic_ms = std::chrono::duration<double, std::milli>(elastic_end - elastic_start).count();
        HardFirstCandidate candidate;
        std::vector<KnapsackResult> results(m);
        std::map<int, UserResult> users;
        for (int k = 0; k < m; ++k) results[k].uav_id = k;
        for (int j = 0; j < n; ++j) {
            users[j] = UserResult();
            if (held[j].empty()) continue;
            const int k = slots[*held[j].begin()].uav;
            for (int s : held[j])
                if (slots[s].owner != j || slots[s].uav != k)
                    throw std::logic_error("HardFirst ownership/association mismatch");
            const double bw = j < model.n1
                ? model.users[j].rMin / model.cap_list[k][j]
                : held[j].size() * width;
            User user = model.users[j];
            const double value = j < model.n1 ? user.hard_utility(bw, model.cap_list[k][j], model.SNRave_list[k][j])
                : user.elastic_utility(bw, model.cap_list[k][j]);
            if (!std::isfinite(value)) throw std::overflow_error("HardFirst output utility overflow");
            add_KnapsackResult(results[k], user, bw, value);
            users[j] = {k, bw, value};
            if (j < model.n1) candidate.hard_utility += value;
            candidate.total_utility += value;
        }
        if (!std::isfinite(candidate.hard_utility) || !std::isfinite(candidate.total_utility))
            throw std::overflow_error("HardFirst candidate utility overflow");
        for (const auto& slot : slots)
            candidate.hard_slot_owners.push_back(slot.phase == SlotPhase::FrozenHard ? slot.owner : -1);
        candidate.counters = counters;
        candidate.allocation = {std::move(results), std::move(users)};
        return candidate;
    }
};

/// 顺序生成原序列前candidate_count个候选（1--8），返回总效用最佳分配并发布实际累计诊断。
/// model为最低等级求解视图；无有效hard边时仅求候选0，错误均携带候选编号，不用耗时决定预算。
HardFirstAllocation run_hardfirst_candidates(const SystemMd& model, AllocationDiagnostics* diagnostics,
    int candidate_count) {
    const std::string mechanism = hard_first_mechanism(candidate_count);
    HardFirstAllocation best;
    std::map<std::vector<int>, int> hard_patterns;
    json candidates = json::array();
    double reference_hard = 0, reference_total = 0, best_total = 0;
    double matching_ms = 0, search_ms = 0, elastic_ms = 0;
    uint64_t augmentations = 0;
    int selected = 0, limit = candidate_count;
    for (int id = 0; id < limit; ++id) {
        try {
            auto candidate = HardFirstAllocator(model).run(id);
            validate_hard_first_allocation(model, candidate.allocation.first);
            if (id == 0) {
                reference_hard = candidate.hard_utility;
                reference_total = candidate.total_utility;
                if (candidate.counters.hard_eligible_edges == 0) limit = 1;
            } else if (!allocation_near(candidate.hard_utility, reference_hard)) {
                throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation,
                    "permuted matching changed the optimal hard utility");
            }
            const auto inserted = hard_patterns.emplace(candidate.hard_slot_owners, id);
            json detail = candidate.counters.to_json();
            detail["candidate_id"] = id;
            detail["permutation_seed"] = id == 0 ? json(nullptr) :
                json(HARD_FIRST_CANDIDATE_BASE_SEED + static_cast<uint32_t>(id));
            detail["duplicate_of"] = inserted.first->second; // 首次出现的方案指向自身编号。
            detail["hard_utility"] = candidate.hard_utility;
            detail["total_utility"] = candidate.total_utility;
            candidates.push_back(std::move(detail));
            matching_ms += candidate.counters.hard_matching_ms;
            search_ms += candidate.counters.hard_local_search_ms;
            elastic_ms += candidate.counters.elastic_ms;
            augmentations += candidate.counters.hard_hungarian_augmentations;
            // 已确认hard值一致；仅在总效用超过现有容差时替换，因此平局始终保留较小编号。
            if (id == 0 || candidate.total_utility > best_total +
                allocation_tolerance(candidate.total_utility, best_total)) {
                selected = id;
                best_total = candidate.total_utility;
                best = std::move(candidate.allocation);
            }
        } catch (const AllocationFailure& error) {
            throw AllocationFailure(error.status,
                "HardFirst candidate " + std::to_string(id) + ": " + error.what());
        } catch (const std::exception& error) {
            throw std::runtime_error("HardFirst candidate " + std::to_string(id) + ": " + error.what());
        }
    }
    if (diagnostics) {
        json summary = {{"algorithm", mechanism},
            {"hard_hungarian_rows", candidates.front().at("hard_hungarian_rows")},
            {"hard_hungarian_columns", candidates.front().at("hard_hungarian_columns")},
            {"candidate_count_requested", candidate_count},
            {"candidate_count_completed", candidates.size()},
            {"distinct_hard_candidates", hard_patterns.size()}, {"selected_candidate", selected},
            {"hard_hungarian_augmentations", augmentations},
            {"hard_matching_ms", matching_ms}, {"hard_local_search_ms", search_ms}, {"elastic_ms", elastic_ms},
            {"candidate0_total_utility", reference_total}, {"gain_over_candidate0", best_total - reference_total},
            {"hard_utility", candidates.at(selected).at("hard_utility")}, {"total_utility", best_total},
            {"candidates", std::move(candidates)}};
        diagnostics->events.push_back(summary.dump());
    }
    return best;
}
} // namespace


/// 无诊断参数的 HardFirst 入口；正式方法由包装层先传入最低等级视图。
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HardFirstPriorityMatchingAllocation() {
    return HardFirstPriorityMatchingAllocation(nullptr);
}

/// 八候选矩形 Hungarian 入口：按当前门槛校验各候选并择优，diagnostics接收累计耗时及候选明细。
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HardFirstPriorityMatchingAllocation(AllocationDiagnostics* diagnostics) {
    return HardFirstPriorityMatchingAllocation(diagnostics, HARD_FIRST_CANDIDATE_COUNT);
}

/// 显式候选预算入口；candidate_count限定原序列前缀，返回分配，diagnostics记录真实候选数及累计计时。
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HardFirstPriorityMatchingAllocation(AllocationDiagnostics* diagnostics, int candidate_count) {
    return run_hardfirst_candidates(sysModel, diagnostics, candidate_count);
}

/// @brief 恢复 34d27a72 的 Hungarian/KM 分配，hard 与 elastic 同时匹配且每用户至多一槽。
/// @return 按 UAV 排列的分配和逐用户结果；空用户集返回空分配。
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HungarianMatchingAllocation()
{
    // 直接调用时保护空用户实例；非空实例仍逐步执行 34d27a72 的历史决策。
    if (sysModel.users.empty()) {
        std::vector<KnapsackResult> empty(sysModel.m);
        for (int k = 0; k < sysModel.m; ++k) empty[k].uav_id = k;
        return {empty, construct_user_results(empty)};
    }

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
    // 返回按 1 起始的用户行和子信道列对应的可写代价矩阵元素。
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

/// @file HungarianMatching.cpp
/// @brief Youssef-inspired priority-aware subchannel DA with single-UAV association.
/// Historical filename/API retained. Static normalized-demand hard preferences,
/// finite recovery and elastic marginal preferences are adaptations, not a full
/// DA/NOMA reproduction, classical stability proof or global-optimality claim.
#include "EntityDefinition.h"
#include <set>
#include <cstdint>

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

/// Independently validate effective integer slots, guard-inclusive budgets and
/// minimal hard bundles. It never repairs output; common validation follows.
void validate_hard_first_allocation(const SystemMd& model,
    const std::vector<KnapsackResult>& results) {
    const double width = hard_first_width(model);
    if (results.size() != model.uavs.size())
        throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first result dimension");
    for (const auto& result : results) {
        const int k = result.uav_id;
        if (k < 0 || k >= model.m)
            throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first UAV ID");
        const int budget = hard_first_slot_count(model.uavs[k].total_bandwidth, width);
        int used = 0;
        for (const auto& entry : result.allocatedBandwidth) {
            const int j = entry.first;
            const double bw = entry.second, slots = std::round(bw / width);
            if (j < 0 || j >= static_cast<int>(model.users.size()) ||
                !std::isfinite(bw) || !std::isfinite(slots) || slots < 1 || slots > INT_MAX ||
                std::abs(bw / width - slots) >
                    16 * std::numeric_limits<double>::epsilon() * std::max(1.0, slots))
                throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first noninteger/invalid bundle");
            const int count = static_cast<int>(slots);
            if (count > budget - used)
                throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first guard-inclusive budget exceeded");
            used += count;
            if (model.users[j].uType == HARD_UTILITY) {
                const double cap = model.cap_list.at(k).at(j), rate = model.users[j].rMin;
                if (!hard_qos_satisfied(bw, cap, rate) ||
                    (count > 1 && hard_qos_satisfied((count - 1) * width, cap, rate)))
                    throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first nonminimal/unsatisfied hard bundle");
            }
        }
        const double occupied = used * (width * (10.0 / 9.0));
        if (!std::isfinite(occupied) || occupied > model.uavs[k].total_bandwidth +
            allocation_tolerance(occupied, model.uavs[k].total_bandwidth))
            throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, "Hard-first occupied bandwidth exceeded");
    }
}

namespace {
/// A physical resource slot; phase distinguishes vacant, provisional hard,
/// frozen hard and provisional elastic ownership without duplicating bandwidth.
enum class DASlotPhase { Vacant, Hard, FrozenHard, Elastic };
struct DASlot { int uav, local, owner = -1; DASlotPhase phase = DASlotPhase::Vacant; };

/// Aggregate actual DA work. Counts are diagnostics, not runtime targets.
struct DACounters {
    uint64_t hard_proposals = 0, hard_replacements = 0, hard_reactivations = 0;
    uint64_t hard_switches = 0, hard_released_slots = 0, hard_rounds = 0;
    uint64_t hard_passes = 0, recovery_rounds = 0, recovery_admissions = 0;
    uint64_t elastic_proposals = 0, elastic_replacements = 0, elastic_rounds = 0;
    uint64_t hard_proposal_bound = 0, elastic_proposal_bound = 0;
    /// Serialize counters once through the existing diagnostics event channel.
    json to_json() const {
        return {{"algorithm", "hardfirst-subchannel-da-v1"},
            {"hard_proposals", hard_proposals}, {"hard_replacements", hard_replacements},
            {"hard_reactivations", hard_reactivations}, {"hard_switches", hard_switches},
            {"hard_released_slots", hard_released_slots}, {"hard_rounds", hard_rounds},
            {"hard_passes", hard_passes}, {"recovery_rounds", recovery_rounds},
            {"recovery_admissions", recovery_admissions}, {"elastic_proposals", elastic_proposals},
            {"elastic_replacements", elastic_replacements}, {"elastic_rounds", elastic_rounds},
            {"hard_proposal_bound", hard_proposal_bound}, {"elastic_proposal_bound", elastic_proposal_bound}};
    }
};

/// Engine for static-capacity, single-UAV, priority-aware subchannel DA.
/// It stores real slot ownership and lazy per-UAV preference cursors; it does
/// not reserve bundles, fabricate frequency variation, or perform global heap allocation.
class SubchannelDA {
    const SystemMd& model;
    double width;
    int n, m;
    std::vector<DASlot> slots;
    std::vector<std::set<int>> held;
    std::vector<bool> frozen;
    std::vector<double> hard_value;
    std::vector<std::vector<bool>> links;
    DACounters counters;

    /// Return available slot IDs per UAV, in local-ID order, at the start of a pass.
    std::vector<std::vector<int>> vacant_lists() const {
        std::vector<std::vector<int>> lists(m);
        for (int s = 0; s < static_cast<int>(slots.size()); ++s)
            if (slots[s].owner == -1) lists[slots[s].uav].push_back(s);
        return lists;
    }

    /// Build descending per-slot-rate UAV preferences. For hard, discard UAVs
    /// that cannot independently meet demand using this pass's available slots.
    std::vector<std::vector<int>> preferences(const std::vector<std::vector<int>>& available,
        bool hard) {
        std::vector<std::vector<int>> prefs(n);
        for (int j = hard ? 0 : model.n1; j < (hard ? model.n1 : n); ++j) {
            if (frozen[j] || (!hard && model.users[j].weight == 0)) continue;
            for (int k = 0; k < m; ++k) {
                if (!links[k][j] || available[k].empty()) continue;
                if (hard && hard_first_min_slots(static_cast<int>(available[k].size()),
                    width, model.cap_list[k][j], model.users[j].rMin) == 0) continue;
                prefs[j].push_back(k);
                if (hard) counters.hard_proposal_bound += available[k].size();
                else counters.elastic_proposal_bound += available[k].size();
            }
            // Width is common, so sorting capacity is exactly sorting per-slot rate.
            std::sort(prefs[j].begin(), prefs[j].end(), [&](int a, int b) {
                if (model.cap_list[a][j] != model.cap_list[b][j])
                    return model.cap_list[a][j] > model.cap_list[b][j];
                return a < b;
            });
        }
        return prefs;
    }

    /// Return whether a positive, single-UAV held set meets actual reliable QoS.
    bool satisfied(int j) const {
        if (held[j].empty()) return false;
        const int k = slots[*held[j].begin()].uav;
        return hard_qos_satisfied(held[j].size() * width, model.cap_list[k][j], model.users[j].rMin);
    }

    /// Compare hard contenders on UAV k: zero demand, rate/demand, utility, ID.
    /// Long-double products avoid dividing by zero or overflowing a double ratio.
    bool hard_prefers(int a, int b, int k) const {
        if (b == -1) return true;
        const auto& ua = model.users[a]; const auto& ub = model.users[b];
        if ((ua.rMin == 0) != (ub.rMin == 0)) return ua.rMin == 0;
        if (ua.rMin != 0) {
            // Compare ratios after scaling denominators and capacities into [0,1].
            const double cscale = std::max(model.cap_list[k][a], model.cap_list[k][b]);
            const double rscale = std::max(ua.rMin, ub.rMin);
            const long double left = static_cast<long double>(model.cap_list[k][a] / cscale) * (ub.rMin / rscale);
            const long double right = static_cast<long double>(model.cap_list[k][b] / cscale) * (ua.rMin / rscale);
            if (left != right) return left > right;
        }
        if (hard_value[a] != hard_value[b]) return hard_value[a] > hard_value[b];
        return a < b;
    }

    /// Release a failed hard user's partial allocation, with no change to others.
    void release_hard(int j) {
        for (int s : held[j]) {
            slots[s].owner = -1; slots[s].phase = DASlotPhase::Vacant;
            ++counters.hard_released_slots;
        }
        held[j].clear();
    }

    /// Apply already-decided winners together: remove all losing memberships
    /// first, then add winners. No preference comparison observes partial updates.
    void commit(const std::vector<std::pair<int, int>>& decisions, bool hard) {
        std::vector<int> completed_losers;
        for (const auto& decision : decisions) {
            const int old = slots[decision.first].owner;
            if (old == decision.second) continue;
            if (old >= 0) {
                if (hard && satisfied(old)) completed_losers.push_back(old);
                held[old].erase(decision.first);
                if (hard) ++counters.hard_replacements;
                else ++counters.elastic_replacements;
            }
        }
        for (const auto& decision : decisions) {
            auto& slot = slots[decision.first];
            slot.owner = decision.second;
            slot.phase = hard ? DASlotPhase::Hard : DASlotPhase::Elastic;
            held[decision.second].insert(decision.first);
        }
        std::sort(completed_losers.begin(), completed_losers.end());
        completed_losers.erase(std::unique(completed_losers.begin(), completed_losers.end()), completed_losers.end());
        for (int j : completed_losers)
            if (!satisfied(j)) ++counters.hard_reactivations;
    }

    /// Execute one finite hard pass over current empty slots; return newly
    /// completed users. Rejections reset only between recovery passes.
    int hard_pass(bool recovery) {
        const auto available = vacant_lists();
        const auto prefs = preferences(available, true);
        bool has_candidates = false;
        for (int j = 0; j < model.n1; ++j) has_candidates = has_candidates || !prefs[j].empty();
        if (!has_candidates) return 0;
        ++counters.hard_passes;
        if (recovery) ++counters.recovery_rounds;
        std::vector<size_t> uav_cursor(n, 0), slot_cursor(n, 0);
        for (;;) {
            std::vector<bool> switched(n, false);
            bool moved = false;
            // Exhausted failed users release simultaneously before new proposals.
            for (int j = 0; j < model.n1; ++j) {
                if (frozen[j] || satisfied(j) || uav_cursor[j] == prefs[j].size()) continue;
                const int k = prefs[j][uav_cursor[j]];
                if (slot_cursor[j] == available[k].size()) {
                    release_hard(j); ++uav_cursor[j]; slot_cursor[j] = 0;
                    switched[j] = true; moved = true;
                    if (uav_cursor[j] < prefs[j].size()) ++counters.hard_switches;
                }
            }
            std::map<int, std::vector<int>> incoming;
            for (int j = 0; j < model.n1; ++j) {
                if (frozen[j] || switched[j] || satisfied(j) || uav_cursor[j] == prefs[j].size()) continue;
                const int k = prefs[j][uav_cursor[j]];
                incoming[available[k][slot_cursor[j]++]].push_back(j);
                ++counters.hard_proposals;
            }
            if (incoming.empty() && !moved) break;
            ++counters.hard_rounds;
            std::vector<std::pair<int, int>> decisions;
            for (const auto& requests : incoming) {
                const int s = requests.first, k = slots[s].uav;
                int winner = slots[s].owner;
                for (int j : requests.second)
                    if (hard_prefers(j, winner, k)) winner = j;
                decisions.push_back({s, winner});
            }
            commit(decisions, true);
        }
        int completed = 0;
        for (int j = 0; j < model.n1; ++j) {
            if (frozen[j]) continue;
            if (satisfied(j)) {
                frozen[j] = true; ++completed;
                for (int s : held[j]) slots[s].phase = DASlotPhase::FrozenHard;
            } else release_hard(j);
        }
        if (recovery) counters.recovery_admissions += completed;
        return completed;
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

    /// Match remaining slots with dynamic marginal preferences and frozen counts.
    /// Each user-slot pair is visited once; positive holdings prevent UAV migration.
    void elastic_pass() {
        const auto available = vacant_lists();
        const auto prefs = preferences(available, false);
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
            commit(decisions, false);
        }
    }

public:
    /// Construct finite indexed slot state and validate inputs before direct API use.
    explicit SubchannelDA(const SystemMd& input) : model(input), width(hard_first_width(input)),
        n(static_cast<int>(input.users.size())), m(input.m), held(n), frozen(n, false),
        hard_value(input.n1 >= 0 ? input.n1 : 0) {
        if (m < 0 || m != static_cast<int>(model.uavs.size()) || model.n1 < 0 || model.n2 < 0 ||
            static_cast<int64_t>(model.n1) + model.n2 != n ||
            model.cap_list.size() != static_cast<size_t>(m) || model.dis_list.size() != static_cast<size_t>(m) ||
            model.SNRave_list.size() != static_cast<size_t>(m))
            throw std::invalid_argument("Invalid DA dimensions");
        std::vector<int> budgets(m);
        int total = 0;
        for (int k = 0; k < m; ++k) {
            if (model.uavs[k].ID != k || model.cap_list[k].size() != static_cast<size_t>(n) ||
                model.dis_list[k].size() != static_cast<size_t>(n) || model.SNRave_list[k].size() != static_cast<size_t>(n))
                throw std::invalid_argument("Invalid DA UAV/matrix");
            budgets[k] = hard_first_slot_count(model.uavs[k].total_bandwidth, width);
            if (budgets[k] > INT_MAX - total) throw std::overflow_error("DA total slot overflow");
            total += budgets[k];
            for (int j = 0; j < n; ++j)
                if (!std::isfinite(model.cap_list[k][j]) || model.cap_list[k][j] < 0 ||
                    !std::isfinite(model.dis_list[k][j]) || model.dis_list[k][j] < 0)
                    throw std::invalid_argument("Invalid DA link");
        }
        for (int j = 0; j < n; ++j) {
            const auto& user = model.users[j];
            if (user.ID != j || user.uType != (j < model.n1 ? HARD_UTILITY : ELASTIC_UTILITY) ||
                !std::isfinite(user.weight) || user.weight < 0 || !std::isfinite(user.rMin) || user.rMin < 0)
                throw std::invalid_argument("Invalid DA user");
            if (j < model.n1) {
                hard_value[j] = user.weight * std::log2(1 + user.rMin);
                if (!std::isfinite(hard_value[j])) throw std::overflow_error("DA hard utility overflow");
            }
        }
        links = hard_first_links(model);
        // No resource allocation is required for an empty user population.
        if (n == 0) return;
        slots.reserve(total);
        for (int k = 0; k < m; ++k)
            for (int s = 0; s < budgets[k]; ++s) slots.push_back({k, s});
    }

    /// Run main hard DA, finite recovery, elastic DA and project final effective
    /// allocations. Append a single aggregate diagnostic event if requested.
    std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>> run(AllocationDiagnostics* diagnostics) {
        hard_pass(false);
        // A recovery continues only after a strict increase in frozen hard users.
        while (hard_pass(true) > 0) {}
        elastic_pass();
        std::vector<KnapsackResult> results(m);
        std::map<int, UserResult> users;
        for (int k = 0; k < m; ++k) results[k].uav_id = k;
        for (int j = 0; j < n; ++j) {
            users[j] = UserResult();
            if (held[j].empty()) continue;
            const int k = slots[*held[j].begin()].uav;
            for (int s : held[j])
                if (slots[s].owner != j || slots[s].uav != k)
                    throw std::logic_error("DA ownership/association mismatch");
            const double bw = held[j].size() * width;
            User user = model.users[j];
            const double value = j < model.n1 ? user.hard_utility(bw, model.cap_list[k][j], model.SNRave_list[k][j])
                : user.elastic_utility(bw, model.cap_list[k][j]);
            if (!std::isfinite(value)) throw std::overflow_error("DA output utility overflow");
            add_KnapsackResult(results[k], user, bw, value);
            users[j] = {k, bw, value};
        }
        validate_hard_first_allocation(model, results);
        if (diagnostics) diagnostics->events.push_back(counters.to_json().dump());
        return {results, users};
    }
};
} // namespace


/// Source-compatible entry: perform subchannel DA without collecting diagnostics.
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HardFirstPriorityMatchingAllocation() {
    return HardFirstPriorityMatchingAllocation(nullptr);
}

/// Diagnostic overload: real proposal/replacement counts accompany the unchanged
/// allocation pair. No artificial delay, padding iterations or runtime target.
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HardFirstPriorityMatchingAllocation(AllocationDiagnostics* diagnostics) {
    return SubchannelDA(sysModel).run(diagnostics);
}

/// Historical name retained only as a source-compatible DA forwarding alias.
std::pair<std::vector<KnapsackResult>, std::map<int, UserResult>>
BAProblem::HungarianMatchingAllocation() {
    return HardFirstPriorityMatchingAllocation();
}

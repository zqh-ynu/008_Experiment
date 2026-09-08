#pragma once
// Shared numerical comparisons and failure reporting for the Route A allocators.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

inline constexpr double ALLOCATION_ABS_TOL = 1e-9;
inline constexpr double ALLOCATION_REL_TOL = 1e-8;

/// Return the common absolute-plus-relative tolerance in the operands' units.
inline double allocation_tolerance(double a, double b) {
    return ALLOCATION_ABS_TOL + ALLOCATION_REL_TOL * std::max(std::abs(a), std::abs(b));
}

/// Compare finite values without allowing infinities or NaNs to pass validation.
inline bool allocation_near(double a, double b) {
    return std::isfinite(a) && std::isfinite(b) &&
        std::abs(a - b) <= allocation_tolerance(a, b);
}

/// Test the reliable-rate requirement using the same rule as the reported utility.
inline bool hard_qos_satisfied(double bandwidth, double capacity, double minimum_rate) {
    const double rate = bandwidth * capacity;
    return std::isfinite(bandwidth) && bandwidth >= 0.0 &&
        std::isfinite(capacity) && capacity > 0.0 &&
        std::isfinite(minimum_rate) && minimum_rate >= 0.0 &&
        std::isfinite(rate) && rate + allocation_tolerance(rate, minimum_rate) >= minimum_rate;
}

/// Terminal outcomes; lack of a rounded candidate is not a proof of infeasibility.
enum class AlgorithmRunStatus {
    Success, ZeroAllocation, SolverFailure, NoFeasibleCandidate, InvalidAllocation, Exception
};

/// Return stable machine-readable status names used by the versioned CSV schema.
inline std::string algorithm_status_name(AlgorithmRunStatus status) {
    switch (status) {
    case AlgorithmRunStatus::Success: return "SUCCESS";
    case AlgorithmRunStatus::ZeroAllocation: return "ZERO_ALLOCATION";
    case AlgorithmRunStatus::SolverFailure: return "SOLVER_FAILURE";
    case AlgorithmRunStatus::NoFeasibleCandidate: return "NO_FEASIBLE_CANDIDATE";
    case AlgorithmRunStatus::InvalidAllocation: return "INVALID_ALLOCATION";
    case AlgorithmRunStatus::Exception: return "EXCEPTION";
    }
    throw std::invalid_argument("Unknown algorithm status");
}

/// Parse a status strictly; unrecognized or truncated records must not be resumed.
inline AlgorithmRunStatus parse_algorithm_status(const std::string& name) {
    for (auto s : { AlgorithmRunStatus::Success, AlgorithmRunStatus::ZeroAllocation,
        AlgorithmRunStatus::SolverFailure, AlgorithmRunStatus::NoFeasibleCandidate,
        AlgorithmRunStatus::InvalidAllocation, AlgorithmRunStatus::Exception })
        if (algorithm_status_name(s) == name) return s;
    throw std::invalid_argument("Unknown recorded status: " + name);
}

/// Only completed feasible outputs, including a legitimate empty allocation, are valid.
inline bool algorithm_status_valid(AlgorithmRunStatus status) {
    return status == AlgorithmRunStatus::Success || status == AlgorithmRunStatus::ZeroAllocation;
}

/// Optional solver diagnostics; fallback messages do not replace feasibility checks.
struct AllocationDiagnostics {
    std::vector<std::string> events;
    bool used_fallback = false;

    /// Record the actual stage and IPOPT return code, optionally marking a fallback.
    void record(const std::string& stage, int code, bool fallback = false) {
        events.push_back(stage + ": solver_status=" + std::to_string(code) +
            (fallback ? "; retained fallback, subject to final validation" : ""));
        used_fallback = used_fallback || fallback;
    }
};

/// Carry a typed algorithm failure through the legacy pair-returning interfaces.
class AllocationFailure : public std::runtime_error {
public:
    AlgorithmRunStatus status;
    /// Construct a failure with a terminal status and a human-readable stage/reason.
    AllocationFailure(AlgorithmRunStatus value, const std::string& message)
        : std::runtime_error(message), status(value) {}
};

/// Normalize numerical zero only; a NaN or materially negative solver value must not masquerade as no allocation.
inline double checked_solver_bandwidth(double value, const std::string& stage) {
    if (!std::isfinite(value) || value < -ALLOCATION_ABS_TOL)
        throw AllocationFailure(AlgorithmRunStatus::InvalidAllocation, stage + ": invalid solver bandwidth");
    return std::max(0.0, value);
}

/// Generate an explicitly specified uniform variate, independent of STL distributions.
inline double allocation_uniform01(std::mt19937& generator) {
    return static_cast<double>(generator()) / 4294967296.0;
}

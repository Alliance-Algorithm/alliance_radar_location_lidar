// pcd_trigger.hpp — PCD save trigger + time-tolerance utilities
//
// Task 5 (M5 fix) of the FAST-LIVO2 RGB map/replay plan.
// Pure functions with zero dependencies — extracted for direct unit testability.

#pragma once

#include <cstdint>

namespace radar::fast_livo2::rgb {

/// Transition a PCD save trigger after a save attempt.
/// Consumes the trigger on success; retains it on failure so the
/// caller can retry on the next poll cycle.
///
/// \param trigger       Current trigger state (true = save requested).
/// \param save_result   0 = success, non-zero = failure.
/// \return              New trigger state to write back.
inline auto pcd_trigger_transition(bool trigger, int save_result) -> bool {
    if (!trigger) return false;
    return (save_result != 0);
}

/// Check whether a timestamp delta (in nanoseconds, absolute value)
/// falls within a configured tolerance window.
inline auto within_tolerance_ns(int64_t delta_ns, int64_t tolerance_ns) -> bool {
    return delta_ns >= 0 && delta_ns <= tolerance_ns;
}

} // namespace radar::fast_livo2::rgb

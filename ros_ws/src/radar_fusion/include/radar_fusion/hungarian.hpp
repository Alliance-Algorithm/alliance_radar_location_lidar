#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

/// Kuhn-Munkres (Hungarian) minimum-cost assignment.
///
/// Solves: given an n x m cost matrix, assign each row to a distinct column
/// minimizing total cost. Rectangular matrices: unmatched rows (columns
/// exhausted) get -1. Rows whose best feasible cost exceeds `unreachable`
/// are treated as unmatched.
namespace radar_fusion::association {

/// Returns assignments[i] = matched column for row i, or -1 if unmatched.
inline auto hungarian_min_cost(const std::vector<std::vector<double>>& cost,
    double unreachable = 1e9) -> std::vector<int> {
    const std::size_t n = cost.size();
    if (n == 0) return { };
    std::size_t m = cost[0].size();
    for (const auto& row : cost) {
        m = std::max(m, row.size());
    }

    // Pad to square n x n (n >= m) with unreachable cost.
    const std::size_t size = std::max(n, m);
    std::vector<std::vector<double>> c(size, std::vector<double>(size, unreachable));
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = 0; j < cost[i].size(); ++j) {
            c[i][j] = cost[i][j];
        }
    }

    // Kuhn-Munkres with potentials (O(n^3)).
    std::vector<double> u(size + 1, 0.0), v(size + 1, 0.0);
    std::vector<int> p(size + 1, 0), way(size + 1, 0);
    for (std::size_t i = 1; i <= size; ++i) {
        p[0] = static_cast<int>(i);
        std::size_t j0 = 0;
        std::vector<double> minv(size + 1, unreachable);
        std::vector<bool> used(size + 1, false);
        do {
            used[j0] = true;
            const std::size_t i0 = static_cast<std::size_t>(p[j0]);
            double delta = unreachable;
            std::size_t j1 = 0;
            for (std::size_t j = 1; j <= size; ++j) {
                if (!used[j]) {
                    const double cur = c[i0 - 1][j - 1] - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j]   = static_cast<int>(j0);
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1    = j;
                    }
                }
            }
            for (std::size_t j = 0; j <= size; ++j) {
                if (used[j]) {
                    u[static_cast<std::size_t>(p[j])] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const std::size_t j1 = static_cast<std::size_t>(way[j0]);
            p[j0] = p[j1];
            j0    = j1;
        } while (j0 != 0);
    }

    // Recover assignments; rows matched to padding columns or unreachable
    // cost are unmatched (-1).
    std::vector<int> assignment(n, -1);
    std::vector<bool> col_taken(m, false);
    for (std::size_t j = 1; j <= size; ++j) {
        if (p[j] == 0) continue;
        const std::size_t row = static_cast<std::size_t>(p[j] - 1);
        const std::size_t col = j - 1;
        if (row >= n || col >= m) continue;
        if (c[row][col] >= unreachable) continue;
        assignment[row] = static_cast<int>(col);
        col_taken[col]  = true;
    }
    return assignment;
}

} // namespace radar_fusion::association

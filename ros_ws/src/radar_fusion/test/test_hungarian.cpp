#include <vector>

#include <gtest/gtest.h>

#include "radar_fusion/hungarian.hpp"

namespace radar_fusion::association {

TEST(Hungarian, TwoByTwoPicksMinTotalCost) {
    const std::vector<std::vector<double>> cost = {
        { 1.0, 2.0 },
        { 2.0, 1.0 },
    };
    const auto assignment = hungarian_min_cost(cost);
    // 最优：总代价 2（(0,0)+(1,1) 或 (0,1)+(1,0)）
    double total = 0.0;
    for (size_t i = 0; i < assignment.size(); ++i) {
        ASSERT_GE(assignment[i], 0);
        total += cost[i][static_cast<size_t>(assignment[i])];
    }
    EXPECT_DOUBLE_EQ(total, 2.0);
}

TEST(Hungarian, GreedyFailsButHungarianSucceeds) {
    // 贪心：行 0 取 (0,0)=1 → 行 1 取 (1,2)=1 → 行 2 只剩 (2,1)=3 → 总 5
    // 匈牙利同样得到 5——构造贪心真吃亏的例子：
    const std::vector<std::vector<double>> cost = {
        { 1.0, 2.0 },
        { 2.0, 100.0 },
    };
    // 贪心：行 0 取 (0,0)=1 → 行 1 取 (1,1)=100 → 总 101
    // 匈牙利：行 0 取 (0,1)=2 → 行 1 取 (1,0)=2 → 总 4
    const auto assignment = hungarian_min_cost(cost);
    double total = 0.0;
    for (size_t i = 0; i < assignment.size(); ++i) {
        ASSERT_GE(assignment[i], 0);
        total += cost[i][static_cast<size_t>(assignment[i])];
    }
    EXPECT_DOUBLE_EQ(total, 4.0);
}

TEST(Hungarian, RectangularUnmatchedRowReturnsMinusOne) {
    // 3 行 2 列：最多匹配 2 个；第三行无可用列 → -1
    const std::vector<std::vector<double>> cost = {
        { 1.0, 5.0 },
        { 5.0, 1.0 },
        { 10.0, 10.0 },
    };
    const auto assignment = hungarian_min_cost(cost);
    ASSERT_EQ(assignment.size(), 3u);
    int matched = 0;
    for (int a : assignment) {
        if (a >= 0) ++matched;
    }
    EXPECT_EQ(matched, 2);
    EXPECT_EQ(assignment[2], -1);
}

} // namespace radar_fusion::association

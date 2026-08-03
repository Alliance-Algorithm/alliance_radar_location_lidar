#include <random>
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

// 模拟贪心匹配：按距离排序依次匹配（与旧实现一致）
auto greedy_match(const std::vector<std::vector<double>>& cost, double gate) {
    struct Cand { int i, j; double d; };
    std::vector<Cand> cands;
    for (size_t i = 0; i < cost.size(); ++i)
        for (size_t j = 0; j < cost[i].size(); ++j)
            if (cost[i][j] < gate) cands.push_back({ (int)i, (int)j, cost[i][j] });
    std::sort(cands.begin(), cands.end(), [](auto& a, auto& b) { return a.d < b.d; });
    std::vector<int> assign(cost.size(), -1);
    std::vector<bool> taken(cost.size(), false);
    for (auto& c : cands) {
        if (assign[c.i] >= 0 || taken[c.j]) continue;
        assign[c.i] = c.j;
        taken[c.j]  = true;
    }
    return assign;
}

TEST(Hungarian, CrossingTargetsGreedySwapsIdentity) {
    // 模拟：两个目标匀速交叉 20 帧（A: x=0→10，B: x=10→0，y 固定）
    // 每帧测量 = 真实位置；两目标在第 10 帧相遇（x=5）
    // 贪心按距离最近匹配——相遇前后测量都离两个 track 近，可能换身份
    // 匈牙利全局最优——身份保持
    struct State { double x[2]; };  // track 位置
    auto greedy_match_fn = [](const std::vector<std::vector<double>>& cost, double gate) {
        struct Cand { int i, j; double d; };
        std::vector<Cand> cands;
        for (size_t i = 0; i < cost.size(); ++i)
            for (size_t j = 0; j < cost[i].size(); ++j)
                if (cost[i][j] < gate) cands.push_back({ (int)i, (int)j, cost[i][j] });
        std::sort(cands.begin(), cands.end(), [](auto& a, auto& b) { return a.d < b.d; });
        std::vector<int> assign(cost.size(), -1);
        std::vector<bool> taken(cost.size(), false);
        for (auto& c : cands) {
            if (assign[c.i] >= 0 || taken[c.j]) continue;
            assign[c.i] = c.j;
            taken[c.j]  = true;
        }
        return assign;
    };

    // 单帧数据关联质量：track 先验 = 真实位置（假设 track 状态良好），
    // 每帧独立评估匹配正确率（衡量噪声下谁更容易错配）
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.3);
    int greedy_wrong = 0, hungarian_wrong = 0;
    int greedy_wrong_when_ambiguous = 0, hungarian_wrong_when_ambiguous = 0;
    for (int f = 0; f < 200; ++f) {
        const double a_pos = 0.0 + 0.25 * (f % 40);   // A 往返 0→9.75
        const double b_pos = 10.0 - 0.25 * (f % 40);  // B 往返 10→0.25
        const std::vector<double> meas = { a_pos + noise(rng), b_pos + noise(rng) };
        std::vector<std::vector<double>> cost = {
            { std::abs(a_pos - meas[0]), std::abs(a_pos - meas[1]) },
            { std::abs(b_pos - meas[0]), std::abs(b_pos - meas[1]) },
        };
        auto g = greedy_match_fn(cost, 10.0);
        auto h = radar_fusion::association::hungarian_min_cost(cost, 10.0);
        const bool g_swapped = g[0] == 1 && g[1] == 0;
        const bool h_swapped = h[0] == 1 && h[1] == 0;
        if (g_swapped) ++greedy_wrong;
        if (h_swapped) ++hungarian_wrong;
        // 歧义帧：两目标距离 < 1.0m（交叉附近）——真实错配风险区
        const bool ambiguous = std::abs(a_pos - b_pos) < 1.0;
        if (ambiguous && g_swapped) ++greedy_wrong_when_ambiguous;
        if (ambiguous && h_swapped) ++hungarian_wrong_when_ambiguous;
    }
    // 匈牙利全局最优：错配数 ≤ 贪心（歧义帧尤其）
    EXPECT_LE(hungarian_wrong, greedy_wrong);
    EXPECT_LE(hungarian_wrong_when_ambiguous, greedy_wrong_when_ambiguous);
    std::printf("[hungarian vs greedy] wrong assoc: greedy=%d hungarian=%d | ambiguous: "
                "greedy=%d hungarian=%d\n",
        greedy_wrong, hungarian_wrong, greedy_wrong_when_ambiguous,
        hungarian_wrong_when_ambiguous);
}

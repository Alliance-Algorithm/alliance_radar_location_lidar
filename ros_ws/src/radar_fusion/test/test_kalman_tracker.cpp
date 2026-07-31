#include <gtest/gtest.h>

#include <cstdint>

#include <Eigen/Core>

#include "radar_fusion/data_format.hpp"
#include "radar_fusion/kalman_tracker.hpp"

namespace {

constexpr int64_t kT0_ns = 1'000'000'000; // t=1.0 s

TEST(KalmanTrackerTest, InitialState) {
    radar_fusion::kalman_tracker::KalmanTracker t(42);

    EXPECT_EQ(t.state().track_id, 42);
    EXPECT_EQ(t.state().lifecycle, radar_fusion::kalman_tracker::TrackLifecycle::TENTATIVE);
    EXPECT_FALSE(t.state().is_confirmed());
    EXPECT_FALSE(t.state().is_deleted());
}

TEST(KalmanTrackerTest, ConfirmsAfterMinHits) {
    radar_fusion::kalman_tracker::KalmanTracker t(1);

    t.update(Eigen::Vector2d(0, 0), kT0_ns, 3);
    EXPECT_FALSE(t.state().is_confirmed());

    t.update(Eigen::Vector2d(0.1, 0), kT0_ns + 100'000'000, 3);
    EXPECT_FALSE(t.state().is_confirmed());

    t.update(Eigen::Vector2d(0.2, 0), kT0_ns + 200'000'000, 3);
    EXPECT_TRUE(t.state().is_confirmed());
    EXPECT_FALSE(t.state().is_deleted());
}

TEST(KalmanTrackerTest, TentativeDiesOnFirstMiss) {
    radar_fusion::kalman_tracker::KalmanTracker t(2);

    t.mark_missed(2);
    EXPECT_TRUE(t.state().is_deleted());
    EXPECT_EQ(t.state().lifecycle, radar_fusion::kalman_tracker::TrackLifecycle::DELETED);
}

TEST(KalmanTrackerTest, ConfirmedSurvivesSingleMiss) {
    radar_fusion::kalman_tracker::KalmanTracker t(3);

    t.update(Eigen::Vector2d(0, 0), kT0_ns, 1);
    ASSERT_TRUE(t.state().is_confirmed());

    t.mark_missed(2);
    EXPECT_EQ(t.state().miss_count, 1);
    EXPECT_FALSE(t.state().is_deleted());
    EXPECT_EQ(t.state().lifecycle, radar_fusion::kalman_tracker::TrackLifecycle::CONFIRMED);
}

TEST(KalmanTrackerTest, ConfirmedDeletedAfterMaxMisses) {
    radar_fusion::kalman_tracker::KalmanTracker t(3);

    t.update(Eigen::Vector2d(0, 0), kT0_ns, 1);
    ASSERT_TRUE(t.state().is_confirmed());

    t.mark_missed(2); // miss_count = 1 < 2
    ASSERT_FALSE(t.state().is_deleted());

    t.mark_missed(2); // miss_count = 2 >= 2 -> DELETED
    EXPECT_EQ(t.state().miss_count, 2);
    EXPECT_TRUE(t.state().is_deleted());
    EXPECT_EQ(t.state().lifecycle, radar_fusion::kalman_tracker::TrackLifecycle::DELETED);
}

TEST(KalmanTrackerTest, UpdateResetsMissCount) {
    radar_fusion::kalman_tracker::KalmanTracker t(4);

    t.update(Eigen::Vector2d(0, 0), kT0_ns, 1);
    ASSERT_TRUE(t.state().is_confirmed());

    t.mark_missed(2);
    ASSERT_EQ(t.state().miss_count, 1);
    ASSERT_EQ(t.state().hit_count, 1);

    t.update(Eigen::Vector2d(0.1, 0), kT0_ns + 100'000'000, 1);
    EXPECT_EQ(t.state().miss_count, 0);
    EXPECT_EQ(t.state().hit_count, 2);
}

TEST(KalmanTrackerTest, DistanceSquaredTo) {
    radar_fusion::kalman_tracker::KalmanTracker t(5);

    EXPECT_NEAR(t.distance_squared_to(Eigen::Vector2d(3, 4)), 25.0, 1e-9);

    t.update(Eigen::Vector2d(1, 2), kT0_ns, 1);
    auto pos    = t.state().position();
    double d_sq = (pos.x() - 4) * (pos.x() - 4) + (pos.y() - 6) * (pos.y() - 6);
    EXPECT_NEAR(t.distance_squared_to(Eigen::Vector2d(4, 6)), d_sq, 1e-9);
}

TEST(KalmanTrackerTest, PredictHandlesTimeDeltas) {
    radar_fusion::kalman_tracker::KalmanTracker t(6);
    t.update(Eigen::Vector2d(1, 0), kT0_ns, 1);

    double P_before = t.state().P.trace();

    t.predict(kT0_ns - 1'000'000'000);
    EXPECT_NEAR(t.state().P.trace(), P_before, 1e-9);

    t.predict(kT0_ns + 1'000'000'000);
    EXPECT_GT(t.state().P.trace(), P_before);
}

} // namespace

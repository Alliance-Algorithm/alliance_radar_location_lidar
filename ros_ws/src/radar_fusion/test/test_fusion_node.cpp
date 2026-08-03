#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <tuple>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "radar_fusion/default_slots.hpp"
#include "radar_fusion/match_timer.hpp"
#include "radar_fusion/radar_fusion_node.hpp"
#include "radar_interfaces/msg/camera_detection_pose.hpp"
#include "radar_interfaces/msg/lidar_location.hpp"
#include "radar_interfaces/msg/lidar_location.hpp"

namespace {

using namespace std::chrono_literals;

class FusionNodeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override {
        fusion_node_     = std::make_shared<radar_fusion::node::RadarFusionNode>();
        publisher_node_  = std::make_shared<rclcpp::Node>("fusion_node_test_publisher");
        subscriber_node_ = std::make_shared<rclcpp::Node>("fusion_node_test_subscriber");

        cluster_pub_ =
            publisher_node_->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10);
        lidar_pose_pub_ =
            publisher_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/lidar/pose", 10);
        camera_detection_pub_ =
            publisher_node_->create_publisher<radar_interfaces::msg::CameraDetectionPose>(
                "/radar_camera/robot_pose", 10);

        pose_sub_ =
            subscriber_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/localization/pose", 10,
                [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_pose_ = *msg;
                    ++pose_gen_;
                    cv_.notify_all();
                });
        tracks_sub_ =
            subscriber_node_->create_subscription<visualization_msgs::msg::MarkerArray>("/fusion/"
                                                                                        "tracks",
                10, [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_track_marker_count_ = msg->markers.size();
                    ++track_pub_gen_;
                    cv_.notify_all();
                });
        fused_tracks_sub_ =
            subscriber_node_->create_subscription<visualization_msgs::msg::MarkerArray>("/fusion/"
                                                                                        "fused_"
                                                                                        "tracks",
                10, [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_fused_track_marker_count_ = msg->markers.size();
                    ++fused_track_pub_gen_;
                    cv_.notify_all();
                });
        status_sub_ =
            subscriber_node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>("/localiz"
                                                                                          "ation/"
                                                                                          "status",
                10, [this](const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_status_ = *msg;
                    ++status_gen_;
                    cv_.notify_all();
                });
        location_sub_ = subscriber_node_->create_subscription<radar_interfaces::msg::LidarLocation>(
            "/lidar/location", 10,
            [this](const radar_interfaces::msg::LidarLocation::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                last_location_ = *msg;
                ++location_gen_;
                cv_.notify_all();
            });

        executor_.add_node(fusion_node_);
        executor_.add_node(publisher_node_);
        executor_.add_node(subscriber_node_);
        spin_thread_ = std::thread([this]() { executor_.spin(); });

        ASSERT_TRUE(wait_for_discovery()) << "ROS entities failed to discover each other";
    }

    void TearDown() override {
        executor_.cancel();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }

        // Drain any remaining work queued on the executor before removing nodes.
        // This prevents stale callbacks from firing after node destruction.
        executor_.remove_node(subscriber_node_);
        executor_.remove_node(publisher_node_);
        executor_.remove_node(fusion_node_);

        pose_sub_.reset();
        tracks_sub_.reset();
        fused_tracks_sub_.reset();
        status_sub_.reset();
        location_sub_.reset();
        cluster_pub_.reset();
        lidar_pose_pub_.reset();
        camera_detection_pub_.reset();
        subscriber_node_.reset();
        publisher_node_.reset();
        fusion_node_.reset();

        // Reset per-test value fields (gens are monotonic — never reset).
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_track_marker_count_       = 0;
            last_fused_track_marker_count_ = 0;
            last_pose_                     = geometry_msgs::msg::PoseWithCovarianceStamped();
            last_status_                   = diagnostic_msgs::msg::DiagnosticStatus();
            last_location_                 = radar_interfaces::msg::LidarLocation();
        }
    }

    auto wait_for_discovery(bool expect_camera = false) -> bool {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (cluster_pub_->get_subscription_count() > 0
                && lidar_pose_pub_->get_subscription_count() > 0
                && pose_sub_->get_publisher_count() > 0 && tracks_sub_->get_publisher_count() > 0
                && fused_tracks_sub_->get_publisher_count() > 0
                && status_sub_->get_publisher_count() > 0
                && (!expect_camera || camera_detection_pub_->get_subscription_count() > 0)) {
                return true;
            }
            std::this_thread::sleep_for(20ms);
        }
        return false;
    }

    // Monotonic generation counters — never reset.
    // Each callback increments a per-topic generation; tests capture a baseline and wait for a
    // delta. Stale or delayed callbacks only raise the count, so the condition is monotonic-safe.
    auto wait_for_pose_gen(std::uint64_t expected, std::chrono::milliseconds timeout) -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [&]() { return pose_gen_ >= expected; });
    }

    auto wait_for_track_pub_gen(std::uint64_t expected, std::chrono::milliseconds timeout) -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [&]() { return track_pub_gen_ >= expected; });
    }

    auto wait_for_fused_track_pub_gen(std::uint64_t expected, std::chrono::milliseconds timeout)
        -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [&]() { return fused_track_pub_gen_ >= expected; });
    }

    auto wait_for_status_gen(std::uint64_t expected, std::chrono::milliseconds timeout) -> bool {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, timeout, [&]() { return status_gen_ >= expected; });
    }

    auto make_cluster_msg(double x, double y, double z, int32_t sec, uint32_t nanosec)
        -> sensor_msgs::msg::PointCloud2 {
        return make_cluster_array_msg({ { x, y, z } }, sec, nanosec);
    }

    auto make_cluster_array_msg(const std::vector<std::tuple<double, double, double>>& points,
        int32_t sec, uint32_t nanosec) -> sensor_msgs::msg::PointCloud2 {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->reserve(points.size());
        for (const auto& [x, y, z] : points) {
            cloud->emplace_back(
                static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        }
        cloud->width    = cloud->size();
        cloud->height   = 1;
        cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp.sec     = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.header.frame_id      = "map";
        return msg;
    }

    auto make_empty_cluster_msg(int32_t sec, uint32_t nanosec) -> sensor_msgs::msg::PointCloud2 {
        return make_cluster_array_msg({ }, sec, nanosec);
    }

    auto make_camera_detection(double x, double y, int32_t sec, uint32_t nanosec)
        -> radar_interfaces::msg::CameraDetectionPose {
        radar_interfaces::msg::CameraDetectionPose msg;
        msg.header.frame_id      = "map";
        msg.header.stamp.sec     = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.hero_position.x      = x;
        msg.hero_position.y      = y;
        msg.hero_position.z      = 0.0;
        msg.hero_confidence      = 0.9;
        return msg;
    }

    auto make_empty_camera_detection(int32_t sec, uint32_t nanosec)
        -> radar_interfaces::msg::CameraDetectionPose {
        radar_interfaces::msg::CameraDetectionPose msg;
        msg.header.frame_id      = "map";
        msg.header.stamp.sec     = sec;
        msg.header.stamp.nanosec = nanosec;
        return msg;
    }

    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<radar_fusion::node::RadarFusionNode> fusion_node_;
    rclcpp::Node::SharedPtr publisher_node_;
    rclcpp::Node::SharedPtr subscriber_node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr lidar_pose_pub_;
    rclcpp::Publisher<radar_interfaces::msg::CameraDetectionPose>::SharedPtr camera_detection_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr tracks_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fused_tracks_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<radar_interfaces::msg::LidarLocation>::SharedPtr location_sub_;
    std::thread spin_thread_;

    std::mutex mutex_;
    std::condition_variable cv_;
    // Monotonic generation counters — never reset between tests.
    std::uint64_t pose_gen_            = 0;
    std::uint64_t track_pub_gen_       = 0;
    std::uint64_t fused_track_pub_gen_ = 0;
    std::uint64_t status_gen_          = 0;
    std::uint64_t location_gen_        = 0;
    // Per-callback snapshot fields (reset in TearDown).
    std::size_t last_track_marker_count_       = 0;
    std::size_t last_fused_track_marker_count_ = 0;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
    diagnostic_msgs::msg::DiagnosticStatus last_status_;
    radar_interfaces::msg::LidarLocation last_location_;
};

// 相机检测：指定槽位（class_id）填充
auto make_camera_slot(double x, double y, int class_id, int32_t sec, uint32_t nanosec)
    -> radar_interfaces::msg::CameraDetectionPose {
    radar_interfaces::msg::CameraDetectionPose msg;
    msg.header.frame_id      = "map";
    msg.header.stamp.sec     = sec;
    msg.header.stamp.nanosec = nanosec;
    auto set = [&](geometry_msgs::msg::Point& pos, double& conf) {
        pos.x = x;
        pos.y = y;
        conf  = 0.9f;
    };
    switch (class_id) {
    case 0: set(msg.hero_position, msg.hero_confidence); break;
    case 1: set(msg.engine_position, msg.engine_confidence); break;
    case 2: set(msg.infantry_3_position, msg.infantry_3_confidence); break;
    case 3: set(msg.infantry_4_position, msg.infantry_4_confidence); break;
    case 4: set(msg.sentry_position, msg.sentry_confidence); break;
    case 5: set(msg.drone_position, msg.drone_confidence); break;
    }
    return msg;
}

}

TEST_F(FusionNodeTest, ClusterOnlyInputPublishesStatusWithoutLocalizationPose) {
    auto pose_bl   = pose_gen_;
    auto status_bl = status_gen_;

    cluster_pub_->publish(make_cluster_msg(1.0, 2.0, 0.0, 0, 123456789u));

    EXPECT_FALSE(wait_for_pose_gen(pose_bl + 1, 300ms));
    EXPECT_TRUE(wait_for_status_gen(status_bl + 1, 300ms));
}

TEST_F(FusionNodeTest, ClusterTrackingUsesMessageTimeInsteadOfWallTime) {
    auto bl = track_pub_gen_;

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 3, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_GE(last_track_marker_count_, 3u);
    }

    auto bl2 = track_pub_gen_;

    // Wall-clock sleep IS needed here: the test verifies that tracking uses message time
    // (not wall time). 1.7 s of wall time elapses while the message timestamp advances
    // only 0.1 s, which is well within the 1.5 s track timeout.
    std::this_thread::sleep_for(1700ms);
    cluster_pub_->publish(make_cluster_msg(1.68, 0.0, 0.0, 2, 100000000u));

    ASSERT_TRUE(wait_for_track_pub_gen(bl2 + 1, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_track_marker_count_, 3u);
}

TEST_F(FusionNodeTest, TentativeTrackIsDroppedAfterSingleMiss) {
    auto bl = track_pub_gen_;

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    cluster_pub_->publish(make_empty_cluster_msg(0, 100000000u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    ASSERT_FALSE(wait_for_track_pub_gen(bl + 3, 200ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_LT(last_track_marker_count_, 3u);
    }
}

TEST_F(FusionNodeTest, ConfirmedTrackSurvivesSingleMissButDropsAfterSecondMiss) {
    auto bl = track_pub_gen_;

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 3, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_GE(last_track_marker_count_, 3u);
    }

    auto bl2 = track_pub_gen_;
    cluster_pub_->publish(make_empty_cluster_msg(2, 100000000u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl2 + 1, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_track_marker_count_, 3u);
    }

    auto bl3 = track_pub_gen_;
    cluster_pub_->publish(make_empty_cluster_msg(2, 200000000u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl3 + 1, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_track_marker_count_, 0u);
    }
}

TEST_F(FusionNodeTest, GlobalGreedyAssociationKeepsTwoConfirmedTracks) {
    auto bl = track_pub_gen_;

    cluster_pub_->publish(make_cluster_array_msg({ { 0.0, 0.0, 0.0 }, { 1.5, 0.0, 0.0 } }, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    cluster_pub_->publish(make_cluster_array_msg({ { 0.4, 0.0, 0.0 }, { 1.9, 0.0, 0.0 } }, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    cluster_pub_->publish(make_cluster_array_msg({ { 0.8, 0.0, 0.0 }, { 2.3, 0.0, 0.0 } }, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 3, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_GE(last_track_marker_count_, 6u);
    }

    auto bl2 = track_pub_gen_;
    cluster_pub_->publish(make_cluster_array_msg({ { 1.1, 0.0, 0.0 }, { 1.2, 0.0, 0.0 } }, 3, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl2 + 1, 500ms));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        EXPECT_EQ(last_track_marker_count_, 6u);
    }
}

TEST_F(FusionNodeTest, LidarPoseIsForwardedToLocalizationPose) {
    auto bl = pose_gen_;

    geometry_msgs::msg::PoseWithCovarianceStamped lidar_pose;
    lidar_pose.header.stamp.sec        = 0;
    lidar_pose.header.stamp.nanosec    = 987654321u;
    lidar_pose.header.frame_id         = "map";
    lidar_pose.pose.pose.position.x    = 1.25;
    lidar_pose.pose.pose.position.y    = -0.75;
    lidar_pose.pose.pose.position.z    = 0.5;
    lidar_pose.pose.pose.orientation.x = 0.1;
    lidar_pose.pose.pose.orientation.y = -0.2;
    lidar_pose.pose.pose.orientation.z = 0.3;
    lidar_pose.pose.pose.orientation.w = 0.9;

    lidar_pose_pub_->publish(lidar_pose);
    ASSERT_TRUE(wait_for_pose_gen(bl + 1, 1s));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_pose_.header.frame_id, lidar_pose.header.frame_id);
    EXPECT_EQ(last_pose_.header.stamp.sec, lidar_pose.header.stamp.sec);
    EXPECT_EQ(last_pose_.header.stamp.nanosec, lidar_pose.header.stamp.nanosec);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.x, lidar_pose.pose.pose.position.x);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.y, lidar_pose.pose.pose.position.y);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.z, lidar_pose.pose.pose.position.z);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.x, lidar_pose.pose.pose.orientation.x);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.y, lidar_pose.pose.pose.orientation.y);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.z, lidar_pose.pose.pose.orientation.z);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.w, lidar_pose.pose.pose.orientation.w);
    EXPECT_DOUBLE_EQ(last_pose_.pose.covariance[0], lidar_pose.pose.covariance[0]);
}

TEST_F(FusionNodeTest, ConfirmedTracksArePublishedToFusedTracks) {
    auto bl_track = track_pub_gen_;
    auto bl_fused = fused_track_pub_gen_;

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl_track + 1, 500ms));

    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl_track + 2, 500ms));

    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl_track + 3, 500ms));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl_fused + 3, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GE(last_track_marker_count_, 3u);
    EXPECT_EQ(last_fused_track_marker_count_, 1u);
}

TEST_F(FusionNodeTest, LidarPosePublishesRadarOnlyStatus) {
    auto bl = status_gen_;

    geometry_msgs::msg::PoseWithCovarianceStamped lidar_pose;
    lidar_pose.header.stamp.sec = 1;
    lidar_pose.header.frame_id  = "map";

    lidar_pose_pub_->publish(lidar_pose);
    ASSERT_TRUE(wait_for_status_gen(bl + 1, 1s));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_status_.message, "RADAR_ONLY");
}

TEST_F(FusionNodeTest, CameraDetectionSwitchesFusionModeWhenEnabled) {
    auto enabled_node = std::make_shared<radar_fusion::node::RadarFusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(/*expect_camera=*/true)) << "ROS entities failed to rediscover "
                                                               "after enabling camera fusion";

    auto bl = status_gen_;
    camera_detection_pub_->publish(make_camera_detection(1.0, 2.0, 2, 0u));

    ASSERT_TRUE(wait_for_status_gen(bl + 1, 1s));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_status_.message, "RADAR_CAMERA");
}

TEST_F(FusionNodeTest, CameraDetectionsCreateConfirmedTracksWhenFusionEnabled) {
    auto enabled_node = std::make_shared<radar_fusion::node::RadarFusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(true)) << "ROS entities failed to rediscover after enabling "
                                             "camera fusion";

    auto bl = fused_track_pub_gen_;

    camera_detection_pub_->publish(make_camera_detection(1.0, 2.0, 0, 0u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl + 1, 500ms));

    camera_detection_pub_->publish(make_camera_detection(1.4, 2.0, 1, 0u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl + 2, 500ms));

    camera_detection_pub_->publish(make_camera_detection(1.8, 2.0, 2, 0u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl + 3, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GT(last_fused_track_marker_count_, 0u);
}

TEST_F(FusionNodeTest, CameraDetectionNearLidarTrackKeepsFusedOutputActive) {
    auto enabled_node = std::make_shared<radar_fusion::node::RadarFusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(true)) << "ROS entities failed to rediscover after enabling "
                                             "camera fusion";

    auto bl = track_pub_gen_;

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 3, 500ms));

    auto bl2 = fused_track_pub_gen_;
    camera_detection_pub_->publish(make_camera_detection(1.8, 0.0, 3, 0u));

    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl2 + 1, 500ms));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GT(last_fused_track_marker_count_, 0u);
}

TEST_F(FusionNodeTest, EmptyCameraFramesDoNotDeleteLidarTrack) {
    auto enabled_node = std::make_shared<radar_fusion::node::RadarFusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(true)) << "ROS entities failed to rediscover after enabling "
                                             "camera fusion";

    auto bl = track_pub_gen_;

    cluster_pub_->publish(make_cluster_msg(0.0, 0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 1, 500ms));

    cluster_pub_->publish(make_cluster_msg(0.8, 0.0, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 2, 500ms));

    cluster_pub_->publish(make_cluster_msg(1.6, 0.0, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_track_pub_gen(bl + 3, 500ms));

    auto bl2 = fused_track_pub_gen_;
    camera_detection_pub_->publish(make_empty_camera_detection(2, 100000000u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl2 + 1, 500ms));

    auto bl3 = fused_track_pub_gen_;
    camera_detection_pub_->publish(make_empty_camera_detection(2, 200000000u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl3 + 1, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_GT(last_fused_track_marker_count_, 0u);
}

TEST_F(FusionNodeTest, CameraSlotFilteringFiltersInvalidConfidenceAndNaN) {
    auto enabled_node = std::make_shared<radar_fusion::node::RadarFusionNode>(
        rclcpp::NodeOptions().append_parameter_override("enable_camera_fusion", true));
    executor_.remove_node(fusion_node_);
    fusion_node_.reset();
    fusion_node_ = enabled_node;
    executor_.add_node(fusion_node_);
    ASSERT_TRUE(wait_for_discovery(true)) << "ROS entities failed to rediscover after enabling "
                                             "camera fusion";

    auto bl_fused = fused_track_pub_gen_;

    camera_detection_pub_->publish(make_camera_detection(0.0, 0.0, 0, 0u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl_fused + 1, 500ms));

    camera_detection_pub_->publish(make_camera_detection(0.4, 0.0, 1, 0u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl_fused + 2, 500ms));

    camera_detection_pub_->publish(make_camera_detection(0.8, 0.0, 2, 0u));
    ASSERT_TRUE(wait_for_fused_track_pub_gen(bl_fused + 3, 500ms));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_fused_track_marker_count_, 1u);
}

// ---------------------------------------------------------------------------
// Task 3: match timer + default slot filling (pure unit tests, no ROS spin)
// ---------------------------------------------------------------------------

namespace {

using radar_fusion::default_positions::DefaultPosition;
using radar_fusion::match_timer::MatchTimer;

constexpr auto kSec = 1'000'000'000LL;

// Fake query with the same signature as default_positions::query_clamped.
auto make_fake_query() {
    return [](int camp, const std::string& robot_class, int t, DefaultPosition& out) -> bool {
        (void)t;
        if (camp != 0) return false;
        static const std::unordered_map<std::string, DefaultPosition> table = {
            { "hero", { 5.5, 2.25, 20 } },
            { "engineer", { 1.0, 1.0, 10 } },
            { "infantry3", { 2.0, 2.0, 10 } },
            { "infantry4", { 3.0, 3.0, 10 } },
            { "aerial", { 4.0, 4.0, 10 } },
            { "sentry", { 6.0, 6.0, 10 } },
        };
        const auto it = table.find(robot_class);
        if (it == table.end()) return false;
        out = it->second;
        return true;
    };
}

} // namespace

TEST(FusionNode, MatchTimerStartsOnProgress4) {
    MatchTimer mt;
    EXPECT_FALSE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(0), -1);

    mt.on_game_state(0, 0, 1 * kSec); // 未开始: no start
    EXPECT_FALSE(mt.started());

    mt.on_game_state(4, 300, 2 * kSec); // 比赛中: start
    EXPECT_TRUE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(2 * kSec), 0);
    EXPECT_EQ(mt.elapsed_sec(5 * kSec), 3);
}

TEST(FusionNode, MatchTimerStartsOnRemainAbove400) {
    MatchTimer mt;
    mt.on_game_state(0, 400, 1 * kSec); // exactly 400: no start
    EXPECT_FALSE(mt.started());

    mt.on_game_state(0, 401, 2 * kSec); // > 400: start
    EXPECT_TRUE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(2 * kSec), 0);
}

TEST(FusionNode, MatchTimerNoFalseStartBeforeBattle) {
    MatchTimer mt;
    for (int progress = 0; progress <= 3; ++progress) {
        mt.on_game_state(static_cast<uint8_t>(progress), 5, 1 * kSec);
        EXPECT_FALSE(mt.started());
        EXPECT_EQ(mt.elapsed_sec(1 * kSec), -1);
    }
    mt.on_game_state(3, 3, 2 * kSec); // 五秒倒计时 + small remain
    EXPECT_FALSE(mt.started());
}

TEST(FusionNode, MatchTimerResetsOnProgress5) {
    MatchTimer mt;
    mt.on_game_state(4, 300, 1 * kSec);
    ASSERT_TRUE(mt.started());

    mt.on_game_state(5, 300, 1 * kSec + 100 * kSec); // 结算
    EXPECT_FALSE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(1 * kSec + 100 * kSec), -1);
}

TEST(FusionNode, MatchTimerDoesNotResetDuringBattleLowRemain) {
    MatchTimer mt;
    mt.on_game_state(4, 300, 1 * kSec);
    ASSERT_TRUE(mt.started());

    // During the battle (progress 4) the remaining time counts down through
    // <10 without ending the round: the timer keeps running.
    mt.on_game_state(4, 10, 2 * kSec); // exactly 10: still running
    EXPECT_TRUE(mt.started());

    mt.on_game_state(4, 9, 3 * kSec); // < 10: still running (battle phase)
    EXPECT_TRUE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(3 * kSec), 2);
}

TEST(FusionNode, MatchTimerResetsOnRemainBelow10OutsideBattle) {
    MatchTimer mt;
    mt.on_game_state(4, 300, 1 * kSec);
    ASSERT_TRUE(mt.started());

    // Round ends; the next message outside the battle phase with a small
    // remain (e.g. 准备阶段 180 -> countdown tail) resets the timer.
    mt.on_game_state(5, 0, 1 * kSec + 100 * kSec);
    ASSERT_FALSE(mt.started());

    mt.on_game_state(3, 5, 1 * kSec + 150 * kSec); // 五秒倒计时 tail: reset stays
    EXPECT_FALSE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(1 * kSec + 150 * kSec), -1);
}

TEST(FusionNode, MatchTimerReentersFromZeroForSecondRound) {
    MatchTimer mt;
    mt.on_game_state(4, 300, 1 * kSec);
    ASSERT_TRUE(mt.started());
    mt.on_game_state(4, 250, 1 * kSec + 50 * kSec);
    EXPECT_EQ(mt.elapsed_sec(1 * kSec + 60 * kSec), 60);

    mt.on_game_state(5, 0, 1 * kSec + 100 * kSec); // round 1 ends
    ASSERT_FALSE(mt.started());

    mt.on_game_state(4, 420, 1 * kSec + 500 * kSec); // round 2 starts
    ASSERT_TRUE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(1 * kSec + 500 * kSec), 0);
    EXPECT_EQ(mt.elapsed_sec(1 * kSec + 505 * kSec), 5);
}

TEST(FusionNode, MatchTimerElapsedClampedAtZero) {
    MatchTimer mt;
    mt.on_game_state(4, 300, 10 * kSec);
    ASSERT_TRUE(mt.started());

    EXPECT_EQ(mt.elapsed_sec(10 * kSec - 5 * kSec), 0); // now before start
    EXPECT_EQ(mt.elapsed_sec(10 * kSec), 0);
}

TEST(FusionNode, MatchTimerDoesNotRestartOnRoundEndingMessage) {
    MatchTimer mt;
    mt.on_game_state(4, 300, 1 * kSec);
    ASSERT_TRUE(mt.started());

    // Battle-phase low remain does not reset (and therefore cannot restart).
    mt.on_game_state(4, 5, 2 * kSec);
    EXPECT_TRUE(mt.started());

    // 结算 message with remain > 400: reset, no restart
    mt.on_game_state(5, 450, 3 * kSec);
    EXPECT_FALSE(mt.started());
    EXPECT_EQ(mt.elapsed_sec(3 * kSec), -1);
}

TEST(FusionNode, SlotFilledWhenNoTrack) {
    radar_interfaces::msg::LidarLocation msg;
    const std::array<bool, 6> occupied = { false, false, false, false, false, false };

    radar_fusion::default_positions::fill_empty_slots(msg,
        radar_fusion::default_positions::kOpponentSlots, occupied, /*camp=*/0, /*t=*/0,
        make_fake_query());

    // Defaults are in the official referee frame: plain (med * 100) centimeters,
    // with NO map_to_rm_offset. A regression here (adding 14/7.5) shifts
    // positions by half a field.
    EXPECT_EQ(msg.opponent_hero_x, static_cast<uint16_t>(5.5 * 100.0));
    EXPECT_EQ(msg.opponent_hero_y, static_cast<uint16_t>(2.25 * 100.0));
    EXPECT_EQ(msg.opponent_engineer_x, static_cast<uint16_t>(1.0 * 100.0));
    EXPECT_EQ(msg.opponent_infantry_3_x, static_cast<uint16_t>(2.0 * 100.0));
    EXPECT_EQ(msg.opponent_infantry_4_x, static_cast<uint16_t>(3.0 * 100.0));
    EXPECT_EQ(msg.opponent_aerial_x, static_cast<uint16_t>(4.0 * 100.0));
    EXPECT_EQ(msg.opponent_sentry_x, static_cast<uint16_t>(6.0 * 100.0));
    EXPECT_EQ(msg.opponent_sentry_y, static_cast<uint16_t>(6.0 * 100.0));
}

TEST(FusionNode, NegativeDefaultMediansClampToZero) {
    radar_interfaces::msg::LidarLocation msg;
    const std::array<bool, 6> occupied = { false, false, false, false, false, false };
    const auto negative_query = [](int, const std::string&, int, DefaultPosition& out) -> bool {
        out = DefaultPosition { -2.0, -0.5, 10 }; // padded-field clip allows -2 m
        return true;
    };

    radar_fusion::default_positions::fill_empty_slots(msg,
        radar_fusion::default_positions::kOpponentSlots, occupied, /*camp=*/0, /*t=*/0,
        negative_query);

    // Negative medians must clamp to 0, not wrap in uint16_t.
    EXPECT_EQ(msg.opponent_hero_x, 0);
    EXPECT_EQ(msg.opponent_hero_y, 0);
    EXPECT_EQ(msg.opponent_sentry_x, 0);
}

TEST(FusionNode, OccupiedSlotsAreNotOverwritten) {
    radar_interfaces::msg::LidarLocation msg;
    msg.opponent_hero_x                = 111; // occupied -> preserved
    msg.opponent_hero_y                = 222;
    const std::array<bool, 6> occupied = { true, false, false, false, false, false };

    radar_fusion::default_positions::fill_empty_slots(msg,
        radar_fusion::default_positions::kOpponentSlots, occupied, /*camp=*/0, /*t=*/0,
        make_fake_query());

    EXPECT_EQ(msg.opponent_hero_x, 111);
    EXPECT_EQ(msg.opponent_hero_y, 222);
    EXPECT_EQ(msg.opponent_engineer_x, static_cast<uint16_t>(1.0 * 100.0));
}

TEST(FusionNode, QueryFailureLeavesSlotZero) {
    radar_interfaces::msg::LidarLocation msg;
    const std::array<bool, 6> occupied = { false, false, false, false, false, false };
    const auto failing_query = [](int, const std::string&, int, DefaultPosition&) -> bool {
        return false;
    };

    radar_fusion::default_positions::fill_empty_slots(msg,
        radar_fusion::default_positions::kOpponentSlots, occupied, /*camp=*/0, /*t=*/0,
        failing_query);

    EXPECT_EQ(msg.opponent_hero_x, 0);
    EXPECT_EQ(msg.opponent_hero_y, 0);
    EXPECT_EQ(msg.opponent_sentry_x, 0);
}

TEST(FusionNode, AllySlotsFilledWithCampInjectedQuery) {
    radar_interfaces::msg::LidarLocation msg;
    const std::array<bool, 6> occupied = { false, false, false, false, false, false };

    const auto ally_query = [](int camp, const std::string& robot_class, int t,
                                DefaultPosition& out) -> bool {
        (void)robot_class;
        (void)t;
        if (camp != 1) return false;
        out = DefaultPosition { 20.0, 12.0, 15 };
        return true;
    };

    radar_fusion::default_positions::fill_empty_slots(msg,
        radar_fusion::default_positions::kAllySlots, occupied, /*camp=*/1, /*t=*/0, ally_query);

    EXPECT_EQ(msg.ally_hero_x, static_cast<uint16_t>(20.0 * 100.0));
    EXPECT_EQ(msg.ally_hero_y, static_cast<uint16_t>(12.0 * 100.0));
    EXPECT_EQ(msg.ally_sentry_x, static_cast<uint16_t>(20.0 * 100.0));
    EXPECT_EQ(msg.opponent_hero_x, 0);
}

TEST_F(FusionNodeTest, LidarClusterInheritsCameraClassAndOutputsLocation) {
    // 雷达聚类确认 track：位置 (1.0, 0.0) 附近连续 3 帧
    cluster_pub_->publish(make_cluster_msg(0.5, 0.0, 0.0, 0, 0u));
    cluster_pub_->publish(make_cluster_msg(1.0, 0.0, 0.0, 1, 0u));
    cluster_pub_->publish(make_cluster_msg(1.5, 0.0, 0.0, 2, 0u));

    // 等待 lidar track 确认（/fusion/tracks 出现 lidar_clusters marker）
    const auto track_bl = track_pub_gen_;
    ASSERT_TRUE(wait_for_track_pub_gen(track_bl + 1, 1s)) << "lidar track not confirmed";

    // 相机检测：同一位置 + class 2（inf3）→ 给 lidar track 贴类别
    camera_detection_pub_->publish(make_camera_slot(1.0, 0.0, 2, 3, 0u));

    // 等待 LidarLocation 输出 inf3 槽位（官方坐标 = map + offset）
    const auto loc_bl = location_gen_;
    radar_interfaces::msg::LidarLocation got;
    bool found = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (last_location_.opponent_infantry_3_x > 0) {
                got   = last_location_;
                found = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(found) << "lidar track with camera class did not reach LidarLocation";

    // 官方坐标 = map + offset(14, 7.5)，×100 cm；雷达 track 位置 ≈ (1.0, 0.0)
    EXPECT_NEAR(got.opponent_infantry_3_x, (1.0 + 14.0) * 100.0, 50.0);
    EXPECT_NEAR(got.opponent_infantry_3_y, (0.0 + 7.5) * 100.0, 50.0);
}

TEST_F(FusionNodeTest, LidarClusterClassPersistsAfterCameraStops) {
    // 雷达聚类确认 track
    cluster_pub_->publish(make_cluster_msg(0.5, 0.0, 0.0, 0, 0u));
    cluster_pub_->publish(make_cluster_msg(1.0, 0.0, 0.0, 1, 0u));
    cluster_pub_->publish(make_cluster_msg(1.5, 0.0, 0.0, 2, 0u));
    const auto track_bl = track_pub_gen_;
    ASSERT_TRUE(wait_for_track_pub_gen(track_bl + 1, 1s)) << "lidar track not confirmed";

    // 相机贴类别后停止相机
    camera_detection_pub_->publish(make_camera_slot(1.0, 0.0, 2, 3, 0u));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    camera_detection_pub_->publish(make_empty_camera_detection(4, 0u));

    // 雷达聚类继续（track 保持 class）→ LidarLocation 持续输出
    cluster_pub_->publish(make_cluster_msg(2.0, 0.0, 0.0, 5, 0u));
    bool found = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (last_location_.opponent_infantry_3_x > 0) {
                found = true;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    EXPECT_TRUE(found) << "class-tagged lidar track should keep outputting after camera stops";
}

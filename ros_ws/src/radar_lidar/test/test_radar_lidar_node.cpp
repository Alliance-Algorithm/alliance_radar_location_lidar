#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <sys/time.h>
#include <thread>
#include <unistd.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <radar_interfaces/msg/registration_status.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include "radar_lidar/geometry_utils.hpp"
#include "radar_lidar/radar_lidar_node.hpp"

namespace {

using namespace std::chrono_literals;

// ── Per-process unique resource generators ─────────────────────────

static std::atomic<int> g_fixture_seq { 0 };

auto make_temp_dir() -> std::string {
    // Unique temp dir: PID + atomic counter avoids cross-process collisions
    auto dir = std::filesystem::path("/tmp")
        / ("radar_lidar_surface_test_" + std::to_string(getpid()) + "_"
            + std::to_string(g_fixture_seq.fetch_add(1)));
    std::filesystem::create_directories(dir);
    return dir.string();
}

auto make_scan_topic(int seq) -> std::string {
    return "/radar_lidar_surface_test_" + std::to_string(getpid()) + "_" + std::to_string(seq)
        + "/scan";
}

// ── Fixed production output topics ─────────────────────────────────

constexpr auto kPoseTopic               = "/lidar/pose";
constexpr auto kDiagTopic               = "/diagnostics";
constexpr auto kDynamicTopic            = "/lidar/dynamic";
constexpr auto kClusterTopic            = "/lidar/cluster";
constexpr auto kClusterVizTopic         = "/lidar/cluster_viz";
constexpr auto kOutputFrame             = "map";
constexpr auto kDiagnosticName          = "radar_lidar/localization";
constexpr auto kHardwareId              = "test_hw";
constexpr auto kRegistrationStatusTopic = "/localization/registration_status";

// ── Test fixture ────────────────────────────────────────────────────

class RadarLidarSurfaceTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            // ── Domain isolation contract ──────────────────────────────
            // ROS_DOMAIN_ID must be ≤ 232 (Fast-DDS port derivation cap).
            // Priority: RADAR_LIDAR_TEST_DOMAIN_ID override > PID ⊕ μs-time.
            // PID-modulo alone does NOT guarantee uniqueness across parallel
            // executions; explicit distinct env IDs are required for safety.
            // Range is validated as [0, 232]; 0 means "let RMW choose".
            //
            // Save current ROS_DOMAIN_ID so it can be restored on teardown
            // when running inside a parent process that already set one.
            const char* prior = getenv("ROS_DOMAIN_ID");
            if (prior) {
                s_saved_domain_ = prior;
            } else {
                s_saved_domain_.reset(); // nothing to restore
            }

            const char* explicit_domain = getenv("RADAR_LIDAR_TEST_DOMAIN_ID");
            int domain                  = -1;
            if (explicit_domain) {
                domain = std::stoi(explicit_domain);
            } else {
                // Derive from PID + microsecond-granularity start time for
                // collision resistance. mod 233 gives [0, 232] inclusive.
                struct timeval tv { };
                gettimeofday(&tv, nullptr);
                auto us = static_cast<int64_t>(tv.tv_sec * 1'000'000 + tv.tv_usec);
                domain  = static_cast<int>((static_cast<int64_t>(getpid()) ^ us) % 233);
            }

            // Validate range 0..232
            if (domain < 0 || domain > 232) {
                FAIL() << "ROS_DOMAIN_ID " << domain
                       << " out of valid range [0, 232] — check RADAR_LIDAR_TEST_DOMAIN_ID";
            }

            setenv("ROS_DOMAIN_ID", std::to_string(domain).c_str(), 1);
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
        s_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
            rclcpp::ExecutorOptions { });
        s_spin_thread_      = std::thread([]() { s_executor_->spin(); });
        s_executor_stopped_ = false;
    }

    static void TearDownTestSuite() {
        if (!s_executor_stopped_) {
            s_executor_->cancel();
            if (s_spin_thread_.joinable()) {
                s_spin_thread_.join();
            }
        }
        s_executor_.reset();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        // Restore prior ROS_DOMAIN_ID so the parent process/environment
        // is not polluted by our test-suite-local override.
        if (s_saved_domain_.has_value()) {
            setenv("ROS_DOMAIN_ID", s_saved_domain_->c_str(), 1);
        } else {
            unsetenv("ROS_DOMAIN_ID");
        }
    }

    void SetUp() override {
        // Verify domain isolation; fail early if env mismatch.
        const char* env_domain = getenv("ROS_DOMAIN_ID");
        ASSERT_NE(env_domain, nullptr) << "ROS_DOMAIN_ID not set";
        int domain_val = std::stoi(env_domain);
        ASSERT_GE(domain_val, 0) << "ROS_DOMAIN_ID " << domain_val << " < 0";
        ASSERT_LE(domain_val, 232) << "ROS_DOMAIN_ID " << domain_val << " > 232";

        // Restart executor if previous TearDown stopped it (safe lifecycle)
        if (s_executor_stopped_) {
            s_spin_thread_      = std::thread([]() { s_executor_->spin(); });
            s_executor_stopped_ = false;
        }

        // Per-fixture unique resources
        temp_dir_      = make_temp_dir();
        map_path_      = temp_dir_ + "/map.pcd";
        scan_topic_    = make_scan_topic(g_fixture_seq.load());
        pub_node_name_ = "surface_test_pub_" + std::to_string(getpid()) + "_"
            + std::to_string(g_fixture_seq.load());
        sub_node_name_ = "surface_test_sub_" + std::to_string(getpid()) + "_"
            + std::to_string(g_fixture_seq.load());

        create_test_map_pcd();

        pub_node_              = std::make_shared<rclcpp::Node>(pub_node_name_);
        extrinsic_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*pub_node_);
        extrinsic_broadcaster_->sendTransform(make_transform("radar_base", "lidar_link", 0, 0u,
            Eigen::Vector3d(0.4, -0.2, 0.1), Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ())));

        // ── Pipeline requiring one real GICP lock ──────────────────
        rclcpp::NodeOptions opts;
        opts.automatically_declare_parameters_from_overrides(true);
        opts.append_parameter_override("map_path", map_path_);
        opts.append_parameter_override("scan_topic", scan_topic_);
        opts.append_parameter_override("use_odin_relocalization_tf", true);
        opts.append_parameter_override("hardware_id", std::string(kHardwareId));
        opts.append_parameter_override("initial_pose_enabled", true);
        opts.append_parameter_override("initial_pose_tx", 2.0);
        opts.append_parameter_override("initial_pose_ty", -1.0);
        opts.append_parameter_override("initial_pose_tz", 0.5);
        opts.append_parameter_override("initial_pose_roll", 0.0);
        opts.append_parameter_override("initial_pose_pitch", 0.0);
        opts.append_parameter_override("initial_pose_yaw", 0.3);
        opts.append_parameter_override("lock_fitness", 20.0);

        pipeline_ = std::make_shared<radar_lidar::node::RadarLidarNode>(opts);
        sub_node_ = std::make_shared<rclcpp::Node>(sub_node_name_);

        // ── Event-driven discovery barrier (no polling, no warmup) ────
        discovery_promise_ = std::make_shared<std::promise<void>>();
        discovery_future_  = discovery_promise_->get_future().share();
        discovery_flag_.store(false);

        auto check_discovery = [this]() {
            if (discovery_flag_.load()) return;
            // Guard: callbacks may fire before all pub/sub are created
            if (!scan_pub_ || !pose_sub_ || !diag_sub_) return;
            if (scan_pub_->get_subscription_count() > 0 && pose_sub_->get_publisher_count() > 0
                && diag_sub_->get_publisher_count() > 0) {
                if (!discovery_flag_.exchange(true)) {
                    try {
                        discovery_promise_->set_value();
                    } catch (const std::future_error&) {
                        // Already resolved — second callback race, safe to ignore
                    }
                }
            }
        };

        // Input publisher on unique scan topic — SensorDataQoS + matched callback
        {
            rclcpp::PublisherOptions pub_opts;
            pub_opts.event_callbacks.matched_callback = [check_discovery](
                                                            rmw_matched_status_t& status) {
                if (status.current_count > 0) check_discovery();
            };
            scan_pub_ = pub_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
                scan_topic_, rclcpp::SensorDataQoS(), pub_opts);
        }

        // Output subscribers with matched callbacks
        {
            rclcpp::SubscriptionOptions pose_opts;
            pose_opts.event_callbacks.matched_callback = [check_discovery](
                                                             rmw_matched_status_t& status) {
                if (status.current_count > 0) check_discovery();
            };
            pose_sub_ =
                sub_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    kPoseTopic, 10,
                    [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        last_pose_ = *msg;
                        ++pose_gen_;
                        ++gen_;
                        cv_.notify_all();
                    },
                    pose_opts);
        }

        {
            rclcpp::SubscriptionOptions diag_opts;
            diag_opts.event_callbacks.matched_callback = [check_discovery](
                                                             rmw_matched_status_t& status) {
                if (status.current_count > 0) check_discovery();
            };
            diag_sub_ = sub_node_->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
                kDiagTopic, 10,
                [this](diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    last_diag_ = *msg;
                    ++diag_gen_;
                    ++gen_;
                    cv_.notify_all();
                },
                diag_opts);
        }

        s_executor_->add_node(pipeline_);
        s_executor_->add_node(pub_node_);
        s_executor_->add_node(sub_node_);

        // Initial synchronous check — handles race where DDS discovery
        // completes before the executor processes matched events.
        check_discovery();

        // Wait for all three matched-callbacks to confirm discovery (bounded deadline)
        ASSERT_TRUE(discover()) << "ROS entities failed to discover each other";
    }

    void TearDown() override {
        // Safe lifecycle: stop executor and drain pending callbacks before
        // destroying subscriptions that capture `this`. Without this order,
        // a callback dispatched before reset but executed after reset could
        // dereference freed memory.
        s_executor_->cancel();
        if (s_spin_thread_.joinable()) {
            s_spin_thread_.join();
        }
        s_executor_stopped_ = true;

        // Now safe: no callbacks can run. Reset data-plane entities.
        pose_sub_.reset();
        diag_sub_.reset();
        scan_pub_.reset();
        extrinsic_broadcaster_.reset();

        s_executor_->remove_node(sub_node_);
        s_executor_->remove_node(pub_node_);
        s_executor_->remove_node(pipeline_);

        sub_node_.reset();
        pub_node_.reset();
        pipeline_.reset();

        // Drain shared state under lock (no executor drain needed —
        // SingleThreadedExecutor serialises callbacks)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            gen_       = 0;
            pose_gen_  = 0;
            diag_gen_  = 0;
            last_pose_ = geometry_msgs::msg::PoseWithCovarianceStamped();
            last_diag_ = diagnostic_msgs::msg::DiagnosticStatus();
        }

        if (!temp_dir_.empty()) {
            std::filesystem::remove_all(temp_dir_);
        }
    }

    // ── Discovery (event-driven, no data-plane warmup) ──────────

    /// Event-driven graph discovery via rclcpp matched-callback events.
    /// Waits for each of the three required endpoint matches:
    ///   - scan_pub_ ↔ pipeline scan subscription  (check_discovery gate)
    ///   - pose_sub_ ↔ pipeline pose publisher     (check_discovery gate)
    ///   - diag_sub_ ↔ pipeline diag publisher     (check_discovery gate)
    /// A single bounded deadline guards the wait; callbacks fire
    /// exactly once per match from the executor/DDS event thread.
    /// No warmup scans, retries, sleeps, or polling.
    auto discover() -> bool {
        constexpr auto kDeadline = 15s;
        auto status              = discovery_future_.wait_for(kDeadline);
        if (status == std::future_status::ready) return true;

        // Deadline expired — report which endpoint(s) failed to match
        if (scan_pub_->get_subscription_count() == 0) {
            ADD_FAILURE() << "Discovery deadline (" << kDeadline.count() << "s) expired: "
                          << "scan publisher '" << scan_topic_ << "' has 0 matched subscriptions";
        }
        if (pose_sub_->get_publisher_count() == 0) {
            ADD_FAILURE() << "Discovery deadline (" << kDeadline.count() << "s) expired: "
                          << "pose subscriber '" << kPoseTopic << "' has 0 matched publishers";
        }
        if (diag_sub_->get_publisher_count() == 0) {
            ADD_FAILURE() << "Discovery deadline (" << kDeadline.count() << "s) expired: "
                          << "diag subscriber '" << kDiagTopic << "' has 0 matched publishers";
        }
        return false;
    }

    // ── Single-scan wait helpers (timestamp/frame-correlated, no retries) ────

    /// Publish one scan and wait for the pose message carrying its timestamp.
    /// Returns true when last_pose_ matches the published (sec, nsec) exactly.
    auto publish_and_await_pose(int32_t sec, uint32_t nsec, const std::string& frame_id) -> bool {
        scan_pub_->publish(make_scan_msg(sec, nsec, frame_id));
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, 5s, [&]() {
            return last_pose_.header.stamp.sec == sec && last_pose_.header.stamp.nanosec == nsec;
        });
    }

    /// Publish one scan and wait for the diagnostic message reporting
    /// the next expected frame number. Each fixture starts at frame 0,
    /// so the first call expects frame=1.
    auto publish_and_await_diag(
        int32_t sec, uint32_t nsec, const std::string& frame_id, int64_t expected_frame) -> bool {
        scan_pub_->publish(make_scan_msg(sec, nsec, frame_id));
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(lock, 5s, [&]() {
            for (const auto& kv : last_diag_.values) {
                if (kv.key == "frame") {
                    return std::stoll(kv.value) == expected_frame;
                }
            }
            return false;
        });
    }

    // ── Generation counter (kept for discovery/info only) ────────

    auto capture_diag_gen() -> std::size_t {
        std::lock_guard<std::mutex> lock(mutex_);
        return diag_gen_;
    }

    // ── Message factory ─────────────────────────────────────────

    static auto make_scan_msg(int32_t sec, uint32_t nanosec, const std::string& frame_id)
        -> sensor_msgs::msg::PointCloud2 {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (double x = 6.0; x < 25.0; x += 0.5) {
            for (double y = -8.0; y < 6.0; y += 0.5) {
                cloud.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.5f);
            }
        }
        cloud.width    = cloud.size();
        cloud.height   = 1;
        cloud.is_dense = true;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp.sec     = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.header.frame_id      = frame_id;
        return msg;
    }

    static auto make_rejected_scan_msg(int32_t sec, uint32_t nanosec, const std::string& frame_id)
        -> sensor_msgs::msg::PointCloud2 {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (double x = 106.0; x < 125.0; x += 0.5) {
            for (double y = -8.0; y < 6.0; y += 0.5) {
                cloud.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.5f);
            }
        }
        cloud.width    = cloud.size();
        cloud.height   = 1;
        cloud.is_dense = true;

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp.sec     = sec;
        msg.header.stamp.nanosec = nanosec;
        msg.header.frame_id      = frame_id;
        return msg;
    }

    static auto make_transform(const std::string& parent, const std::string& child, int32_t sec,
        uint32_t nanosec, const Eigen::Vector3d& translation, const Eigen::AngleAxisd& rotation)
        -> geometry_msgs::msg::TransformStamped {
        geometry_msgs::msg::TransformStamped msg;
        msg.header.frame_id         = parent;
        msg.child_frame_id          = child;
        msg.header.stamp.sec        = sec;
        msg.header.stamp.nanosec    = nanosec;
        msg.transform.translation.x = translation.x();
        msg.transform.translation.y = translation.y();
        msg.transform.translation.z = translation.z();
        const Eigen::Quaterniond quaternion(rotation);
        msg.transform.rotation.x = quaternion.x();
        msg.transform.rotation.y = quaternion.y();
        msg.transform.rotation.z = quaternion.z();
        msg.transform.rotation.w = quaternion.w();
        return msg;
    }

    // ── PCD fixture generator ───────────────────────────────────

    void create_test_map_pcd() {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (double x = 0.0; x < 20.0; x += 0.5) {
            for (double y = -7.0; y < 7.0; y += 0.5) {
                cloud.emplace_back(static_cast<float>(x), static_cast<float>(y), 1.0f);
            }
        }
        cloud.width    = cloud.size();
        cloud.height   = 1;
        cloud.is_dense = true;
        pcl::io::savePCDFileBinary(map_path_, cloud);
    }

    // ── Shared state ────────────────────────────────────────────

    std::string temp_dir_;
    std::string map_path_;
    std::string scan_topic_;
    std::string pub_node_name_;
    std::string sub_node_name_;

    std::shared_ptr<radar_lidar::node::RadarLidarNode> pipeline_;
    rclcpp::Node::SharedPtr pub_node_;
    rclcpp::Node::SharedPtr sub_node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> extrinsic_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diag_sub_;

    static inline rclcpp::executors::SingleThreadedExecutor::SharedPtr s_executor_;
    static inline std::thread s_spin_thread_;
    static inline std::optional<std::string> s_saved_domain_;
    static inline bool s_executor_stopped_ { false };

    // Event-driven discovery (no polling)
    std::shared_ptr<std::promise<void>> discovery_promise_;
    std::shared_future<void> discovery_future_;
    std::atomic<bool> discovery_flag_ { false };

    std::mutex mutex_;
    std::condition_variable cv_;
    std::size_t gen_      = 0;
    std::size_t pose_gen_ = 0;
    std::size_t diag_gen_ = 0;
    geometry_msgs::msg::PoseWithCovarianceStamped last_pose_;
    diagnostic_msgs::msg::DiagnosticStatus last_diag_;
};

} // namespace

// ═══════════════════════════════════════════════════════════════════
// Input topic contract — name, type, exact QoS
// ═══════════════════════════════════════════════════════════════════

TEST_F(RadarLidarSurfaceTest, ScanInputTopicExactQoS) {
    // Verify the configured scan topic exists in graph
    auto topics = sub_node_->get_topic_names_and_types();
    bool found  = false;
    for (const auto& [name, types] : topics) {
        if (name == scan_topic_) {
            found = true;
            EXPECT_EQ(types.size(), 1u);
            EXPECT_EQ(types[0], "sensor_msgs/msg/PointCloud2");
        }
    }
    EXPECT_TRUE(found) << "Scan topic " << scan_topic_ << " not found in graph";

    // Assert QoS from local publisher — avoids graph-endpoint depth uncertainty.
    // SensorDataQoS = BestEffort + Volatile + KeepLast(5)
    auto pub_qos = scan_pub_->get_actual_qos();
    EXPECT_EQ(pub_qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
    EXPECT_EQ(pub_qos.durability(), rclcpp::DurabilityPolicy::Volatile);
    EXPECT_EQ(pub_qos.history(), rclcpp::HistoryPolicy::KeepLast);
    EXPECT_EQ(pub_qos.depth(), 5u);

    // Publisher has discovered the subscription (proves QoS compatibility)
    EXPECT_GT(scan_pub_->get_subscription_count(), 0u);
}

// ═══════════════════════════════════════════════════════════════════
// Output topic contract — names, types, QoS
// ═══════════════════════════════════════════════════════════════════

TEST_F(RadarLidarSurfaceTest, OutputTopicsPreservedWithExactQoS) {
    auto topics = sub_node_->get_topic_names_and_types();

    // Expected {topic_name → type}
    std::map<std::string, std::string> expected {
        { kPoseTopic, "geometry_msgs/msg/PoseWithCovarianceStamped" },
        { kDiagTopic, "diagnostic_msgs/msg/DiagnosticStatus" },
        { kDynamicTopic, "sensor_msgs/msg/PointCloud2" },
        { kClusterTopic, "sensor_msgs/msg/PointCloud2" },
        { kClusterVizTopic, "visualization_msgs/msg/MarkerArray" },
        { kRegistrationStatusTopic, "radar_interfaces/msg/RegistrationStatus" },
    };

    std::map<std::string, std::string> actual;
    for (const auto& [name, types] : topics) {
        if (!types.empty()) actual[name] = types[0];
    }

    for (const auto& [topic, type] : expected) {
        auto it = actual.find(topic);
        ASSERT_NE(it, actual.end()) << "Output topic missing: " << topic;
        EXPECT_EQ(it->second, type) << "Wrong type for topic: " << topic;

        // Verify exactly one type per output topic
        auto entry = std::find_if(
            topics.begin(), topics.end(), [&topic](const auto& p) { return p.first == topic; });
        ASSERT_NE(entry, topics.end());
        EXPECT_EQ(entry->second.size(), 1u) << "Output topic " << topic << " has multiple types";
    }

    // ── Production publisher QoS via graph endpoint info ──────────
    // Assert reliability, durability, history, depth from the publisher
    // endpoint owned by the pipeline node. Local subscriber QoS matched
    // against this endpoint serves as supplemental verification.
    {
        auto pub_info = pipeline_->get_publishers_info_by_topic(kPoseTopic);
        ASSERT_GE(pub_info.size(), 1u) << "No publisher endpoint found for topic: " << kPoseTopic;
        auto it = std::ranges::find_if(
            pub_info, [](const auto& info) { return info.node_name() == "radar_lidar_node"; });
        ASSERT_NE(it, pub_info.end())
            << "No publisher endpoint owned by radar_lidar_node for: " << kPoseTopic;

        const auto& qos = it->qos_profile().get_rmw_qos_profile();
        EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE) << "Pose publisher "
                                                                           "endpoint should offer "
                                                                           "RELIABLE";
        EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE) << "Pose publisher endpoint "
                                                                         "should offer VOLATILE";
        // RMW may not expose history/depth in endpoint queries (RMW_QOS_POLICY_HISTORY_UNKNOWN).
        // Assert these only when the RMW layer reports them.
        if (qos.history != RMW_QOS_POLICY_HISTORY_UNKNOWN) {
            EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST) << "Pose publisher endpoint "
                                                                        "history should be "
                                                                        "KEEP_LAST";
            EXPECT_EQ(qos.depth, 10u) << "Pose publisher endpoint depth should be 10";
        }

        // Supplemental: local subscriber QoS matches publisher endpoint
        auto pose_qos = pose_sub_->get_actual_qos();
        EXPECT_EQ(pose_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
        EXPECT_EQ(pose_qos.durability(), rclcpp::DurabilityPolicy::Volatile);
        EXPECT_EQ(pose_qos.history(), rclcpp::HistoryPolicy::KeepLast);
        EXPECT_EQ(pose_qos.depth(), 10u);
        EXPECT_GT(pose_sub_->get_publisher_count(), 0u);
    }
    {
        auto pub_info = pipeline_->get_publishers_info_by_topic(kDiagTopic);
        ASSERT_GE(pub_info.size(), 1u) << "No publisher endpoint found for topic: " << kDiagTopic;
        auto it = std::ranges::find_if(
            pub_info, [](const auto& info) { return info.node_name() == "radar_lidar_node"; });
        ASSERT_NE(it, pub_info.end())
            << "No publisher endpoint owned by radar_lidar_node for: " << kDiagTopic;

        const auto& qos = it->qos_profile().get_rmw_qos_profile();
        EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE) << "Diag publisher "
                                                                           "endpoint should offer "
                                                                           "RELIABLE";
        EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE) << "Diag publisher endpoint "
                                                                         "should offer VOLATILE";
        if (qos.history != RMW_QOS_POLICY_HISTORY_UNKNOWN) {
            EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST) << "Diag publisher endpoint "
                                                                        "history should be "
                                                                        "KEEP_LAST";
            EXPECT_EQ(qos.depth, 10u) << "Diag publisher endpoint depth should be 10";
        }

        // Supplemental: local subscriber QoS
        auto diag_qos = diag_sub_->get_actual_qos();
        EXPECT_EQ(diag_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
        EXPECT_EQ(diag_qos.durability(), rclcpp::DurabilityPolicy::Volatile);
        EXPECT_EQ(diag_qos.history(), rclcpp::HistoryPolicy::KeepLast);
        EXPECT_EQ(diag_qos.depth(), 10u);
        EXPECT_GT(diag_sub_->get_publisher_count(), 0u);
    }
    {
        auto pub_info = pipeline_->get_publishers_info_by_topic(kRegistrationStatusTopic);
        ASSERT_GE(pub_info.size(), 1u)
            << "No publisher endpoint found for topic: " << kRegistrationStatusTopic;
        auto it = std::ranges::find_if(
            pub_info, [](const auto& info) { return info.node_name() == "radar_lidar_node"; });
        ASSERT_NE(it, pub_info.end())
            << "No publisher endpoint owned by radar_lidar_node for: " << kRegistrationStatusTopic;

        const auto& qos = it->qos_profile().get_rmw_qos_profile();
        EXPECT_EQ(qos.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(qos.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        if (qos.history != RMW_QOS_POLICY_HISTORY_UNKNOWN) {
            EXPECT_EQ(qos.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
            EXPECT_EQ(qos.depth, 1u);
        }
    }
}

TEST_F(
    RadarLidarSurfaceTest, AcceptedRegistrationPublishesImmutableStaticTransformAndLatchedStatus) {
    struct Observations {
        std::mutex mutex;
        std::condition_variable cv;
        std::vector<geometry_msgs::msg::TransformStamped> static_transforms;
        std::vector<geometry_msgs::msg::TransformStamped> dynamic_transforms;
        std::optional<radar_interfaces::msg::RegistrationStatus> status;
    };
    auto observations = std::make_shared<Observations>();

    auto capture_edge = [observations](const tf2_msgs::msg::TFMessage::SharedPtr msg,
                            std::vector<geometry_msgs::msg::TransformStamped>& destination) {
        std::lock_guard<std::mutex> lock(observations->mutex);
        for (const auto& transform : msg->transforms) {
            if (transform.header.frame_id == kOutputFrame
                && transform.child_frame_id == "radar_base") {
                destination.push_back(transform);
            }
        }
        observations->cv.notify_all();
    };

    auto static_sub  = sub_node_->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
        rclcpp::QoS(1).reliable().transient_local(),
        [observations, capture_edge](tf2_msgs::msg::TFMessage::SharedPtr msg) {
            capture_edge(msg, observations->static_transforms);
        });
    auto dynamic_sub = sub_node_->create_subscription<tf2_msgs::msg::TFMessage>("/tf",
        rclcpp::QoS(100), [observations, capture_edge](tf2_msgs::msg::TFMessage::SharedPtr msg) {
            capture_edge(msg, observations->dynamic_transforms);
        });

    ASSERT_TRUE([&]() {
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (static_sub->get_publisher_count() > 0) {
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        return false;
    }()) << "Static TF subscription did not discover the pipeline broadcaster";

    {
        std::unique_lock<std::mutex> lock(observations->mutex);
        EXPECT_FALSE(observations->cv.wait_for(lock, 200ms, [&]() {
            return !observations->static_transforms.empty()
                || !observations->dynamic_transforms.empty();
        })) << "map -> radar_base must not exist before registration acceptance";
    }

    ASSERT_TRUE(publish_and_await_pose(300, 0u, "scan"));
    {
        std::unique_lock<std::mutex> lock(observations->mutex);
        ASSERT_TRUE(observations->cv.wait_for(lock, 5s, [&]() {
            return observations->static_transforms.size() == 1;
        })) << "Accepted registration did not publish map -> radar_base on /tf_static";
    }

    auto status_sub = sub_node_->create_subscription<radar_interfaces::msg::RegistrationStatus>(
        kRegistrationStatusTopic, rclcpp::QoS(1).reliable().transient_local(),
        [observations](radar_interfaces::msg::RegistrationStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(observations->mutex);
            observations->status = *msg;
            observations->cv.notify_all();
        });
    {
        std::unique_lock<std::mutex> lock(observations->mutex);
        ASSERT_TRUE(observations->cv.wait_for(lock, 5s, [&]() {
            return observations->status.has_value();
        })) << "Late subscriber did not receive registration status";
        EXPECT_EQ(observations->status->state, radar_interfaces::msg::RegistrationStatus::LOCKED);
        EXPECT_TRUE(observations->status->reason.empty());
    }

    ASSERT_TRUE(publish_and_await_pose(301, 0u, "scan"));
    std::this_thread::sleep_for(200ms);
    {
        std::lock_guard<std::mutex> lock(observations->mutex);
        ASSERT_EQ(observations->static_transforms.size(), 1u) << "Later scans must not republish "
                                                                 "or change the accepted static "
                                                                 "transform";
        EXPECT_TRUE(observations->dynamic_transforms.empty()) << "map -> radar_base must never be "
                                                                 "published on /tf";
        const auto& transform = observations->static_transforms.front();
        EXPECT_EQ(transform.header.frame_id, kOutputFrame);
        EXPECT_EQ(transform.child_frame_id, "radar_base");
        EXPECT_EQ(transform.header.stamp.sec, 300);
        EXPECT_LT(transform.transform.translation.x, 1.9) << "Non-identity radar_base -> "
                                                             "lidar_link extrinsic was not "
                                                             "composed";
        EXPECT_GT(transform.transform.translation.z, 0.35);
        std::lock_guard<std::mutex> pose_lock(mutex_);
        EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.x, transform.transform.translation.x);
        EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.y, transform.transform.translation.y);
        EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.z, transform.transform.translation.z);
        EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.x, transform.transform.rotation.x);
        EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.y, transform.transform.rotation.y);
        EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.z, transform.transform.rotation.z);
        EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.w, transform.transform.rotation.w);
    }
}

TEST_F(RadarLidarSurfaceTest, OdinEstimateCannotBypassOrChangeFrozenGicpRegistration) {
    extrinsic_broadcaster_->sendTransform(make_transform(kOutputFrame, "scan", 0, 0u,
        Eigen::Vector3d(30.0, 20.0, 5.0), Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ())));

    ASSERT_TRUE(publish_and_await_pose(400, 0u, "scan")) << "Odin estimate suppressed the required "
                                                            "GICP registration";
    geometry_msgs::msg::Pose first_pose;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        first_pose = last_pose_.pose.pose;
        EXPECT_LT(first_pose.position.x, 5.0) << "Pose output used the Odin estimate instead of "
                                                 "GICP";
    }

    ASSERT_TRUE(publish_and_await_pose(401, 0u, "scan"));
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.x, first_pose.position.x);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.y, first_pose.position.y);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.position.z, first_pose.position.z);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.x, first_pose.orientation.x);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.y, first_pose.orientation.y);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.z, first_pose.orientation.z);
    EXPECT_DOUBLE_EQ(last_pose_.pose.pose.orientation.w, first_pose.orientation.w);
}

TEST_F(RadarLidarSurfaceTest, RejectedRegistrationPublishesNoPoseOrDetectionBeforeAcceptance) {
    std::atomic<int> dynamic_count { 0 };
    std::atomic<int> cluster_count { 0 };
    std::atomic<int> visualization_count { 0 };
    std::atomic<uint8_t> registration_state { radar_interfaces::msg::RegistrationStatus::FAILED };
    auto dynamic_sub = sub_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        kDynamicTopic, 10, [&](sensor_msgs::msg::PointCloud2::SharedPtr) { ++dynamic_count; });
    auto cluster_sub = sub_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        kClusterTopic, 10, [&](sensor_msgs::msg::PointCloud2::SharedPtr) { ++cluster_count; });
    auto visualization_sub =
        sub_node_->create_subscription<visualization_msgs::msg::MarkerArray>(kClusterVizTopic, 10,
            [&](visualization_msgs::msg::MarkerArray::SharedPtr) { ++visualization_count; });
    auto status_sub = sub_node_->create_subscription<radar_interfaces::msg::RegistrationStatus>(
        kRegistrationStatusTopic, rclcpp::QoS(1).reliable().transient_local(),
        [&](radar_interfaces::msg::RegistrationStatus::SharedPtr msg) {
            registration_state = msg->state;
        });

    scan_pub_->publish(make_rejected_scan_msg(500, 0u, "scan"));
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT_TRUE(cv_.wait_for(lock, 5s, [&]() { return diag_gen_ > 0; }));
    EXPECT_EQ(pose_gen_, 0u);
    lock.unlock();
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(dynamic_count.load(), 0);
    EXPECT_EQ(cluster_count.load(), 0);
    EXPECT_EQ(visualization_count.load(), 0);
    EXPECT_EQ(registration_state.load(), radar_interfaces::msg::RegistrationStatus::REGISTERING);

    ASSERT_TRUE(publish_and_await_pose(501, 0u, "scan"));
}

TEST(RadarLidarTimeoutTest, RegistrationTimeoutAfterInsufficientScanFailsWithoutPoseOrStaticTf) {
    const auto temp_dir = make_temp_dir();
    const auto map_path = temp_dir + "/map.pcd";
    pcl::PointCloud<pcl::PointXYZ> map_cloud;
    for (double x = 0.0; x < 20.0; x += 0.5) {
        for (double y = -7.0; y < 7.0; y += 0.5) {
            map_cloud.emplace_back(static_cast<float>(x), static_cast<float>(y), 1.0f);
        }
    }
    map_cloud.width = map_cloud.size();
    map_cloud.height = 1;
    map_cloud.is_dense = true;
    ASSERT_EQ(pcl::io::savePCDFileBinary(map_path, map_cloud), 0);

    if (!rclcpp::ok()) {
        int argc = 0;
        rclcpp::init(argc, nullptr);
    }
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    const auto scan_topic = make_scan_topic(g_fixture_seq.fetch_add(1));
    options.append_parameter_override("map_path", map_path);
    options.append_parameter_override("scan_topic", scan_topic);
    options.append_parameter_override("registration_timeout_sec", 0.2);
    options.append_parameter_override("initial_pose_enabled", true);
    options.append_parameter_override("initial_pose_tx", 2.0);
    options.append_parameter_override("initial_pose_ty", -1.0);
    options.append_parameter_override("initial_pose_tz", 0.5);
    options.append_parameter_override("initial_pose_yaw", 0.3);
    options.append_parameter_override("lock_fitness", 20.0);

    auto pipeline = std::make_shared<radar_lidar::node::RadarLidarNode>(options);
    auto observer = std::make_shared<rclcpp::Node>("timeout_observer", options);
    auto extrinsic_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*observer);
    geometry_msgs::msg::TransformStamped extrinsic;
    extrinsic.header.frame_id = "radar_base";
    extrinsic.child_frame_id = "lidar_link";
    extrinsic.transform.rotation.w = 1.0;
    extrinsic_broadcaster->sendTransform(extrinsic);
    auto scan_pub = observer->create_publisher<sensor_msgs::msg::PointCloud2>(
        scan_topic, rclcpp::SensorDataQoS());
    pcl::PointCloud<pcl::PointXYZ> registration_cloud;
    for (double x = 6.0; x < 25.0; x += 0.5) {
        for (double y = -8.0; y < 6.0; y += 0.5) {
            registration_cloud.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.5f);
        }
    }
    registration_cloud.width = registration_cloud.size();
    registration_cloud.height = 1;
    registration_cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 registration_scan;
    pcl::toROSMsg(registration_cloud, registration_scan);
    registration_scan.header.stamp.sec = 1;
    registration_scan.header.frame_id = "lidar_link";

    std::mutex mutex;
    std::condition_variable cv;
    std::optional<radar_interfaces::msg::RegistrationStatus> failed_status;
    std::atomic<int> pose_count { 0 };
    std::atomic<int> static_tf_count { 0 };
    auto status_sub = observer->create_subscription<radar_interfaces::msg::RegistrationStatus>(
        kRegistrationStatusTopic, rclcpp::QoS(1).reliable().transient_local(),
        [&](radar_interfaces::msg::RegistrationStatus::SharedPtr msg) {
            if (msg->state != radar_interfaces::msg::RegistrationStatus::FAILED) return;
            std::lock_guard<std::mutex> lock(mutex);
            failed_status = *msg;
            scan_pub->publish(registration_scan);
            cv.notify_all();
        });
    auto pose_sub = observer->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        kPoseTopic, 10,
        [&](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr) { ++pose_count; });
    auto static_tf_sub = observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
        rclcpp::QoS(1).reliable().transient_local(), [&](tf2_msgs::msg::TFMessage::SharedPtr msg) {
            for (const auto& transform : msg->transforms) {
                if (transform.header.frame_id == kOutputFrame
                    && transform.child_frame_id == "radar_base") {
                    ++static_tf_count;
                }
            }
        });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(pipeline);
    executor.add_node(observer);
    std::thread spin_thread([&]() { executor.spin(); });

    const auto discovery_deadline = std::chrono::steady_clock::now() + 5s;
    while (scan_pub->get_subscription_count() == 0
        && std::chrono::steady_clock::now() < discovery_deadline) {
        std::this_thread::sleep_for(10ms);
    }
    const bool discovered = scan_pub->get_subscription_count() > 0;

    pcl::PointCloud<pcl::PointXYZ> insufficient_cloud;
    for (int i = 0; i < 10; ++i) {
        insufficient_cloud.emplace_back(static_cast<float>(i), 0.0f, 0.5f);
    }
    insufficient_cloud.width = insufficient_cloud.size();
    insufficient_cloud.height = 1;
    insufficient_cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 insufficient_scan;
    pcl::toROSMsg(insufficient_cloud, insufficient_scan);
    insufficient_scan.header.frame_id = "lidar_link";
    if (discovered) scan_pub->publish(insufficient_scan);

    bool received_failed_status = false;
    {
        std::unique_lock<std::mutex> lock(mutex);
        received_failed_status =
            cv.wait_for(lock, 2s, [&]() { return failed_status.has_value(); });
        if (received_failed_status) {
            EXPECT_EQ(failed_status->reason, "Registration timeout: Too few points: 10");
        }
    }

    const auto shutdown_deadline = std::chrono::steady_clock::now() + 2s;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < shutdown_deadline) {
        std::this_thread::sleep_for(10ms);
    }
    EXPECT_FALSE(rclcpp::ok()) << "Timeout must request process context shutdown";
    EXPECT_EQ(pose_count.load(), 0);
    EXPECT_EQ(static_tf_count.load(), 0);

    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    if (rclcpp::ok()) rclcpp::shutdown();
    std::filesystem::remove_all(temp_dir);
    EXPECT_TRUE(discovered);
    EXPECT_TRUE(received_failed_status);
}

TEST(RadarLidarTransformTest, ComposesNonIdentityRadarBaseToLidarExtrinsic) {
    Eigen::Isometry3d t_map_lidar = Eigen::Isometry3d::Identity();
    t_map_lidar.translation()     = Eigen::Vector3d(4.0, 3.0, 2.0);
    t_map_lidar.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Isometry3d t_radar_base_lidar = Eigen::Isometry3d::Identity();
    t_radar_base_lidar.translation()     = Eigen::Vector3d(0.5, -0.25, 0.1);
    t_radar_base_lidar.linear() =
        Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // T_map_radar_base = T_map_lidar * inverse(T_radar_base_lidar).
    const auto actual = radar_lidar::geom::map_radar_base_pose(t_map_lidar, t_radar_base_lidar);

    EXPECT_NEAR(actual.translation().x(), 3.456524484548333, 1e-12);
    EXPECT_NEAR(actual.translation().y(), 2.869101703202276, 1e-12);
    EXPECT_NEAR(actual.translation().z(), 1.9, 1e-12);
    EXPECT_NEAR(Eigen::AngleAxisd(actual.rotation()).angle(), 0.7, 1e-12);
}

// ═══════════════════════════════════════════════════════════════════
// Pose output — frame, timestamp, finite values
// ═══════════════════════════════════════════════════════════════════

TEST_F(RadarLidarSurfaceTest, PoseOutputFrameAndTimestamp) {
    constexpr int32_t kSec   = 12345;
    constexpr uint32_t kNsec = 678900000u;

    ASSERT_TRUE(publish_and_await_pose(kSec, kNsec, "test_scan_frame")) << "Pose not delivered "
                                                                           "after single scan "
                                                                           "publish";

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_pose_.header.frame_id, kOutputFrame);
    EXPECT_EQ(last_pose_.header.stamp.sec, kSec);
    EXPECT_EQ(last_pose_.header.stamp.nanosec, kNsec);

    // All pose fields must be finite
    EXPECT_TRUE(std::isfinite(last_pose_.pose.pose.position.x));
    EXPECT_TRUE(std::isfinite(last_pose_.pose.pose.position.y));
    EXPECT_TRUE(std::isfinite(last_pose_.pose.pose.position.z));
    EXPECT_TRUE(std::isfinite(last_pose_.pose.pose.orientation.x));
    EXPECT_TRUE(std::isfinite(last_pose_.pose.pose.orientation.y));
    EXPECT_TRUE(std::isfinite(last_pose_.pose.pose.orientation.z));
    EXPECT_TRUE(std::isfinite(last_pose_.pose.pose.orientation.w));
}

// ═══════════════════════════════════════════════════════════════════
// Pose covariance — accepted GICP result is finite
// ═══════════════════════════════════════════════════════════════════

TEST_F(RadarLidarSurfaceTest, PoseCovarianceFromAcceptedRegistrationIsFinite) {
    ASSERT_TRUE(publish_and_await_pose(100, 0u, "scan")) << "Pose not delivered after single scan "
                                                            "publish";

    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < 36; ++i) {
        EXPECT_TRUE(std::isfinite(last_pose_.pose.covariance[i]))
            << "Covariance[" << i << "] is not finite";
    }
    for (int i = 0; i < 6; ++i) {
        EXPECT_GT(last_pose_.pose.covariance[i * 6 + i], 0.0)
            << "Accepted GICP covariance diagonal[" << i << "] should be positive";
    }
}

// ═══════════════════════════════════════════════════════════════════
// Diagnostics — exact identity and value semantics
// ═══════════════════════════════════════════════════════════════════

TEST_F(RadarLidarSurfaceTest, DiagnosticPublishedWithExactSemantics) {
    ASSERT_TRUE(publish_and_await_diag(200, 500000000u, "scan", 1)) << "No diagnostic after "
                                                                       "publishing scan";

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(last_diag_.name, kDiagnosticName);
    EXPECT_EQ(last_diag_.hardware_id, kHardwareId);
    EXPECT_EQ(last_diag_.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_EQ(last_diag_.message, "TRACKING");

    // Parse key-value pairs and assert exact value semantics
    double fitness   = -1.0;
    double time_ms   = -1.0;
    int64_t frame    = -1;
    bool converged   = false;
    bool has_fitness = false, has_time = false, has_frame = false, has_converged = false;

    for (const auto& kv : last_diag_.values) {
        if (kv.key == "fitness") {
            has_fitness = true;
            fitness     = std::stod(kv.value);
        }
        if (kv.key == "time_ms") {
            has_time = true;
            time_ms  = std::stod(kv.value);
        }
        if (kv.key == "frame") {
            has_frame = true;
            frame     = std::stoll(kv.value);
        }
        if (kv.key == "converged") {
            has_converged = true;
            converged     = (kv.value == "true");
        }
    }

    EXPECT_TRUE(has_fitness) << "Missing 'fitness' key";
    EXPECT_TRUE(has_time) << "Missing 'time_ms' key";
    EXPECT_TRUE(has_frame) << "Missing 'frame' key";
    EXPECT_TRUE(has_converged) << "Missing 'converged' key";

    if (has_fitness) {
        EXPECT_GT(fitness, 0.0) << "First locked pose must come from real GICP";
        EXPECT_LE(fitness, 20.0) << "Accepted pose fitness must satisfy the configured threshold";
    }
    if (has_time) {
        EXPECT_GT(time_ms, 0.0) << "time_ms should be positive";
        EXPECT_TRUE(std::isfinite(time_ms));
    }
    if (has_frame) {
        EXPECT_GT(frame, 0) << "frame counter should be positive";
    }
    if (has_converged) {
        EXPECT_TRUE(converged) << "Locked pose should be converged";
    }
}

// ═══════════════════════════════════════════════════════════════════
// Multiple scans — frame counter advances per scan
// ═══════════════════════════════════════════════════════════════════

TEST_F(RadarLidarSurfaceTest, MultipleScansAdvanceFrameCount) {
    // Each fixture starts with a fresh pipeline (frame_count_ = 0).
    // Assert exact frames via diagnostic history without generation-counter polling.

    ASSERT_TRUE(publish_and_await_diag(1, 0u, "scan", 1)) << "Frame 1 not delivered";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int64_t frame = -1;
        for (const auto& kv : last_diag_.values) {
            if (kv.key == "frame") frame = std::stoll(kv.value);
        }
        EXPECT_EQ(frame, 1);
    }

    ASSERT_TRUE(publish_and_await_diag(2, 100000000u, "scan", 2)) << "Frame 2 not delivered";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int64_t frame = -1;
        for (const auto& kv : last_diag_.values) {
            if (kv.key == "frame") frame = std::stoll(kv.value);
        }
        EXPECT_EQ(frame, 2);
    }

    ASSERT_TRUE(publish_and_await_diag(3, 200000000u, "scan", 3)) << "Frame 3 not delivered";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int64_t frame = -1;
        for (const auto& kv : last_diag_.values) {
            if (kv.key == "frame") frame = std::stoll(kv.value);
        }
        EXPECT_EQ(frame, 3);
    }
}

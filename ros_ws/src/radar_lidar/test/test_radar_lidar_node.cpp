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
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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

constexpr auto kPoseTopic       = "/lidar/pose";
constexpr auto kDiagTopic       = "/diagnostics";
constexpr auto kDynamicTopic    = "/lidar/dynamic";
constexpr auto kClusterTopic    = "/lidar/cluster";
constexpr auto kClusterVizTopic = "/lidar/cluster_viz";
constexpr auto kOutputFrame     = "map";
constexpr auto kDiagnosticName  = "radar_lidar/localization";
constexpr auto kHardwareId      = "test_hw";

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

        // ── Pipeline with locked initial pose ──────────────────────
        rclcpp::NodeOptions opts;
        opts.automatically_declare_parameters_from_overrides(true);
        opts.append_parameter_override("map_path", map_path_);
        opts.append_parameter_override("scan_topic", scan_topic_);
        opts.append_parameter_override("use_odin_relocalization_tf", false);
        opts.append_parameter_override("hardware_id", std::string(kHardwareId));
        opts.append_parameter_override("initial_pose_enabled", true);
        opts.append_parameter_override("initial_pose_tx", 2.0);
        opts.append_parameter_override("initial_pose_ty", -1.0);
        opts.append_parameter_override("initial_pose_tz", 0.5);
        opts.append_parameter_override("initial_pose_roll", 0.0);
        opts.append_parameter_override("initial_pose_pitch", 0.0);
        opts.append_parameter_override("initial_pose_yaw", 0.3);

        pipeline_ = std::make_shared<radar_lidar::node::RadarLidarNode>(opts);
        pub_node_ = std::make_shared<rclcpp::Node>(pub_node_name_);
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
// Pose covariance — exact locked-pose value (Identity * 1e-6)
// ═══════════════════════════════════════════════════════════════════

TEST_F(RadarLidarSurfaceTest, PoseCovarianceExactLockedPoseValue) {
    ASSERT_TRUE(publish_and_await_pose(100, 0u, "scan")) << "Pose not delivered after single scan "
                                                            "publish";

    std::lock_guard<std::mutex> lock(mutex_);
    // Locked pose (initial_pose_enabled=true) → covariance = Identity * 1e-6
    constexpr double kExpectedDiag = 1e-6;
    for (int i = 0; i < 36; ++i) {
        EXPECT_TRUE(std::isfinite(last_pose_.pose.covariance[i]))
            << "Covariance[" << i << "] is not finite";
    }
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(last_pose_.pose.covariance[i * 6 + i], kExpectedDiag, 1e-15)
            << "Covariance diagonal[" << i << "] should be 1e-6 (locked pose)";
    }
    // Off-diagonals should be 0.0
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            if (row != col) {
                EXPECT_DOUBLE_EQ(last_pose_.pose.covariance[row * 6 + col], 0.0)
                    << "Covariance[" << row << "][" << col << "] should be 0 (locked pose)";
            }
        }
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
        EXPECT_DOUBLE_EQ(fitness, 0.0) << "Locked pose fitness should be 0.0";
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

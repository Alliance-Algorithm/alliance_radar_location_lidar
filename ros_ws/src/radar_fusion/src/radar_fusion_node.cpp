#include "radar_fusion/radar_fusion_node.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <tuple>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace radar_fusion::node {

using PoseCov = geometry_msgs::msg::PoseWithCovarianceStamped;

namespace {

    struct AssociationCandidate {
        std::size_t track_idx;
        std::size_t measurement_idx;
        double distance_sq;
    };

    auto to_string(radar_fusion::fusion_config::FusionMode mode) -> std::string_view {
        switch (mode) {
        case radar_fusion::fusion_config::FusionMode::RADAR_ONLY:
            return "RADAR_ONLY";
        case radar_fusion::fusion_config::FusionMode::RADAR_CAMERA:
            return "RADAR_CAMERA";
        case radar_fusion::fusion_config::FusionMode::DEGRADED:
            return "DEGRADED";
        }
        return "UNKNOWN";
    }

} // namespace

RadarFusionNode::RadarFusionNode(const rclcpp::NodeOptions& options)
    : Node("radar_fusion_node", options) {

    this->declare_parameter("gate_distance", 2.0);
    this->declare_parameter("track_timeout_sec", 1.5);
    this->declare_parameter("min_hits_to_confirm", 3);
    this->declare_parameter("max_misses_before_delete", 2);
    this->declare_parameter("max_tracks", 20);
    this->declare_parameter("enable_camera_fusion", false);
    this->declare_parameter("camera_timeout_sec", 1.5);
    this->declare_parameter("camera_topic", std::string("/radar_camera/robot_pose"));
    this->declare_parameter("map_to_rm_offset_x", 14.0);
    this->declare_parameter("map_to_rm_offset_y", 7.5);
    this->declare_parameter("default_positions_path", "");
    this->declare_parameter("enemy_color", "blue");

    cfg_.gate_distance            = this->get_parameter("gate_distance").as_double();
    cfg_.track_timeout_sec        = this->get_parameter("track_timeout_sec").as_double();
    cfg_.min_hits_to_confirm      = this->get_parameter("min_hits_to_confirm").as_int();
    cfg_.max_misses_before_delete = this->get_parameter("max_misses_before_delete").as_int();
    cfg_.max_tracks               = this->get_parameter("max_tracks").as_int();
    cfg_.enable_camera_fusion     = this->get_parameter("enable_camera_fusion").as_bool();
    cfg_.camera_timeout_sec       = this->get_parameter("camera_timeout_sec").as_double();
    camera_topic_                 = this->get_parameter("camera_topic").as_string();
    cfg_.map_to_rm_offset_x       = this->get_parameter("map_to_rm_offset_x").as_double();
    cfg_.map_to_rm_offset_y       = this->get_parameter("map_to_rm_offset_y").as_double();
    default_positions_path_       = this->get_parameter("default_positions_path").as_string();
    enemy_color_                  = this->get_parameter("enemy_color").as_string();
    tracks_.reserve(static_cast<std::size_t>(cfg_.max_tracks));

    sub_lidar_pose_ = this->create_subscription<PoseCov>(
        "/lidar/pose", 10, [this](const PoseCov::SharedPtr msg) { on_lidar_pose(msg); });

    sub_cluster_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cluster(msg); });

    if (cfg_.enable_camera_fusion) {
        sub_camera_detection_ =
            this->create_subscription<radar_interfaces::msg::CameraDetectionPose>(camera_topic_, 10,
                [this](const radar_interfaces::msg::CameraDetectionPose::SharedPtr msg) {
                    on_camera_detection(msg);
                });
    }

    if (!default_positions_path_.empty()) {
        if (radar_fusion::default_positions::load(default_positions_path_)) {
            sub_game_state_ = this->create_subscription<radar_interfaces::msg::GameState>("/bridge/"
                                                                                          "game_"
                                                                                          "state",
                10, [this](const radar_interfaces::msg::GameState::SharedPtr msg) {
                    on_game_state(msg);
                });
            RCLCPP_INFO(get_logger(), "default positions loaded from '%s' (enemy=%s)",
                default_positions_path_.c_str(), enemy_color_.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "failed to load default positions from '%s'; feature disabled",
                default_positions_path_.c_str());
        }
    }

    pub_tracks_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/fusion/tracks", 10);
    pub_fused_tracks_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/fusion/fused_tracks", 10);
    pub_lidar_location_ =
        this->create_publisher<radar_interfaces::msg::LidarLocation>("/lidar/location", 10);
    pub_pose_ = this->create_publisher<PoseCov>("/localization/pose", 10);
    pub_status_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/localization/status", 10);
    // 输出节奏统一 10Hz（跟随雷达点云频率）：
    // camera(7Hz)/cluster(10Hz) 回调只更新测量，location/marker 由本定时器统一发布，
    // 避免 camera 路径把输出拖慢或双路径叠加（此前 ~17Hz）。
    location_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        [this]() {
            const auto stamp = this->now();
            publish_tracks(tracks_, stamp);
            publish_fused_tracks(tracks_, stamp);
            publish_lidar_tracks(lidar_tracks_, stamp);
            publish_lidar_location(tracks_, lidar_tracks_);
            publish_status(stamp);
        });
    update_fusion_mode(this->now().nanoseconds());

    RCLCPP_INFO(get_logger(),
        "radar_fusion ready. gate=%.1fm timeout=%.1fs camera=%s arena_offset=(%.1f,%.1f)m",
        cfg_.gate_distance, cfg_.track_timeout_sec, cfg_.enable_camera_fusion ? "on" : "off",
        cfg_.map_to_rm_offset_x, cfg_.map_to_rm_offset_y);
}

void RadarFusionNode::on_lidar_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    publish_localization_pose(*msg);
    publish_status(rclcpp::Time(msg->header.stamp));
}

void RadarFusionNode::on_camera_detection(
    const radar_interfaces::msg::CameraDetectionPose::SharedPtr msg) {
    auto stamp              = rclcpp::Time(msg->header.stamp);
    latest_camera_stamp_ns_ = stamp.nanoseconds();
    latest_camera_observations_.clear();

    struct {
        geometry_msgs::msg::Point pos;
        double conf;
        int class_id;
    } slots[6] = {
        { msg->hero_position, msg->hero_confidence, 0 },
        { msg->engine_position, msg->engine_confidence, 1 },
        { msg->infantry_3_position, msg->infantry_3_confidence, 2 },
        { msg->infantry_4_position, msg->infantry_4_confidence, 3 },
        { msg->sentry_position, msg->sentry_confidence, 4 },
        { msg->drone_position, msg->drone_confidence, 5 },
    };

    std::vector<Eigen::Vector2d> measurements;
    std::vector<int> classes;
    for (const auto& slot : slots) {
        if (slot.conf > 0.0 && std::isfinite(slot.pos.x) && std::isfinite(slot.pos.y)) {
            latest_camera_observations_.push_back(
                radar_fusion::camera_observation::CameraObservation {
                    .x          = slot.pos.x,
                    .y          = slot.pos.y,
                    .z          = slot.pos.z,
                    .confidence = slot.conf,
                    .class_id   = slot.class_id,
                });
            measurements.emplace_back(slot.pos.x, slot.pos.y);
            classes.push_back(slot.class_id);
        }
    }

    update_fusion_mode(latest_camera_stamp_ns_);
    process_measurements(measurements, stamp.nanoseconds(), false, classes);

    // 相机识别给雷达聚类 track 贴类别——雷达 track 无类别（class_id=-1）
    // 无法进官方坐标；被相机识别一次后继承类别，之后即使相机停发也持续输出坐标。
    if (!lidar_tracks_.empty()) {
        const double gate_sq = cfg_.gate_distance * cfg_.gate_distance;
        for (const auto& obs : latest_camera_observations_) {
            double best_d = gate_sq;
            int best_idx  = -1;
            for (size_t i = 0; i < lidar_tracks_.size(); ++i) {
                const auto& s = lidar_tracks_[i].state();
                if (!s.is_confirmed()) continue;
                const double dx = obs.x - s.x(0);
                const double dy = obs.y - s.x(1);
                const double d  = dx * dx + dy * dy;
                if (d < best_d) {
                    best_d  = d;
                    best_idx = static_cast<int>(i);
                }
            }
            if (best_idx >= 0) {
                lidar_tracks_[static_cast<size_t>(best_idx)].set_class_id(obs.class_id);
            }
        }
    }
    // 输出统一由 10Hz location_timer_ 发布（见构造函数），回调不再直接发布。
}

void RadarFusionNode::process_measurements(const std::vector<Eigen::Vector2d>& measurements,
    int64_t now_ns, bool mark_unmatched_tracks, const std::vector<int>& classes) {
    for (auto& track : tracks_) {
        track.predict(now_ns);
    }

    std::vector<bool> matched_tracks(tracks_.size(), false);
    std::vector<bool> matched_meas(measurements.size(), false);
    const double gate_distance_sq = cfg_.gate_distance * cfg_.gate_distance;
    std::vector<AssociationCandidate> candidates;
    candidates.reserve(tracks_.size() * measurements.size());

    for (size_t i = 0; i < tracks_.size(); ++i) {
        for (size_t j = 0; j < measurements.size(); ++j) {
            // 带类别时只匹配同类别（camera 路径防跨类跳变）；无类别（lidar 路径）全匹配。
            if (!classes.empty() && tracks_[i].state().class_id != classes[j]) continue;
            const double d_sq = tracks_[i].distance_squared_to(measurements[j]);
            if (d_sq < gate_distance_sq) {
                candidates.push_back({ i, j, d_sq });
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const AssociationCandidate& lhs, const AssociationCandidate& rhs) {
            return lhs.distance_sq < rhs.distance_sq;
        });

    for (const auto& candidate : candidates) {
        if (matched_tracks[candidate.track_idx] || matched_meas[candidate.measurement_idx]) {
            continue;
        }

        tracks_[candidate.track_idx].update(
            measurements[candidate.measurement_idx], now_ns, cfg_.min_hits_to_confirm);
        matched_tracks[candidate.track_idx]     = true;
        matched_meas[candidate.measurement_idx] = true;
    }

    if (mark_unmatched_tracks) {
        for (size_t i = 0; i < tracks_.size(); ++i) {
            if (!matched_tracks[i]) {
                tracks_[i].mark_missed(cfg_.max_misses_before_delete);
            }
        }
    }

    for (size_t j = 0; j < measurements.size(); ++j) {
        if (matched_meas[j]) continue;
        if (tracks_.size() >= static_cast<size_t>(cfg_.max_tracks)) break;

        radar_fusion::kalman_tracker::KalmanTracker new_track(next_track_id_++);
        if (!classes.empty()) new_track.set_class_id(classes[j]);
        new_track.update(measurements[j], now_ns, cfg_.min_hits_to_confirm);
        tracks_.push_back(new_track);
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                      [&](const radar_fusion::kalman_tracker::KalmanTracker& t) {
                          return t.state().is_deleted()
                              || t.state().is_stale(now_ns, cfg_.track_timeout_sec);
                      }),
        tracks_.end());
}

void RadarFusionNode::on_cluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto stamp  = rclcpp::Time(msg->header.stamp);
    auto now_ns = stamp.nanoseconds();
    update_fusion_mode(now_ns);

    std::vector<Eigen::Vector2d> measurements;
    measurements.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y)) {
            measurements.emplace_back(*iter_x, *iter_y);
        }
    }

    // lidar 聚类独立 track 池：与 camera 池完全解耦，
    // 脏聚类点不会创建/删除/污染 camera 的 class track。
    process_lidar_clusters(measurements, now_ns);
    // 输出统一由 10Hz location_timer_ 发布（见构造函数），回调不再直接发布。
}

void RadarFusionNode::process_lidar_clusters(
    const std::vector<Eigen::Vector2d>& measurements, int64_t now_ns) {
    for (auto& track : lidar_tracks_) {
        track.predict(now_ns);
    }

    std::vector<bool> matched_tracks(lidar_tracks_.size(), false);
    std::vector<bool> matched_meas(measurements.size(), false);
    const double gate_distance_sq = cfg_.gate_distance * cfg_.gate_distance;
    std::vector<AssociationCandidate> candidates;
    candidates.reserve(lidar_tracks_.size() * measurements.size());

    for (size_t i = 0; i < lidar_tracks_.size(); ++i) {
        for (size_t j = 0; j < measurements.size(); ++j) {
            const double d_sq = lidar_tracks_[i].distance_squared_to(measurements[j]);
            if (d_sq < gate_distance_sq) {
                candidates.push_back({ i, j, d_sq });
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const AssociationCandidate& lhs, const AssociationCandidate& rhs) {
            return lhs.distance_sq < rhs.distance_sq;
        });

    for (const auto& candidate : candidates) {
        if (matched_tracks[candidate.track_idx] || matched_meas[candidate.measurement_idx]) {
            continue;
        }
        lidar_tracks_[candidate.track_idx].update(
            measurements[candidate.measurement_idx], now_ns, cfg_.min_hits_to_confirm);
        matched_tracks[candidate.track_idx]     = true;
        matched_meas[candidate.measurement_idx] = true;
    }

    for (size_t i = 0; i < lidar_tracks_.size(); ++i) {
        if (!matched_tracks[i]) {
            lidar_tracks_[i].mark_missed(cfg_.max_misses_before_delete);
        }
    }

    for (size_t j = 0; j < measurements.size(); ++j) {
        if (matched_meas[j]) continue;
        if (lidar_tracks_.size() >= static_cast<size_t>(cfg_.max_tracks)) break;
        radar_fusion::kalman_tracker::KalmanTracker new_track(next_lidar_track_id_++);
        new_track.set_class_id(-1);
        new_track.update(measurements[j], now_ns, cfg_.min_hits_to_confirm);
        lidar_tracks_.push_back(new_track);
    }

    lidar_tracks_.erase(std::remove_if(lidar_tracks_.begin(), lidar_tracks_.end(),
                            [&](const radar_fusion::kalman_tracker::KalmanTracker& t) {
                                return t.state().is_deleted()
                                    || t.state().is_stale(now_ns, cfg_.track_timeout_sec);
                            }),
        lidar_tracks_.end());
}

void RadarFusionNode::publish_tracks(
    const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
    const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < tracks.size(); ++i) {
        const auto& s = tracks[i].state();
        if (!s.is_confirmed()) continue;

        // 轨迹位置（球体）
        visualization_msgs::msg::Marker m;
        m.header.stamp    = stamp;
        m.header.frame_id = "map";
        m.ns              = "tracks";
        m.id              = s.track_id;
        m.type            = visualization_msgs::msg::Marker::SPHERE;
        m.action          = visualization_msgs::msg::Marker::ADD;

        m.pose.position.x    = s.x(0);
        m.pose.position.y    = s.x(1);
        m.pose.position.z    = 0.5;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.3;
        m.scale.y = 0.3;
        m.scale.z = 0.3;

        // 颜色：confirmed=绿色，颜色根据 color 字段
        if (s.color == 0) { // blue
            m.color.b = 1.0f;
            m.color.a = 0.8f;
        } else if (s.color == 2) { // red
            m.color.r = 1.0f;
            m.color.a = 0.8f;
        } else {
            m.color.g = 1.0f;
            m.color.a = 0.8f;
        }

        m.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(m);

        // 速度箭头
        visualization_msgs::msg::Marker arrow;
        arrow.header.stamp    = stamp;
        arrow.header.frame_id = "map";
        arrow.ns              = "velocity";
        arrow.id              = s.track_id;
        arrow.type            = visualization_msgs::msg::Marker::ARROW;
        arrow.action          = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start, end;
        start.x      = s.x(0);
        start.y      = s.x(1);
        start.z      = 0.5;
        end.x        = s.x(0) + s.x(2) * 0.5;
        end.y        = s.x(1) + s.x(3) * 0.5;
        end.z        = 0.5;
        arrow.points = { start, end };

        arrow.scale.x = 0.05;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;

        arrow.color.r  = 1.0f;
        arrow.color.a  = 0.6f;
        arrow.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(arrow);

        // track ID 文字
        visualization_msgs::msg::Marker text;
        text.header.stamp    = stamp;
        text.header.frame_id = "map";
        text.ns              = "track_id";
        text.id              = s.track_id;
        text.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action          = visualization_msgs::msg::Marker::ADD;

        text.pose.position.x    = s.x(0);
        text.pose.position.y    = s.x(1);
        text.pose.position.z    = 1.0;
        text.pose.orientation.w = 1.0;

        text.scale.z  = 0.3;
        text.color.r  = 1.0f;
        text.color.g  = 1.0f;
        text.color.b  = 1.0f;
        text.color.a  = 1.0f;
        text.text     = std::to_string(s.track_id);
        text.lifetime = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(text);
    }

    pub_tracks_->publish(markers);
}

void RadarFusionNode::publish_lidar_tracks(
    const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
    const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < tracks.size(); ++i) {
        const auto& s = tracks[i].state();
        if (!s.is_confirmed()) continue;

        visualization_msgs::msg::Marker m;
        m.header.stamp       = stamp;
        m.header.frame_id    = "map";
        m.ns                 = "lidar_clusters";
        m.id                 = s.track_id;
        m.type               = visualization_msgs::msg::Marker::SPHERE;
        m.action             = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x    = s.x(0);
        m.pose.position.y    = s.x(1);
        m.pose.position.z    = 0.5;
        m.pose.orientation.w = 1.0;
        m.scale.x            = 0.4;
        m.scale.y            = 0.4;
        m.scale.z            = 0.4;
        m.color.g            = 1.0f; // 黄色系: R+G
        m.color.r            = 1.0f;
        m.color.a            = 0.6f;
        m.lifetime           = rclcpp::Duration::from_seconds(0.5);
        markers.markers.push_back(m);
    }
    pub_tracks_->publish(markers);
}

void RadarFusionNode::publish_fused_tracks(
    const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
    const rclcpp::Time& stamp) {
    auto fused_markers = visualization_msgs::msg::MarkerArray();
    for (const auto& track : tracks) {
        const auto& state = track.state();
        if (!state.is_confirmed()) {
            continue;
        }

        auto marker               = visualization_msgs::msg::Marker();
        marker.header.stamp       = stamp;
        marker.header.frame_id    = "map";
        marker.ns                 = "fused_tracks";
        marker.id                 = state.track_id;
        marker.type               = visualization_msgs::msg::Marker::SPHERE;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = state.x(0);
        marker.pose.position.y    = state.x(1);
        marker.pose.position.z    = 0.6;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = 0.2;
        marker.scale.y            = 0.2;
        marker.scale.z            = 0.2;
        marker.color.r            = 1.0f;
        marker.color.g =
            fusion_mode_ == radar_fusion::fusion_config::FusionMode::RADAR_CAMERA ? 0.6f : 0.2f;
        marker.color.b  = 1.0f;
        marker.color.a  = 0.9f;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        fused_markers.markers.push_back(marker);
    }

    pub_fused_tracks_->publish(fused_markers);
}

void RadarFusionNode::on_game_state(const radar_interfaces::msg::GameState::SharedPtr msg) {
    match_timer_.on_game_state(msg->game_progress, msg->stage_remain_time, steady_now_ns());
}

int64_t RadarFusionNode::steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

void RadarFusionNode::fill_default_positions(
    radar_interfaces::msg::LidarLocation& msg, int64_t now_ns) {
    if (default_positions_path_.empty()) return;
    const auto t = match_timer_.elapsed_sec(now_ns);
    if (t < 0) return;

    const int enemy_camp = (enemy_color_ == "red") ? 0 : 1;
    const int ally_camp  = 1 - enemy_camp;

    // Confirmed tracks occupy opponent slots in track order (same loop and
    // order as publish_lidar_location); remaining slots get defaults.
    std::array<bool, 6> occupied_opponent { };
    std::array<bool, 6> occupied_ally { };
    std::size_t slot_idx = 0;
    for (const auto& track : tracks_) {
        const auto& s = track.state();
        if (!s.is_confirmed()) continue;
        if (slot_idx >= 6) break;
        occupied_opponent[slot_idx] = true;
        ++slot_idx;
    }

    const auto& query = radar_fusion::default_positions::query_clamped;
    // Defaults are already in the official referee frame; no map_to_rm_offset.
    radar_fusion::default_positions::fill_empty_slots(msg,
        radar_fusion::default_positions::kOpponentSlots, occupied_opponent, enemy_camp,
        static_cast<int>(t), query);
    radar_fusion::default_positions::fill_empty_slots(msg,
        radar_fusion::default_positions::kAllySlots, occupied_ally, ally_camp, static_cast<int>(t),
        query);
}

void RadarFusionNode::publish_lidar_location(
    const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks,
    const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& lidar_tracks) {
    auto msg = radar_interfaces::msg::LidarLocation { };

    uint16_t* const slots_x[] = {
        &msg.opponent_hero_x,
        &msg.opponent_engineer_x,
        &msg.opponent_infantry_3_x,
        &msg.opponent_infantry_4_x,
        &msg.opponent_aerial_x,
        &msg.opponent_sentry_x,
    };
    uint16_t* const slots_y[] = {
        &msg.opponent_hero_y,
        &msg.opponent_engineer_y,
        &msg.opponent_infantry_3_y,
        &msg.opponent_infantry_4_y,
        &msg.opponent_aerial_y,
        &msg.opponent_sentry_y,
    };

    // 按 class_id 填对应槽位（class_id: 0=hero 1=eng 2=inf3 3=inf4 4=sentry 5=drone）。
    // 协议槽位顺序: hero(0), engineer(1), inf3(2), inf4(3), aerial无人机(4), sentry(5)。
    // 注意 sentry/drone 槽位与 class_id 交叉: class 4=sentry -> slot 5; class 5=drone -> slot 4。
    static constexpr int kClassToSlot[6] = { 0, 1, 2, 3, 5, 4 };
    // 相机池先填；雷达池（继承相机类别）后填覆盖——雷达聚类位置更持续准确。
    const auto fill_slots = [&](const auto& pool) {
        for (const auto& track : pool) {
            const auto& s = track.state();
            if (!s.is_confirmed()) continue;
            if (s.class_id < 0 || s.class_id >= 6) continue;
            const int slot_idx = kClassToSlot[s.class_id];
            // RoboMaster 0x0305 / radar-egui 约定: 厘米 (米 -> cm 乘 100)
            // 截断负值：track 外推误差可能让 map+offset 为负，uint16_t 转换是 UB
            *slots_x[slot_idx] =
                static_cast<uint16_t>(std::max(0.0, s.x(0) + cfg_.map_to_rm_offset_x) * 100.0);
            *slots_y[slot_idx] =
                static_cast<uint16_t>(std::max(0.0, s.x(1) + cfg_.map_to_rm_offset_y) * 100.0);
        }
    };
    fill_slots(tracks);
    fill_slots(lidar_tracks);

    fill_default_positions(msg, steady_now_ns());

    msg.cmd_id = radar_interfaces::msg::LidarLocation::CMD_ID;
    pub_lidar_location_->publish(msg);
}

void RadarFusionNode::publish_localization_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose) {
    pub_pose_->publish(pose);
}

void RadarFusionNode::publish_status(const rclcpp::Time& stamp) const {
    auto status    = diagnostic_msgs::msg::DiagnosticStatus();
    status.level   = (fusion_mode_ == radar_fusion::fusion_config::FusionMode::DEGRADED)
        ? diagnostic_msgs::msg::DiagnosticStatus::WARN
        : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.name    = "radar_fusion/status";
    status.message = std::string(to_string(fusion_mode_));

    auto add_value = [&status](const std::string& key, const std::string& value) {
        auto item  = diagnostic_msgs::msg::KeyValue();
        item.key   = key;
        item.value = value;
        status.values.push_back(item);
    };
    add_value("stamp_ns", std::to_string(stamp.nanoseconds()));
    add_value("mode", std::string(to_string(fusion_mode_)));
    add_value("camera_enabled", cfg_.enable_camera_fusion ? "true" : "false");
    add_value("camera_observations", std::to_string(latest_camera_observations_.size()));
    add_value("track_count", std::to_string(tracks_.size()));

    pub_status_->publish(status);
}

void RadarFusionNode::update_fusion_mode(int64_t reference_stamp_ns) {
    if (!cfg_.enable_camera_fusion) {
        fusion_mode_ = radar_fusion::fusion_config::FusionMode::RADAR_ONLY;
        return;
    }

    const auto timeout_ns   = static_cast<int64_t>(cfg_.camera_timeout_sec * 1e9);
    const bool camera_stale = latest_camera_observations_.empty()
        || (reference_stamp_ns - latest_camera_stamp_ns_) > timeout_ns;

    fusion_mode_ = camera_stale ? radar_fusion::fusion_config::FusionMode::DEGRADED
                                : radar_fusion::fusion_config::FusionMode::RADAR_CAMERA;
}

} // namespace radar_fusion::node

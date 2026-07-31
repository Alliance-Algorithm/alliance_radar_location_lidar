#include "radar_fusion/radar_fusion_node.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
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

    this->declare_parameter("gate_distance", 1.0);
    this->declare_parameter("track_timeout_sec", 1.5);
    this->declare_parameter("min_hits_to_confirm", 3);
    this->declare_parameter("max_misses_before_delete", 2);
    this->declare_parameter("max_tracks", 20);
    this->declare_parameter("enable_camera_fusion", false);
    this->declare_parameter("camera_lidar_consistency_distance", 1.0);
    this->declare_parameter("identity_retention_sec", 1.5);
    this->declare_parameter("map_to_rm_offset_x", 14.0);
    this->declare_parameter("map_to_rm_offset_y", 7.5);

    cfg_.gate_distance            = this->get_parameter("gate_distance").as_double();
    cfg_.track_timeout_sec        = this->get_parameter("track_timeout_sec").as_double();
    cfg_.min_hits_to_confirm      = this->get_parameter("min_hits_to_confirm").as_int();
    cfg_.max_misses_before_delete = this->get_parameter("max_misses_before_delete").as_int();
    cfg_.max_tracks               = this->get_parameter("max_tracks").as_int();
    cfg_.enable_camera_fusion     = this->get_parameter("enable_camera_fusion").as_bool();
    cfg_.camera_lidar_consistency_distance =
        this->get_parameter("camera_lidar_consistency_distance").as_double();
    cfg_.identity_retention_sec = this->get_parameter("identity_retention_sec").as_double();
    cfg_.map_to_rm_offset_x       = this->get_parameter("map_to_rm_offset_x").as_double();
    cfg_.map_to_rm_offset_y       = this->get_parameter("map_to_rm_offset_y").as_double();
    if (!std::isfinite(cfg_.camera_lidar_consistency_distance)
        || cfg_.camera_lidar_consistency_distance <= 0.0) {
        throw std::invalid_argument("camera_lidar_consistency_distance must be positive");
    }
    if (!std::isfinite(cfg_.identity_retention_sec) || cfg_.identity_retention_sec <= 0.0) {
        throw std::invalid_argument("identity_retention_sec must be positive");
    }
    tracks_.reserve(static_cast<std::size_t>(cfg_.max_tracks));

    sub_lidar_pose_ = this->create_subscription<PoseCov>(
        "/lidar/pose", 10, [this](const PoseCov::SharedPtr msg) { on_lidar_pose(msg); });

    sub_cluster_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cluster(msg); });

    if (cfg_.enable_camera_fusion) {
        sub_camera_detection_ =
            this->create_subscription<radar_interfaces::msg::CameraDetectionArray>("/camera/"
                                                                                     "detection",
                10, [this](const radar_interfaces::msg::CameraDetectionArray::SharedPtr msg) {
                    on_camera_detection(msg);
                });
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
    const radar_interfaces::msg::CameraDetectionArray::SharedPtr msg) {
    auto stamp              = rclcpp::Time(msg->header.stamp);
    latest_camera_stamp_ns_ = stamp.nanoseconds();
    latest_camera_observations_.clear();

    using Team = radar_fusion::camera_observation::Team;
    using SemanticClass = radar_fusion::camera_observation::SemanticClass;
    for (const auto& detection : msg->detections) {
        const auto team = static_cast<Team>(detection.team);
        const auto semantic_class = static_cast<SemanticClass>(detection.semantic_class);
        if (team == Team::UNKNOWN || semantic_class == SemanticClass::UNKNOWN
            || !std::isfinite(detection.confidence) || detection.confidence <= 0.0f
            || !std::isfinite(detection.position.x) || !std::isfinite(detection.position.y)
            || !std::isfinite(detection.position.z)) {
            continue;
        }
        latest_camera_observations_.push_back({
            .x = detection.position.x,
            .y = detection.position.y,
            .z = detection.position.z,
            .confidence = detection.confidence,
            .team = team,
            .semantic_class = semantic_class,
        });
    }

    for (const auto& observation : latest_camera_observations_) {
        const auto slot = find_semantic_slot(observation.team, observation.semantic_class);
        if (slot.has_value()) {
            semantic_slots_[*slot].position = Eigen::Vector2d(observation.x, observation.y);
            semantic_slots_[*slot].last_update_ns = stamp.nanoseconds();
        } else {
            semantic_slots_.push_back({ observation.team, observation.semantic_class,
                Eigen::Vector2d(observation.x, observation.y), stamp.nanoseconds() });
        }
    }

    update_fusion_mode(latest_camera_stamp_ns_);
    for (const auto& observation : latest_camera_observations_) {
        const Eigen::Vector2d measurement(observation.x, observation.y);
        auto existing = std::find_if(tracks_.begin(), tracks_.end(), [&](const auto& track) {
            const auto& state = track.state();
            return state.team == observation.team
                && state.semantic_class == observation.semantic_class;
        });
        if (existing != tracks_.end()) {
            existing->update(measurement, stamp.nanoseconds(), cfg_.min_hits_to_confirm);
            continue;
        }
        if (tracks_.size() >= static_cast<std::size_t>(cfg_.max_tracks)) continue;
        radar_fusion::kalman_tracker::KalmanTracker track(next_track_id_++);
        track.update_identity(measurement, stamp.nanoseconds(), cfg_.min_hits_to_confirm,
            observation.team, observation.semantic_class);
        tracks_.push_back(track);
    }

    publish_tracks(tracks_, stamp);
    publish_fused_tracks(tracks_, stamp);
    publish_lidar_location(tracks_, stamp.nanoseconds());
    publish_status(stamp);
}

void RadarFusionNode::process_measurements(
    const std::vector<Eigen::Vector2d>& measurements, int64_t now_ns, bool mark_unmatched_tracks) {
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

    process_measurements(measurements, now_ns, true);

    publish_tracks(tracks_, stamp);
    publish_fused_tracks(tracks_, stamp);
    update_semantic_slots_from_lidar(measurements, now_ns);
    publish_lidar_location(tracks_, now_ns);
    publish_status(stamp);
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
        if (s.team == radar_fusion::camera_observation::Team::BLUE) {
            m.color.b = 1.0f;
            m.color.a = 0.8f;
        } else if (s.team == radar_fusion::camera_observation::Team::RED) {
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

void RadarFusionNode::publish_lidar_location(
    const std::vector<radar_fusion::kalman_tracker::KalmanTracker>& tracks, int64_t stamp_ns) {
    auto msg = radar_interfaces::msg::LidarLocation { };

    uint16_t* const opponent_x[] = {
        &msg.opponent_hero_x,
        &msg.opponent_engineer_x,
        &msg.opponent_infantry_3_x,
        &msg.opponent_infantry_4_x,
        &msg.opponent_aerial_x,
        &msg.opponent_sentry_x,
    };
    uint16_t* const opponent_y[] = {
        &msg.opponent_hero_y,
        &msg.opponent_engineer_y,
        &msg.opponent_infantry_3_y,
        &msg.opponent_infantry_4_y,
        &msg.opponent_aerial_y,
        &msg.opponent_sentry_y,
    };
    uint16_t* const ally_x[] = {
        &msg.ally_hero_x,
        &msg.ally_engineer_x,
        &msg.ally_infantry_3_x,
        &msg.ally_infantry_4_x,
        &msg.ally_aerial_x,
        &msg.ally_sentry_x,
    };
    uint16_t* const ally_y[] = {
        &msg.ally_hero_y,
        &msg.ally_engineer_y,
        &msg.ally_infantry_3_y,
        &msg.ally_infantry_4_y,
        &msg.ally_aerial_y,
        &msg.ally_sentry_y,
    };

    expire_semantic_slots(stamp_ns);
    for (const auto& slot : semantic_slots_) {
        if (slot.team == radar_fusion::camera_observation::Team::UNKNOWN
            || slot.semantic_class == radar_fusion::camera_observation::SemanticClass::UNKNOWN) {
            continue;
        }
        const bool opponent = slot.team == radar_fusion::camera_observation::Team::BLUE;
        const int class_idx = static_cast<int>(slot.semantic_class) - 1;
        if (class_idx < 0 || class_idx >= 6) continue;
        auto* slots_x = opponent ? opponent_x : ally_x;
        auto* slots_y = opponent ? opponent_y : ally_y;
        *slots_x[class_idx] = radar_fusion::location::meters_to_cm(
            slot.position.x(), cfg_.map_to_rm_offset_x);
        *slots_y[class_idx] = radar_fusion::location::meters_to_cm(
            slot.position.y(), cfg_.map_to_rm_offset_y);
    }

    msg.cmd_id = radar_interfaces::msg::LidarLocation::CMD_ID;
    pub_lidar_location_->publish(msg);
}

auto RadarFusionNode::find_semantic_slot(camera_observation::Team team,
    camera_observation::SemanticClass semantic_class) -> std::optional<std::size_t> {
    for (std::size_t index = 0; index < semantic_slots_.size(); ++index) {
        if (semantic_slots_[index].team == team
            && semantic_slots_[index].semantic_class == semantic_class) {
            return index;
        }
    }
    return std::nullopt;
}

void RadarFusionNode::expire_semantic_slots(int64_t now_ns) {
    const auto retention_ns = static_cast<int64_t>(cfg_.identity_retention_sec * 1e9);
    semantic_slots_.erase(std::remove_if(semantic_slots_.begin(), semantic_slots_.end(),
                              [&](const SemanticSlot& slot) {
                                  return now_ns - slot.last_update_ns > retention_ns;
                              }),
        semantic_slots_.end());
}

void RadarFusionNode::update_semantic_slots_from_lidar(
    const std::vector<Eigen::Vector2d>& measurements, int64_t stamp_ns) {
    expire_semantic_slots(stamp_ns);
    const double consistency_sq =
        cfg_.camera_lidar_consistency_distance * cfg_.camera_lidar_consistency_distance;
    std::vector<bool> used(measurements.size(), false);
    for (auto& slot : semantic_slots_) {
        std::size_t best = measurements.size();
        double best_distance = consistency_sq;
        for (std::size_t index = 0; index < measurements.size(); ++index) {
            if (used[index] || !measurements[index].allFinite()) continue;
            const double distance = (measurements[index] - slot.position).squaredNorm();
            if (distance <= best_distance) {
                best = index;
                best_distance = distance;
            }
        }
        if (best != measurements.size()) {
            slot.position = measurements[best];
            slot.last_update_ns = stamp_ns;
            used[best] = true;
        }
    }
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

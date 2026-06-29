#include "radar_fusion/fusion_node.hpp"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace radar::fusion {

FusionNode::FusionNode()
    : Node("radar_fusion_node") {

    this->declare_parameter("gate_distance", 1.0);
    this->declare_parameter("track_timeout_sec", 1.5);
    this->declare_parameter("min_hits_to_confirm", 3);
    this->declare_parameter("max_tracks", 20);

    cfg_.gate_distance       = this->get_parameter("gate_distance").as_double();
    cfg_.track_timeout_sec   = this->get_parameter("track_timeout_sec").as_double();
    cfg_.min_hits_to_confirm = this->get_parameter("min_hits_to_confirm").as_int();
    cfg_.max_tracks          = this->get_parameter("max_tracks").as_int();
    tracks_.reserve(static_cast<std::size_t>(cfg_.max_tracks));

    sub_lidar_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/li"
                                                                                               "dar"
                                                                                               "/po"
                                                                                               "se",
        10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            on_lidar_pose(msg);
        });

    sub_cluster_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar/cluster", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { on_cluster(msg); });

    pub_tracks_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/fusion/tracks", 10);
    pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);

    RCLCPP_INFO(get_logger(), "radar_fusion ready. gate=%.1fm timeout=%.1fs", cfg_.gate_distance,
        cfg_.track_timeout_sec);
}

void FusionNode::on_lidar_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose.pose;
    pub_pose_->publish(pose);
}

void FusionNode::on_cluster(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto stamp  = rclcpp::Time(msg->header.stamp);
    auto now_ns = stamp.nanoseconds();

    std::vector<Eigen::Vector2d> measurements;
    measurements.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
        if (std::isfinite(*iter_x) && std::isfinite(*iter_y)) {
            measurements.emplace_back(*iter_x, *iter_y);
        }
    }

    // 1. 预测所有现有轨迹
    for (auto& track : tracks_) {
        track.predict(now_ns);
    }

    // 2. 数据关联（最近邻贪婪匹配）
    std::vector<bool> matched_meas(measurements.size(), false);
    const double gate_distance_sq = cfg_.gate_distance * cfg_.gate_distance;

    for (size_t i = 0; i < tracks_.size(); ++i) {
        double min_dist_sq = gate_distance_sq;
        int min_idx        = -1;
        for (size_t j = 0; j < measurements.size(); ++j) {
            if (matched_meas[j]) continue;
            const double d_sq = tracks_[i].distance_squared_to(measurements[j]);
            if (d_sq < min_dist_sq) {
                min_dist_sq = d_sq;
                min_idx     = static_cast<int>(j);
            }
        }

        if (min_idx >= 0) {
            tracks_[i].update(measurements[min_idx], now_ns);
            matched_meas[min_idx] = true;
        }
    }

    // 3. 未匹配的观测 → 新建轨迹
    for (size_t j = 0; j < measurements.size(); ++j) {
        if (matched_meas[j]) continue;
        if (tracks_.size() >= static_cast<size_t>(cfg_.max_tracks)) break;

        KalmanTracker new_track(next_track_id_++);
        new_track.update(measurements[j], now_ns);
        tracks_.push_back(new_track);
    }

    // 4. 删除超时轨迹
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                      [&](const KalmanTracker& t) {
                          return t.state().is_stale(now_ns, cfg_.track_timeout_sec);
                      }),
        tracks_.end());

    // 5. 发布
    publish_tracks(tracks_, stamp);
}

void FusionNode::publish_tracks(
    const std::vector<KalmanTracker>& tracks, const rclcpp::Time& stamp) {
    visualization_msgs::msg::MarkerArray markers;

    for (size_t i = 0; i < tracks.size(); ++i) {
        const auto& s = tracks[i].state();
        if (s.hit_count < cfg_.min_hits_to_confirm) continue;

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

} // namespace radar::fusion

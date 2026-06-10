#include "pointcloud_capture.hpp"
#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
using namespace Radar::capture;
class PointCloudCapture::Impl : public rclcpp::Node {
public:
    std::mutex mutex_;
    PointCloudData latest_pointcloud_;
    Impl(std::string pointcloud_topic)
        : Node("pointcloud_capture_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic,
            10, std::bind(&Impl::PointCloudCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) { }
};
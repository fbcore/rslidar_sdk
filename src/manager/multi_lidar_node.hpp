
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "lidar_handler.hpp"

class MultiLidarNode : public rclcpp::Node
{
public:
  MultiLidarNode(const rclcpp::NodeOptions& options);

private:
  void loadParameters();
  void mergeAndPublish();

  std::vector<std::shared_ptr<LidarHandler>> lidar_handlers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string base_frame_id_;

  void runInitialCalibration();
};

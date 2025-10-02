#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "lidar_handler.hpp"

// Struct to hold a single ROI filter configuration
struct RoiFilterConfig
{
  std::string name;
  std::string type; // "positive" or "negative"
  float min_x, max_x, min_y, max_y, min_z, max_z;
};

class MultiLidarNode : public rclcpp::Node
{
public:
  MultiLidarNode(const rclcpp::NodeOptions& options);

private:
  void loadParameters();
  void loadFilterParameters();
  void mergeAndPublish();
  void runInitialCalibration();

  std::vector<std::shared_ptr<LidarHandler>> lidar_handlers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string base_frame_id_;

  // Filter parameters
  bool enable_roi_filter_;
  std::vector<RoiFilterConfig> roi_filters_;
  bool enable_voxel_filter_;
  float voxel_leaf_size_;
};
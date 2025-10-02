#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> // New include for LaserScan
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include "gpu_lidar_handler.hpp"

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
  void loadFlatScanParameters(); // New function to load FlatScan parameters
  void mergeAndPublish();
  void runInitialCalibration();

  std::vector<std::shared_ptr<GPULidarHandler>> lidar_handlers_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
  bool publish_3d_pcd_; // New member variable
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr flatscan_pub_; // LaserScan publisher
  rclcpp::TimerBase::SharedPtr timer_;

  std::string base_frame_id_;

  // Filter parameters
  bool enable_roi_filter_;
  std::vector<RoiFilterConfig> roi_filters_;
  bool enable_voxel_filter_;
  float voxel_leaf_size_;

  // FlatScan parameters
  bool publish_flatscan_;
  std::string flatscan_topic_name_;
  float flatscan_min_height_, flatscan_max_height_;
  float flatscan_angle_min_, flatscan_angle_max_, flatscan_angle_increment_;
  float flatscan_range_min_, flatscan_range_max_;
};
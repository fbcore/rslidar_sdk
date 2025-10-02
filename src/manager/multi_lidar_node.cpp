
#include "multi_lidar_node.hpp"
#include <rslidar_sdk/utility/yaml_reader.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>
#include <thread>
#include <algorithm>


MultiLidarNode::MultiLidarNode(const rclcpp::NodeOptions& options)
  : Node("multi_lidar_node", options)
{
  loadParameters();
  runInitialCalibration();
  loadFilterParameters();
  
  double publish_frequency = this->declare_parameter("publish_frequency", 10.0);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency)),
      std::bind(&MultiLidarNode::mergeAndPublish, this));
}

void MultiLidarNode::loadParameters()
{
  base_frame_id_ = this->declare_parameter("base_frame_id", "base_link");
  std::string merged_topic_name = this->declare_parameter("merged_topic_name", "/points_merged");
  merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_topic_name, 10);

  auto lidar_params = this->declare_parameter<std::vector<std::string>>("lidars", std::vector<std::string>());
  
  // This is a workaround to get nested parameters (a list of structs)
  // A better approach might use a different YAML structure or a more advanced parsing library
  for (size_t i = 0; i < lidar_params.size(); ++i) 
  {
      std::string lidar_prefix = "lidars." + std::to_string(i) + ".";
      bool enabled = this->declare_parameter(lidar_prefix + "enabled", false);
      if (!enabled) continue;

      RsDriverParam driver_param;
      driver_param.lidar_type = (LidarType)this->declare_parameter(lidar_prefix + "driver.lidar_type", (int)LidarType::RS16);
      driver_param.input_type = (InputType)this->declare_parameter(lidar_prefix + "driver.input_type", (int)InputType::ONLINE_LIDAR);
      driver_param.msop_port = this->declare_parameter(lidar_prefix + "driver.msop_port", 6699);
      driver_param.difop_port = this->declare_parameter(lidar_prefix + "driver.difop_port", 7788);

      double x = this->declare_parameter(lidar_prefix + "tf.x", 0.0);
      double y = this->declare_parameter(lidar_prefix + "tf.y", 0.0);
      double z = this->declare_parameter(lidar_prefix + "tf.z", 0.0);
      double roll = this->declare_parameter(lidar_prefix + "tf.roll", 0.0);
      double pitch = this->declare_parameter(lidar_prefix + "tf.pitch", 0.0);
      double yaw = this->declare_parameter(lidar_prefix + "tf.yaw", 0.0);

      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      Eigen::Translation3f translation(x, y, z);
      Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
      Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
      transform = (translation * rot_z * rot_y * rot_x).matrix();

      lidar_handlers_.emplace_back(std::make_shared<LidarHandler>(driver_param, transform));
      RCLCPP_INFO(this->get_logger(), "Initialized lidar: %s", this->declare_parameter(lidar_prefix + "name", "").c_str());
  }
}

void MultiLidarNode::mergeAndPublish()
{
  pcl::PointCloud<pcl::PointXYZI> merged_cloud;

  for (const auto& handler : lidar_handlers_)
  {
    auto cloud_msg = handler->getPointCloud();
    if (cloud_msg)
    {
      pcl::PointCloud<pcl::PointXYZI> cloud_part;
      // This is a simplified conversion. You may need a proper converter 
      // depending on the exact point cloud format from the driver.
      for (const auto& p : cloud_msg->points)
      {
          pcl::PointXYZI new_point;
          new_point.x = p.x;
          new_point.y = p.y;
          new_point.z = p.z;
          new_point.intensity = p.intensity;
          cloud_part.points.push_back(new_point);
      }

      pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
      pcl::transformPointCloud(cloud_part, transformed_cloud, handler->getTransform());
      merged_cloud += transformed_cloud;
    }
  }

  if (!merged_cloud.empty())
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *filtered_cloud = merged_cloud;

    // 1. Apply ROI Filters (multiple positive/negative with priority)
    if (enable_roi_filter_ && !filtered_cloud->empty() && !roi_filters_.empty())
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr roi_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      for (const auto& point : filtered_cloud->points)
      {
        bool in_any_positive_box = false;
        bool in_any_negative_box = false;

        for (const auto& filter_config : roi_filters_)
        {
          bool in_box = (point.x >= filter_config.min_x && point.x <= filter_config.max_x &&
                         point.y >= filter_config.min_y && point.y <= filter_config.max_y &&
                         point.z >= filter_config.min_z && point.z <= filter_config.max_z);

          if (in_box)
          {
            if (filter_config.type == "positive")
            {
              in_any_positive_box = true;
            }
            else if (filter_config.type == "negative")
            {
              in_any_negative_box = true;
            }
          }
        }

        // Priority logic: If in any positive box, keep. Else if not in any negative box, keep.
        if (in_any_positive_box || (!in_any_negative_box && !in_any_positive_box))
        {
          roi_filtered_cloud->points.push_back(point);
        }
      }
      roi_filtered_cloud->width = roi_filtered_cloud->points.size();
      roi_filtered_cloud->height = 1;
      roi_filtered_cloud->is_dense = true;
      filtered_cloud = roi_filtered_cloud;
    }

    // 2. Apply Voxel Grid Filter
    if (enable_voxel_filter_ && !filtered_cloud->empty())
    {
      pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
      voxel_filter.setInputCloud(filtered_cloud);
      voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel_filter.filter(*filtered_cloud);
    }

    if (filtered_cloud->empty())
    {
      RCLCPP_WARN(this->get_logger(), "Filtered cloud is empty. Not publishing.");
      return;
    }

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header.stamp = this->get_clock()->now();
    output_msg.header.frame_id = base_frame_id_;
    merged_pub_->publish(output_msg);
  }
}

void MultiLidarNode::loadFilterParameters()
{
  enable_roi_filter_ = this->declare_parameter("enable_roi_filter", false);
  if (enable_roi_filter_)
  {
    RCLCPP_INFO(this->get_logger(), "[Filter] ROI filter enabled.");
    auto roi_filters_param = this->declare_parameter<std::vector<rclcpp::Parameter>>("roi_filters", std::vector<rclcpp::Parameter>());

    for (size_t i = 0; i < roi_filters_param.size(); ++i)
    {
      std::string prefix = "roi_filters." + std::to_string(i) + ".";
      RoiFilterConfig config;
      config.name = this->declare_parameter(prefix + "name", "unnamed_roi");
      config.type = this->declare_parameter(prefix + "type", "positive");
      config.min_x = this->declare_parameter(prefix + "min_x", -100.0f);
      config.max_x = this->declare_parameter(prefix + "max_x", 100.0f);
      config.min_y = this->declare_parameter(prefix + "min_y", -100.0f);
      config.max_y = this->declare_parameter(prefix + "max_y", 100.0f);
      config.min_z = this->declare_parameter(prefix + "min_z", -100.0f);
      config.max_z = this->declare_parameter(prefix + "max_z", 100.0f);
      roi_filters_.push_back(config);

      RCLCPP_INFO(this->get_logger(), "  - ROI Filter '%s' (type: %s): X=[%f, %f], Y=[%f, %f], Z=[%f, %f]",
                  config.name.c_str(), config.type.c_str(),
                  config.min_x, config.max_x, config.min_y, config.max_y, config.min_z, config.max_z);
    }
  }

  enable_voxel_filter_ = this->declare_parameter("enable_voxel_filter", false);
  if (enable_voxel_filter_)
  {
    voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.1f);
    RCLCPP_INFO(this->get_logger(), "[Filter] Voxel filter enabled: leaf_size=%f", voxel_leaf_size_);
  }
}

void MultiLidarNode::runInitialCalibration()
{
  if (lidar_handlers_.size() < 2)
  {
    RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Not enough LiDARs (%zu) for ICP calibration. Skipping.", lidar_handlers_.size());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Starting initial ICP calibration...");

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> initial_clouds(lidar_handlers_.size());
  std::vector<bool> cloud_received(lidar_handlers_.size(), false);
  auto start_time = std::chrono::high_resolution_clock::now();
  const std::chrono::seconds timeout(15); // 15 seconds to collect initial clouds

  // 1. Collect initial point clouds from all LiDARs
  while (std::any_of(cloud_received.begin(), cloud_received.end(), [](bool b){ return !b; }))
  {
    auto current_time = std::chrono::high_resolution_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time) > timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "[ICP Calibration] Timeout while collecting initial point clouds. Calibration aborted.");
      return;
    }

    for (size_t i = 0; i < lidar_handlers_.size(); ++i)
    {
      if (!cloud_received[i])
      {
        auto cloud_msg = lidar_handlers_[i]->getPointCloud();
        if (cloud_msg && !cloud_msg->points.empty())
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
          for (const auto& p : cloud_msg->points)
          {
            pcl::PointXYZI new_point;
            new_point.x = p.x;
            new_point.y = p.y;
            new_point.z = p.z;
            new_point.intensity = p.intensity;
            pcl_cloud->points.push_back(new_point);
          }
          pcl_cloud->width = pcl_cloud->points.size();
          pcl_cloud->height = 1;
          pcl_cloud->is_dense = true;

          initial_clouds[i] = pcl_cloud;
          cloud_received[i] = true;
          RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Received initial cloud from LiDAR %zu.", i);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait a bit before checking again
  }

  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] All initial point clouds collected. Proceeding with ICP.");

  // 2. Perform ICP for each source LiDAR against the first LiDAR (target)
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = initial_clouds[0];
  // The transform of the target LiDAR (lidar_handlers_[0]) relative to base_link
  Eigen::Matrix4f base_to_target_tf = lidar_handlers_[0]->getTransform();

  for (size_t i = 1; i < lidar_handlers_.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = initial_clouds[i];
    std::shared_ptr<LidarHandler> source_handler = lidar_handlers_[i];

    // Get the initial guess for source_lidar_link to base_link
    Eigen::Matrix4f initial_source_to_base_tf = source_handler->getTransform();

    // Calculate the initial guess for source_lidar_link to target_lidar_link
    // This is: (base_link -> target_lidar_link)^-1 * (base_link -> source_lidar_link)
    Eigen::Matrix4f initial_guess_source_to_target = base_to_target_tf.inverse() * initial_source_to_base_tf;

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(0.5); // Max distance for a point to be considered a correspondence
    icp.setMaximumIterations(100);         // Max number of ICP iterations
    icp.setTransformationEpsilon(1e-8);    // Minimum transformation difference between iterations
    icp.setEuclideanFitnessEpsilon(1e-5);  // Minimum fitness score difference between iterations

    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    pcl::PointCloud<pcl::PointXYZI> unused_aligned_cloud; // To store the aligned cloud, not directly used after ICP
    icp.align(unused_aligned_cloud, initial_guess_source_to_target);

    if (icp.hasConverged())
    {
      Eigen::Matrix4f final_source_to_target_tf = icp.getFinalTransformation();
      
      // Calculate the new transform for source_lidar_link to base_link
      // This is: (base_link -> target_lidar_link) * (target_lidar_link -> source_lidar_link)
      Eigen::Matrix4f calibrated_source_to_base_tf = base_to_target_tf * final_source_to_target_tf;

      source_handler->setTransform(calibrated_source_to_base_tf);
      RCLCPP_INFO(this->get_logger(), "[ICP Calibration] LiDAR %zu calibrated successfully. Fitness score: %f", i, icp.getFitnessScore());
      // Optionally print the new transform matrix
      // RCLCPP_INFO(this->get_logger(), "New transform for LiDAR %zu:\n%s", i, Eigen::Matrix4f(calibrated_source_to_base_tf).format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "[", "]", "", "")).c_str());
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[ICP Calibration] ICP did not converge for LiDAR %zu. Using initial guess.", i);
    }
  }
  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Initial ICP calibration finished.");
}

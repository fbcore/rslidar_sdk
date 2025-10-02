
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
#include "../core/cuda_transform_merge.cuh"
#include "../core/cuda_roi_filter.cuh"
#include "../core/cuda_voxel_grid.cuh"


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

      lidar_handlers_.emplace_back(std::make_shared<GPULidarHandler>(driver_param, transform));
      RCLCPP_INFO(this->get_logger(), "Initialized lidar: %s", this->declare_parameter(lidar_prefix + "name", "").c_str());
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

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> initial_cpu_clouds(lidar_handlers_.size());
  std::vector<bool> cloud_received(lidar_handlers_.size(), false);
  auto start_time = std::chrono::high_resolution_clock::now();
  const std::chrono::seconds timeout(15); // 15 seconds to collect initial clouds

  // 1. Collect initial point clouds from all LiDARs (copy from GPU to CPU for PCL ICP)
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
        auto gpu_cloud_data = lidar_handlers_[i]->getGPUPointCloud();
        if (gpu_cloud_data && gpu_cloud_data->d_points_ptr && gpu_cloud_data->num_points > 0)
        {
          pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
          pcl_cloud->points.resize(gpu_cloud_data->num_points);

          // Copy from GPU to CPU for PCL ICP
          cudaError_t err = cudaMemcpy(pcl_cloud->points.data(), gpu_cloud_data->d_points_ptr.get(), 
                                       gpu_cloud_data->num_points * sizeof(CudaPointXYZI), cudaMemcpyDeviceToHost);
          if (err != cudaSuccess)
          {
              RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for ICP) failed: %s", cudaGetErrorString(err));
              continue;
          }

          pcl_cloud->width = pcl_cloud->points.size();
          pcl_cloud->height = 1;
          pcl_cloud->is_dense = true;

          initial_cpu_clouds[i] = pcl_cloud;
          cloud_received[i] = true;
          RCLCPP_INFO(this->get_logger(), "[ICP Calibration] Received initial cloud from LiDAR %zu.", i);
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait a bit before checking again
  }

  RCLCPP_INFO(this->get_logger(), "[ICP Calibration] All initial point clouds collected. Proceeding with ICP.");

  // 2. Perform ICP for each source LiDAR against the first LiDAR (target)
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = initial_cpu_clouds[0];
  // The transform of the target LiDAR (lidar_handlers_[0]) relative to base_link
  Eigen::Matrix4f base_to_target_tf = lidar_handlers_[0]->getTransform();

  for (size_t i = 1; i < lidar_handlers_.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = initial_cpu_clouds[i];
    std::shared_ptr<GPULidarHandler> source_handler = lidar_handlers_[i];

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

void MultiLidarNode::mergeAndPublish()
{
  std::vector<CudaPointXYZI*> d_input_clouds;
  std::vector<size_t> h_input_counts;
  std::vector<CudaMatrix4f> h_transforms;
  size_t total_points_to_merge = 0;

  for (const auto& handler : lidar_handlers_)
  {
    auto gpu_cloud_data = handler->getGPUPointCloud();
    if (gpu_cloud_data && gpu_cloud_data->d_points_ptr && gpu_cloud_data->num_points > 0)
    {
      d_input_clouds.push_back(gpu_cloud_data->d_points_ptr.get());
      h_input_counts.push_back(gpu_cloud_data->num_points);
      total_points_to_merge += gpu_cloud_data->num_points;

      CudaMatrix4f cuda_transform;
      cuda_transform.fromEigen(handler->getTransform().data());
      h_transforms.push_back(cuda_transform);
    }
  }

  if (total_points_to_merge == 0)
  {
    RCLCPP_WARN(this->get_logger(), "No points to merge. Not publishing.");
    return;
  }

  CudaPointXYZI* d_merged_cloud = nullptr;
  cudaError_t err = cudaMalloc((void**)&d_merged_cloud, total_points_to_merge * sizeof(CudaPointXYZI));
  if (err != cudaSuccess)
  {
      RCLCPP_ERROR(this->get_logger(), "cudaMalloc for merged cloud failed: %s", cudaGetErrorString(err));
      return;
  }

  // Launch CUDA kernel for transform and merge
  err = transformAndMergeGPU(d_input_clouds, h_input_counts, h_transforms, d_merged_cloud, total_points_to_merge);
  if (err != cudaSuccess)
  {
      RCLCPP_ERROR(this->get_logger(), "transformAndMergeGPU kernel launch failed: %s", cudaGetErrorString(err));
      cudaFree(d_merged_cloud);
      return;
  }

  CudaPointXYZI* d_roi_filtered_cloud = nullptr;
  size_t num_roi_filtered_points = 0;

  // 1. Apply ROI Filters (multiple positive/negative with priority) on GPU
  if (enable_roi_filter_ && !roi_filters_.empty())
  {
    std::vector<CudaRoiFilterConfig> h_cuda_roi_filters;
    for (const auto& config : roi_filters_)
    {
      CudaRoiFilterConfig cuda_config;
      cuda_config.min_x = config.min_x; cuda_config.max_x = config.max_x;
      cuda_config.min_y = config.min_y; cuda_config.max_y = config.max_y;
      cuda_config.min_z = config.min_z; cuda_config.max_z = config.max_z;
      cuda_config.type = (config.type == "positive") ? 0 : 1;
      h_cuda_roi_filters.push_back(cuda_config);
    }

    err = roiFilterGPU(d_merged_cloud, total_points_to_merge,
                       h_cuda_roi_filters.data(), h_cuda_roi_filters.size(),
                       &d_roi_filtered_cloud, &num_roi_filtered_points);
    if (err != cudaSuccess)
    {
        RCLCPP_ERROR(this->get_logger(), "roiFilterGPU kernel launch failed: %s", cudaGetErrorString(err));
        cudaFree(d_merged_cloud);
        return;
    }
    cudaFree(d_merged_cloud); // Free merged cloud after filtering
  }
  else
  {
    d_roi_filtered_cloud = d_merged_cloud; // No ROI filter, pass through
    num_roi_filtered_points = total_points_to_merge;
  }

  CudaPointXYZI* d_voxel_filtered_cloud = nullptr;
  size_t num_voxel_filtered_points = 0;

  // 2. Apply Voxel Grid Filter on GPU
  if (enable_voxel_filter_ && num_roi_filtered_points > 0)
  {
    err = voxelGridDownsampleGPU(d_roi_filtered_cloud, num_roi_filtered_points,
                                 voxel_leaf_size_, &d_voxel_filtered_cloud, &num_voxel_filtered_points);
    if (err != cudaSuccess)
    {
        RCLCPP_ERROR(this->get_logger(), "voxelGridDownsampleGPU kernel launch failed: %s", cudaGetErrorString(err));
        cudaFree(d_roi_filtered_cloud);
        return;
    }
    cudaFree(d_roi_filtered_cloud); // Free ROI filtered cloud after voxel filtering
  }
  else
  {
    d_voxel_filtered_cloud = d_roi_filtered_cloud; // No Voxel filter, pass through
    num_voxel_filtered_points = num_roi_filtered_points;
  }

  if (num_voxel_filtered_points == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Final filtered cloud is empty. Not publishing.");
    if (d_voxel_filtered_cloud) cudaFree(d_voxel_filtered_cloud);
    return;
  }

  // Copy final GPU cloud back to CPU for ROS publishing
  pcl::PointCloud<pcl::PointXYZI>::Ptr final_cpu_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  final_cpu_cloud->points.resize(num_voxel_filtered_points);
  err = cudaMemcpy(final_cpu_cloud->points.data(), d_voxel_filtered_cloud, 
                   num_voxel_filtered_points * sizeof(CudaPointXYZI), cudaMemcpyDeviceToHost);
  if (err != cudaSuccess)
  {
      RCLCPP_ERROR(this->get_logger(), "cudaMemcpy (GPU to CPU for ROS publish) failed: %s", cudaGetErrorString(err));
      cudaFree(d_voxel_filtered_cloud);
      return;
  }
  final_cpu_cloud->width = num_voxel_filtered_points;
  final_cpu_cloud->height = 1;
  final_cpu_cloud->is_dense = true;

  cudaFree(d_voxel_filtered_cloud); // Free final GPU cloud

  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*final_cpu_cloud, output_msg);
  output_msg.header.stamp = this->get_clock()->now();
  output_msg.header.frame_id = base_frame_id_;
  merged_pub_->publish(output_msg);
}

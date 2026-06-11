#pragma once

#include <rclcpp/rclcpp.hpp>

#include <drake/geometry/rgba.h>
#include <drake/math/rigid_transform.h>
#include <drake/common/text_logging.h>
#include <drake/multibody/meshcat/contact_visualizer.h>

#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/pointcloud_voxelization.hpp>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <drake/perception/point_cloud.h>
#include <drake/systems/sensors/image.h>
#include <drake/systems/sensors/camera_info.h>


using drake::geometry::Rgba;
using drake::math::RigidTransformd;
using drake::geometry::Meshcat;

using voxelized_geometry_tools::CollisionMap;
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudVoxelizationFilterOptions;
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudVoxelizationInterface;

using MarkerArray = visualization_msgs::msg::MarkerArray;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using ColorRGBA = std_msgs::msg::ColorRGBA;

using drake::perception::PointCloud;
using drake::systems::sensors::CameraInfo;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageRgb8U;


namespace qr_mapping {

struct VoxelGridConfig {
  double grid_resolution { 0.04 };

  double x_size { 2.0 };
  double y_size { 2.0 };
  double z_size { 2.0 };

  double x_origin { -1.0};
  double y_origin { -1.0 };
  double z_origin {  0.0 };
  double roll_origin { 0.0 };
  double pitch_origin { 0.0 };
  double yaw_origin { 0.0 };

  float min_depth { 0.0f };
  float max_depth { 5.0f };

  double percent_seen_free { 1.0 };

  double outlier_points_threshold { 1.0 };
  int num_cameras_seen_free { 1 };
  double step_size_multiplier { 0.5 };

  std::string voxelizer_option { "best" };
  int cuda_device { 0 };
  int opencl_platform_index { 0 };
  int opencl_device_index { 0 };

  Rgba filled_color { 1.0, 0.0, 0.0, 1.0 };
  Rgba unknown_color { 0.0, 0.0, 0.0, 1.0 };

  // ROS-related
  std::string occupancy_display_topic { "/occupancy_display" };
  std::string pointcloud_topic { "/camera_hand/pointcloud" };

};


// TODO (Sid): Remove this once we stop using this as a ROS node
inline void RunRclcppInit() {
  drake::log()->info("Running rclcpp::init()");
  rclcpp::init(0, {});
}


class VoxelGridEstimator : public rclcpp::Node {
public:
  explicit VoxelGridEstimator(VoxelGridConfig config, std::shared_ptr<Meshcat> meshcat);
  void ConvertDepthImageToPointCloudAndPublishRosMsg(
    const ImageDepth32F& depth_image,
    const Eigen::Matrix3d& intrinsics,
    const Eigen::Matrix4d& extrinsics,
    const ImageRgb8U* color_image = nullptr) const;
  PointCloud2 ConvertPointCloudToRosMsg(const PointCloud& cloud) const;
  void UpdateFromDepthImage(
    const ImageDepth32F& depth_image,
    const Eigen::Matrix3d& intrinsics,
    const Eigen::Matrix4d& extrinsics,
    const ImageRgb8U* color_image = nullptr,
    int max_depth_points_to_publish = -1,  // means publish all points
    double viz_point_size = 0.001,
    bool publish_to_ros = false
  );
  void UpdateFromRosPointCloudMsg(PointCloud2::SharedPtr msg, const RigidTransformd& X_WC);
  void PublishVoxelGridToRos() const;
  void PublishVoxelGridAsBoxesToMeshcat() const;
  void PublishVoxelGridAsPointCloudToMeshcat() const;
  void PublishVoxelGridAsTwoPointCloudsToMeshcat() const;


private:
  VoxelGridConfig m_config;
  std::shared_ptr<Meshcat> m_meshcat;
  CollisionMap m_empty_grid;  // this is always empty, acts as the "static environment"
  CollisionMap m_voxel_grid;  // the "cumulative" voxel grid that is updated
  double m_step_size_multiplier = 0.5;
  PointCloudVoxelizationFilterOptions m_filter_options{};
  std::unique_ptr<PointCloudVoxelizationInterface> m_voxelizer{};
  ColorRGBA m_filled_color{}, m_free_color{}, m_unknown_color{};

  // ROS Publishers
  rclcpp::Publisher<MarkerArray>::SharedPtr m_display_pub;
  rclcpp::Publisher<PointCloud2>::SharedPtr m_pointcloud_pub;

  void UpdateCumulativeOccupancy(const CollisionMap& latest_occupancy);

  PointCloud ConvertDepthImageToPointCloud(
    const ImageDepth32F& depth_image,
    const Eigen::Matrix3d& intrinsics,
    const ImageRgb8U* color_image = nullptr) const;

};

}

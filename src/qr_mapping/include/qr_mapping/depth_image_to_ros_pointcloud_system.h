#pragma once

#include <map>
#include <memory>
#include <string>
#include <string_view>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/sensors/camera_info.h"
#include "drake/systems/sensors/image.h"
#include "drake/systems/sensors/pixel_types.h"
#include "qr_mapping/color_depth_images_to_pointcloud_generator.h"

namespace anzu {
namespace sim {

/// System to create a depth-only sensor_msgs::msg::PointCloud2 message from a
/// depth image. Depth image must be a single-channel uint16 image with depth
/// in mm. See color_depth_images_to_pointcloud_generator.h for more details on
/// expected image behavior and limitations. Has one input port for the depth
/// image plus one output port for the pointcloud message.
class DepthImageToRosPointCloudSystem
    : public drake::systems::LeafSystem<double> {
 public:
  /// Construct with the provided values:
  /// @param frame_id Frame name for the pointcloud message.
  /// @param depth_camera_info CameraInfo for the depth camera.
  /// @param generator_options Options to configure the internal pointcloud
  ///     generator. See color_depth_images_to_pointcloud_generator.h for more
  ///     information on possible configuration options.
  DepthImageToRosPointCloudSystem(
      std::string_view frame_id,
      const drake::systems::sensors::CameraInfo& depth_camera_info,
      const std::map<std::string, int>& generator_options);

  ~DepthImageToRosPointCloudSystem() override;

  const drake::systems::InputPort<double>& get_depth_image_input_port() const {
    return LeafSystem<double>::get_input_port(depth_image_input_port_index_);
  }

  const drake::systems::OutputPort<double>& get_pointcloud_output_port() const {
    return LeafSystem<double>::get_output_port(pointcloud_output_port_index_);
  }

 private:
  using PixelType = drake::systems::sensors::PixelType;
  using DepthImageType = drake::systems::sensors::Image<PixelType::kDepth16U>;

  void CalcPointCloudFromDepthImage(
      const DepthImageType& depth_image,
      const drake::systems::sensors::CameraInfo& depth_camera_info,
      sensor_msgs::msg::PointCloud2* pointcloud_message) const;

  const std::string frame_id_;
  const drake::systems::sensors::CameraInfo depth_camera_info_;

  drake::systems::InputPortIndex depth_image_input_port_index_;
  drake::systems::OutputPortIndex pointcloud_output_port_index_;

  std::unique_ptr<internal::ColorDepthImagesToPointCloudGenerator>
      pointcloud_generator_;
};

}  // namespace sim
}  // namespace anzu

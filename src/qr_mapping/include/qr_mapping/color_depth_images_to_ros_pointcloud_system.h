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

/// System to create a color and depth sensor_msgs::msg::PointCloud2 message
/// from a color image and a depth image. Color image must be four-channel RGBA
/// with 8 bits/channel. Depth image must be a single-channel uint16 image with
/// depth in mm. See color_depth_images_to_pointcloud_generator.h for more
/// details on expected image behavior and limitations. Has two input ports, one
/// for the color image and one for the depth image, plus one output port for
/// the pointcloud message.
class ColorDepthImagesToRosPointCloudSystem
    : public drake::systems::LeafSystem<double> {
 public:
  /// Construct with the provided values:
  /// @param frame_id Frame name for the pointcloud message.
  /// @param color_camera_info CameraInfo for the color camera.
  /// @param depth_camera_info CameraInfo for the depth camera.
  /// @param generator_options Options to configure the internal pointcloud
  ///     generator. See color_depth_images_to_pointcloud_generator.h for more
  ///     information on possible configuration options.
  ColorDepthImagesToRosPointCloudSystem(
      std::string_view frame_id,
      const drake::systems::sensors::CameraInfo& color_camera_info,
      const drake::systems::sensors::CameraInfo& depth_camera_info,
      const std::map<std::string, int>& generator_options);

  ~ColorDepthImagesToRosPointCloudSystem() override;

  const drake::systems::InputPort<double>& get_color_image_input_port() const {
    return LeafSystem<double>::get_input_port(color_image_input_port_index_);
  }

  const drake::systems::InputPort<double>& get_depth_image_input_port() const {
    return LeafSystem<double>::get_input_port(depth_image_input_port_index_);
  }

  const drake::systems::OutputPort<double>& get_pointcloud_output_port() const {
    return LeafSystem<double>::get_output_port(pointcloud_output_port_index_);
  }

 private:
  using PixelType = drake::systems::sensors::PixelType;
  using ColorImageType = drake::systems::sensors::Image<PixelType::kRgba8U>;
  using DepthImageType = drake::systems::sensors::Image<PixelType::kDepth16U>;

  void CalcPointCloudFromColorDepthImages(
      const ColorImageType& color_image,
      const drake::systems::sensors::CameraInfo& color_camera_info,
      const DepthImageType& depth_image,
      const drake::systems::sensors::CameraInfo& depth_camera_info,
      sensor_msgs::msg::PointCloud2* pointcloud_message) const;

  const std::string frame_id_;
  const drake::systems::sensors::CameraInfo color_camera_info_;
  const drake::systems::sensors::CameraInfo depth_camera_info_;

  drake::systems::InputPortIndex color_image_input_port_index_;
  drake::systems::InputPortIndex depth_image_input_port_index_;
  drake::systems::OutputPortIndex pointcloud_output_port_index_;

  std::unique_ptr<internal::ColorDepthImagesToPointCloudGenerator>
      pointcloud_generator_;
};

}  // namespace sim
}  // namespace anzu

#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <string>

// Use the vendored copy of cl.hpp provided by voxelized_geometry_tools.
#include <voxelized_geometry_tools/cl.hpp>

#include "drake/common/drake_copyable.h"
#include "drake/common/parallelism.h"

namespace anzu {
namespace sim {
namespace internal {

/// Core implementation of a {Color+Depth,Depth} image->PointCloud generator,
/// supporting (preferably) OpenCL GPU acceleration and OpenMP parallel CPU
/// fallback. This implementation is specialized for the case of color and depth
/// images rendered in Drake simulation, and may not work in other cases; namely
/// the following limitations apply:
/// - no distortion is present in either color or depth image(s)
/// - color and depth images are in the same frame, i.e. the color<->depth
///   transform is identity
/// - color and depth images have the same resolution
/// - color and depth images have the same focal length(s) and principal point
/// - color and depth images are stored densely, with no additional padding
/// - all color images are RGBA, 8 bits/channel
/// - all depth images are uint16, with depth in mm
class ColorDepthImagesToPointCloudGenerator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ColorDepthImagesToPointCloudGenerator);

  /// Construct a pointcloud generator with the provided options. Recognized
  /// options include:
  /// - CPU_PARALLELIZE: should CPU parallelization be enabled?
  ///     Values > 0 enable CPU parallelization.
  ///    Default: 1 (enable)
  /// - CPU_NUM_THREADS: how many threads can be used for CPU parallelization?
  ///     Values > 0 specify a fixed number of threads, values <= 0 specify
  ///     the maximum available (i.e. the result of drake::Parallelism::Max()).
  ///     Default: -1 (automatic max # of threads)
  /// - GPU_POINT_BLOCK_SIZE: how many points can be processed per GPU thread?
  ///     Generally it would be inefficient to process each point in its own
  ///     GPU thread, and thus we process a block of points in each GPU thread
  ///     instead. If value is <= 0, disables GPU implementation.
  ///     Default: 48; optimal number likely depends on the specific GPU in use.
  /// - OPENCL_PLATFORM_INDEX: what is the index of the OpenCL platform to use?
  ///     A computer may have multiple OpenCL platforms available, e.g. both a
  ///     Nvidia OpenCL implementation for a PCIe GPU and an Intel OpenCL
  ///     implementation for the integrated GPU.
  ///     Default: 0, i.e. pick the first available platform.
  /// - OPENCL_DEVICE_INDEX: what is the index of the OpenCL device to use?
  ///     For a given OpenCL platform, there may be multiple devices available,
  ///     e.g. if multiple Nvidia GPUs are present.
  ///     Default: 0, i.e. pick first device of first platform available.
  /// For more information on the available OpenCL platform(s) and device(s),
  /// use the clinfo command. Note that depending on installation/configuration,
  /// you may have platforms available with no supported devices present.
  ColorDepthImagesToPointCloudGenerator(
      const std::map<std::string, int>& options);

  /// Generate a XYZ+color pointcloud from the provided images and parameters.
  /// See color_depth_images_to_ros_pointcloud_system.cc for intended usage.
  void CalcPointCloudFromColorDepthImages(
      float ppx, float ppy, float inv_fx, float inv_fy, int width,
      int num_points, int color_pixel_step, int point_step,
      const uint8_t* color_image_buffer, const uint16_t* depth_image_buffer,
      uint8_t* pointcloud_buffer) const;

  /// Generate a XYZ pointcloud from the provided images and parameters.
  /// See depth_image_to_ros_pointcloud_system.cc for intended usage.
  void CalcPointCloudFromDepthImage(
      float ppx, float ppy, float inv_fx, float inv_fy, int width,
      int num_points, int point_step, const uint16_t* depth_image_buffer,
      uint8_t* pointcloud_buffer) const;

 private:
  void TrySetupOpenCL(const std::map<std::string, int>& options);

  void DoCalcPointCloudFromColorDepthImagesGpu(
      float ppx, float ppy, float inv_fx, float inv_fy, int width,
      int num_points, int color_pixel_step, int point_step,
      const uint8_t* color_image_buffer, const uint16_t* depth_image_buffer,
      uint8_t* pointcloud_buffer) const;

  void DoCalcPointCloudFromColorDepthImagesCpu(
      float ppx, float ppy, float inv_fx, float inv_fy, int width,
      int num_points, int color_pixel_step, int point_step,
      const uint8_t* color_image_buffer, const uint16_t* depth_image_buffer,
      uint8_t* pointcloud_buffer) const;

  void DoCalcPointCloudFromDepthImageGpu(
      float ppx, float ppy, float inv_fx, float inv_fy, int width,
      int num_points, int point_step, const uint16_t* depth_image_buffer,
      uint8_t* pointcloud_buffer) const;

  void DoCalcPointCloudFromDepthImageCpu(
      float ppx, float ppy, float inv_fx, float inv_fy, int width,
      int num_points, int point_step, const uint16_t* depth_image_buffer,
      uint8_t* pointcloud_buffer) const;

  int gpu_point_block_size_ = 0;
  drake::Parallelism parallelism_;

  std::unique_ptr<cl::Context> opencl_context_;
  std::unique_ptr<cl::CommandQueue> opencl_command_queue_;
  std::unique_ptr<cl::Program> opencl_calc_pointcloud_program_;
};

}  // namespace internal
}  // namespace sim
}  // namespace anzu

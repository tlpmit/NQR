#include "qr_mapping/color_depth_images_to_pointcloud_generator.h"

#include <array>
#include <vector>

#include <common_robotics_utilities/parallelism.hpp>

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

using common_robotics_utilities::parallelism::DegreeOfParallelism;
using common_robotics_utilities::parallelism::StaticParallelForIndexLoop;
using common_robotics_utilities::parallelism::ParallelForBackend;

namespace anzu {
namespace sim {
namespace internal {

static int32_t RetrieveOptionOrDefault(
    const std::map<std::string, int32_t>& options, const std::string& option,
    const int32_t default_value,
    const std::function<void(const std::string&)>& logging_fn)
{
  auto found_itr = options.find(option);
  if (found_itr != options.end()) {
    const int32_t value = found_itr->second;
    logging_fn(
        "Option [" + option + "] found, value [" + std::to_string(value) + "]");
    return value;
  } else {
    logging_fn(
        "Option [" + option + "] not found, default value ["
        + std::to_string(default_value) + "]");
    return default_value;
  }
}

constexpr int kDefaultGpuPointBlockSize = 48;

namespace {
const char* kCalcPointCloudColorDepthKernelCode = R"(
void kernel CalcPointCloudColorDepth(
    const int block_size, const float ppx, const float ppy, const float inv_fx,
    const float inv_fy, const int width, const int num_points,
    const int color_pixel_step, const int point_step,
    global const uchar* const color_image_buffer,
    global const ushort* const depth_image_buffer,
    global float* const pointcloud_buffer) {
  // Which block are we working on?
  const int block_num = get_global_id(0);

  const int start_index = block_size * block_num;
  const int nominal_end_index = block_size * (block_num + 1);
  const int end_index = min(nominal_end_index, num_points);

  for (int index = start_index; index < end_index; ++index) {
    const int row = (int)(index / width);
    const int column = (int)(index % width);

    // Use a union to reinterpret the color bytes to float.
    union { float f; uchar bgr[4]; } color;

    color.bgr[0] = color_image_buffer[(index * color_pixel_step) + 2];
    color.bgr[1] = color_image_buffer[(index * color_pixel_step) + 1];
    color.bgr[2] = color_image_buffer[(index * color_pixel_step) + 0];
    color.bgr[3] = 0x00;

    const ushort depth_mm = depth_image_buffer[index];

    const float depth_meters = (float)(depth_mm) / 1000.0f;

    // This is simplified for the no-distortion case of Drake sim images.
    const float x = ((float)(column) - ppx) * inv_fx * depth_meters;
    const float y = ((float)(row) - ppy) * inv_fy * depth_meters;
    const float z = depth_meters;

    pointcloud_buffer[(index * point_step) + 0] = x;
    pointcloud_buffer[(index * point_step) + 1] = y;
    pointcloud_buffer[(index * point_step) + 2] = z;
    pointcloud_buffer[(index * point_step) + 3] = color.f;
  }
}
)";

const char* kCalcPointCloudDepthKernelCode = R"(
void kernel CalcPointCloudDepth(
    const int block_size, const float ppx, const float ppy, const float inv_fx,
    const float inv_fy, const int width, const int num_points,
    const int point_step, global const ushort* const depth_image_buffer,
    global float* const pointcloud_buffer) {
  // Which block are we working on?
  const int block_num = get_global_id(0);

  const int start_index = block_size * block_num;
  const int nominal_end_index = block_size * (block_num + 1);
  const int end_index = min(nominal_end_index, num_points);

  for (int index = start_index; index < end_index; ++index) {
    const int row = (int)(index / width);
    const int column = (int)(index % width);

    const ushort depth_mm = depth_image_buffer[index];

    const float depth_meters = (float)(depth_mm) / 1000.0f;

    // This is simplified for the no-distortion case of Drake sim images.
    const float x = ((float)(column) - ppx) * inv_fx * depth_meters;
    const float y = ((float)(row) - ppy) * inv_fy * depth_meters;
    const float z = depth_meters;

    pointcloud_buffer[(index * point_step) + 0] = x;
    pointcloud_buffer[(index * point_step) + 1] = y;
    pointcloud_buffer[(index * point_step) + 2] = z;
  }
}
)";

static std::string GetCalcPointCloudColorDepthKernelSource() {
  return std::string(kCalcPointCloudColorDepthKernelCode);
}

static std::string GetCalcPointCloudDepthKernelSource() {
  return std::string(kCalcPointCloudDepthKernelCode);
}
}  // namespace

ColorDepthImagesToPointCloudGenerator::
ColorDepthImagesToPointCloudGenerator(const std::map<std::string, int>& options)
{
  // Load configuration options for CPU processing.
  const auto logging_function = [](const std::string& message) {
    drake::log()->info(message);
  };
  const int32_t cpu_parallelize = RetrieveOptionOrDefault(
      options, "CPU_PARALLELIZE", 1, logging_function);
  const int32_t cpu_num_threads = RetrieveOptionOrDefault(
      options, "CPU_NUM_THREADS", -1, logging_function);

  if (cpu_parallelize > 0 && cpu_num_threads >= 1) {
    parallelism_ = drake::Parallelism(cpu_num_threads);
    drake::log()->info(
        "ColorDepthImagesToPointCloudGenerator: "
        "parallelism using provided number of threads {}",
        cpu_num_threads);
  } else if (cpu_parallelize > 0) {
    parallelism_ = drake::Parallelism::Max();
    drake::log()->info(
        "ColorDepthImagesToPointCloudGenerator: "
        "parallelism using Parallelism::Max() threads {}",
        parallelism_.num_threads());
  } else {
    parallelism_ = drake::Parallelism::None();
    drake::log()->info(
        "ColorDepthImagesToPointCloudGenerator: CPU parallelism disabled");
  }

  gpu_point_block_size_ = RetrieveOptionOrDefault(
      options, "GPU_POINT_BLOCK_SIZE", kDefaultGpuPointBlockSize,
      logging_function);
  drake::log()->info(
      "ColorDepthImagesToPointCloudGenerator: "
      "GPU point block size {}",
      gpu_point_block_size_);

  // Try to initialize OpenCL for GPU processing.
  if (gpu_point_block_size_ > 0) {
    TrySetupOpenCL(options);
  } else {
    drake::log()->info(
        "ColorDepthImagesToPointCloudGenerator: "
        "GPU implementation disabled");
  }
}

void ColorDepthImagesToPointCloudGenerator::
CalcPointCloudFromColorDepthImages(
    const float ppx, const float ppy, const float inv_fx, const float inv_fy,
    const int width, const int num_points, const int color_pixel_step,
    const int point_step, const uint8_t* const color_image_buffer,
    const uint16_t* const depth_image_buffer,
    uint8_t* const pointcloud_buffer) const {
  DRAKE_THROW_UNLESS(color_image_buffer != nullptr);
  DRAKE_THROW_UNLESS(depth_image_buffer != nullptr);
  DRAKE_THROW_UNLESS(pointcloud_buffer != nullptr);

  if (opencl_calc_pointcloud_program_ != nullptr) {
    drake::log()->debug(
        "ColorDepthImagesToPointCloudGenerator: "
        "Calling DoCalcPointCloudFromColorDepthImagesGpu");
    DoCalcPointCloudFromColorDepthImagesGpu(
        ppx, ppy, inv_fx, inv_fy, width, num_points, color_pixel_step,
        point_step, color_image_buffer, depth_image_buffer, pointcloud_buffer);
  } else {
    drake::log()->debug(
        "ColorDepthImagesToPointCloudGenerator: "
        "Calling DoCalcPointCloudFromColorDepthImagesCpu");
    DoCalcPointCloudFromColorDepthImagesCpu(
        ppx, ppy, inv_fx, inv_fy, width, num_points, color_pixel_step,
        point_step, color_image_buffer, depth_image_buffer, pointcloud_buffer);
  }
}

void ColorDepthImagesToPointCloudGenerator::
CalcPointCloudFromDepthImage(
    const float ppx, const float ppy, const float inv_fx, const float inv_fy,
    const int width, const int num_points, const int point_step,
    const uint16_t* const depth_image_buffer,
    uint8_t* const pointcloud_buffer) const {
  DRAKE_THROW_UNLESS(depth_image_buffer != nullptr);
  DRAKE_THROW_UNLESS(pointcloud_buffer != nullptr);

  if (opencl_calc_pointcloud_program_ != nullptr) {
    drake::log()->debug(
        "ColorDepthImagesToPointCloudGenerator: "
        "Calling DoCalcPointCloudFromDepthImageGpu");
    DoCalcPointCloudFromDepthImageGpu(
        ppx, ppy, inv_fx, inv_fy, width, num_points, point_step,
        depth_image_buffer, pointcloud_buffer);
  } else {
    drake::log()->debug(
        "ColorDepthImagesToPointCloudGenerator: "
        "Calling DoCalcPointCloudFromDepthImageCpu");
    DoCalcPointCloudFromDepthImageCpu(
        ppx, ppy, inv_fx, inv_fy, width, num_points, point_step,
        depth_image_buffer, pointcloud_buffer);
  }
}

void ColorDepthImagesToPointCloudGenerator::TrySetupOpenCL(
    const std::map<std::string, int>& options) {
  // Load options (or defaults).
  const auto logging_function = [](const std::string& msg) {
    drake::log()->info("ColorDepthImagesToPointCloudGenerator: [{}]", msg);
  };
  const int opencl_platform_index = RetrieveOptionOrDefault(
      options, "OPENCL_PLATFORM_INDEX", 0, logging_function);
  const int opencl_device_index = RetrieveOptionOrDefault(
      options, "OPENCL_DEVICE_INDEX", 0, logging_function);

  // Discover OpenCL devices.
  std::vector<cl::Platform> all_opencl_platforms;
  cl::Platform::get(&all_opencl_platforms);

  if (all_opencl_platforms.size() > 0) {
    DRAKE_THROW_UNLESS(
        opencl_platform_index < static_cast<int>(all_opencl_platforms.size()));
    auto& opencl_platform = all_opencl_platforms.at(opencl_platform_index);

    std::string platform_name;
    opencl_platform.getInfo(CL_PLATFORM_NAME, &platform_name);
    std::string platform_vendor;
    opencl_platform.getInfo(CL_PLATFORM_VENDOR, &platform_vendor);

    drake::log()->info(
        "ColorDepthImagesToPointCloudGenerator: "
        "Using OpenCL platform: [{}] vendor: [{}]",
        platform_name, platform_vendor);

    std::vector<cl::Device> platform_opencl_devices;
    opencl_platform.getDevices(CL_DEVICE_TYPE_ALL, &platform_opencl_devices);

    if (platform_opencl_devices.size() > 0) {
      DRAKE_THROW_UNLESS(
          opencl_device_index <
          static_cast<int>(platform_opencl_devices.size()));
      auto& opencl_device = platform_opencl_devices.at(opencl_device_index);

      std::string device_name;
      opencl_device.getInfo(CL_DEVICE_NAME, &device_name);

      drake::log()->info(
          "ColorDepthImagesToPointCloudGenerator: Using OpenCL device [{}]",
          device_name);

      opencl_context_ = std::unique_ptr<cl::Context>(
          new cl::Context({opencl_device}));
      opencl_command_queue_ =
          std::make_unique<cl::CommandQueue>(*opencl_context_, opencl_device);

      const std::string build_options = "-Werror -cl-fast-relaxed-math";

      const std::string calc_pointcloud_color_depth_kernel_source =
          GetCalcPointCloudColorDepthKernelSource();
      const std::string calc_pointcloud_depth_kernel_source =
          GetCalcPointCloudDepthKernelSource();

      cl::Program::Sources calc_pointcloud_sources;
      calc_pointcloud_sources.push_back(
          {calc_pointcloud_color_depth_kernel_source.c_str(),
           calc_pointcloud_color_depth_kernel_source.length()});
      calc_pointcloud_sources.push_back(
          {calc_pointcloud_depth_kernel_source.c_str(),
           calc_pointcloud_depth_kernel_source.length()});

      opencl_calc_pointcloud_program_ = std::make_unique<cl::Program>(
            *opencl_context_, calc_pointcloud_sources);

      if (opencl_calc_pointcloud_program_->build(
              {opencl_device}, build_options.c_str())
          == CL_SUCCESS) {
        drake::log()->info(
            "ColorDepthImagesToPointCloudGenerator: OpenCL program built");
      } else {
        drake::log()->error(
            "ColorDepthImagesToPointCloudGenerator: "
            "Error building calc pointcloud kernel: [{}]",
            opencl_calc_pointcloud_program_->getBuildInfo<CL_PROGRAM_BUILD_LOG>(
                opencl_device));

        opencl_calc_pointcloud_program_.reset();
      }
    } else {
      drake::log()->warn(
          "ColorDepthImagesToPointCloudGenerator: "
          "No OpenCL device(s) available for platform");
    }
  } else {
    drake::log()->warn(
        "ColorDepthImagesToPointCloudGenerator: "
        "No OpenCL platform(s) available");
  }
}

void ColorDepthImagesToPointCloudGenerator::
DoCalcPointCloudFromColorDepthImagesGpu(
    const float ppx, const float ppy, const float inv_fx, const float inv_fy,
    const int width, const int num_points, const int color_pixel_step,
    const int point_step, const uint8_t* const color_image_buffer,
    const uint16_t* const depth_image_buffer,
    uint8_t* const pointcloud_buffer) const {
  // Copy data to the accelerator device.
  cl_int err = 0;

  cl::Buffer device_color_image_buffer(
      *opencl_context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
      static_cast<size_t>(num_points * color_pixel_step),
      const_cast<uint8_t*>(color_image_buffer), &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failed to allocate and copy color image buffer");
  }

  cl::Buffer device_depth_image_buffer(
      *opencl_context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
      num_points * sizeof(uint16_t), const_cast<uint16_t*>(depth_image_buffer),
      &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failed to allocate and copy depth image buffer");
  }

  cl::Buffer device_pointcloud_buffer(
      *opencl_context_, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
      static_cast<size_t>(num_points * point_step), pointcloud_buffer, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failed to allocate and copy pointcloud buffer");
  }

  // Calc block sizes.
  const int block_size = gpu_point_block_size_;
  const int num_blocks = (num_points + (block_size - 1)) / block_size;

  // Note that the kernel treats the pointcloud as a float buffer, not in bytes.
  constexpr int kernel_point_step = 4;

  // Build kernel.
  cl::Kernel calc_pointcloud_kernel(
      *opencl_calc_pointcloud_program_, "CalcPointCloudColorDepth");
  calc_pointcloud_kernel.setArg(0, block_size);
  calc_pointcloud_kernel.setArg(1, ppx);
  calc_pointcloud_kernel.setArg(2, ppy);
  calc_pointcloud_kernel.setArg(3, inv_fx);
  calc_pointcloud_kernel.setArg(4, inv_fy);
  calc_pointcloud_kernel.setArg(5, width);
  calc_pointcloud_kernel.setArg(6, num_points);
  calc_pointcloud_kernel.setArg(7, color_pixel_step);
  calc_pointcloud_kernel.setArg(8, kernel_point_step);
  calc_pointcloud_kernel.setArg(9, device_color_image_buffer);
  calc_pointcloud_kernel.setArg(10, device_depth_image_buffer);
  calc_pointcloud_kernel.setArg(11, device_pointcloud_buffer);

  // Dispatch kernel.
  err = opencl_command_queue_->enqueueNDRangeKernel(
      calc_pointcloud_kernel, cl::NullRange, cl::NDRange(num_blocks),
      cl::NullRange);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failed to enqueue CalcPointCloud kernel");
  }

  opencl_command_queue_->finish();
  err = opencl_command_queue_->enqueueReadBuffer(
      device_pointcloud_buffer, CL_TRUE, 0, num_points * point_step,
      pointcloud_buffer);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("PointCloud buffer enqueueReadBuffer failed");
  }
}

void ColorDepthImagesToPointCloudGenerator::
DoCalcPointCloudFromColorDepthImagesCpu(
    const float ppx, const float ppy, const float inv_fx, const float inv_fy,
    const int width, const int num_points, const int color_pixel_step,
    const int point_step, const uint8_t* const color_image_buffer,
    const uint16_t* const depth_image_buffer,
    uint8_t* const pointcloud_buffer) const {
  // Helper for per-point work.
  const auto per_point_work =
      [ppx, ppy, inv_fx, inv_fy, width, color_pixel_step, point_step,
       color_image_buffer, depth_image_buffer, pointcloud_buffer](
          const int, const int64_t index) {
    const int row = static_cast<int>(index / width);
    const int column = static_cast<int>(index % width);

    const uint8_t* color_pixel =
        color_image_buffer + (index * color_pixel_step);

    const uint16_t depth_mm = depth_image_buffer[index];

    const float depth_meters = static_cast<float>(depth_mm) / 1000.0f;

    // This is simplified for the no-distortion case of Drake sim images.
    const float x = (static_cast<float>(column) - ppx) * inv_fx * depth_meters;
    const float y = (static_cast<float>(row) - ppy) * inv_fy * depth_meters;
    const float z = depth_meters;

    uint8_t* point = pointcloud_buffer + (index * point_step);

    std::memcpy(point + (sizeof(float) * 0), &x, sizeof(float));
    std::memcpy(point + (sizeof(float) * 1), &y, sizeof(float));
    std::memcpy(point + (sizeof(float) * 2), &z, sizeof(float));

    const std::array<uint8_t, 4> color =
        {color_pixel[0], color_pixel[1], color_pixel[2], 0x00};
    std::memcpy(point + (sizeof(float) * 3), color.data(), color.size());
  };

  StaticParallelForIndexLoop(
      DegreeOfParallelism(parallelism_.num_threads()), 0, num_points,
      per_point_work, ParallelForBackend::BEST_AVAILABLE);
}

void ColorDepthImagesToPointCloudGenerator::
DoCalcPointCloudFromDepthImageGpu(
    const float ppx, const float ppy, const float inv_fx, const float inv_fy,
    const int width, const int num_points, const int point_step,
    const uint16_t* const depth_image_buffer,
    uint8_t* const pointcloud_buffer) const {
  // Copy data to the accelerator device.
  cl_int err = 0;

  cl::Buffer device_depth_image_buffer(
      *opencl_context_, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
      num_points * sizeof(uint16_t), const_cast<uint16_t*>(depth_image_buffer),
      &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failed to allocate and copy depth image buffer");
  }

  cl::Buffer device_pointcloud_buffer(
      *opencl_context_, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
      static_cast<size_t>(num_points * point_step), pointcloud_buffer, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failed to allocate and copy pointcloud buffer");
  }

  // Calc block sizes.
  const int block_size = gpu_point_block_size_;
  const int num_blocks = (num_points + (block_size - 1)) / block_size;

  // Note that the kernel treats the pointcloud as a float buffer, not in bytes.
  constexpr int kernel_point_step = 3;

  // Build kernel.
  cl::Kernel calc_pointcloud_kernel(
      *opencl_calc_pointcloud_program_, "CalcPointCloudDepth");
  calc_pointcloud_kernel.setArg(0, block_size);
  calc_pointcloud_kernel.setArg(1, ppx);
  calc_pointcloud_kernel.setArg(2, ppy);
  calc_pointcloud_kernel.setArg(3, inv_fx);
  calc_pointcloud_kernel.setArg(4, inv_fy);
  calc_pointcloud_kernel.setArg(5, width);
  calc_pointcloud_kernel.setArg(6, num_points);
  calc_pointcloud_kernel.setArg(7, kernel_point_step);
  calc_pointcloud_kernel.setArg(8, device_depth_image_buffer);
  calc_pointcloud_kernel.setArg(9, device_pointcloud_buffer);

  // Dispatch kernel.
  err = opencl_command_queue_->enqueueNDRangeKernel(
      calc_pointcloud_kernel, cl::NullRange, cl::NDRange(num_blocks),
      cl::NullRange);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failed to enqueue CalcPointCloud kernel");
  }

  opencl_command_queue_->finish();
  err = opencl_command_queue_->enqueueReadBuffer(
      device_pointcloud_buffer, CL_TRUE, 0, num_points * point_step,
      pointcloud_buffer);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("PointCloud buffer enqueueReadBuffer failed");
  }
}

void ColorDepthImagesToPointCloudGenerator::
DoCalcPointCloudFromDepthImageCpu(
    const float ppx, const float ppy, const float inv_fx, const float inv_fy,
    const int width, const int num_points, const int point_step,
    const uint16_t* const depth_image_buffer,
    uint8_t* const pointcloud_buffer) const {
  // Helper for per-point work.
  const auto per_point_work =
      [ppx, ppy, inv_fx, inv_fy, width, point_step, depth_image_buffer,
       pointcloud_buffer](
          const int, const int64_t index) {
    const int row = static_cast<int>(index / width);
    const int column = static_cast<int>(index % width);

    const uint16_t depth_mm = depth_image_buffer[index];

    const float depth_meters = static_cast<float>(depth_mm) / 1000.0f;

    // This is simplified for the no-distortion case of Drake sim images.
    const float x = (static_cast<float>(column) - ppx) * inv_fx * depth_meters;
    const float y = (static_cast<float>(row) - ppy) * inv_fy * depth_meters;
    const float z = depth_meters;

    uint8_t* point = pointcloud_buffer + (index * point_step);

    std::memcpy(point + (sizeof(float) * 0), &x, sizeof(float));
    std::memcpy(point + (sizeof(float) * 1), &y, sizeof(float));
    std::memcpy(point + (sizeof(float) * 2), &z, sizeof(float));
  };

  StaticParallelForIndexLoop(
      DegreeOfParallelism(parallelism_.num_threads()), 0, num_points,
      per_point_work, ParallelForBackend::BEST_AVAILABLE);
}

}  // namespace internal
}  // namespace sim
}  // namespace anzu

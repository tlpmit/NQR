#include <qr_mapping/voxel_grid.hpp>
#include <qr_mapping/drake_pointcloud_voxelization_interface.hpp>

#include <cstring>
#include <utility>
#include <vector>
#include <ranges>

#include <Eigen/Geometry>

#include <drake/geometry/rgba.h>
#include <drake/perception/point_cloud.h>
#include <drake/perception/point_cloud_flags.h>
#include <drake/common/text_logging.h>

#include <common_robotics_utilities/conversions.hpp>
#include <common_robotics_utilities/color_builder.hpp>
#include <voxelized_geometry_tools/collision_map.hpp>
#include <voxelized_geometry_tools/pointcloud_voxelization.hpp>


using drake::geometry::Rgba;

using common_robotics_utilities::conversions::TransformFromUrdfXYZRPY;
using common_robotics_utilities::color_builder::MakeFromFloatColors;

using voxelized_geometry_tools::CollisionMap;
using voxelized_geometry_tools::CollisionCell;
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudVoxelizationFilterOptions;
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudVoxelizationInterface;
using voxelized_geometry_tools::pointcloud_voxelization::PointCloudWrapperSharedPtr;
using voxelized_geometry_tools::pointcloud_voxelization::BackendOptions;
using voxelized_geometry_tools::pointcloud_voxelization::VoxelizerRuntime;
using voxelized_geometry_tools::pointcloud_voxelization::OwningDrakePointCloudWrapper;

using drake::math::RigidTransformd;
using drake::perception::PointCloud;
using drake::perception::pc_flags::Fields;
using drake::perception::pc_flags::BaseField;
using drake::systems::sensors::ImageDepth32F;
using drake::systems::sensors::ImageRgb8U;
using drake::geometry::Box;

namespace qr_mapping {

VoxelGridEstimator::VoxelGridEstimator(VoxelGridConfig config, std::shared_ptr<Meshcat> meshcat)
  : m_config(std::move(config)), m_meshcat(std::move(meshcat))
{
  const common_robotics_utilities::voxel_grid::GridSizes grid_sizes(
    m_config.grid_resolution, m_config.x_size, m_config.y_size, m_config.z_size
  );
  const Eigen::Isometry3d grid_origin_transform = TransformFromUrdfXYZRPY(
    m_config.x_origin, m_config.y_origin, m_config.z_origin,
    m_config.roll_origin, m_config.pitch_origin, m_config.yaw_origin
  );

  m_empty_grid = CollisionMap(
    grid_origin_transform, "grid_frame", grid_sizes, CollisionCell(0.5f) );
  m_voxel_grid = CollisionMap(
    grid_origin_transform, "grid_frame", grid_sizes, CollisionCell(0.5f) );
  m_voxel_grid.SetFrame("world");

  m_filter_options = PointCloudVoxelizationFilterOptions(
    m_config.percent_seen_free,
    static_cast<int32_t>(m_config.outlier_points_threshold),
    m_config.num_cameras_seen_free
  );

  BackendOptions voxelizer_option = BackendOptions::BEST_AVAILABLE;
  if (m_config.voxelizer_option == "best")
    voxelizer_option = BackendOptions::BEST_AVAILABLE;
  else if (m_config.voxelizer_option == "cpu")
    voxelizer_option = BackendOptions::CPU;
  else if (m_config.voxelizer_option == "opencl")
    voxelizer_option = BackendOptions::OPENCL;
  else if (m_config.voxelizer_option == "cuda")
    voxelizer_option = BackendOptions::CUDA;
  else
    drake::log()->error("[%s] is not a valid voxelizer option", m_config.voxelizer_option.c_str());

  std::map<std::string, int32_t> options;
  options["CUDA_DEVICE"] = m_config.cuda_device;
  options["OPENCL_PLATFORM_INDEX"] = m_config.opencl_platform_index;
  options["OPENCL_DEVICE_INDEX"] = m_config.opencl_device_index;
  options["CPU_PARALLELIZE"] = 1;
  options["CPU_NUM_THREADS"] = m_config.max_threads;
  options["DISPATCH_PARALLELIZE"] = 1;
  options["DISPATCH_NUM_THREADS"] = m_config.max_threads;

  auto logging_fn = [&](const std::string& msg) {
    drake::log()->info(msg.c_str());
  };
  m_voxelizer = MakePointCloudVoxelizer(voxelizer_option, options, logging_fn);
  drake::log()->info("Voxelizer type: {}", typeid(*m_voxelizer).name());

  // Publish display
  PublishVoxelGridAsTwoPointCloudsToMeshcat();
}


void VoxelGridEstimator::UpdateCumulativeOccupancy (const CollisionMap& latest_occupancy)
{
  for (int64_t index = 0; index < latest_occupancy.GetTotalCells(); ++index) {
    const float& observed_occupancy = latest_occupancy.GetDataIndexImmutable(index).Occupancy();
    float& current_occupancy = m_voxel_grid.GetDataIndexMutable(index).Occupancy();
    if (observed_occupancy != 0.5f) {
      current_occupancy = observed_occupancy;
    }
  }
}

Eigen::Tensor<uint8_t, 3> VoxelGridEstimator::GetOccupancyAsEigen() const {
  Eigen::Tensor<uint8_t, 3> result(
    m_voxel_grid.GetNumXCells(),
    m_voxel_grid.GetNumYCells(),
    m_voxel_grid.GetNumZCells()
  );

  for (int64_t x = 0; x < m_voxel_grid.GetNumXCells(); ++x) {
    for (int64_t y = 0; y < m_voxel_grid.GetNumYCells(); ++y) {
      for (int64_t z = 0; z < m_voxel_grid.GetNumZCells(); ++z) {
        const CollisionCell cell = m_voxel_grid.GetIndexImmutable(x, y, z).Value();
        const float occupancy = cell.Occupancy();

        if (occupancy < 0.5f) {
          result(x, y, z) = 0;  // Free
        } else if (occupancy > 0.5f) {
          result(x, y, z) = 1;  // Occupied
        } else {
          result(x, y, z) = 255;  // Unknown
        }
      }
    }
  }

  return result;
}

Eigen::Tensor<double, 4> VoxelGridEstimator::GetVoxelCentersAsEigen() const {
  Eigen::Tensor<double, 4> result(
    m_voxel_grid.GetNumXCells(),
    m_voxel_grid.GetNumYCells(),
    m_voxel_grid.GetNumZCells(),
    3
  );

  // Compute X_WG.
  const Eigen::Isometry3d pose = m_voxel_grid.GetOriginTransform();
  const RigidTransformd X_WG (pose);

  for (int64_t x = 0; x < m_voxel_grid.GetNumXCells(); ++x) {
    for (int64_t y = 0; y < m_voxel_grid.GetNumYCells(); ++y) {
      for (int64_t z = 0; z < m_voxel_grid.GetNumZCells(); ++z) {
        // Get location in the grid's frame.
        const Eigen::Vector3d p_G
            = m_voxel_grid.GridIndexToLocationInGridFrame(x, y, z).head<3>();

        // Transform to location in the world frame.
        const Eigen::Vector3d p_W = X_WG * p_G;

        // Copy into result tensor.
        result(x, y, z, 0) = p_W.x();
        result(x, y, z, 1) = p_W.y();
        result(x, y, z, 2) = p_W.z();
      }
    }
  }

  return result;
}



static PointCloud TransformPointCloudToWorldFrame(const PointCloud& src, const Eigen::Matrix4d& extrinsics)
{
  // Create a new point cloud with the same fields and size as the source
  PointCloud cloud (src.size(), src.fields());

  // Create transform object from extrinsics matrix
  const RigidTransformd X_DW (extrinsics);
  const RigidTransformd X_WD = X_DW.inverse();

  // Transform XYZ coordinates
  Eigen::Ref<drake::Matrix3X<float>> dst_xyz = cloud.mutable_xyzs();
  for (int i = 0; i < src.size(); ++i) {
    const Eigen::Vector3d p_D = src.xyz(i).cast<double>();
    const Eigen::Vector3d p_W = X_WD * p_D;
    dst_xyz.col(i) = p_W.cast<float>();
  }

  // Copy RGB values if they exist
  if (src.has_rgbs()) {
    cloud.mutable_rgbs() = src.rgbs();
  }

  return cloud ;
}

static PointCloud SubsamplePointCloud(const PointCloud& src, int max_num_points)
{
    if (max_num_points >= src.size()) return src;

    // Create output vector to store sampled indices
    std::vector<int> sampled_indices (max_num_points);

    // Create input range of indices
    auto indices = std::views::iota(0, src.size());
    
    // Sample max_num_points indices without replacement
    std::random_device rd;
    std::mt19937 gen(rd());
    std::ranges::sample(indices, sampled_indices.begin(), max_num_points, gen);

    int num_points = static_cast<int>(sampled_indices.size());

    // Create new point cloud with subsampled points
    const Fields fields = src.fields();
    PointCloud subsampled_cloud(num_points, fields);

    // Copy selected points
    for (int i = 0; i < max_num_points; i++) {
        const int orig_idx = sampled_indices[i];
        subsampled_cloud.mutable_xyz(i) = src.xyz(orig_idx);
        if (src.has_rgbs()) {
            subsampled_cloud.mutable_rgb(i) = src.rgb(orig_idx);
        }
    }

    return subsampled_cloud;
}


void VoxelGridEstimator::UpdateFromDepthImage(
  const ImageDepth32F& depth_image,
  const Eigen::Matrix3d& intrinsics,
  const Eigen::Matrix4d& extrinsics,
  const ImageRgb8U *color_image,
  int max_depth_points_to_publish,
  double viz_point_size)
{
  const PointCloud drake_cloud = ConvertDepthImageToPointCloud(depth_image, intrinsics, color_image);

  // Publish point cloud if required
  if (max_depth_points_to_publish != 0) {
    PointCloud viz_cloud = TransformPointCloudToWorldFrame(drake_cloud, extrinsics);

    int num_points = viz_cloud.size();
    if (0 < max_depth_points_to_publish && max_depth_points_to_publish < num_points)
      viz_cloud = SubsamplePointCloud(viz_cloud, max_depth_points_to_publish);

    // Publish to Meshcat
    m_meshcat->SetObject("/voxel_grid/pointcloud", viz_cloud, viz_point_size);
  }

  const Eigen::Isometry3d X_DW (extrinsics);
  const Eigen::Isometry3d X_WD = X_DW.inverse();

  PointCloudWrapperSharedPtr cloud (
    new OwningDrakePointCloudWrapper(
      std::make_shared<PointCloud>(drake_cloud), X_WD, m_config.max_depth
  ));

  const auto log_runtime = [&] (const VoxelizerRuntime& voxelizer_runtime)
  {
    const double raycasting_time = voxelizer_runtime.RaycastingTime();
    const double filtering_time = voxelizer_runtime.FilteringTime();
    drake::log()->info(
      "Raycasting time {:.2f}ms, filtering time {:.2f}ms, total time {:.2f}ms",
      raycasting_time * 1000,
      filtering_time * 1000,
      (raycasting_time + filtering_time) * 1000
    );
  };
  const CollisionMap latest_occupancy = m_voxelizer->VoxelizePointClouds(
    m_empty_grid, m_step_size_multiplier, m_filter_options, {cloud}, log_runtime
  );

  // Update cumulative occupancy for latest voxelized pointcloud.
  UpdateCumulativeOccupancy(latest_occupancy);

  // Publish display
  PublishVoxelGridAsTwoPointCloudsToMeshcat();
}

void VoxelGridEstimator::PublishVoxelGridAsBoxesToMeshcat() const
{
  // const Eigen::Isometry3d pose = m_voxel_grid.GetOriginTransform();
  const Eigen::Vector3d scale = m_voxel_grid.GetCellSizes();
  for (int64_t x_index = 0; x_index < m_voxel_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < m_voxel_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < m_voxel_grid.GetNumZCells(); z_index++)
      {
        const CollisionCell cell = m_voxel_grid.GetIndexImmutable(x_index, y_index, z_index).Value();

        Rgba color = (cell.Occupancy() > 0.5) ? m_config.filled_color :  // filled
                     (cell.Occupancy() == 0.5) ? m_config.unknown_color :  // unknown
                     Rgba(0, 0, 0, 0);  // free

        // Get location in the grid's frame.
        const Eigen::Vector4d location
            = m_voxel_grid.GridIndexToLocationInGridFrame(x_index, y_index, z_index);
        const RigidTransformd X_WG (location.head<3>());

        const Box box (scale);

        std::string path = fmt::format("/voxel_grid/box_x{}_y{}_z{}", x_index, y_index, z_index);
        m_meshcat->SetObject(path, box, color);
        m_meshcat->SetTransform(path, X_WG);
      }
    }
  }
}

void VoxelGridEstimator::PublishVoxelGridAsPointCloudToMeshcat() const
{
  // First count the number of points to be added
  int num_points = 0;
  for (int64_t x_index = 0; x_index < m_voxel_grid.GetNumXCells(); x_index++) {
    for (int64_t y_index = 0; y_index < m_voxel_grid.GetNumYCells(); y_index++) {
      for (int64_t z_index = 0; z_index < m_voxel_grid.GetNumZCells(); z_index++) {
        const CollisionCell cell = m_voxel_grid.GetIndexImmutable(x_index, y_index, z_index).Value();
        if (cell.Occupancy() >= 0.5) num_points++;
      }
    }
  }

  // Create pointcloud.
  const Fields fields = BaseField::kXYZs | BaseField::kRGBs;
  PointCloud viz_cloud (num_points, fields);

  int col = 0;
  for (int64_t x_index = 0; x_index < m_voxel_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < m_voxel_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < m_voxel_grid.GetNumZCells(); z_index++)
      {
        const CollisionCell cell = m_voxel_grid.GetIndexImmutable(x_index, y_index, z_index).Value();

        // Don't add free points.
        if (cell.Occupancy() < 0.5) continue;

        Rgba color = cell.Occupancy() > 0.5 ? m_config.filled_color : m_config.unknown_color;

        const uint8_t r = static_cast<uint8_t>(255.0 * color.r());
        const uint8_t g = static_cast<uint8_t>(255.0 * color.g());
        const uint8_t b = static_cast<uint8_t>(255.0 * color.b());

        // Get location in the grid's frame.
        const Eigen::Vector4d location
            = m_voxel_grid.GridIndexToLocationInGridFrame(x_index, y_index, z_index);

        viz_cloud.mutable_xyz(col) = location.cast<float>().head<3>();
        viz_cloud.mutable_rgb(col) << r, g, b;
        
        col++; // Increment the counter
      }
    }
  }

  // Compute point size.
  const Eigen::Vector3d scale = m_voxel_grid.GetCellSizes();
  const double point_size = std::min({scale.x(), scale.y(), scale.z()});

  std::string path = "/voxel_grid/grid";
  m_meshcat->SetObject(path, viz_cloud, point_size);

  // Compute transform
  const Eigen::Isometry3d pose = m_voxel_grid.GetOriginTransform();
  const RigidTransformd X_WG (pose);
  m_meshcat->SetTransform(path, X_WG);
}

void VoxelGridEstimator::PublishVoxelGridAsTwoPointCloudsToMeshcat() const
{
  // First count the number of points to be added
  int num_unknown_points = 0, num_occupied_points = 0;
  for (int64_t x_index = 0; x_index < m_voxel_grid.GetNumXCells(); x_index++) {
    for (int64_t y_index = 0; y_index < m_voxel_grid.GetNumYCells(); y_index++) {
      for (int64_t z_index = 0; z_index < m_voxel_grid.GetNumZCells(); z_index++) {
        const CollisionCell cell = m_voxel_grid.GetIndexImmutable(x_index, y_index, z_index).Value();
        if (cell.Occupancy() == 0.5) num_unknown_points++;
        if (cell.Occupancy() >  0.5) num_occupied_points++;
      }
    }
  }

  // Create pointclouds.
  const Fields fields = BaseField::kXYZs;
  PointCloud unknown_cloud (num_unknown_points, fields);
  PointCloud occupied_cloud (num_occupied_points, fields);

  int unknown_col = 0, occupied_col = 0;
  for (int64_t x_index = 0; x_index < m_voxel_grid.GetNumXCells(); x_index++)
  {
    for (int64_t y_index = 0; y_index < m_voxel_grid.GetNumYCells(); y_index++)
    {
      for (int64_t z_index = 0; z_index < m_voxel_grid.GetNumZCells(); z_index++)
      {
        const CollisionCell cell = m_voxel_grid.GetIndexImmutable(x_index, y_index, z_index).Value();

        // Don't add free points.
        if (cell.Occupancy() < 0.5) continue;

        // Get location in the grid's frame.
        const Eigen::Vector4d location
            = m_voxel_grid.GridIndexToLocationInGridFrame(x_index, y_index, z_index);
        const Eigen::Vector3f xyz = location.cast<float>().head<3>();

        if (cell.Occupancy() == 0.5) {
          unknown_cloud.mutable_xyz(unknown_col++) = xyz;
        }
        else if (cell.Occupancy() > 0.5) {
          occupied_cloud.mutable_xyz(occupied_col++) = xyz;
        }
      }
    }
  }

  // Compute point size.
  const Eigen::Vector3d scale = m_voxel_grid.GetCellSizes();
  const double point_size = std::min({scale.x(), scale.y(), scale.z()});

  // Compute transform
  const Eigen::Isometry3d pose = m_voxel_grid.GetOriginTransform();
  const RigidTransformd X_WG (pose);

  // Unknown cloud
  std::string path = "/voxel_grid/unknown_grid";
  m_meshcat->SetObject(path, unknown_cloud, point_size, m_config.unknown_color);
  m_meshcat->SetTransform(path, X_WG);

  // Occupied cloud
  path = "/voxel_grid/occupied_grid";
  m_meshcat->SetObject(path, occupied_cloud, point_size, m_config.filled_color);
  m_meshcat->SetTransform(path, X_WG);
}

PointCloud VoxelGridEstimator::ConvertDepthImageToPointCloud(
    const ImageDepth32F& depth_image,
    const Eigen::Matrix3d& intrinsics,
    const ImageRgb8U* color_image) const
{
  // Create point cloud with XYZ and optionally RGB fields
  const int height = depth_image.height();
  const int width = depth_image.width();
  const int num_points = height * width;
  const Fields fields  = color_image ? BaseField::kXYZs | BaseField::kRGBs : BaseField::kXYZs;
  PointCloud output(num_points, fields);

  Eigen::Ref<drake::Matrix3X<float>> output_xyz = output.mutable_xyzs();
  std::optional<Eigen::Ref<Eigen::Matrix3X<uint8_t>>> output_rgb;
  if (color_image) {
    output_rgb = output.mutable_rgbs();
  }

  // Get camera intrinsics from the passed 3x3 matrix
  const double fx = intrinsics(0, 0);
  const double fy = intrinsics(1, 1);
  const double cx = intrinsics(0, 2);
  const double cy = intrinsics(1, 2);
  const double fx_inv = 1.0 / fx;
  const double fy_inv = 1.0 / fy;

  for (int v = 0; v < height; ++v) {
    for (int u = 0; u < width; ++u) {
      const int col = v * width + u;
      const float z = depth_image.at(u, v)[0];

      if (!std::isfinite(z) || z <= 0 ||
          z < m_config.min_depth || z > m_config.max_depth) {
        output_xyz.col(col).array() = std::numeric_limits<float>::infinity();
      }
      else
      {
        // Convert from image coordinates to camera coordinates
        // N.B. This clause handles both true depths *and* NaNs.
        Eigen::Vector3d xyz(
            z * (static_cast<float>(u) - cx) * fx_inv,
            z * (static_cast<float>(v) - cy) * fy_inv,
            z);

        output_xyz.col(col) = xyz.cast<float>();
      }

      // Add color information if available
      if (color_image) {
        const uint8_t* color = color_image->at(u, v);
        output_rgb->col(col) = Eigen::Vector3<uint8_t>(color[0], color[1], color[2]);
      }
    }
  }

  return output;
}

}  // namespace qr_mapping
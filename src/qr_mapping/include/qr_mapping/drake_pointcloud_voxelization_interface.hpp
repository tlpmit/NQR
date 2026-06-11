#pragma once

#include <cstring>
#include <limits>
#include <memory>

#include <drake/perception/point_cloud.h>

#include <Eigen/Geometry>
#include <voxelized_geometry_tools/pointcloud_voxelization_interface.hpp>

namespace voxelized_geometry_tools
{
VGT_NAMESPACE_BEGIN
namespace pointcloud_voxelization
{

using drake::perception::PointCloud;

class DrakePointCloudWrapper : public PointCloudWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double MaxRange() const override { return max_range_; }

  int64_t Size() const override
  {
    return static_cast<int64_t>(cloud_ptr_->size());
  }

  const Eigen::Isometry3d& GetPointCloudOriginTransform() const override
  {
    return origin_transform_;
  }

  void SetPointCloudOriginTransform(
      const Eigen::Isometry3d& origin_transform) override
  {
    origin_transform_ = origin_transform;
  }

  const PointCloud& Cloud() const { return *cloud_ptr_; }

protected:
  DrakePointCloudWrapper(
      const PointCloud* const cloud_ptr,
      const Eigen::Isometry3d& origin_transform, const double max_range);

private:
  void CopyPointLocationIntoDoublePtrImpl(
      const int64_t point_index, double* destination) const override
  {
    const Eigen::Vector4d point =
        GetPointLocationVector4f(point_index).cast<double>();
    std::memcpy(destination, point.data(), sizeof(double) * 3);
  }

  void CopyPointLocationIntoFloatPtrImpl(
      const int64_t point_index, float* destination) const override
  {
    // const size_t starting_offset = GetStartingOffsetForPointXYZ(point_index);
    Eigen::Vector3f xyz = cloud_ptr_->xyz(static_cast<int>(point_index));
    std::memcpy(destination, xyz.data(),
                sizeof(float) * 3);
  }

  // size_t GetStartingOffsetForPointXYZ(const int64_t point_index) const
  // {
  //   const size_t starting_offset =
  //       (static_cast<size_t>(point_index)
  //        * static_cast<size_t>(cloud_ptr_->point_step))
  //       + xyz_offset_from_point_start_;
  //   return starting_offset;
  // }

  const PointCloud* const cloud_ptr_ = nullptr;
  // size_t xyz_offset_from_point_start_ = 0;
  Eigen::Isometry3d origin_transform_ = Eigen::Isometry3d::Identity();
  double max_range_ = std::numeric_limits<double>::infinity();
};

class NonOwningDrakePointCloudWrapper : public DrakePointCloudWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NonOwningDrakePointCloudWrapper(
      const PointCloud* const cloud_ptr,
      const Eigen::Isometry3d& origin_transform,
      const double max_range = std::numeric_limits<double>::infinity())
      : DrakePointCloudWrapper(cloud_ptr, origin_transform, max_range) {}
};

class OwningDrakePointCloudWrapper : public DrakePointCloudWrapper
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OwningDrakePointCloudWrapper(
      const std::shared_ptr<PointCloud>& cloud_ptr,
      const Eigen::Isometry3d& origin_transform,
      const double max_range = std::numeric_limits<double>::infinity())
      : DrakePointCloudWrapper(cloud_ptr.get(), origin_transform, max_range),
        owned_cloud_ptr_(cloud_ptr) {}

private:
  std::shared_ptr<PointCloud> owned_cloud_ptr_;
};
}  // namespace pointcloud_voxelization
VGT_NAMESPACE_END
}  // namespace voxelized_geometry_tools

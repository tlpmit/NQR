// SPDX-License-Identifier: MIT-0

/**
 * @file
 * Provides pybind11 bindings for qr_mapping voxel-grid utilities.
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>  // for Eigen <-> Numpy conversions
#include <pybind11/eigen/tensor.h>  // for Eigen::Tensor <-> Numpy conversions

#include <qr_mapping/voxel_grid.hpp>

namespace py = pybind11;

namespace qr_mapping {

PYBIND11_MODULE(cpp, m) {
  m.doc() = "qr_mapping C++ bindings";

  {
    py::module::import("pydrake.geometry");  // for Rgba

    py::class_<VoxelGridConfig>(m, "VoxelGridConfig")
        .def(py::init<>())
        .def_readwrite("grid_resolution", &VoxelGridConfig::grid_resolution)
        .def_readwrite("x_size", &VoxelGridConfig::x_size)
        .def_readwrite("y_size", &VoxelGridConfig::y_size)
        .def_readwrite("z_size", &VoxelGridConfig::z_size)
        .def_readwrite("x_origin", &VoxelGridConfig::x_origin)
        .def_readwrite("y_origin", &VoxelGridConfig::y_origin)
        .def_readwrite("z_origin", &VoxelGridConfig::z_origin)
        .def_readwrite("roll_origin", &VoxelGridConfig::roll_origin)
        .def_readwrite("pitch_origin", &VoxelGridConfig::pitch_origin)
        .def_readwrite("yaw_origin", &VoxelGridConfig::yaw_origin)
        .def_readwrite("percent_seen_free", &VoxelGridConfig::percent_seen_free)
        .def_readwrite("outlier_points_threshold", &VoxelGridConfig::outlier_points_threshold)
        .def_readwrite("num_cameras_seen_free", &VoxelGridConfig::num_cameras_seen_free)
        .def_readwrite("step_size_multiplier", &VoxelGridConfig::step_size_multiplier)
        .def_readwrite("voxelizer_option", &VoxelGridConfig::voxelizer_option)
        .def_readwrite("cuda_device", &VoxelGridConfig::cuda_device)
        .def_readwrite("opencl_platform_index", &VoxelGridConfig::opencl_platform_index)
        .def_readwrite("opencl_device_index", &VoxelGridConfig::opencl_device_index)
        .def_readwrite("max_threads", &VoxelGridConfig::max_threads)
        .def_readwrite("filled_color", &VoxelGridConfig::filled_color)
        .def_readwrite("unknown_color", &VoxelGridConfig::unknown_color)
        .def_readwrite("min_depth", &VoxelGridConfig::min_depth)
        .def_readwrite("max_depth", &VoxelGridConfig::max_depth)
        ;

    py::class_<VoxelGridEstimator, std::shared_ptr<VoxelGridEstimator>>(m, "VoxelGridEstimator")
        .def(py::init<VoxelGridConfig, std::shared_ptr<Meshcat>>(),
                py::arg("config"),
                py::arg("meshcat"))
        .def("UpdateFromDepthImage", &VoxelGridEstimator::UpdateFromDepthImage,
                py::arg("depth_image"),
                py::arg("intrinsics"),
                py::arg("extrinsics"),
                py::arg("color_image")=nullptr,
                py::arg("max_depth_points_to_publish")=-1,
                py::arg("viz_point_size")=0.001)
        .def("GetOccupancy", &VoxelGridEstimator::GetOccupancyAsEigen)
        .def("GetVoxelCenters", &VoxelGridEstimator::GetVoxelCentersAsEigen)
        ;
  }
}

}  // namespace qr_mapping

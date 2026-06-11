"""Smoke-test the installed qr_mapping C++ extension."""

import numpy as np
from pydrake.all import ImageDepth32F, Meshcat, MeshcatParams, Rgba

import qr_mapping.cpp as qr_mapping_cpp


def main() -> None:
    params = MeshcatParams()
    params.port = 0
    meshcat = Meshcat(params)

    config = qr_mapping_cpp.VoxelGridConfig()
    config.x_size = 0.4
    config.y_size = 0.4
    config.z_size = 1.0
    config.x_origin = -0.2
    config.y_origin = -0.2
    config.z_origin = 0.0
    config.grid_resolution = 0.1
    config.max_threads = 1
    config.filled_color = Rgba(1.0, 0.0, 0.0, 1.0)
    config.unknown_color = Rgba(0.0, 0.0, 0.0, 0.5)

    voxel_grid = qr_mapping_cpp.VoxelGridEstimator(config, meshcat)

    depth = ImageDepth32F(width=4, height=4)
    np.copyto(
        depth.mutable_data,  # ty:ignore[invalid-argument-type]
        np.full((4, 4, 1), 0.5, dtype=np.float32),
    )

    intrinsics = np.array(
        [
            [100.0, 0.0, 1.5],
            [0.0, 100.0, 1.5],
            [0.0, 0.0, 1.0],
        ]
    )
    extrinsics = np.eye(4)

    voxel_grid.UpdateFromDepthImage(
        depth,
        intrinsics,
        extrinsics,
        None,
        0,
        0.001,
    )

    centers = voxel_grid.GetVoxelCenters()
    occupancy = voxel_grid.GetOccupancy()

    assert centers.shape == (4, 4, 10, 3), centers.shape
    assert occupancy.shape == (4, 4, 10), occupancy.shape
    assert occupancy.dtype == np.uint8, occupancy.dtype
    assert set(np.unique(occupancy)).issubset({0, 1, 255})
    assert np.any(occupancy == 1), "Synthetic update did not mark any occupied cells"

    print(f"Imported {qr_mapping_cpp.__file__}")
    print(f"Voxel centers: shape={centers.shape}, dtype={centers.dtype}")
    print(f"Occupancy: shape={occupancy.shape}, values={np.unique(occupancy).tolist()}")


if __name__ == "__main__":
    main()

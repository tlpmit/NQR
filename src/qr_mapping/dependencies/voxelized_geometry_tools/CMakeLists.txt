cmake_minimum_required(VERSION 3.5)
project(voxelized_geometry_tools)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenMP)
find_package(CUDA)
find_package(OpenCL)

## We don't depend on Drake, but we do use different build flags if present.
find_package(drake QUIET)

# =============================================================================
# common_robotics_utilities
# =============================================================================
# Include the following if you've installed common_robotics_utilities
# find_package(common_robotics_utilities REQUIRED)

# Else, include this if you're adding it as a subdirectory
# add_subdirectory(common_robotics_utilities)
# include_directories(${common_robotics_utilities_SOURCE_DIR}/include)
# =============================================================================

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

## Build options
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
cmake_policy(SET CMP0069 NEW)

add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-Werror=non-virtual-dtor)
add_compile_options(-Wold-style-cast)
add_compile_options(-Wpessimizing-move)
add_compile_options(-Wuninitialized)
add_compile_options(-Wmissing-declarations)

if(drake_FOUND)
    message(STATUS "Drake found, disabling -march=native")
else()
    message(STATUS "Drake NOT found, enabling -march=native")
    add_compile_options(-march=native)
endif()

## It's not clear if add_compile_options does the right things for flags that
## may differ between languages and target type.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

# Voxelized geometry tools library
add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/collision_map.hpp
            include/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.hpp
            include/${PROJECT_NAME}/mesh_rasterizer.hpp
            include/${PROJECT_NAME}/signed_distance_field.hpp
            include/${PROJECT_NAME}/signed_distance_field_generation.hpp
            include/${PROJECT_NAME}/tagged_object_collision_map.hpp
            include/${PROJECT_NAME}/topology_computation.hpp
            src/${PROJECT_NAME}/collision_map.cpp
            src/${PROJECT_NAME}/mesh_rasterizer.cpp
            src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
            src/${PROJECT_NAME}/signed_distance_field.cpp
            src/${PROJECT_NAME}/signed_distance_field_generation.cpp
            src/${PROJECT_NAME}/tagged_object_collision_map.cpp)
target_link_libraries(${PROJECT_NAME} common_robotics_utilities)

# Cuda voxelization helpers library
if(CUDA_FOUND)
    message(STATUS "CUDA found. Building CUDA voxelization helpers")
    include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O3 -use_fast_math")
    set(CUDA_NVCC_FLAGS
        "${CUDA_NVCC_FLAGS} -Xcompiler -D__CORRECT_ISO_CPP11_MATH_H_PROTO")
    cuda_add_library(${PROJECT_NAME}_cuda_voxelization_helpers
                     include/${PROJECT_NAME}/device_voxelization_interface.hpp
                     include/${PROJECT_NAME}/cuda_voxelization_helpers.h
                     src/${PROJECT_NAME}/cuda_voxelization_helpers.cu)
    target_link_libraries(${PROJECT_NAME}_cuda_voxelization_helpers
                          cuda ${CUDA_LIBRARIES})
    add_library(${PROJECT_NAME}_dummy_cuda_voxelization_helpers
                include/${PROJECT_NAME}/device_voxelization_interface.hpp
                include/${PROJECT_NAME}/cuda_voxelization_helpers.h
                src/${PROJECT_NAME}/dummy_cuda_voxelization_helpers.cc)
else()
    message(WARNING "CUDA not found. Building stubs for CUDA helpers.")
    add_library(${PROJECT_NAME}_cuda_voxelization_helpers
                include/${PROJECT_NAME}/device_voxelization_interface.hpp
                include/${PROJECT_NAME}/cuda_voxelization_helpers.h
                src/${PROJECT_NAME}/dummy_cuda_voxelization_helpers.cc)
endif()

# OpenCL voxelization helpers library
if(OpenCL_FOUND)
    message(STATUS "OpenCL found. Building OpenCL voxelization helpers")
    include_directories(SYSTEM ${OpenCL_INCLUDE_DIRS})
    add_library(${PROJECT_NAME}_opencl_voxelization_helpers
                include/${PROJECT_NAME}/device_voxelization_interface.hpp
                include/${PROJECT_NAME}/opencl_voxelization_helpers.h
                src/${PROJECT_NAME}/opencl_voxelization_helpers.cc)
    target_link_libraries(${PROJECT_NAME}_opencl_voxelization_helpers
                          ${OpenCL_LIBRARIES})
    add_library(${PROJECT_NAME}_dummy_opencl_voxelization_helpers
                include/${PROJECT_NAME}/device_voxelization_interface.hpp
                include/${PROJECT_NAME}/opencl_voxelization_helpers.h
                src/${PROJECT_NAME}/dummy_opencl_voxelization_helpers.cc)
else()
    message(WARNING "OpenCL not found. Building stubs for OpenCL helpers.")
    add_library(${PROJECT_NAME}_opencl_voxelization_helpers
                include/${PROJECT_NAME}/device_voxelization_interface.hpp
                include/${PROJECT_NAME}/opencl_voxelization_helpers.h
                src/${PROJECT_NAME}/dummy_opencl_voxelization_helpers.cc)
endif()

# Pointcloud voxelization
add_library(${PROJECT_NAME}_pointcloud_voxelization
            include/${PROJECT_NAME}/cpu_pointcloud_voxelization.hpp
            include/${PROJECT_NAME}/device_pointcloud_voxelization.hpp
            include/${PROJECT_NAME}/device_voxelization_interface.hpp
            include/${PROJECT_NAME}/pointcloud_voxelization_interface.hpp
            include/${PROJECT_NAME}/pointcloud_voxelization.hpp
            src/${PROJECT_NAME}/cpu_pointcloud_voxelization.cpp
            src/${PROJECT_NAME}/device_pointcloud_voxelization.cpp
            src/${PROJECT_NAME}/pointcloud_voxelization.cpp)
target_link_libraries(${PROJECT_NAME}_pointcloud_voxelization
                      ${PROJECT_NAME}_cuda_voxelization_helpers
                      ${PROJECT_NAME}_opencl_voxelization_helpers
                      ${PROJECT_NAME})

if(BUILD_TESTING)
    # Tests
    find_package(GTest REQUIRED)
    enable_testing()

    add_executable(pointcloud_voxelization_test
        test/pointcloud_voxelization_test.cpp)
    target_link_libraries(pointcloud_voxelization_test
        ${PROJECT_NAME}_pointcloud_voxelization
        GTest::GTest
        GTest::Main)
    add_test(NAME pointcloud_voxelization_test
             COMMAND pointcloud_voxelization_test)

    add_executable(mesh_rasterization_test test/mesh_rasterization_test.cpp)
    target_link_libraries(mesh_rasterization_test
        ${PROJECT_NAME}
        GTest::GTest
        GTest::Main)
    add_test(NAME mesh_rasterization_test
             COMMAND mesh_rasterization_test)

    add_executable(sdf_generation_test test/sdf_generation_test.cpp)
    target_link_libraries(sdf_generation_test
        ${PROJECT_NAME}
        GTest::GTest
        GTest::Main)
    add_test(NAME sdf_generation_test
             COMMAND sdf_generation_test)
endif()

#############
## Install ##
#############

install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_cuda_voxelization_helpers
    ${PROJECT_NAME}_opencl_voxelization_helpers
    ${PROJECT_NAME}_pointcloud_voxelization
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN ".svn" EXCLUDE
)

cmake_minimum_required(VERSION 3.5)
project(qr_mapping)

# Find required packages
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenMP)

find_package(drake REQUIRED)

# Include directories
include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

# To compile all code (including dependencies) with -fPIC, which is required
# when creating shared libraries (in our case, the Python module).
# COLCON does this automatically, but we need to do it manually here.
# This must be set before adding subdirectories.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_CXX_FLAGS MATCHES "-fPIC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
endif()

# =============================================================================
# Dependencies: common_robotics_utilities and voxelized_geometry_tools
# =============================================================================

# Do not build tests on macOS; sometimes there is an issue with compiling
# and linking GTest.
# Also disable on Linux to avoid compilation issues with pointcloud_voxelization_test
if (APPLE OR UNIX)
    set(BUILD_TESTING OFF)
endif()

# These vendored libraries are implementation details of qr_mapping. Marking
# them EXCLUDE_FROM_ALL keeps their own install rules out of the Python wheel,
# while still allowing their targets to build when qr_mapping links them.
add_subdirectory(dependencies/common_robotics_utilities EXCLUDE_FROM_ALL)
include_directories(${common_robotics_utilities_SOURCE_DIR}/include)

add_subdirectory(dependencies/voxelized_geometry_tools EXCLUDE_FROM_ALL)
include_directories(${voxelized_geometry_tools_SOURCE_DIR}/include)
# =============================================================================

# Build options
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
cmake_policy(SET CMP0069 NEW)

# This previously was -std=c++17; had to update it to work on macOS.
add_compile_options(-std=c++20)
add_compile_options(-Wall)
add_compile_options(-Wextra)
# MacOS throws a bunch of sign-conversion errors with this flag while it works
# just fine on Linux.
if (NOT APPLE)
    add_compile_options(-Werror)
endif()
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-Werror=non-virtual-dtor)
add_compile_options(-Wold-style-cast)
add_compile_options(-Wpessimizing-move)
add_compile_options(-Wuninitialized)
add_compile_options(-Wmissing-declarations)

# OpenMP flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

# =============================================================================
# region qr_mapping library
# =============================================================================

add_library(${PROJECT_NAME} STATIC
    include/${PROJECT_NAME}/voxel_grid.hpp
    include/${PROJECT_NAME}/drake_pointcloud_voxelization_interface.hpp
    include/${PROJECT_NAME}/color_depth_images_to_pointcloud_generator.h
    src/${PROJECT_NAME}/voxel_grid.cpp
    src/${PROJECT_NAME}/drake_pointcloud_voxelization_interface.cpp
    src/${PROJECT_NAME}/color_depth_images_to_pointcloud_generator.cc
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${drake_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${drake_LIBRARIES}
    common_robotics_utilities
    voxelized_geometry_tools
    voxelized_geometry_tools_pointcloud_voxelization
)

# endregion
# =============================================================================
# region qr_mapping.cpp Pybind11 bindings
# =============================================================================

# Search for venv's Python first.
if (APPLE)
    if(DEFINED ENV{VIRTUAL_ENV})
        set(Python3_ROOT_DIR $ENV{VIRTUAL_ENV})
        set(Python3_EXECUTABLE $ENV{VIRTUAL_ENV}/bin/python3)
        set(Python3_FIND_VIRTUALENV FIRST)  # Changed from FIRST to ONLY
    endif()
endif()

# Find Python3 and Pybind11 in this order.
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Debug info
message(STATUS "Python3_VERSION: ${Python3_VERSION}")
message(STATUS "Python3_EXECUTABLE: ${Python3_EXECUTABLE}")
message(STATUS "Python3_LIBRARIES: ${Python3_LIBRARIES}")
message(STATUS "Python3_INCLUDE_DIRS: ${Python3_INCLUDE_DIRS}")
message(STATUS "pybind11_VERSION: ${pybind11_VERSION}")
message(STATUS "PYTHON_MODULE_EXTENSION: ${PYTHON_MODULE_EXTENSION}")

pybind11_add_module(
    cpp
    src/qr_mapping/bindings.cpp
)
if(UNIX AND NOT APPLE)
    # The Linux pydrake wheel is built with GCC 11-era pybind internals
    # (cxxabi1016). Our Pixi compiler is newer, so align the pybind registry
    # key for this module with pydrake to allow shared Drake types like Rgba.
    target_compile_definitions(cpp PRIVATE PYBIND11_BUILD_ABI="_cxxabi1016")
endif()
target_link_libraries(
    cpp
    PUBLIC
    ${PROJECT_NAME}
    drake::drake
)

# Link OpenMP only if found
if(OpenMP_CXX_FOUND)
    target_link_libraries(cpp PUBLIC OpenMP::OpenMP_CXX)
endif()

# Build against the fetched Drake SDK, but load the PyPI pydrake copy of
# libdrake.so after installation. That keeps qr_mapping.cpp and pydrake on a
# single runtime libdrake and avoids duplicate Objective-C class warnings.
get_filename_component(QR_DRAKE_LIBRARY_DIR "${drake_DIR}/../.." ABSOLUTE)
if(APPLE)
    set(QR_DRAKE_INSTALL_RPATH "@loader_path/../pydrake/lib")
else()
    set(QR_DRAKE_INSTALL_RPATH "$ORIGIN/../pydrake/lib")
endif()

set_target_properties(cpp PROPERTIES
    CXX_VISIBILITY_PRESET default
    BUILD_RPATH "${QR_DRAKE_LIBRARY_DIR}"
    INSTALL_RPATH "${QR_DRAKE_INSTALL_RPATH}"
)

install(TARGETS cpp
    LIBRARY DESTINATION qr_mapping
)
install(FILES cpp.pyi DESTINATION qr_mapping)

# endregion
# =============================================================================

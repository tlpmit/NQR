cmake_minimum_required(VERSION 3.5)
project(common_robotics_utilities)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenMP)
find_package(ZLIB REQUIRED)

# We don't depend on Drake, but we do use different build flags if present.
find_package(drake QUIET)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS})

# Build options
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
cmake_policy(SET CMP0069 NEW)

add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-Wextra)
# MacOS throws a bunch of sign-conversion errors with this flag while it works
# just fine on Linux.
if(NOT APPLE)
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

if(drake_FOUND)
    message(STATUS "Drake found, disabling -march=native")
else()
    message(STATUS "Drake NOT found, enabling -march=native")
    add_compile_options(-march=native)
endif()

# OpenMP flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

# Utility library
add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/base64_helpers.hpp
            include/${PROJECT_NAME}/color_builder.hpp
            include/${PROJECT_NAME}/conversions.hpp
            include/${PROJECT_NAME}/cru_namespace.hpp
            include/${PROJECT_NAME}/dynamic_spatial_hashed_voxel_grid.hpp
            include/${PROJECT_NAME}/gaussian_distributions.hpp
            include/${PROJECT_NAME}/math.hpp
            include/${PROJECT_NAME}/maybe.hpp
            include/${PROJECT_NAME}/openmp_helpers.hpp
            include/${PROJECT_NAME}/parallelism.hpp
            include/${PROJECT_NAME}/path_processing.hpp
            include/${PROJECT_NAME}/print.hpp
            include/${PROJECT_NAME}/random_rotation_generator.hpp
            include/${PROJECT_NAME}/ros_conversions.hpp
            include/${PROJECT_NAME}/ros_helpers.hpp
            include/${PROJECT_NAME}/serialization.hpp
            include/${PROJECT_NAME}/simple_astar_search.hpp
            include/${PROJECT_NAME}/simple_dtw.hpp
            include/${PROJECT_NAME}/simple_graph.hpp
            include/${PROJECT_NAME}/simple_graph_search.hpp
            include/${PROJECT_NAME}/simple_hausdorff_distance.hpp
            include/${PROJECT_NAME}/simple_hierarchical_clustering.hpp
            include/${PROJECT_NAME}/simple_kmeans_clustering.hpp
            include/${PROJECT_NAME}/simple_knearest_neighbors.hpp
            include/${PROJECT_NAME}/simple_prm_planner.hpp
            include/${PROJECT_NAME}/simple_prngs.hpp
            include/${PROJECT_NAME}/simple_robot_model_interface.hpp
            include/${PROJECT_NAME}/simple_rrt_planner.hpp
            include/${PROJECT_NAME}/simple_task_planner.hpp
            include/${PROJECT_NAME}/time_optimal_trajectory_parametrization.hpp
            include/${PROJECT_NAME}/utility.hpp
            include/${PROJECT_NAME}/voxel_grid.hpp
            include/${PROJECT_NAME}/zlib_helpers.hpp
            src/${PROJECT_NAME}/base64_helpers.cpp
            src/${PROJECT_NAME}/conversions.cpp
            src/${PROJECT_NAME}/math.cpp
            src/${PROJECT_NAME}/serialization.cpp
            src/${PROJECT_NAME}/time_optimal_trajectory_parametrization.cpp
            src/${PROJECT_NAME}/zlib_helpers.cpp)
target_link_libraries(${PROJECT_NAME} ${ZLIB_LIBRARIES})

# Examples
add_executable(clustering_example example/clustering_example.cpp)
target_link_libraries(clustering_example ${PROJECT_NAME})

add_executable(dtw_example example/dtw_example.cpp)
target_link_libraries(dtw_example ${PROJECT_NAME})

# Tests
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    enable_testing()

    add_executable(hausdorff_distance_test test/hausdorff_distance_test.cpp)
    target_link_libraries(hausdorff_distance_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    add_test(NAME hausdorff_distance_test COMMAND hausdorff_distance_test)

    add_executable(maybe_test test/maybe_test.cpp)
    target_link_libraries(maybe_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    add_test(NAME maybe_test COMMAND maybe_test)

    add_executable(parallelism_test test/parallelism_test.cpp)
    target_link_libraries(parallelism_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    add_test(NAME parallelism_test COMMAND parallelism_test)

    add_executable(planning_test test/planning_test.cpp)
    target_link_libraries(planning_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    add_test(NAME planning_test COMMAND planning_test)

    add_executable(task_planning_test test/task_planning_test.cpp)
    target_link_libraries(task_planning_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    add_test(NAME task_planning_test COMMAND task_planning_test)

    add_executable(utility_test test/utility_test.cpp)
    target_link_libraries(utility_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    add_test(NAME utility_test COMMAND utility_test)

    add_executable(voxel_grid_test test/voxel_grid_test.cpp)
    target_link_libraries(voxel_grid_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    add_test(NAME voxel_grid_test COMMAND voxel_grid_test)

    # add_executable(print_test test/print_test.cpp)
    # target_link_libraries(print_test ${PROJECT_NAME} GTest::GTest GTest::Main)
    # add_test(NAME print_test COMMAND print_test)
endif()

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY include/
    DESTINATION include
)

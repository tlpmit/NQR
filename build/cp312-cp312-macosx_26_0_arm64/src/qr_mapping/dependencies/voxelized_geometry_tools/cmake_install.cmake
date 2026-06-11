# Install script for directory: /Users/tlp/Documents/GitHub/NQR/src/qr_mapping/dependencies/voxelized_geometry_tools

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/var/folders/y2/fghxh2fs6tsd8v8837_d5b400000gn/T/tmp95vgnwlv/wheel/platlib")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Users/tlp/Documents/GitHub/NQR/.pixi/envs/default/bin/llvm-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/tlp/Documents/GitHub/NQR/build/cp312-cp312-macosx_26_0_arm64/src/qr_mapping/dependencies/voxelized_geometry_tools/libvoxelized_geometry_tools.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools.a")
    execute_process(COMMAND "/Users/tlp/Documents/GitHub/NQR/.pixi/envs/default/bin/arm64-apple-darwin20.0.0-ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/tlp/Documents/GitHub/NQR/build/cp312-cp312-macosx_26_0_arm64/src/qr_mapping/dependencies/voxelized_geometry_tools/libvoxelized_geometry_tools_cuda_voxelization_helpers.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_cuda_voxelization_helpers.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_cuda_voxelization_helpers.a")
    execute_process(COMMAND "/Users/tlp/Documents/GitHub/NQR/.pixi/envs/default/bin/arm64-apple-darwin20.0.0-ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_cuda_voxelization_helpers.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/tlp/Documents/GitHub/NQR/build/cp312-cp312-macosx_26_0_arm64/src/qr_mapping/dependencies/voxelized_geometry_tools/libvoxelized_geometry_tools_opencl_voxelization_helpers.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_opencl_voxelization_helpers.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_opencl_voxelization_helpers.a")
    execute_process(COMMAND "/Users/tlp/Documents/GitHub/NQR/.pixi/envs/default/bin/arm64-apple-darwin20.0.0-ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_opencl_voxelization_helpers.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/tlp/Documents/GitHub/NQR/build/cp312-cp312-macosx_26_0_arm64/src/qr_mapping/dependencies/voxelized_geometry_tools/libvoxelized_geometry_tools_pointcloud_voxelization.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_pointcloud_voxelization.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_pointcloud_voxelization.a")
    execute_process(COMMAND "/Users/tlp/Documents/GitHub/NQR/.pixi/envs/default/bin/arm64-apple-darwin20.0.0-ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libvoxelized_geometry_tools_pointcloud_voxelization.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/Users/tlp/Documents/GitHub/NQR/src/qr_mapping/dependencies/voxelized_geometry_tools/include/" FILES_MATCHING REGEX "/[^/]*\\.hpp$" REGEX "/\\.svn$" EXCLUDE)
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/tlp/Documents/GitHub/NQR/build/cp312-cp312-macosx_26_0_arm64/src/qr_mapping/dependencies/voxelized_geometry_tools/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

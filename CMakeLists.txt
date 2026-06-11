cmake_minimum_required(VERSION 3.15)
project(NQR)

include(FetchContent)

# Pixi exposes compilers and libraries through the active conda prefix. Add it
# first so CMake finds Pixi-provided packages before system/Homebrew installs.
if(DEFINED ENV{CONDA_PREFIX})
  list(PREPEND CMAKE_PREFIX_PATH "$ENV{CONDA_PREFIX}")
endif()

# The Drake PyPI wheel does not ship the C++ headers/configs needed by
# qr_mapping, so the Pixi build fetches the pinned binary SDK directly.
# Must match the pip drake version exactly: qr_mapping shares Drake objects
# (Meshcat, images) with pydrake across the pybind boundary.
set(QR_DRAKE_VERSION "1.45.0")

# Drake release tarballs are named by platform
if(APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
  set(QR_DRAKE_PLATFORM "mac-arm64")
elseif(UNIX AND EXISTS "/etc/os-release")
  file(READ "/etc/os-release" QR_OS_RELEASE)
  if(QR_OS_RELEASE MATCHES "VERSION_CODENAME=noble")
    set(QR_DRAKE_PLATFORM "noble")
  elseif(QR_OS_RELEASE MATCHES "VERSION_CODENAME=jammy")
    set(QR_DRAKE_PLATFORM "jammy")
  else()
    message(FATAL_ERROR "Unsupported Ubuntu version. Expected jammy or noble.")
  endif()
else()
  message(FATAL_ERROR "Unsupported platform. Expected macOS arm64 or Ubuntu.")
endif()

# Keep the downloaded SDK under .pixi so all generated install/build state
# lives in the Pixi-owned part of the checkout (and survives rebuilds).
set(FETCHCONTENT_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.pixi/fetchcontent")
set(QR_DRAKE_SDK_DIR
    "${FETCHCONTENT_BASE_DIR}/drake-${QR_DRAKE_VERSION}-${QR_DRAKE_PLATFORM}")
FetchContent_Declare(
  drake_sdk
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_DIR "${QR_DRAKE_SDK_DIR}"
  URL "https://github.com/RobotLocomotion/drake/releases/download/v${QR_DRAKE_VERSION}/drake-${QR_DRAKE_VERSION}-${QR_DRAKE_PLATFORM}.tar.gz"
)
FetchContent_MakeAvailable(drake_sdk)

# FetchContent extracts Drake's archive into drake_sdk_SOURCE_DIR. Cache
# drake_DIR so only Drake comes from the SDK; other packages such as pybind11
# should still come from Pixi's prefix.
set(QR_DRAKE_PREFIX "${drake_sdk_SOURCE_DIR}")
set(drake_DIR "${QR_DRAKE_PREFIX}/lib/cmake/drake" CACHE PATH
    "Drake CMake package directory" FORCE)

find_package(drake CONFIG REQUIRED)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(src/qr_mapping)

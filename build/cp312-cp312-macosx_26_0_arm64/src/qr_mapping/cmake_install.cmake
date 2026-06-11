# Install script for directory: /Users/tlp/Documents/GitHub/NQR/src/qr_mapping

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/qr_mapping" TYPE MODULE FILES "/Users/tlp/Documents/GitHub/NQR/build/cp312-cp312-macosx_26_0_arm64/src/qr_mapping/cpp.cpython-312-darwin.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/qr_mapping/cpp.cpython-312-darwin.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/qr_mapping/cpp.cpython-312-darwin.so")
    execute_process(COMMAND /Users/tlp/Documents/GitHub/NQR/.pixi/envs/default/bin/arm64-apple-darwin20.0.0-install_name_tool
      -delete_rpath "/Users/tlp/Documents/GitHub/NQR/.pixi/fetchcontent/drake-1.45.0-mac-arm64/lib"
      -add_rpath "@loader_path/../pydrake/lib"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/qr_mapping/cpp.cpython-312-darwin.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/Users/tlp/Documents/GitHub/NQR/.pixi/envs/default/bin/arm64-apple-darwin20.0.0-strip" -x "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/qr_mapping/cpp.cpython-312-darwin.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/qr_mapping" TYPE FILE FILES "/Users/tlp/Documents/GitHub/NQR/src/qr_mapping/cpp.pyi")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/tlp/Documents/GitHub/NQR/build/cp312-cp312-macosx_26_0_arm64/src/qr_mapping/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

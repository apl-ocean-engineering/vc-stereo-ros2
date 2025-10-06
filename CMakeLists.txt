
cmake_minimum_required(VERSION 3.10)
project(vc_stereo_ros2
        LANGUAGES CUDA CXX)

set(CMAKE_CUDA_ARCHITECTURES "53")

find_package(CUDAToolkit)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# Location set by the `nvidia-l4t-jetson-multimedia-api` apt package
set(ARGUS_DIR /usr/src/jetson_multimedia_api/argus)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake "${CMAKE_MODULE_PATH}")

set(AMENT_DEPENDS
  ament_cmake
  camera_info_manager
  generate_parameter_library
  image_transport
  imaging_msgs
  rclcpp
  rclcpp_components
)

foreach(Dependency IN ITEMS ${AMENT_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(EGL REQUIRED)
find_package(Argus REQUIRED)
find_package(X11 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK REQUIRED IMPORTED_TARGET gtk+-3.0)
pkg_check_modules(EGL REQUIRED IMPORTED_TARGET egl)


# == Build ==
#
#

add_library(nvidia_multimedia_api SHARED
  thirdparty/nvidia_multimedia_api/src/ArgusHelpers.cpp
  thirdparty/nvidia_multimedia_api/src/CUDAHelper.cpp
  thirdparty/nvidia_multimedia_api/src/EGLGlobal.cpp
  thirdparty/nvidia_multimedia_api/src/Thread.cpp
)

target_include_directories(
    nvidia_multimedia_api
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nvidia_multimedia_api/include>"
        "$<BUILD_INTERFACE:${ARGUS_DIR}/../include/>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(
  nvidia_multimedia_api
  PUBLIC
      CUDA::cudart
      X11::X11
      PkgConfig::GTK
      PkgConfig::EGL
      ${ARGUS_LIBRARIES}
)

#
#

generate_parameter_library(
  stereo_parameters    # cmake target name for the parameter library
  src/parameters.yaml  # path to input yaml file
)

add_library(stereo_driver_component SHARED
    src/convert.cu
    src/cuda_frame_acquire.cpp
    src/gpio_trigger_thread.cpp
    src/stereo_node.cpp
    src/stereo_consumer.cpp
    src/v4l_device.cpp
)

target_include_directories(
    stereo_driver_component
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(stereo_driver_component
  PUBLIC
        camera_info_manager::camera_info_manager
        image_transport::image_transport
        ${imaging_msgs_TARGETS}
        rclcpp::rclcpp
        rclcpp_components::component
        nvidia_multimedia_api
        stereo_parameters
)

rclcpp_components_register_node(
    stereo_driver_component
    PLUGIN "vc_stereo_ros2::SyncedStereoNode"
    EXECUTABLE stereo_node
)

# == Install ==
#

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_stereo_driver_component nvidia_multimedia_api stereo_parameters)
ament_export_dependencies(stereo_parameters nvidia_multimedia_api)

install(TARGETS stereo_parameters
        EXPORT stereo_parameters
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS nvidia_multimedia_api
        EXPORT nvidia_multimedia_api
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS stereo_driver_component
        EXPORT export_stereo_driver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config/calibrations
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Run linters found in package.xml except those below
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_uncrustify_FOUND TRUE)
#   set(ament_cmake_pep257_FOUND TRUE)
#   set(ament_cmake_flake8_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

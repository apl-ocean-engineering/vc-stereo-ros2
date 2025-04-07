
cmake_minimum_required(VERSION 3.3)
project(vc_stereo_ros2)

if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

set(ARGUS_DIR /usr/src/jetson_multimedia_api/argus)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake "${CMAKE_MODULE_PATH}")

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  camera_info_manager
  generate_parameter_library
  image_transport
  imaging_msgs
  rclcpp
  rclcpp_components
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(EGL REQUIRED)
find_package(CUDA REQUIRED)
find_package(Argus REQUIRED)
find_package(X11 REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

# == Build ==
#
#


add_library(nvidia_multimedia_api
  thirdparty/nvidia_multimedia_api/src/ArgusHelpers.cpp
  thirdparty/nvidia_multimedia_api/src/CUDAHelper.cpp
  thirdparty/nvidia_multimedia_api/src/EGLGlobal.cpp
  thirdparty/nvidia_multimedia_api/src/Observed.cpp
  thirdparty/nvidia_multimedia_api/src/Thread.cpp
)
ament_target_dependencies(nvidia_multimedia_api ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(nvidia_multimedia_api
  ${GTK3_LIBRARIES}
  ${EGL_LIBRARIES}
  ${X11_LIBRARIES}
)

target_include_directories(nvidia_multimedia_api
  PUBLIC
  thirdparty/nvidia_multimedia_api/include
  ${ARGUS_DIR}/../include
  PRIVATE
  ${GTK3_INCLUDE_DIRS}
  ${EGL_INCLUDE_DIRS}
)

#
#

generate_parameter_library(
  stereo_parameters # cmake target name for the parameter library
  src/parameters.yaml # path to input yaml file
)

cuda_add_executable(stereo_node
    src/convert.cu
    src/cuda_frame_acquire.cpp
    src/gpio_trigger_thread.cpp
    src/main.cpp
    src/stereo_consumer.cpp
    src/v4l_device.cpp
)

target_include_directories(stereo_node
  PUBLIC
  ${ARGUS_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(stereo_node
  ${catkin_LIBRARIES}
  ${ARGUS_LIBRARIES}
  cuda
  nvidia_multimedia_api
  stereo_parameters
)

# == Install ==
#

install(
  DIRECTORY include/
  DESTINATION include
)

# # Installation of the library has an unexpected dependency on
# # "arena_camera_parameters".   Resolve if downstream packages
# # ever want to use the arena_camera library.
# install(
#   TARGETS ${arena_camera_lib}
# #  EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
# )
# # ament_export_targets(export_${arena_camera_lib} HAS_LIBRARY_TARGET)
# # ament_export_dependencies(arena_camera_parameters)

install(TARGETS stereo_node
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY launch
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

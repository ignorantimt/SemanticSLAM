cmake_minimum_required(VERSION 3.5)
project(semanticSLAM)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

set(ncnn_DIR "/home/tengfei/LLM_Mani/SemanticSLAM/src/SemanticSLAM/Thirdparty/ncnn/build/install/lib/cmake/ncnn" CACHE PATH "/home/tengfei/LLM_Mani/SemanticSLAM/src/sg-slam/Thirdparty/ncnn/build/install/lib/cmake/ncnn")

set(NCNN_TARGET_LIB ncnn)

find_package(ncnn REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
SET(EIGEN3_INCLUDE_DIR "/home/tengfei/LLM_Mani/SemanticSLAM/eigen/install/include/eigen3")
find_package(Pangolin REQUIRED)
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include/SemanticSLAM
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Viewer.cc
src/Detector2D.cc
src/Detector3D.cc
src/ObjectDatabase.cc
src/PointcloudMapping.cc
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  cv_bridge
  image_geometry
  message_filters
  tf2
  tf2_ros
  nav_msgs
  visualization_msgs
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${PCL_LIBRARIES}
${NCNN_TARGET_LIB}
${OCTOMAP_LIBRARIES}
-lgomp
)

# Build examples
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples)

# add_executable(sg_slam_tum Examples/rgbd_tum.cc)
# target_link_libraries(sg_slam_tum ${PROJECT_NAME})

add_executable(semantic_ros_rgbd src/ros_rgbd.cc)
target_link_libraries(semantic_ros_rgbd ${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

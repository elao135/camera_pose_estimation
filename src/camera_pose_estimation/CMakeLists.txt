cmake_minimum_required(VERSION 3.0.2)
project(camera_pose_estimation)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package (OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_pose_estimation
 CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rgb_pose_estimation src/rgb_pose_estimation.cpp)
target_link_libraries(rgb_pose_estimation ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


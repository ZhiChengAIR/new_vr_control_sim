cmake_minimum_required(VERSION 3.8)
project(my_pinocchio_robot)

find_package (Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(Boost_PYTHON_VERSION 3.10)
find_package(Boost 1.74 REQUIRED COMPONENTS python310)
find_package(rclcpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


# 添加你的可执行文件或库
add_executable(my_node src/test_node.cpp)

ament_target_dependencies(
  my_node
   rclcpp sensor_msgs 
   nav_msgs 
  geometry_msgs
  message_filters
  cv_bridge
  pinocchio
  )

target_link_libraries(my_node
  ${OpenCV_LIBS}
  ${HDF5_LIBRARIES}
  ${HDF5_C_LIBRARIES}
  ${HDF5_CXX_LIBRARIES}
  curl
)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
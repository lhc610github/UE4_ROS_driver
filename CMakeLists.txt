cmake_minimum_required(VERSION 3.0.2)
project(ue4_ros_drivers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  tf2
  tf
  rosgraph_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  nodelet
  pluginlib
  nav_msgs
  geometry_msgs
  mavros_msgs
  tf2
  tf
  rosgraph_msgs
  DEPENDS
  EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(image_driver_nodelet src/image_driver_nodelet.cpp)
target_include_directories(image_driver_nodelet PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(image_driver_nodelet PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(image_driver_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
TARGETS image_driver_nodelet 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(imu_driver_nodelet src/imu_driver_nodelet.cpp)
target_include_directories(imu_driver_nodelet PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(imu_driver_nodelet PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(imu_driver_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
TARGETS imu_driver_nodelet 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(lidar_driver_nodelet src/lidar_driver_nodelet.cpp)
target_include_directories(lidar_driver_nodelet PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(lidar_driver_nodelet PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(lidar_driver_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
TARGETS lidar_driver_nodelet 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(control_driver_nodelet src/control_driver_nodelet.cpp)
target_include_directories(control_driver_nodelet PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(control_driver_nodelet PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(control_driver_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
TARGETS control_driver_nodelet 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(groundtruth_driver_nodelet src/groundtruth_driver_nodelet.cpp)
target_include_directories(groundtruth_driver_nodelet PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(groundtruth_driver_nodelet PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(groundtruth_driver_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
TARGETS groundtruth_driver_nodelet 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(tf_publish_nodelet src/tf_publish_nodelet.cpp)
target_include_directories(tf_publish_nodelet PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(tf_publish_nodelet PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(tf_publish_nodelet ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(
TARGETS tf_publish_nodelet 
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nodelet_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
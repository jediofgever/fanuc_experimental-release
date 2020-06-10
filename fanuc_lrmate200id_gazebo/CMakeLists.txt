cmake_minimum_required(VERSION 2.8.3)
project(fanuc_lrmate200id_gazebo)

find_package(OpenCV REQUIRED )
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
  gazebo_ros
  gazebo_plugins
  roscpp
  fanuc_arm_perception_utilities
  image_transport
  camera_info_manager
  sensor_msgs
 )

find_package(gazebo 9 REQUIRED)
 
if(NOT EIGEN3_INCLUDE_DIRS)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}  )
include_directories(include ${fanuc_arm_perception_utilities_INCLUDE_DIRS})
include_directories(include ~/catkin_ws/src/PICK_PLACE_with_ROS_on_FANUC_ARM/fanuc_arm_perception_utilities/include/)

catkin_package(
  LIBRARIES  
  CATKIN_DEPENDS 
  roscpp 
  gazebo_ros 
  image_transport
  camera_info_manager
  sensor_msgs
 )  


add_library(model_push SHARED plugins/MoveObject.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(random_model_push SHARED plugins/RandomMoveObject.cc)
target_link_libraries(random_model_push ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(sim_pickplace SHARED plugins/SimulationPickandPlace.cc  ${SOURCES})
target_link_libraries(sim_pickplace ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBS} ${fanuc_arm_perception_utilities})
install(TARGETS sim_pickplace DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(sim_pickplace ${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(realsense_gazebo_plugin
    plugins/RealSensePlugin.cpp
    plugins/gazebo_ros_realsense.cpp
)
target_link_libraries(realsense_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_gazebo_plugin ${catkin_EXPORTED_TARGETS})


install(TARGETS  
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(image_handler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  message_generation
  sensor_msgs
)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS 
  cv_bridge 
  image_transport 
  message_runtime 
  sensor_msgs
)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(image_publisher src/image_publisher.cpp)
add_dependencies(image_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS image_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



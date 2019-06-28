cmake_minimum_required(VERSION 2.8.3)
project(LH_laser_driver)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(LH_laser_publisher src/LH_laser_publisher.cpp src/LH_laser.cpp)
add_dependencies(LH_laser_publisher LH_laser_driver_generate_messages_cpp)
target_link_libraries(LH_laser_publisher ${catkin_LIBRARIES})

install(TARGETS LH_laser_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 3.0.2)
project(servo_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS "ON")

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  geometry_msgs
  itolab_senior_car_msgs
  roscpp
  roslint
  std_msgs
  )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}_core
 CATKIN_DEPENDS
   autoware_msgs
   itolab_senior_car_msgs
   )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}_node_main.cpp
  src/${PROJECT_NAME}_node.cpp
  )

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

add_library(
  ${PROJECT_NAME}_core
  src/${PROJECT_NAME}.cpp
  )

target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_core ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )
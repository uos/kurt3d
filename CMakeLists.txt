cmake_minimum_required(VERSION 2.4.6)

project(kurt3d)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  laser_geometry
  message_generation
  sensor_msgs
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   ServoCommand.srv
   Scan.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  DEPENDS 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Set module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
set(CMAKE_CXX_FLAGS "-std=c++11" ${CMAKE_CXX_FLAGS})

# Include libusb1
find_package(libusb-1.0 REQUIRED)
include_directories(${LIBUSB_1_INCLUDE_DIRS})

add_executable(laserscanner_node src/laserscanner_node.cpp)
add_executable(servo_node src/USBInterface.cpp src/servo_node.cpp)
add_executable(ps3joy_kurt3d src/ps3joy_kurt3d.cpp)

add_dependencies(laserscanner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ps3joy_kurt3d ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# link executables
target_link_libraries(servo_node ${LIBUSB_1_LIBRARY})

target_link_libraries(laserscanner_node ${catkin_LIBRARIES})
target_link_libraries(servo_node ${catkin_LIBRARIES})
target_link_libraries(ps3joy_kurt3d ${catkin_LIBRARIES})
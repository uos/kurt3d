cmake_minimum_required(VERSION 2.4.6)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Set module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
message(STATUS "Module Path: ${CMAKE_MODULE_PATH}")
list( APPEND ROS_COMPILE_FLAGS "-std=c++0x")

# Include libusb1
find_package(libusb-1.0 REQUIRED)
include_directories(${LIBUSB_1_INCLUDE_DIRS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
generate_messages()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(laserscanner_node src/laserscanner_node.cpp)
add_executable(servo_node src/USBInterface.cpp src/servo_node.cpp)
add_executable(ps3joy_kurt3d src/ps3joy_kurt3d.cpp)

#target_link_libraries(example ${PROJECT_NAME})


# link executable
target_link_libraries(servo_node ${LIBUSB_1_LIBRARY})

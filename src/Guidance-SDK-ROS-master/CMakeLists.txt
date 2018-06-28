cmake_minimum_required(VERSION 2.8.3)
project(guidance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  sensor_msgs
  geometry_msgs
  image_transport 
  message_generation
)
find_package(OpenCV)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LandinData.msg
#   Message2.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs  # Or other packages containing msgs
 )

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs geometry_msgs roscpp)


include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

message("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message("-- 32bit detected")
    link_directories(lib/x86)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("-- 64bit detected")
    link_directories(lib/x64)
  endif ()
else()
  message("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
  message("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
  link_directories(lib/XU3)
else ()
endif ()

link_libraries(
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp)



add_executable(guidanceNode
  src/GuidanceNode.cpp
  src/DJI_utility.cpp)

add_executable(guidanceNodeTest
  src/GuidanceNodeTest.cpp)

add_executable(guidanceNodeCalibration
  src/GuidanceNodeCalibration.cpp
  src/DJI_utility.cpp)

target_link_libraries(
  guidanceNode
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  guidanceNodeTest
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  guidanceNodeCalibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 :

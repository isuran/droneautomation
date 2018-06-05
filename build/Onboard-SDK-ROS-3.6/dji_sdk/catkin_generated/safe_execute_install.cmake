execute_process(COMMAND "/home/ivica/catkin_ws/build/Onboard-SDK-ROS-3.6/dji_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ivica/catkin_ws/build/Onboard-SDK-ROS-3.6/dji_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

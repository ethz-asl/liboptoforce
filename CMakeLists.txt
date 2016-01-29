cmake_minimum_required(VERSION 2.6)
project(liboptoforce)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)

include_directories(src/lib/optoforce)
include_directories(${catkin_INCLUDE_DIRS})

add_library(optoforce
  src/lib/optoforce/Sensor.cpp
  src/lib/optoforce/SensorConfig.cpp
  src/lib/optoforce/SensorPackage.cpp
  src/lib/optoforce/SensorReading.cpp
  src/lib/optoforce/SerialDevice.cpp
  src/lib/optoforce/StateMachine.cpp
)

target_link_libraries(optoforce
   ${catkin_LIBRARIES}
)


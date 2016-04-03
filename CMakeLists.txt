cmake_minimum_required(VERSION 2.6)
project(optoforce)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS chrono)

catkin_package(
  INCLUDE_DIRS src/lib 
  LIBRARIES optoforce
  CATKIN_DEPENDS
  DEPENDS boost
  )

include_directories(src/lib)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

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
   ${Boost_LIBRARIES}
)


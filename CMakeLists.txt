
cmake_minimum_required(VERSION 2.8)

PROJECT(optoforceInterface)

ADD_DEFINITIONS( -ggdb -Wall -DLINUX -DANSI_DECLARATORS)

INCLUDE_DIRECTORIES(
  /usr/local/include
  )

LINK_DIRECTORIES(
  /usr/local/lib
  )


#Find boost

find_package(Boost REQUIRED COMPONENTS system thread )
if ( Boost_FOUND )
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
  add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})
  # Remember to include ${Boost_LIBRARIES} in the target_link_libraries() statement
endif ( Boost_FOUND )


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

add_executable(optoforceInterface
  src/main.cpp
  src/ForceSensorVector.cpp
  src/ForceSensorBase.cpp
  src/SerialDevice.cpp
  src/state_machine_wo_qt.cpp #Optoforce file, replaced QT dependence
  src/optoforce_parameters.cpp
  )

SET_TARGET_PROPERTIES(optoforceInterface PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(optoforceInterface
  pthread rt SM m boost_system boost_thread pthread #udev
)

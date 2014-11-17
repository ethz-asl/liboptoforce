cmake_minimum_required (VERSION 2.6)  

#SET(CMAKE_CXX_FLAGS "-std=c++0x")
#ADD_DEFINITIONS( -ggdb -Wall -DLINUX -DANSI_DECLARATORS)


######################
### SUBDIRECTORIES ###
######################

################
### INCLUDES ###
################
set(CAN_INCL ${CAN_INCL} 
	${CMAKE_CURRENT_SOURCE_DIR}/src 
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(CAN_SRCS ${CAN_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/src/ForceSensorVector.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/ForceSensorBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/SerialDevice.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/state_machine_wo_qt.cpp #Optoforce file, replaced QT dependence	
	${CMAKE_CURRENT_SOURCE_DIR}/src/optoforce_parameters.cpp
PARENT_SCOPE)


########################
### LINK DIRECTORIES ###
########################
set(CAN_LINKDIRS ${CAN_LINKDIRS} 

PARENT_SCOPE)


#################
### LIBRARIES ###
#################
set(CAN_LIBS ${CAN_LIBS} 
	SM boost_system boost_thread 
PARENT_SCOPE)
 
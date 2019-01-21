cmake_minimum_required(VERSION 2.8.3)
project(ai_robot_lcsfl)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
        geometry_msgs
        sensor_fusion_comm
        msf_core
        msf_updates
)
include_directories(
	${catkin_INCLUDE_DIRS}
	src
	)
#message("$$$$$$$$$" ${msf_updates_INCLUDE_DIRS})
generate_dynamic_reconfigure_options(
    cfg/DoublePoseSensor.cfg
)

catkin_package(
    CATKIN_DEPENDS 
		roscpp 
        geometry_msgs
        sensor_fusion_comm
		msf_core 
        msf_updates
    INCLUDE_DIRS 
    	#include 
    	${catkin_INCLUDE_DIRS}
    LIBRARIES 
    	pose_distorter
)

add_executable(lcsfl_node src/main.cpp)

target_link_libraries(lcsfl_node
    ${catkin_LIBRARIES}
)

#dd_dependencies(fusion_eth ${${PROJECT_NAME}_EXPORTED_TARGETS})



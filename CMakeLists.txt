cmake_minimum_required(VERSION 2.8.3)
project(ai_robot_lcsfl)
add_definitions(-DBOOST_NO_CXX11_VARIADIC_TEMPLATES)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
        geometry_msgs
        sensor_fusion_comm
        msf_core
        msf_updates
)
include_directories(
	${catkin_INCLUDE_DIRS}
#	src
        include
  #      cfg
	)
#message("$$$$$$$$$" ${include_directories})
generate_dynamic_reconfigure_options(
    cfg/SinglePoseSensor.cfg
)

catkin_package(
    CATKIN_DEPENDS 
	roscpp 
        geometry_msgs
        sensor_fusion_comm
		msf_core 
        msf_updates
    INCLUDE_DIRS 
        include
    	${catkin_INCLUDE_DIRS}
    LIBRARIES 
    	pose_distorter
)
add_executable(lcsfl_node src/main.cpp)

target_link_libraries(lcsfl_node pose_distorter
    ${catkin_LIBRARIES}
)

add_dependencies(lcsfl_node ${${PROJECT_NAME}_EXPORTED_TARGETS})



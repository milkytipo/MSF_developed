cmake_minimum_required(VERSION 2.8.3)
project(ai_robot_lcsfl)
add_definitions(-DBOOST_NO_CXX11_VARIADIC_TEMPLATES)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
        geometry_msgs
        sensor_fusion_comm
        msf_core
        msf_updates
		cv_bridge
		sensor_msgs
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(
	${catkin_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
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
		sensor_msgs
    INCLUDE_DIRS 
        include
    	${catkin_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
    LIBRARIES 
    	pose_distorter
)
add_executable(read_node  src/read_data_set/main.cpp)
add_executable(lcsfl_node src/position_msf/main.cpp)

target_link_libraries(lcsfl_node pose_distorter
    ${catkin_LIBRARIES}
)
target_link_libraries(read_node
		${catkin_LIBRARIES}
		${OpenCV_LIBS}
		)

add_dependencies(lcsfl_node  ${${PROJECT_NAME}_EXPORTED_TARGETS})
#add_dependencies(read_node ${${PROJECT_NAME}_EXPORTED_TARGETS})



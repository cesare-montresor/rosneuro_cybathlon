cmake_minimum_required(VERSION 2.8.3)
project(rosneuro_cybathlon)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  	roscpp
  	std_msgs
	rosneuro_msgs
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	std_msgs
	rosneuro_msgs

  DEPENDS
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS} 
					include
)

add_library(${PROJECT_NAME}
	src/Control.cpp
	src/udp_client_server.cpp
)

target_link_libraries(${PROJECT_NAME} 
					  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} 
				 ${catkin_EXPORTED_TARGETS}
)

#################
## Executables ##
#################
add_executable(control src/control.cpp)
target_link_libraries(control rosneuro_cybathlon)


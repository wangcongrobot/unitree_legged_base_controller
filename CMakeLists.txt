cmake_minimum_required(VERSION 2.8.3)
project(unitree_legged_base_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   # controller_manager
    roscpp
    tf
   
   geometry_msgs

   
   unitree_legged_msgs
    
)

find_package(Eigen3 REQUIRED)




catkin_package(

 )

include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ../unitree_legged_sdk/include
    ../aliengo_sdk/include
 
  
)

link_directories(
	../unitree_legged_sdk/lib 
	../aliengo_sdk/lib
)

#string(CONCAT LEGGED_SDK_NAME libunitree_legged_sdk_$ENV{UNITREE_PLATFORM}.so)
set(EXTRA_LIBS libunitree_legged_sdk_amd64.so libaliengo_comm.so lcm)

set(CMAKE_CXX_FLAGS "-O3")

add_executable(unitree_legged_base_controller src/unitree_legged_base_controller.cpp)
target_link_libraries(unitree_legged_base_controller ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(unitree_legged_base_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lcm_server_3_2 ../unitree_legged_sdk/examples/lcm_server.cpp)
target_link_libraries(lcm_server_3_2 ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(lcm_server_3_2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  ../unitree_legged_sdk/lib/libunitree_legged_sdk_amd64.so
  ../aliengo_sdk/lib/libaliengo_comm.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS  unitree_legged_base_controller lcm_server_3_2 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


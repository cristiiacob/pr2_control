cmake_minimum_required(VERSION 2.8.3)
project(pr2_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  kdl_conversions
  kdl_parser
  pr2_controllers_msgs
  pr2_mechanism_model
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(orocos_kdl REQUIRED)

link_directories(
 ${catkin_LIBRARY_DIRS}
 ${orocos_kdl_LIBRARY_DIRS}
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

add_executable(r_arm_controller src/${PROJECT_NAME}/r_arm_controller.cpp)
target_link_libraries(r_arm_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(l_arm_controller src/${PROJECT_NAME}/l_arm_controller.cpp)
target_link_libraries(l_arm_controller ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(r_arm_ik src/${PROJECT_NAME}/r_arm_ik.cpp)
target_link_libraries(r_arm_ik ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(r_arm_ik_jl src/${PROJECT_NAME}/r_arm_ik_jl.cpp)
target_link_libraries(r_arm_ik_jl ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(l_arm_ik_jl src/${PROJECT_NAME}/l_arm_ik_jl.cpp)
target_link_libraries(l_arm_ik_jl ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(l_arm_ik src/${PROJECT_NAME}/l_arm_ik.cpp)
target_link_libraries(l_arm_ik ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(r_arm_fk src/${PROJECT_NAME}/r_arm_fk.cpp)
target_link_libraries(r_arm_fk ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(l_arm_fk src/${PROJECT_NAME}/l_arm_fk.cpp)
target_link_libraries(l_arm_fk ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(pr2_keyboard_r src/${PROJECT_NAME}/pr2_keyboard_r.cpp)
target_link_libraries(pr2_keyboard_r ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(pr2_keyboard_l src/${PROJECT_NAME}/pr2_keyboard_l.cpp)
target_link_libraries(pr2_keyboard_l ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(client src/${PROJECT_NAME}/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

set(TEST_SRCS
   test/main.cpp
  #test/${PROJECT_NAME}/kdl_functions.cpp 
)

catkin_add_gtest(${PROJECT_NAME}-test ${TEST_SRCS}
		 test/main.cpp
		 #test/kdl_training/kdl_functions.cpp
		 WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test_data)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
endif()

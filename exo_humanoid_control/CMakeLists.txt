cmake_minimum_required(VERSION 3.5)

project(exo_humanoid_control)
set(CMAKE_CXX_STANDARD 17)
include_directories(${CMAKE_BINARY_DIR})
set(CMAKE_C_FLAGS "-O3  -ggdb  -march=native -std=gnu99 -I.")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-lboost_system -O3") 
# set(CMAKE_CXX_FLAGS "-lboost_system -fext-numeric-literals -O3") 
add_definitions(-DEIGEN_STACK_ALLOCATION_LIMIT=0)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    rospy
    gazebo_ros
    gazebo_msgs
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs  
    trac_ik_lib
)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)
find_package(orocos_kdl REQUIRED)

#find_package(qpoases)
catkin_package(
    CATKIN_DEPENDS 
    unitree_legged_msgs 
    trac_ik_lib
    DEPENDS
    Boost
    orocos_kdl
    pkg_nlopt
)

include_directories(
    # include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PINOCCHIO_INCLUDE_DIRS}
    #${GAZEBO_LIBRARY_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${pkg_nlopt_INCLUDE_DIRS}

    include
)

link_directories(
    include/sdk/lib
    ${GAZEBO_LIBRARY_DIRS}
)

add_subdirectory(third_party/qpOASES)
add_subdirectory(third_party/BS_thread_pool)

file(GLOB_RECURSE SRC_LIST
"src/*/*.cpp"
"src/*/*.h"
"MPC/*.cpp"
"WBC/*.cpp"
)
add_executable(exo_humanoid_ctrl src/main.cpp  ${SRC_LIST} )
target_link_libraries(exo_humanoid_ctrl 
    ${catkin_LIBRARIES} 
    ${PINOCCHIO_LIBRARIES} 
    ${Boost_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${pkg_nlopt_LIBRARIES}

    qpOASES_NEW 
    -pthread lcm

    Threads::Threads)

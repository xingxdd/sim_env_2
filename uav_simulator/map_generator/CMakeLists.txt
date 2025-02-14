cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  pcl_conversions
  message_generation
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  ObjectTpyeArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable (random_forest src/random_forest_sensing.cpp )
target_link_libraries(random_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest src/dynamic_forest_sensing.cpp)
target_link_libraries(dynamic_forest
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(dynamic_forest_seq src/dynamic_forest_seq_sensing.cpp)
target_link_libraries(dynamic_forest_seq
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

add_executable(talker src/talker.cpp)
target_link_libraries(talker
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})


# add_executable(test2 src/test2.cpp)
# target_link_libraries(test2
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES})  
    # add_executable(listener src/listener.cpp)
    # target_link_libraries(listener
    #   ${catkin_LIBRARIES}
    #   ${PCL_LIBRARIES})
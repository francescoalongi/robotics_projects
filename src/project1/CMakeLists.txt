cmake_minimum_required(VERSION 2.8.3)
project(project1)


find_package(catkin REQUIRED COMPONENTS
  roscpp 
  tf
  std_msgs
  message_generation
)

add_service_files(
    FILES
    DistanceComputer.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ecef_to_enu src/ecef_to_enu.cpp)
target_link_libraries(ecef_to_enu ${catkin_LIBRARIES})

add_executable(distance_computer src/distance_computer.cpp)
target_link_libraries(distance_computer ${catkin_LIBRARIES})
add_dependencies(distance_computer ${catkin_EXPORTED_TARGETS})

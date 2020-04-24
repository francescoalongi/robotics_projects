cmake_minimum_required(VERSION 2.8.3)
project(project1)


find_package(catkin REQUIRED COMPONENTS
  roscpp 
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ecef_to_enu src/ecef_to_enu.cpp)
target_link_libraries(ecef_to_enu ${catkin_LIBRARIES})


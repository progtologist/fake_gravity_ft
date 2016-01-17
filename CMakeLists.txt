cmake_minimum_required(VERSION 2.8.3)
project(fake_gravity_ft)
find_package(catkin REQUIRED COMPONENTS roscpp tf geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(fake_gravity_ft_node src/fake_gravity_ft_node.cpp)
target_link_libraries(fake_gravity_ft_node ${catkin_LIBRARIES})

install(TARGETS fake_gravity_ft_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

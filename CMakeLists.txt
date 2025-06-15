cmake_minimum_required(VERSION 3.0.2)
project(hiwin_robot_client_library)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hrsdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(hrsdk
  src/socket/tcp_client.cpp
  src/hiwin_driver.cpp
  src/commander.cpp
)

target_link_libraries(hrsdk ${catkin_LIBRARIES})

install(TARGETS hrsdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

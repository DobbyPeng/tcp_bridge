cmake_minimum_required(VERSION 2.8.3)
project(tcp_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(
  include ${catkin_INCLUDE_DIRS}
    install(DIRECTORY include/tcp_bridge/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
      PATTERN ".svn" EXCLUDE
    )
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(tcpServer src/tcpServer.cpp)
target_link_libraries(tcpServer ${catkin_LIBRARIES})

add_executable(tcpClient src/tcpClient.cpp)
target_link_libraries(tcpClient ${catkin_LIBRARIES})

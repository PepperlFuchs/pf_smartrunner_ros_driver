cmake_minimum_required(VERSION 2.8.3)
project(pepperl_fuchs_smartrunner)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    SET(OS "linux")
endif()

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64" OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64")
    SET(CPU "x64")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86")
    SET(CPU "x86")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm")
    SET(CPU "arm")
elseif (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64" or ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
    SET(CPU "arm64")
endif()

SET (VSX_SDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib/VsxSdk")
message(STATUS "${VSX_SDK_PATH}")
SET(VSX_SDK_LIB_PATH "${VSX_SDK_PATH}/C/lib/${OS}-${CPU}/")
message(STATUS "${VSX_SDK_LIB_PATH}")
SET (VSX_SDK_INCLUDE_PATH "${VSX_SDK_PATH}/C/include/")
message(STATUS "${VSX_SDK_INCLUDE_PATH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${VSX_SDK_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
)

add_executable(smartrunner_node
  src/rosnode/smartrunner_node.cpp
)

find_library(VSX_LIB PF.VsxProtocolDriver.WrapperNE ${VSX_SDK_LIB_PATH})

target_link_libraries(smartrunner_node
  ${catkin_LIBRARIES}
  ${VSX_LIB}
)
install(TARGETS smartrunner_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES launch/smartrunner.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES rviz/PointCloud.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(FILES rviz/PointCloud2.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

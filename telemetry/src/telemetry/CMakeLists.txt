cmake_minimum_required(VERSION 2.8.11)
project(telemetry)

## Set @result to list of all @curdir subdirectories which contain <include> folder and set them to @result
MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child}/include)
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

set(CMAKE_CXX_FLAGS "-std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_runtime message_generation)

## Declare ROS messages and services
add_message_files(FILES Telemetry_data.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Set trikRuntime project directory
set(TRIK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../)

## Find all trikRuntime subdirectories which contain <include> folder
SUBDIRLIST(TRIK_SUBDIRS ${TRIK_DIR})

FOREACH(subdir ${TRIK_SUBDIRS})
  LIST(APPEND TRIK_INCLUDE_DIRS ${TRIK_DIR}/${subdir}/include)
ENDFOREACH()

## Set Qt directory
SET(QT_DIR /opt/trik-sdk/sysroots/arm926ejse-oe-linux-gnueabi/usr/include/qt5/)

## Find all Qt subdirectories
FILE(GLOB children RELATIVE ${QT_DIR} ${QT_DIR}/*)
SET(QT_INCLUDE_DIRS "")
FOREACH(child ${children})
  LIST(APPEND QT_INCLUDE_DIRS ${QT_DIR}/${child})
ENDFOREACH()

include_directories(include 
	${catkin_INCLUDE_DIRS} 
	${TRIK_INCLUDE_DIRS}
	${QT_DIR}
	${QT_INCLUDE_DIRS}
)

link_directories(${TRIK_DIR}/bin/arm-release/)

add_executable(telemetry_daemon src/telemetry_daemon.cpp)
target_link_libraries(telemetry_daemon ${catkin_LIBRARIES} trikControl Qt5Core)
add_dependencies(telemetry_daemon telemetry_generate_messages_cpp)

add_executable(telemetry_server src/telemetry_server.cpp)
target_link_libraries(telemetry_server ${catkin_LIBRARIES})
add_dependencies(telemetry_server telemetry_generate_messages_cpp)

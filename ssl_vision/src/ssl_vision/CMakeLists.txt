cmake_minimum_required(VERSION 2.8.3)
project(ssl_vision)

set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

ADD_SUBDIRECTORY(src/protobuf)

catkin_package()
set(CMAKE_AUTOMOC ON)
message("CMAKE_CROSSCOMPILING " ${CMAKE_CROSSCOMPILING})

if(${CMAKE_CROSSCOMPILING})
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

    ## Set trikRuntime project directory
    set(TRIK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../)

    ## Find all trikRuntime subdirectories which contain <include> folder
    SUBDIRLIST(TRIK_SUBDIRS ${TRIK_DIR})

    FOREACH(subdir ${TRIK_SUBDIRS})
        LIST(APPEND TRIK_INCLUDE_DIRS ${TRIK_DIR}/${subdir}/include)
    ENDFOREACH()

    ## Set Qt directory
    SET(QT_DIR ${TRIK_SDK_ROOT}/sysroots/arm926ejse-oe-linux-gnueabi/usr/include/qt5/)

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
            ${PROTOBUF_INCLUDE_DIRS}
            src/staticObjectsPlugin
            ${CMAKE_BINARY_DIR}/ssl_vision/src/protobuf
            src)

    link_directories(${TRIK_DIR}/bin/arm-release/)

    add_executable(ssl_listener
            src/sslClient.cpp)

    target_link_libraries(ssl_listener
            Qt5Core
            ${catkin_LIBRARIES}
            trikControl
            proto
            ${PROTOBUF_LIBRARY})

else(${CMAKE_CROSSCOMPILING})
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    find_package(Qt5Core)
    find_package(Qt5Network)
    find_package(Qt5Widgets)

    include_directories(include
            ${Qt5Core_INCLUDE_DIRS}
            ${Qt5Network_INCLUDE_DIRS}
            ${Qt5Widgets_INCLUDE_DIRS}
            ${catkin_INCLUDE_DIRS}
            ${PROTOBUF_INCLUDE_DIRS}
            src/staticObjectsPlugin
            ${CMAKE_BINARY_DIR}/ssl_vision/src/protobuf
            src)

    add_executable(sslServer src/sslServer.cpp)
    target_link_libraries(sslServer
            ${catkin_LIBRARIES}
            Qt5::Widgets)

    add_executable(sslClient
            src/sslClient.cpp)
    target_link_libraries(sslClient
            ${catkin_LIBRARIES}
            ${QT_LIBRARIES}
            ${PROTOBUF_LIBRARY}
            proto
            Qt5::Core)

    add_executable(staticObjectsServer
            src/staticObjectsPlugin/staticObjectsServer.cpp
            src/staticObjectsPlugin/main.cpp)
    target_link_libraries(staticObjectsServer
            ${catkin_LIBRARIES}
            ${QT_LIBRARIES}
            Qt5::Core
            Qt5::Network
            Qt5::Widgets)
endif(${CMAKE_CROSSCOMPILING})
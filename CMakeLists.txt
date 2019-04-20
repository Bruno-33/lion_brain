cmake_minimum_required(VERSION 2.8.3)
project(lion_brain)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    FILES
    rc.msg
    target_position.msg
    virtual_rc.msg
    chassis_control.msg
)
  
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
    INCLUDE_DIRS 
        inc
        inc/core
    CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        message_runtime
    DEPENDS
        ${LIBS}
)

include_directories(
    inc
    inc/core
    ${catkin_INCLUDE_DIRS}
)

add_executable(lion_brain 
    src/core/lion_protocol.cpp
    src/core/lion_serial_impl.cpp
    src/core/lion_serial.cpp
    src/core/lion_thread.cpp
    src/core/lion_control.cpp
    src/lion_brain_node.cpp 
    src/lion_brain_publisher.cpp
    src/lion_brain_subscriber.cpp
    src/lion_brain.cpp
    src/main.cpp)

target_link_libraries(lion_brain 
    ${catkin_LIBRARIES}
)





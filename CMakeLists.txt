cmake_minimum_required(VERSION 3.0.2)
project(ros_bmp280)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
)

add_message_files(
    FILES
    Barometer.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_subdirectory(lib/raspberry-bmp280)

add_executable(bmp280_node src/bmp280_node.cpp)
target_link_libraries(bmp280_node raspberry_bmp280 ${catkin_LIBRARIES})

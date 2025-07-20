cmake_minimum_required(VERSION 3.5)
project(scalpelspace_momentum_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
        include
        include/scalpelspace_momentum_ros/momentum_driver
)

add_executable(momentum_can_node
        src/momentum_can_node.cpp
        src/momentum_driver/momentum_can_dbc.c
        src/momentum_driver/momentum_can_driver.c
        src/momentum_driver/momentum_spi_driver.c
)

ament_target_dependencies(momentum_can_node
        rclcpp
        can_msgs
        std_msgs
)

install(TARGETS momentum_can_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()

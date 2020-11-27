cmake_minimum_required(VERSION 2.8.3)
project(fastsense_msgs)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        )

add_message_files(
        FILES
        mover_cmd.msg
        mover_status.msg
        polar_map.msg
        marker_detector_msg.msg
        object_msg.msg
        clear_area.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime
)




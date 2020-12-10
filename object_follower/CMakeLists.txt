cmake_minimum_required(VERSION 3.0.2)
project(object_follower LANGUAGES CXX)

add_definitions(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  actionlib
  move_base_msgs
  dynamic_reconfigure
)

find_package(rostest REQUIRED)

generate_dynamic_reconfigure_options(cfg/ObjectFollower.cfg)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_follower
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include/object_follower
  ${catkin_INCLUDE_DIRS}
)

 add_library(object_follower_lib 

   src/object_follower/ObjectFollowerCore.cpp

   src/object_follower/GoalChecker.cpp
   src/object_follower/GoalGenerator.cpp
   src/object_follower/GoalGeneratorBaselink2D.cpp
   src/object_follower/GoalGeneratorNearest2D.cpp
   src/object_follower/GoalPublisherTf.cpp

   src/object_follower/Converter.cpp
   )

add_executable(object_follower_tf_node src/object_follower/ObjectFollowerMain.cpp )
# add_executable(object_follower_move_base_node src/object_follower/objf_move_base_main.cpp )

add_dependencies(object_follower_tf_node ${PROJECT_NAME}_gencfg)
# add_dependencies(object_follower_move_base_node ${PROJECT_NAME}_gencfg)

target_link_libraries(object_follower_tf_node
  object_follower_lib
  ${catkin_LIBRARIES}
)

# target_link_libraries(object_follower_move_base_node
#   object_follower_lib
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
add_executable(test_tf_publisher_node test/tf_publisher_node.cpp)
target_link_libraries(test_tf_publisher_node
  ${catkin_LIBRARIES}
)

add_rostest_gtest(object_follower_test_x test/launch/object_follower_forward_x.test test/object_follower_test.cpp)
add_rostest_gtest(object_follower_test_z test/launch/object_follower_forward_z.test test/object_follower_test.cpp)

target_link_libraries(object_follower_test_x
  object_follower_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(object_follower_test_z
  object_follower_lib
  ${catkin_LIBRARIES}
)

add_dependencies(object_follower_test_x ${PROJECT_NAME}_gencfg)
add_dependencies(object_follower_test_z ${PROJECT_NAME}_gencfg)

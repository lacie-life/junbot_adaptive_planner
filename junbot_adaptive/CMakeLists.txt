cmake_minimum_required(VERSION 3.0.2)
project(junbot_adaptive)

option(TEST_BUILD "Build test" ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        tf
        roscpp
        rospy
        std_msgs
        genmsg
        move_base_msgs
        custom_msgs
        geometry_msgs
        vision_msgs
        )

find_package(jsoncpp REQUIRED)

## Declare a catkin package
catkin_package(
        CATKIN_DEPENDS custom_msgs geometry_msgs)

## Build talker and listener
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${jsoncpp_INCLUDE_DIRS}
)

add_executable(junbot_planner
        src/junbot_planner.cpp
        src/SemanticPlanner.cpp
        )
target_link_libraries(junbot_planner ${catkin_LIBRARIES} -ljsoncpp)
add_dependencies(junbot_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

################################################################################
# Test
################################################################################

add_executable(mission_control test/mission_control.cpp src/SemanticPlanner.cpp)
target_link_libraries(mission_control ${catkin_LIBRARIES} -ljsoncpp)
add_dependencies(mission_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(add_layer test/add_layer.cpp src/SemanticPlanner.cpp)
target_link_libraries(add_layer ${catkin_LIBRARIES} -ljsoncpp)
add_dependencies(add_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(subGridMap test/subGridMap.cpp)
target_link_libraries(subGridMap ${catkin_LIBRARIES})
add_dependencies(subGridMap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(adaptiveControl test/adaptiveControl.cpp src/SemanticPlanner.cpp)
target_link_libraries(adaptiveControl ${catkin_LIBRARIES} -ljsoncpp)
add_dependencies(adaptiveControl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(objectLayer test/ObjectLayer.cpp)
target_link_libraries(objectLayer ${catkin_LIBRARIES})
add_dependencies(objectLayer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(subscribeTraj test/subscribeTraj.cpp)
target_link_libraries(subscribeTraj ${catkin_LIBRARIES})
add_dependencies(subscribeTraj ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


################################################################################
# Install
################################################################################
install(TARGETS junbot_planner
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS mission_control
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(TARGETS add_layer
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS subGridMap
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS adaptiveControl
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS objectLayer
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS subscribeTraj
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY bag config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

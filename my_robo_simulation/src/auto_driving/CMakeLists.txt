
find_package(catkin REQUIRED COMPONENTS
    sensor_msgs
    darknet_ros_msgs
    geometry_msgs
    message_generation
    my_robo_description
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS move_base my_robo_description
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(nakbot_auto_driving
    main.cpp
    System.cpp
    Goal.cpp
    Image.cpp
    Velocity.cpp
    Scan.cpp
)

target_link_libraries(nakbot_auto_driving
    ${catkin_LIBRARIES}
)
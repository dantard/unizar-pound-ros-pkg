cmake_minimum_required(VERSION 2.4.6)

project(ros_pound)

if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
        cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs roscpp_serialization)
find_package(Boost REQUIRED COMPONENTS thread)
catkin_package(CATKIN_DEPENDS roscpp roscpp_serialization roscpp_tutorials)

include_directories(include ${catkin_INCLUDE_DIRS})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration")
SET(CMAKE_CXX_FLAGS "-std=c++0x -O3")

add_executable (ros-pound
    ros-pound.cpp
)

target_link_libraries(ros-pound wrapper  z pthread rt pthread ${Boost_LIBRARIES}  ${catkin_LIBRARIES} )

add_custom_command(TARGET ros-pound
        POST_BUILD
        COMMAND echo  "Linking configuration files..."
        COMMAND touch ${CMAKE_SOURCE_DIR}/ros_pound/config
        COMMAND rm ${CMAKE_SOURCE_DIR}/ros_pound/config
        COMMAND ln -s ${CMAKE_SOURCE_DIR}/libwrapper/config ${CMAKE_SOURCE_DIR}/ros_pound/
)

add_custom_command(TARGET ros-pound
        POST_BUILD
        COMMAND echo "Applying sticky bit to the executable..."
        COMMAND echo ${CMAKE_BINARY_DIR}/../devel/lib/ros_pound/ros-pound
        COMMAND sudo chmod a+rwx ${CMAKE_BINARY_DIR}/../devel/lib/ros_pound/ros-pound
        COMMAND sudo chown root:root ${CMAKE_BINARY_DIR}/../devel/lib/ros_pound/ros-pound
        COMMAND sudo chmod +s ${CMAKE_BINARY_DIR}/../devel/lib/ros_pound/ros-pound
)

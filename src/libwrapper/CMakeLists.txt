cmake_minimum_required(VERSION 2.4.6)

project(libwrapper)

if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
        cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs topic_tools)
find_package(Boost REQUIRED COMPONENTS thread)

add_message_files(FILES
        Info.msg
)

add_service_files(FILES
        Manage.srv
)
generate_messages()


catkin_package(CATKIN_DEPENDS roscpp roscpp_serialization)
include_directories(include ${catkin_INCLUDE_DIRS})

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration")
SET(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(wrapper
    main.cpp
    include/ShapeShifterManager.h
    include/ShapeShifterMessage.h
    include/ShapeShifterListener.h
    include/ShapeShifterSender.h
    include/TopicManager.h
    interface/interface.h
    include/ServiceManager.h
    include/TFTopicManager.h
    include/QOSTopicManager.h
    include/Manager.h
    include/argon.h
    include/TimeSpec.h
    include/compressor.h
    config/config.h
    config/data_types.h
)
add_dependencies(wrapper ${PROJECT_NAME}_gencpp)

target_link_libraries(wrapper z pthread rt pthread yaml-cpp bz2 ${Boost_LIBRARIES}  ${catkin_LIBRARIES} )

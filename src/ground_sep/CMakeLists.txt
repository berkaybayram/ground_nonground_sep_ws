cmake_minimum_required(VERSION 3.8)
project(ground_sep)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
ament_auto_find_build_dependencies()

set(PCL_DIR "/home/berkay/libraries/libpcl-1.12.1/share/pcl-1.12/")
find_package(PCL 1.12.1 REQUIRED)

ament_auto_add_library(ground_sep_node SHARED
        include/ground_sep/ground_sep_node.hpp
        src/ground_sep_node.cpp
        )

include_directories(
        include
        SYSTEM
        ${PCL_COMMON_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#add_executable(ground_sep_node src/ground_sep_node.cpp)

target_link_libraries(ground_sep_node ${PCL_LIBRARIES})

ament_target_dependencies(ground_sep_node
        rclcpp
        std_msgs
        sensor_msgs)


target_include_directories(ground_sep_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(ground_sep_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS ground_sep_node
        DESTINATION lib/${PROJECT_NAME})



rclcpp_components_register_node(ground_sep_node
        PLUGIN "ground_sep::GroundSeparationNode"
        EXECUTABLE ground_sep_node_exe
        )

ament_auto_package(
        INSTALL_TO_SHARE
        launch
        param
)



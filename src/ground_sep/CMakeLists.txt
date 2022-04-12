cmake_minimum_required(VERSION 3.8)
project(ground_sep)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

#SET(PCL_DIR /home/berkay/libraries/libpcl-1.12.1/share/pcl-1.12/)
#find_package(ament_cmake_auto REQUIRED)
#find_package(PCL  REQUIRED)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
        include
        SYSTEM
        ${PCL_COMMON_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(ground_sep_node
        src/ground_sep_node.cpp)

target_link_libraries(ground_sep_node ${PCL_LIBRARIES})

ament_target_dependencies(ground_sep_node
        rclcpp
        std_msgs
        sensor_msgs
        PCL)


target_include_directories(ground_sep_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(ground_sep_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ground_sep_node
        DESTINATION lib/${PROJECT_NAME})


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()

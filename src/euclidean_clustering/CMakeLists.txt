cmake_minimum_required(VERSION 3.8)
project(euclidean_clustering)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(PCL_DIR "/home/berkay/libraries/libpcl-1.12.1/share/pcl-1.12/")
find_package(PCL 1.12.1 REQUIRED)

ament_auto_add_library(euclidean_clustering_node SHARED
        include/euclidean_clustering/euclidean_clustering_node.hpp
        src/euclidean_clustering_node.cpp
        )

include_directories(
        include
        SYSTEM
        ${PCL_COMMON_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable(euclidean_clustering_node src/euclidean_clustering_node.cpp)

target_link_libraries(euclidean_clustering_node ${PCL_LIBRARIES})


target_include_directories(euclidean_clustering_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(euclidean_clustering_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS euclidean_clustering_node
  DESTINATION lib/${PROJECT_NAME})

rclcpp_components_register_node(euclidean_clustering_node
        PLUGIN "EuclideanClustering::EuclideanClusteringNode"
        EXECUTABLE euclidean_clustering_node_exe
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
        INSTALL_TO_SHARE
#        launch
        param
)

//
// Created by berkay on 18.04.2022.
//

#ifndef GROUND_NONGROUND_SEP_WS_EUCLIDEAN_CLUSTERING_NODE_HPP
#define GROUND_NONGROUND_SEP_WS_EUCLIDEAN_CLUSTERING_NODE_HPP

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace EuclideanClustering {

class EuclideanClusteringNode : public rclcpp::Node {
public:
  EuclideanClusteringNode(const rclcpp::NodeOptions &node_options);


private:
  // Node
  std::string _name;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  void topic_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
};

} // namespace EuclideanClustering

#endif // GROUND_NONGROUND_SEP_WS_EUCLIDEAN_CLUSTERING_NODE_HPP

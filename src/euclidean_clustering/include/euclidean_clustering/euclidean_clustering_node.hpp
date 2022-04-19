//
// Created by berkay on 18.04.2022.
//

#ifndef GROUND_NONGROUND_SEP_WS_EUCLIDEAN_CLUSTERING_NODE_HPP
#define GROUND_NONGROUND_SEP_WS_EUCLIDEAN_CLUSTERING_NODE_HPP

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

namespace EuclideanClustering {

class EuclideanClusteringNode : public rclcpp::Node {
public:
  EuclideanClusteringNode(const rclcpp::NodeOptions &node_options);

private:
  // Node
  std::string name_;

  // Algorithm parameters
  float lx_, ly_, lz_;
  float tolerance_;
  int min_cluster_size_, max_cluster_size_;
  float cluster_intencity_increase_amount_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      cluster_publisher_;

  // Callbacks
  void topic_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
};

} // namespace EuclideanClustering

#endif // GROUND_NONGROUND_SEP_WS_EUCLIDEAN_CLUSTERING_NODE_HPP

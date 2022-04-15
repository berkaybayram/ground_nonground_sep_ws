//
// Created by berkay on 13.04.2022.
//

#ifndef GROUND_NONGROUND_SEP_WS_GROUND_SEP_NODE_HPP
#define GROUND_NONGROUND_SEP_WS_GROUND_SEP_NODE_HPP

#include <iostream>
#include <vector>

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ground_sep {

class GroundSeparationNode : public rclcpp::Node {
public:
  GroundSeparationNode(const rclcpp::NodeOptions &node_options);

private:
  // Node
  std::string _name;

  // Algorithm parameters
  int _max_iterations;
  double _distanceThreshold;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      nonground_publisher_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  void pointCloud2_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
};
} // namespace ground_sep

#endif // GROUND_NONGROUND_SEP_WS_GROUND_SEP_NODE_HPP

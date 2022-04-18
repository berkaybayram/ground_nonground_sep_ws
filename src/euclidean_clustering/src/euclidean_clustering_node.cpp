#include "euclidean_clustering/euclidean_clustering_node.hpp"

namespace EuclideanClustering {

EuclideanClusteringNode::EuclideanClusteringNode(
    const rclcpp::NodeOptions &node_options)
    : Node("EuclideanClusteringNode", node_options),
      _name(rclcpp::Node::get_name()) // Get node name
{

  auto callback = [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    this->topic_callback(msg);
  };

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/middle/velodyne_points", 10, callback);
}

void EuclideanClusteringNode::topic_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
  //  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  //  RCLCPP_INFO(this->get_logger(), "callback: loooool");

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  // Convert sensor msg PointCloud2 to PointCloud
  pcl::fromROSMsg(*msg, *cloud);
}

} // namespace EuclideanClustering

RCLCPP_COMPONENTS_REGISTER_NODE(EuclideanClustering::EuclideanClusteringNode)
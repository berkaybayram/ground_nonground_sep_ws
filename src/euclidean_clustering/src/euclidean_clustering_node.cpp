#include "euclidean_clustering/euclidean_clustering_node.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>

namespace EuclideanClustering {

EuclideanClusteringNode::EuclideanClusteringNode(
    const rclcpp::NodeOptions &node_options)
    : Node("EuclideanClusteringNode", node_options),
      name_(rclcpp::Node::get_name()), // Get node name
      cluster_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/clusters", 10)) // Init publisher
{

  // Get parameters
  this->declare_parameter<float>("lx", 0.0);
  this->get_parameter("lx", lx_);

  this->declare_parameter<float>("ly", 0.0);
  this->get_parameter("ly", ly_);

  this->declare_parameter<float>("lz", 0.0);
  this->get_parameter("lz", lz_);

  this->declare_parameter<float>("tolerance", 0.0);
  this->get_parameter("tolerance", tolerance_);

  this->declare_parameter<int>("min_cluster_size", 0);
  this->get_parameter("min_cluster_size", min_cluster_size_);

  this->declare_parameter<int>("max_cluster_size", 0);
  this->get_parameter("max_cluster_size", max_cluster_size_);

  this->declare_parameter<float>("cluster_intencity_increase_amount", 0.0);
  this->get_parameter("cluster_intencity_increase_amount",
                      cluster_intencity_increase_amount_);

//  RCLCPP_INFO(this->get_logger(),
//              "%f, %f, %f\n "
//              "%f\n "
//              "%d, %d\n "
//              "%f\n",
//              lx_, ly_, lz_, tolerance_, min_cluster_size_, max_cluster_size_,
//              cluster_intencity_increase_amount_);

  auto callback = [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    this->topic_callback(msg);
  };

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/non_ground", 10, callback);
}

void EuclideanClusteringNode::topic_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(
      new pcl::PointCloud<pcl::PointXYZI>);

  // Convert sensor msg PointCloud2 to PointCloud
  pcl::fromROSMsg(*msg, *cloud);

  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(lx_, ly_, lz_);
  vg.setDownsampleAllData(true);
  vg.filter(*cloud_out);

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_out);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

  ec.setClusterTolerance(tolerance_); // in meters
  ec.setMinClusterSize(min_cluster_size_);
  ec.setMaxClusterSize(max_cluster_size_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_out);
  ec.extract(cluster_indices);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(
      new pcl::PointCloud<pcl::PointXYZI>);

  float clusterIntencity = 0.0f;
  for (const auto &cluster_indice : cluster_indices) {
    for (const auto &idx : cluster_indice.indices) {
      pcl::PointXYZI p;
      pcl::copyPoint(cloud_out->points[idx], p);
      p.intensity = clusterIntencity;

      cloud_cluster->push_back((p)); //*
    }
    clusterIntencity += cluster_intencity_increase_amount_;
  }

  //   Publish ground points
  sensor_msgs::msg::PointCloud2 clusterMsg;
  pcl::toROSMsg(*cloud_cluster, clusterMsg);
  clusterMsg.header = msg->header;
  cluster_publisher_->publish(clusterMsg);
  //  RCLCPP_INFO(this->get_logger(), "%zu", cloud_cluster->size());
}

} // namespace EuclideanClustering

RCLCPP_COMPONENTS_REGISTER_NODE(EuclideanClustering::EuclideanClusteringNode)
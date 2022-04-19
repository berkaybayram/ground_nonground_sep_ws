#include "euclidean_clustering/euclidean_clustering_node.hpp"


#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace EuclideanClustering {

EuclideanClusteringNode::EuclideanClusteringNode(
    const rclcpp::NodeOptions &node_options)
    : Node("EuclideanClusteringNode", node_options),
      name_(rclcpp::Node::get_name()), // Get node name
      cluster_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/clusters", 10)) // Init publisher
{

  auto callback = [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    this->topic_callback(msg);
  };

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/non_ground", 10, callback);
}

void EuclideanClusteringNode::topic_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
  //  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  //    RCLCPP_INFO(this->get_logger(), "%zu", msg->data.size());

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f(
      new pcl::PointCloud<pcl::PointXYZI>);

  // Convert sensor msg PointCloud2 to PointCloud
  pcl::fromROSMsg(*msg, *cloud);

  // Create the filtering object: downsample the dataset using a leaf size of
  // 1cm
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  vg.setInputCloud(cloud);
  vg.setLeafSize(50.f, 50.f, 50.f);
  vg.filter(*cloud_filtered);
//  RCLCPP_INFO(this->get_logger(), "%zu, %zu", msg->data.size(),
//              cloud_filtered->size());

  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZI>());

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.02);
  int nr_points = (int)cloud_filtered->size();
  while (cloud_filtered->size() > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *cloud_filtered = *cloud_f;
  }
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.5); // 50cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);
  //  int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it)

  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &idx : it->indices) {
      cloud_cluster->push_back((*cloud_filtered)[idx]); //*
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //   Publish ground points
    sensor_msgs::msg::PointCloud2 clusterMsg;
    pcl::toROSMsg(*cloud_cluster, clusterMsg);
    clusterMsg.header = msg->header;
    cluster_publisher_->publish(clusterMsg);
    RCLCPP_INFO(this->get_logger(), "%zu", clusterMsg.data.size());
  }
}

} // namespace EuclideanClustering

RCLCPP_COMPONENTS_REGISTER_NODE(EuclideanClustering::EuclideanClusteringNode)
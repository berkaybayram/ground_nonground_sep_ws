#include "ground_sep/ground_sep_node.hpp"

namespace ground_sep {

GroundSeparationNode::GroundSeparationNode(
    const rclcpp::NodeOptions &node_options)
    : Node("ground_sep_node", node_options) {

  // Get node name
  _name = rclcpp::Node::get_name();

  // Get parameters
  this->declare_parameter<int>("max_iterations", 0.0);
  this->get_parameter("max_iterations", _max_iterations);

  this->declare_parameter<double>("distanceThreshold", 0.0);
  this->get_parameter("distanceThreshold", _distanceThreshold);


  // manually enable topic statistics via options
  //  auto options = rclcpp::SubscriptionOptions();
  //  options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;
  // configure the collection window and publish period (default 1s)
  //    options.topic_stats_options.publish_period = std::chrono::seconds(10);
  // configure the topic name (default '/statistics')
  // options.topic_stats_options.publish_topic = "/topic_statistics"

  auto callback = [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    this->pointCloud2_callback(msg);
  };

  ground_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground", 10);
  nonground_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/non_ground", 10);

  //  "/velodyne_points"
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/middle/velodyne_points", 10, callback);

  // Inform initialized
  RCLCPP_INFO(this->get_logger(), "%s: node initialized.", _name.c_str());
}

void GroundSeparationNode::pointCloud2_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr nonGround(
      new pcl::PointCloud<pcl::PointXYZI>);

  // Convert sensor msg PointCloud2 to PointCloud
  pcl::fromROSMsg(*msg, *cloud);

  // Ransac coefficients
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(_max_iterations);
  seg.setDistanceThreshold(_distanceThreshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  // Log error if no plane found
  if (inliers->indices.empty()) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
  }

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZI> extract;

  // Extract the inliers
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*ground);

  // Extract the outliers
  extract.setNegative(true);
  extract.filter(*nonGround);

  // Publish ground points
  sensor_msgs::msg::PointCloud2 groundPoints;
  pcl::toROSMsg(*ground, groundPoints);
  groundPoints.header = msg->header;
  ground_publisher_->publish(groundPoints);

  // Publish non-ground points
  sensor_msgs::msg::PointCloud2 nongroundPoints;
  pcl::toROSMsg(*nonGround, nongroundPoints);
  nongroundPoints.header = msg->header;
  nonground_publisher_->publish(nongroundPoints);
}

} // namespace ground_sep

RCLCPP_COMPONENTS_REGISTER_NODE(ground_sep::GroundSeparationNode)
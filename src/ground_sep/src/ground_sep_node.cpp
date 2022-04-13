#include "ground_sep/ground_sep_node.hpp"

namespace ground_sep {

GroundSeparationNode::GroundSeparationNode(
    const rclcpp::NodeOptions &node_options)
    : Node("Ground_Cluster", node_options) {

  // Get node name
  _name = rclcpp::Node::get_name();

  //    rclcpp::param::param<double>("~max_distance",_max_distance,0.005);
  //    ros::param::param<double>("~min_percentage",_min_percentage,5);
  //    ros::param::param<bool>("~color_pc_with_error",_color_pc_with_error,false);

  // manually enable topic statistics via options
  auto options = rclcpp::SubscriptionOptions();
  options.topic_stats_options.state = rclcpp::TopicStatisticsState::Disable;
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
      "/middle/velodyne_points", 10, callback, options);

  createColors();

  //    std::cout << PCL_VERSION << std::endl;

  // Inform initialized
  RCLCPP_INFO(this->get_logger(), "%s: node initialized.", _name.c_str());
}

void GroundSeparationNode::createColors() {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  for (int i = 0; i < 20; i++) {
    while (r < 70 && g < 70 && b < 70) {
      r = rand() % (255);
      g = rand() % (255);
      b = rand() % (255);
    }
    Color c(r, g, b);
    r = 0;
    g = 0;
    b = 0;
    colors.push_back(c);
  }
}

void GroundSeparationNode::pointCloud2_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
  /*TODO:
   *
   * read z treshold from yaml
   * ransac?
   *
   * sudo ldconfig
   * check out: pcl in kendi euclidian cluster
   * */

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(
  //        new pcl::PointCloud<pcl::PointXYZ>);
  //    pcl::PointCloud<pcl::PointXYZ>::Ptr nonGround(
  //        new pcl::PointCloud<pcl::PointXYZ>);
  //
  //    //    Convert sensor msg PointCloud2 to PointCloud
  pcl::fromROSMsg(*msg, *cloud);
  //
  //    //    Basic clustering w.r.t. point height
  //    for (auto point : cloud->points) {
  //      // std::cout << point.x << ", " << point.y << ", " << point.z <<
  //      "\n"; if (point.z > 0) {
  //        nonGround->points.push_back(point);
  //      } else {
  //        ground->points.push_back(point);
  //      }
  //    }
  //
  //    //    Convert PointCloud to sensor msg PointCloud2 and publish
  //    sensor_msgs::msg::PointCloud2::UniquePtr groundMSG(
  //        new sensor_msgs::msg::PointCloud2);
  //    pcl::toROSMsg(*ground, *groundMSG);
  //    groundMSG->header.frame_id = "map";
  //    ground_publisher_->publish(*groundMSG);
  //
  //    //    Convert PointCloud to sensor msg PointCloud2 and publish
  //    sensor_msgs::msg::PointCloud2::UniquePtr nonGroundMSG(
  //        new sensor_msgs::msg::PointCloud2);
  //    pcl::toROSMsg(*nonGround, *nonGroundMSG);
  //    nonGroundMSG->header.frame_id = "map";
  //    nonground_publisher_->publish(*nonGroundMSG);

  // Get segmentation ready
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(_max_distance);

  // Create pointcloud to publish inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  int original_size(cloud->height * cloud->width);
  int n_planes(0);
  while (cloud->height * cloud->width > original_size * _min_percentage / 100) {

    // Fit a plane
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Check result
    if (inliers->indices.size() == 0)
      break;

    // Iterate inliers
    double mean_error(0);
    double max_error(0);
    double min_error(100000);
    std::vector<double> err;
    for (int i = 0; i < inliers->indices.size(); i++) {

      // Get Point
      pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

      // Compute distance
      double d = point2planedistnace(pt, coefficients) * 1000; // mm
      err.push_back(d);

      // Update statistics
      mean_error += d;
      if (d > max_error)
        max_error = d;
      if (d < min_error)
        min_error = d;
    }
    mean_error /= inliers->indices.size();

    // Compute Standard deviation
    ColorMap cm(min_error, max_error);
    double sigma(0);
    for (int i = 0; i < inliers->indices.size(); i++) {

      sigma += pow(err[i] - mean_error, 2);

      // Get Point
      pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

      // Copy point to now cloud
      pcl::PointXYZRGB pt_color;
      pt_color.x = pt.x;
      pt_color.y = pt.y;
      pt_color.z = pt.z;
      uint32_t rgb;
      if (_color_pc_with_error) {
        rgb = cm.getColor(err[i]);
      } else {
        rgb = colors[n_planes].getColor();
        //          std::cout << "colors\n";
      }
      pt_color.rgb = *reinterpret_cast<float *>(&rgb);
      cloud_pub->points.push_back(pt_color);
    }
    sigma = sqrt(sigma / inliers->indices.size());

    // Extract inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    pcl::PointCloud<pcl::PointXYZ> cloudF;
    extract.filter(cloudF);
    cloud->swap(cloudF);

    //      // Display infor
    //      ROS_INFO("%s: fitted plane %i: %fx%s%fy%s%fz%s%f=0 (inliers:
    //      %zu/%i)",
    //               _name.c_str(),n_planes,
    //               coefficients->values[0],(coefficients->values[1]>=0?"+":""),
    //               coefficients->values[1],(coefficients->values[2]>=0?"+":""),
    //               coefficients->values[2],(coefficients->values[3]>=0?"+":""),
    //               coefficients->values[3],
    //               inliers->indices.size(),original_size);
    //      ROS_INFO("%s: mean error: %f(mm), standard deviation: %f (mm), max
    //      error: %f(mm)",_name.c_str(),mean_error,sigma,max_error);
    //      ROS_INFO("%s: poitns left in cloud
    //      %i",_name.c_str(),cloud->width*cloud->height);

    // Nest iteration
    n_planes++;
  }

  // Publish points
  sensor_msgs::msg::PointCloud2 cloud_publish;
  pcl::toROSMsg(*cloud_pub, cloud_publish);
  //    cloud_publish.header = msg->header;
  cloud_publish.header.frame_id = "map";

  ground_publisher_->publish(cloud_publish);
}
//
// int main(int argc, char *argv[]) {
//  rclcpp::init(argc, argv);
//  rclcpp::spin(std::make_shared<GroundSeparationNode>());
//  rclcpp::shutdown();
//  return 0;
//}

} // namespace ground_sep

RCLCPP_COMPONENTS_REGISTER_NODE(ground_sep::GroundSeparationNode)
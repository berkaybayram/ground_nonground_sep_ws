#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
//#include "std_msgs/msg/string.hpp"

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/point_types.h>
//#include <pcl/conversions.h>sudo ldconfig
//#include <pcl/pcl_config.h>

#include <pcl_conversions/pcl_conversions.h>

class MinimalSubscriberWithTopicStatistics : public rclcpp::Node {
public:
  MinimalSubscriberWithTopicStatistics()
      : Node("minimal_subscriber_with_topic_statistics") {

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
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/non_ground",
                                                              10);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10, callback, options);

    std::cout << PCL_VERSION << std::endl;
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      nonground_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  void pointCloud2_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
    /*TODO:
     *
     * read z treshold from yaml
     * ransac?
     *
     * sudo ldconfig
     * check out: pcl in kendi euclidian cluster
     * */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nonGround(
        new pcl::PointCloud<pcl::PointXYZ>);

    //    Convert sensor msg PointCloud2 to PointCloud
    pcl::fromROSMsg(*msg, *cloud);

    //    Basic clustering w.r.t. point height
    for (auto point : cloud->points) {
      // std::cout << point.x << ", " << point.y << ", " << point.z << "\n";
      if (point.z > 0) {
        nonGround->points.push_back(point);
      } else {
        ground->points.push_back(point);
      }
    }

    //    Convert PointCloud to sensor msg PointCloud2 and publish
    sensor_msgs::msg::PointCloud2::UniquePtr groundMSG(
        new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*ground, *groundMSG);
    groundMSG->header.frame_id = "map";
    ground_publisher_->publish(*groundMSG);

    //    Convert PointCloud to sensor msg PointCloud2 and publish
    sensor_msgs::msg::PointCloud2::UniquePtr nonGroundMSG(
        new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*nonGround, *nonGroundMSG);
    nonGroundMSG->header.frame_id = "map";
    nonground_publisher_->publish(*nonGroundMSG);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithTopicStatistics>());
  rclcpp::shutdown();
  return 0;
}
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"



class MinimalSubscriberWithTopicStatistics : public rclcpp::Node {
public:
  MinimalSubscriberWithTopicStatistics()
      : Node("minimal_subscriber_with_topic_statistics") {

    // manually enable topic statistics via options
    auto options = rclcpp::SubscriptionOptions();
    options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
    // configure the collection window and publish period (default 1s)
    options.topic_stats_options.publish_period = std::chrono::seconds(10);
    // configure the topic name (default '/statistics')
    // options.topic_stats_options.publish_topic = "/topic_statistics"

    auto callback = [this](sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) {
      this->topic_callback(point_cloud2_msgs);
    };

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10, callback, options);
  }

private:
  //  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud2_msgs) const {
    //    auto lastIndex = point_cloud2_msgs->data.size()-2;
    //    auto pcSize =
    //    std::to_string(point_cloud2_msgs->data[lastIndex]).c_str();

    //    std::string buff = "";
    //    for (const auto d: point_cloud2_msgs->data)
    //    {
    //      buff += std::to_string(d) + "\n";
    //    }

    auto height = std::to_string(point_cloud2_msgs->height).c_str();
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", height);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithTopicStatistics>());
  rclcpp::shutdown();
  return 0;
}
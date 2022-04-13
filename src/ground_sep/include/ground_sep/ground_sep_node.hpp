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
//#include "std_msgs/msg/string.hpp"

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/point_types.h>
//#include <pcl/conversions.h>sudo ldconfig
//#include <pcl/pcl_config.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

double point2planedistnace(pcl::PointXYZ pt,
                           pcl::ModelCoefficients::Ptr coefficients) {
  double f1 =
      fabs(coefficients->values[0] * pt.x + coefficients->values[1] * pt.y +
           coefficients->values[2] * pt.z + coefficients->values[3]);
  double f2 =
      sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) +
           pow(coefficients->values[2], 2));
  return f1 / f2;
}

class ColorMap {
public:
  ColorMap(double mn, double mx) : mn(mn), mx(mx) {}
  void setMinMax(double min, double max) {
    mn = min;
    mx = max;
  }
  void setMin(double min) { mn = min; }
  void setMax(double max) { mx = max; }
  void getColor(double c, uint8_t &R, uint8_t &G, uint8_t &B) {
    double normalized = (c - mn) / (mx - mn) * 2 - 1;
    R = (int)(base(normalized - 0.5) * 255);
    G = (int)(base(normalized) * 255);
    B = (int)(base(normalized + 0.5) * 255);
  }
  void getColor(double c, double &rd, double &gd, double &bd) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    getColor(c, r, g, b);
    rd = (double)r / 255;
    gd = (double)g / 255;
    bd = (double)b / 255;
  }
  uint32_t getColor(double c) {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    getColor(c, r, g, b);
    return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  }

private:
  double interpolate(double val, double y0, double x0, double y1, double x1) {
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
  }
  double base(double val) {
    if (val <= -0.75)
      return 0;
    else if (val <= -0.25)
      return interpolate(val, 0, -0.75, 1, -0.25);
    else if (val <= 0.25)
      return 1;
    else if (val <= 0.75)
      return interpolate(val, 1.0, 0.25, 0.0, 0.75);
    else
      return 0;
  }

private:
  double mn, mx;
};

class Color {
private:
  uint8_t r;
  uint8_t g;
  uint8_t b;

public:
  Color(uint8_t R, uint8_t G, uint8_t B) : r(R), g(G), b(B) {}

  void getColor(uint8_t &R, uint8_t &G, uint8_t &B) {
    R = r;
    G = g;
    B = b;
  }
  void getColor(double &rd, double &gd, double &bd) {
    rd = (double)r / 255;
    gd = (double)g / 255;
    bd = (double)b / 255;
  }
  uint32_t getColor() const {
    return ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  }
};

namespace ground_sep {

class GroundSeparationNode : public rclcpp::Node {
public:
  GroundSeparationNode(const rclcpp::NodeOptions &node_options);

private:
  // Node
  std::string _name;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      nonground_publisher_;

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  // Algorithm parameters
  double _min_percentage = 50;
  double _max_distance = 0.05;
  bool _color_pc_with_error = false;
//  double _min_percentage;
//  double _max_distance;
//  bool _color_pc_with_error;

  std::vector<Color> colors;

  void createColors();
  void pointCloud2_callback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
};
} //namespace ground_sep

#endif // GROUND_NONGROUND_SEP_WS_GROUND_SEP_NODE_HPP

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <rclcpp/rclcpp.hpp>

#include <ros_tools/base_configuration.h>

#include <string>
#include <vector>
#include "helpers.h"

class RoadmapConfig : public RosTools::BaseConfiguration
{

  /**
   * @brief Class for retrieving configuration parameters
   *
   */

public:
  RoadmapConfig();
  ~RoadmapConfig();

  /**
   * @brief intialize:  check parameters on parameter server and read from there
   * @param node_handle_name: node handler initialize from name, as parameter set inside that name
   * @return true all parameter initialize successfully else false
   */
  bool initialize(rclcpp::Node::SharedPtr node);

  // High-level Parameters
  bool debug_output_;

  std::string map_package_name_;
  std::string map_file_name_;

  double update_frequency_;

  // Spline settings
  bool fit_clothoid_;

  double spline_sample_distance_;
  double minimum_waypoint_distance_;
  double clothoid_point_per_xm_;

  // Visual settings
  double scale_;

  // Topics
  std::string external_waypoint_topic_;

  bool success_;

  double autoware_update_interval_;
  double autoware_forward_distance_, autoware_backward_distance_;
};

#endif

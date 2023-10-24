
#include <configuration.h>

RoadmapConfig::RoadmapConfig()
{

  success_ = false;
}

RoadmapConfig::~RoadmapConfig()
{
}

// read predicitve configuration paramter from paramter server
bool RoadmapConfig::initialize(rclcpp::Node::SharedPtr node) // const std::string& node_handle_name
{
  retrieveParameter(node, "roadmap.debug_output", debug_output_, true);
  retrieveParameter(node, "roadmap.map_package_name", map_package_name_);
  retrieveParameter(node, "roadmap.map_file_name", map_file_name_);
  retrieveParameter(node, "roadmap.update_frequency", update_frequency_, 10.);

  retrieveParameter(node, "roadmap.spline.fit_clothoid", fit_clothoid_);
  retrieveParameter(node, "roadmap.spline.spline_sample_distance", spline_sample_distance_);
  retrieveParameter(node, "roadmap.spline.minimum_waypoint_distance", minimum_waypoint_distance_);
  retrieveParameter(node, "roadmap.spline.clothoid_point_per_xm", clothoid_point_per_xm_);

  retrieveParameter(node, "roadmap.scale", scale_);

  retrieveParameter(node, "roadmap.external_waypoint_topic", external_waypoint_topic_, std::string());

  success_ = true;

  if (debug_output_)
    RCLCPP_INFO(node->get_logger().get_child("configuration"), "Initialized");

  return true;
}

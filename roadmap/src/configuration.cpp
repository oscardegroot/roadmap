
#include <configuration.h>

RoadmapConfig::RoadmapConfig()
{

  success_ = false;
}

RoadmapConfig::~RoadmapConfig()
{
}

// read predicitve configuration paramter from paramter server
bool RoadmapConfig::initialize() //const std::string& node_handle_name
{
  ros::NodeHandle nh;

  retrieveParameter(nh, "debug_output", debug_output_, true);
  retrieveParameter(nh, "map_file_name", map_file_name_);
  retrieveParameter(nh, "update_frequency", update_frequency_, 10.);

  retrieveParameter(nh, "fit_clothoid", fit_clothoid_);
  retrieveParameter(nh, "spline_sample_distance", spline_sample_distance_);
  retrieveParameter(nh, "minimum_waypoint_distance", minimum_waypoint_distance_);
  retrieveParameter(nh, "clothoid_point_per_xm", clothoid_point_per_xm_);

  retrieveParameter(nh, "scale", scale_);

  success_ = true;

  if (debug_output_)
    ROS_WARN("[Roadmap]: Configuration initialized");

  return true;
}

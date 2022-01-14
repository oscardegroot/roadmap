#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include "helpers.h"

class RoadmapConfig
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
  bool initialize();

  // Parameters
  bool debug_output_;
  bool fit_clothoid_;

  // Spline settings
  double spline_sample_distance_;
  double minimum_waypoint_distance_;
  double clothoid_point_per_xm_;

  bool success_;

private:
  /* Retrieve paramater, if it doesn't exist return false */
  template <class T>
  bool
  retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value)
  {

    if (!nh.getParam(name, value))
    {
      ROS_WARN_STREAM(" Parameter " << name << " not set on node " << ros::this_node::getName().c_str());
      return false;
    }
    else
    {
      return true;
    }
  }

  /* Retrieve parameter, if it doesn't exist use the default */
  template <class T>
  void retrieveParameter(const ros::NodeHandle &nh, const std::string &name, T &value, const T &default_value)
  {

    if (!retrieveParameter(nh, name, value))
    {
      ROS_WARN_STREAM(" Setting " << name << " to default value: " << default_value);
      value = default_value;
    }
  }
};

#endif

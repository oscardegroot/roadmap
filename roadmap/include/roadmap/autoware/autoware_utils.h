#ifndef __AUTOWARE_UTILS_H__
#define __AUTOWARE_UTILS_H__

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <nav_msgs/msg/path.hpp>

using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;


inline nav_msgs::msg::Path ConvertAutowarePathToNavPath(Path::SharedPtr msg)
{
    nav_msgs::msg::Path nav_path;
    nav_path.header = msg->header;
    nav_path.poses.resize(msg->points.size());
    for (size_t i = 0; i < msg->points.size(); i++)
    {
        nav_path.poses[i].pose = msg->points[i].pose;
    }

    return nav_path;
}

inline nav_msgs::msg::Path ConvertAutowarePathWithLaneIdToNavPath(PathWithLaneId msg)
{
    nav_msgs::msg::Path nav_path;
    nav_path.header = msg.header;
    nav_path.poses.resize(msg.points.size());
    for (size_t i = 0; i < msg.points.size(); i++)
    {
        nav_path.poses[i].pose = msg.points[i].point.pose;
    }

    return nav_path;
}
#endif
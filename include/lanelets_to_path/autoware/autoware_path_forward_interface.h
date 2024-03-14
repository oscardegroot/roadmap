#ifndef __AUTOWARE_PATH_FORWARD_INTERFACE_H__
#define __AUTOWARE_PATH_FORWARD_INTERFACE_H__

#include <lanelets_to_path/autoware/autoware_utils.h>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <lanelets_to_path/roadmap.h>

#include <ros_tools/ros_visuals.h>

#include <memory>

using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;

using std::placeholders::_1;

/** @brief Loads the reference path from an autoware path */
class AutowarePathForwardInterface
{

public:
    AutowarePathForwardInterface(std::shared_ptr<Roadmap> roadmap) : roadmap_ptr_(roadmap)
    {
        waypoints_sub_ = roadmap->create_subscription<autoware_auto_planning_msgs::msg::Path>(
            "/planning/scenario_planning/lane_driving/behavior_planning/path", 1,
            std::bind(&AutowarePathForwardInterface::WaypointCallback, this, _1)); // Subscriber for waypoints (forwarded to the reader)
    }

    void WaypointCallback(autoware_auto_planning_msgs::msg::Path::SharedPtr msg)
    {
        // When we receive an autoware path, convert it to nav_msgs and read it
        roadmap_ptr_->WaypointCallback(ConvertAutowarePathToNavPath(msg));
    }

private:
    std::shared_ptr<Roadmap> roadmap_ptr_;

    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr waypoints_sub_; /** Subscriber for external waypoints */
};
#endif
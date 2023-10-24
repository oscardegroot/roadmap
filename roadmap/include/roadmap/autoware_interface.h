#ifndef __AUTOWARE_INTERFACE_H__
#define __AUTOWARE_INTERFACE_H__

#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <roadmap.h>

#include <memory>

class AutowareInterface
{

public:
    AutowareInterface(std::shared_ptr<Roadmap> roadmap) : roadmap_ptr_(roadmap)
    {
        waypoints_sub_ = roadmap->create_subscription<autoware_auto_planning_msgs::msg::Path>(
            "/planning/scenario_planning/lane_driving/behavior_planning/path", 1,
            std::bind(&AutowareInterface::WaypointCallback, this, std::placeholders::_1)); // Subscriber for waypoints (forwarded to the reader)
    }

    void WaypointCallback(autoware_auto_planning_msgs::msg::Path::SharedPtr msg)
    {
        // When we receive an autoware path, convert it to nav_msgs and read it
        roadmap_ptr_->WaypointCallback(ConvertAutowarePathToNavPath(msg));
    }

private:
    std::shared_ptr<Roadmap> roadmap_ptr_;

    rclcpp::Subscription<autoware_auto_planning_msgs::msg::Path>::SharedPtr waypoints_sub_; /** Subscriber for external waypoints */

    nav_msgs::msg::Path ConvertAutowarePathToNavPath(autoware_auto_planning_msgs::msg::Path::SharedPtr msg)
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
};

#endif
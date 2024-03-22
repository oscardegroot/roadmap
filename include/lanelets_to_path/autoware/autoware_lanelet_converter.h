#ifndef __AUTOWARE_LANELET_CONVERTER_INTERFACE_H__
#define __AUTOWARE_LANELET_CONVERTER_INTERFACE_H__

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <Eigen/Dense>
#include <memory>

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;

using std::placeholders::_1;

namespace lanelet
{
    class ConstLanelet;

}
namespace route_handler
{
    class RouteHandler;
}

class RoadmapConfig;

/** @brief The lanelet converter is an autoware interface that loads the reference path from
 * the map and the mission planner's route */
class AutowareLaneletConverter : public rclcpp::Node
{
public:
    AutowareLaneletConverter();
    void initialize();
    // Trigger a map recompute when respawning
    void onInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg);
    void onOdometry(Odometry::ConstSharedPtr msg);
    void onMap(HADMapBin::ConstSharedPtr msg);
    void onRoute(LaneletRoute::ConstSharedPtr msg);
    void onGoal(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    void WaypointCallback(const nav_msgs::msg::Path &msg);

private:
    void AddRoadBoundaries(PathWithLaneId &path, const std::vector<lanelet::ConstLanelet> &lanelet_sequence);

    std::shared_ptr<RoadmapConfig> config_ptr_;

    Eigen::Vector2d goal_;

    rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_; /** Subscriber for external waypoints */
    HADMapBin::ConstSharedPtr map_ptr_{nullptr};

    rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscriber_; /** Subscriber for external waypoints */
    LaneletRoute::ConstSharedPtr route_ptr_{nullptr};

    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscriber_; /** Subscriber for external waypoints */
    Odometry::ConstSharedPtr odometry_ptr_{nullptr};

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber_; /** Subscriber for external waypoints */

    rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initial_pose_subscriber_; /** Subscriber for external waypoints */

    rclcpp::Publisher<PathWithLaneId>::SharedPtr autoware_path_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;

    std::shared_ptr<route_handler::RouteHandler> route_handler_;
};
#endif
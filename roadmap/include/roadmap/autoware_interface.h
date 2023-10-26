#ifndef __AUTOWARE_INTERFACE_H__
#define __AUTOWARE_INTERFACE_H__

// For lanelets
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <route_handler/route_handler.hpp>
#include <lanelet2_extension/utility/query.hpp> // See route-handler

#include <roadmap/roadmap.h>

#include <memory>

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletRoute;
using nav_msgs::msg::Odometry;
using route_handler::RouteHandler;

using std::placeholders::_1;

namespace
{
    rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node *node_ptr)
    {
        rclcpp::CallbackGroup::SharedPtr callback_group =
            node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group;

        return sub_opt;
    }
} // namespace

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

/** @brief Loads the reference path from an autoware path */
class AutowarePathForwardInterface
{

public:
    AutowarePathForwardInterface(std::shared_ptr<Roadmap> roadmap) : roadmap_ptr_(roadmap)
    {
        waypoints_sub_ = roadmap->create_subscription<autoware_auto_planning_msgs::msg::Path>(
            "/planning/scenario_planning/lane_driving/behavior_planning/path", 1,
            std::bind(&AutowarePathForwardInterface::WaypointCallback, this, std::placeholders::_1)); // Subscriber for waypoints (forwarded to the reader)
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

/** @brief The lanelet converter is an autoware interface that loads the reference path from
 * the map and the mission planner's route */
class AutowareLaneletConverter
{
public:
    AutowareLaneletConverter(std::shared_ptr<Roadmap> roadmap) : roadmap_ptr_(roadmap),
                                                                 logger_(roadmap->get_logger().get_child("lanelet_converter"))
    {
        RCLCPP_INFO(logger_, "Initializing");
        auto qos_transient_local = rclcpp::QoS{1}.transient_local();

        // The map of the world
        vector_map_subscriber_ = roadmap->create_subscription<HADMapBin>(
            "~/input/vector_map", qos_transient_local, std::bind(&AutowareLaneletConverter::onMap, this, _1), createSubscriptionOptions(roadmap_ptr_.get()));

        // The route to be followed on the map
        route_subscriber_ = roadmap->create_subscription<LaneletRoute>(
            "~/input/route", 1, std::bind(&AutowareLaneletConverter::onRoute, this, _1), createSubscriptionOptions(roadmap_ptr_.get()));

        // The vehicle position (to figure out from where we should look up the route)
        odometry_subscriber_ = roadmap->create_subscription<Odometry>(
            "~/input/odometry", 1, std::bind(&AutowareLaneletConverter::onOdometry, this, _1), createSubscriptionOptions(roadmap_ptr_.get()));
        RCLCPP_INFO(logger_, "Ready");
    }

    void onOdometry(Odometry::ConstSharedPtr msg)
    {
        // rclcpp::Time previous_msg_time = odometry_ptr_->header.stamp;
        // rclcpp::Time cur_msg_time = msg->header.stamp;

        // Update the map every 10s
        // if (odometry_ptr_ == nullptr || (cur_msg_time < previous_msg_time + rclcpp::Duration::from_seconds(10.)))
        // {
        odometry_ptr_ = msg;

        // if (route_ptr_ != nullptr)
        // onRoute(route_ptr_);
        // }
    }

    void onMap(HADMapBin::ConstSharedPtr msg)
    {
        RCLCPP_INFO(logger_, "Map Received");
        map_ptr_ = msg;
    }

    void onRoute(LaneletRoute::ConstSharedPtr msg)
    {
        RCLCPP_INFO(logger_, "Route Received");

        // We need the map to understand the route
        if (map_ptr_ == nullptr)
        {
            RCLCPP_INFO_THROTTLE(logger_, *(roadmap_ptr_->get_clock()), 5000, "waiting for lanelet_map msg...");
            return;
        }
        if (odometry_ptr_ == nullptr)
        {
            RCLCPP_INFO_THROTTLE(logger_, *(roadmap_ptr_->get_clock()), 5000, "waiting for odometry msg...");
            return;
        }
        RCLCPP_INFO(logger_, "Map was ready, computing reference path");

        route_ptr_ = msg;

        route_handler_->setMap(*map_ptr_);
        route_handler_->setRoute(*route_ptr_);

        /** @see behavior_path_planner/planner manager */
        geometry_msgs::msg::Pose cur_pose = odometry_ptr_->pose.pose;
        PathWithLaneId reference_path{}; // The final reference path

        const auto backward_length = 10.; // How far back should the path go
        const auto forward_length = 100.; // How far forward should the path go

        reference_path.header = route_handler_->getRouteHeader();

        // Find the closest lanelet to our lanelet
        lanelet::ConstLanelet closest_lane{};
        route_handler_->getClosestLaneletWithinRoute(cur_pose, &closest_lane); // Get the closest lanelet to the pose

        const auto current_lanes = route_handler_->getLaneletSequence(
            closest_lane, cur_pose, backward_length, forward_length);

        reference_path = route_handler_->getCenterLinePath(current_lanes, backward_length, forward_length, true);

        RCLCPP_INFO(logger_, "Reference path ready");

        // Convert to nav path and forward to the rest of the roadmap
        roadmap_ptr_->WaypointCallback(ConvertAutowarePathWithLaneIdToNavPath(reference_path));
    }

private:
    std::shared_ptr<Roadmap> roadmap_ptr_;
    rclcpp::Logger logger_;

    rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_; /** Subscriber for external waypoints */
    HADMapBin::ConstSharedPtr map_ptr_{nullptr};

    rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscriber_; /** Subscriber for external waypoints */
    LaneletRoute::ConstSharedPtr route_ptr_{nullptr};

    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscriber_; /** Subscriber for external waypoints */
    Odometry::ConstSharedPtr odometry_ptr_{nullptr};

    std::shared_ptr<RouteHandler> route_handler_{std::make_shared<RouteHandler>()};
};

// Additional steps to consider (from autoware)
// clip backward length
// const size_t current_seg_idx = data->findEgoSegmentIndex(reference_path.points);
// util::clipPathLength(
//     reference_path, current_seg_idx, p.forward_path_length, p.backward_path_length);
// const auto drivable_lanelets = util::getLaneletsFromPath(reference_path, route_handler);
// const auto drivable_lanes = util::generateDrivableLanes(drivable_lanelets);

// {
//     const int num_lane_change =
//         std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));

//     const double lane_change_buffer = util::calcLaneChangeBuffer(p, num_lane_change);

//     reference_path = util::setDecelerationVelocity(
//         *route_handler, reference_path, current_lanes, parameters_->lane_change_prepare_duration,
//         lane_change_buffer);
// }

// const auto shorten_lanes = util::cutOverlappedLanes(reference_path, drivable_lanes);

// const auto expanded_lanes = util::expandLanelets(
//     shorten_lanes, parameters_->drivable_area_left_bound_offset,
//     parameters_->drivable_area_right_bound_offset, parameters_->drivable_area_types_to_skip);

// util::generateDrivableArea(reference_path, expanded_lanes, p.vehicle_length, data);

// BehaviorModuleOutput output;
// output.path = std::make_shared<PathWithLaneId>(reference_path);
// output.reference_path = std::make_shared<PathWithLaneId>(reference_path);

// return output;

#endif
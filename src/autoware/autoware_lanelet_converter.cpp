#include <lanelets_to_path/autoware/autoware_lanelet_converter.h>

#include <lanelets_to_path/autoware/autoware_utils.h>
#include <lanelets_to_path/spline_fitter.h>
#include <lanelets_to_path/types.h>

#include <ros_tools/logging.h>
#include <ros_tools/visuals.h>

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

AutowareLaneletConverter::AutowareLaneletConverter()
    : rclcpp::Node("lanelets_to_path")
{
}

void AutowareLaneletConverter::initialize()
{
  LOG_INFO("Initializing");

  // Initialize the configuration
  config_ptr_ = std::make_shared<RoadmapConfig>();
  config_ptr_->initialize(shared_from_this());

  VISUALS.init(this);

  auto qos_transient_local = rclcpp::QoS{1}.transient_local();

  // The map of the world
  vector_map_subscriber_ = this->create_subscription<HADMapBin>(
      "~/input/vector_map", qos_transient_local,
      std::bind(&AutowareLaneletConverter::onMap, this, _1),
      createSubscriptionOptions(this));

  // The route to be followed on the map
  route_subscriber_ = this->create_subscription<LaneletRoute>(
      "~/input/route", 1,
      std::bind(&AutowareLaneletConverter::onRoute, this, _1),
      createSubscriptionOptions(this));

  // The vehicle position (to figure out from where we should look up the route)
  odometry_subscriber_ = this->create_subscription<Odometry>(
      "~/input/odometry", 1,
      std::bind(&AutowareLaneletConverter::onOdometry, this, _1),
      createSubscriptionOptions(this));

  initial_pose_subscriber_ = this->create_subscription<PoseWithCovarianceStamped>(
      "/initialpose", 1,
      std::bind(&AutowareLaneletConverter::onInitialPose, this, _1),
      createSubscriptionOptions(this));

  autoware_path_publisher_ = this->create_publisher<PathWithLaneId>(
      "/roadmap/path_with_lane_id", 1);

  reference_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "roadmap/reference", 1);

  LOG_SUCCESS("Ready");
}

void AutowareLaneletConverter::onInitialPose(PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  (void)msg;
  if (route_ptr_)
  {
    LOG_INFO("Vehicle respawned, recomputing the route.");
    onRoute(route_ptr_); // Computing a new part of the map
  }
}

void AutowareLaneletConverter::onOdometry(Odometry::ConstSharedPtr msg)
{
  if (!odometry_ptr_)
  {
    if (config_ptr_->debug_output_)
    {
      LOG_INFO("First odometry received");
    }

    odometry_ptr_ = msg; // Read odometry if it was not received yet
  }
  else
  {
    rclcpp::Time previous_msg_time = odometry_ptr_->header.stamp;
    rclcpp::Time cur_msg_time = msg->header.stamp;
    if ((cur_msg_time - previous_msg_time).seconds() > config_ptr_->autoware_update_interval_)
    { // Otherwise, check how old the data is
      if (config_ptr_->debug_output_)
      {
        LOG_INFO("Updating odometry and route");
      }

      odometry_ptr_ = msg; // And update if necessary
      if (route_ptr_ != nullptr)
      {
        onRoute(route_ptr_); // Computing a new part of the map
      }
    }
  }
}

void AutowareLaneletConverter::onMap(HADMapBin::ConstSharedPtr msg)
{
  if (config_ptr_->debug_output_)
  {
    LOG_INFO("Map Received");
  }
  map_ptr_ = msg;
}

void AutowareLaneletConverter::onRoute(LaneletRoute::ConstSharedPtr msg)
{
  if (config_ptr_->debug_output_)
  {
    LOG_INFO("Route Received");
  }

  // We need the map to understand the route
  if (!map_ptr_)
  {
    LOG_INFO_THROTTLE(5000, "waiting for lanelet_map msg...");
    return;
  }
  if (!odometry_ptr_)
  {
    LOG_INFO_THROTTLE(5000, "waiting for odometry msg...");
    return;
  }
  LOG_INFO("Converting lanelets to the goal into a reference path for planning");

  route_ptr_ = msg;

  route_handler_->setMap(*map_ptr_);
  route_handler_->setRoute(*route_ptr_);

  /** @see behavior_path_planner/planner manager */

  geometry_msgs::msg::Pose cur_pose = odometry_ptr_->pose.pose;
  PathWithLaneId reference_path{}, extended_reference_path{}; // The final reference path

  const auto backward_length = config_ptr_->autoware_backward_distance_; // How far back should the path go
  const auto forward_length = config_ptr_->autoware_forward_distance_;   // How far forward should the path go

  reference_path.header = route_handler_->getRouteHeader();

  // Find the closest lanelet to our lanelet
  lanelet::ConstLanelet closest_lane{};
  route_handler_->getClosestLaneletWithinRoute(cur_pose, &closest_lane); // Get the closest lanelet to the pose

  const auto current_lanes = route_handler_->getLaneletSequence(
      closest_lane, cur_pose, -backward_length, forward_length);
  reference_path = route_handler_->getCenterLinePath(
      current_lanes, -backward_length,
      forward_length, true);
  // getAllSharedLineStringLanelets

  // RCLCPP_INFO(logger_, "Loading reference path");
  // Retrieve road boundaries from the reference path
  double extend = 200.;
  const auto extended_current_lanes = route_handler_->getLaneletSequence(
      closest_lane, cur_pose, -backward_length - extend, forward_length + extend);
  // extended_reference_path = route_handler_->getCenterLinePath(extended_current_lanes, -backward_length - extend, forward_length + extend, true);
  AddRoadBoundaries(reference_path, extended_current_lanes);
  // reference_path.left_bound = extended_reference_path.left_bound;
  // reference_path.right_bound = extended_reference_path.right_bound;
  LaneletFitter lanelet_fitter(config_ptr_.get(), reference_path);
  lanelet_fitter.Visualize();

  autoware_path_publisher_->publish(reference_path);

  // Convert to nav path and forward to the rest of the roadmap
  WaypointCallback(ConvertAutowarePathWithLaneIdToNavPath(reference_path));

  LOG_SUCCESS("Published reference path!");
}

void AutowareLaneletConverter::AddRoadBoundaries(
    PathWithLaneId &path,
    const std::vector<lanelet::ConstLanelet> &lanelet_sequence)
{
  // Go throught the left boundary of the lanelet and add its points
  std::vector<geometry_msgs::msg::Point> left_points;
  for (auto &lanelet : lanelet_sequence)
  {
    // First get the left most lanelet
    // auto left_most_lanelet = route_handler_->getAllSharedLineStringLanelets(lanelet, false, true, true);

    lanelet::ConstLineString2d bound = lanelet.leftBound2d(); // I will assume left most is 0 index?
    for (auto &point : bound)
    {
      left_points.push_back(lanelet::utils::conversion::toGeomMsgPt(point));
    }
  }
  path.left_bound = left_points;

  // Same for the right boundary
  std::vector<geometry_msgs::msg::Point> right_points;
  for (auto &lanelet : lanelet_sequence)
  {
    // Get opposite lanes if they exist
    auto right_most_lanelet = route_handler_->getAllRightSharedLinestringLanelets(
        lanelet, true,
        true);

    // std::cout << right_most_lanelet.size() << std::endl;
    // For the opposite lanes, invert it then take the left bound? Or should it be the right bound
    // THIS ONLY WORKS OCCASIONALLY ON HORIZONTAL ROADS. NOT CLEAR WHY YET -> USE THE TOP OF THE MAP, THE MAP IS PROPERLY MADE THERE
    lanelet::ConstLineString2d bound = right_most_lanelet.size() > 0 ? right_most_lanelet[0].rightBound2d() : // (is already inverted!)
                                           lanelet.rightBound2d();

    for (auto &point : bound)
    {
      right_points.push_back(lanelet::utils::conversion::toGeomMsgPt(point));
    }
  }
  path.right_bound = right_points;
}

void AutowareLaneletConverter::WaypointCallback(const nav_msgs::msg::Path &msg)
{
  Way new_way; // Create a way object
  for (size_t i = 0; i < msg.poses.size(); i++)
  {
    new_way.AddNode(RoadNode(
        msg.poses[i].pose.position.x,
        msg.poses[i].pose.position.y,
        RosTools::quaternionToAngle(msg.poses[i].pose)));
  }
  int id = 0;
  new_way.AddLane("road", 4.0, true, id); // Add a lane
  auto &lane = new_way.lanes.back();

  SplineFitter spline_fitter;
  spline_fitter.Initialize(config_ptr_.get());
  spline_fitter.FitSplineOnLane(lane); // Fit a spline on the received waypoints

  auto ref_msg = std::make_shared<nav_msgs::msg::Path>();
  ref_msg->poses.reserve(lane.nodes.size());

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = rclcpp::Clock().now();
  pose_msg.header.frame_id = "map";

  for (RoadNode &node : lane.nodes)
  {
    pose_msg.pose.position.x = node.x;
    pose_msg.pose.position.y = node.y;
    pose_msg.pose.orientation = RosTools::angleToQuaternion(node.theta);
    ref_msg->poses.push_back(pose_msg);
  }

  ref_msg->header.stamp = rclcpp::Clock().now();
  ref_msg->header.frame_id = "map";

  reference_pub_->publish(*ref_msg); // publish
}

#include "roadmap.h"

Roadmap::Roadmap() : rclcpp::Node("roadmap")
{
}

void Roadmap::Initialize()
{
    // Initialize the configuration
    config_.reset(new RoadmapConfig());
    config_->initialize(shared_from_this());

    // Initialize reading and processing
    reader_.reset(new Reader(config_.get()));
    spline_converter_.reset(new SplineConverter());
    spline_converter_->Initialize(this, config_.get());

    // Read the map
    ReadFromFile();

    // Subscribers
    waypoints_sub_ = this->create_subscription<nav_msgs::msg::Path>(config_->external_waypoint_topic_, 1,
                                                                    std::bind(&Roadmap::WaypointCallback, this, std::placeholders::_1)); // Subscriber for waypoints (forwarded to the reader)
    reset_sub_ = this->create_subscription<std_msgs::msg::Empty>("/lmpcc/reset_environment", 1,
                                                                 std::bind(&Roadmap::ResetCallback, this, std::placeholders::_1));
    reverse_sub_ = this->create_subscription<std_msgs::msg::Empty>("/roadmap/reverse", 1,
                                                                   std::bind(&Roadmap::ReverseCallback, this, std::placeholders::_1));

    // Debug: listens to rviz point for translating the map
    offset_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("roadmap/offset", 1,
                                                                                           std::bind(&Roadmap::OffsetCallback, this, std::placeholders::_1));

    // Publishers
    map_pub_ = this->create_publisher<roadmap_msgs::msg::RoadPolylineArray>("roadmap/road_polylines", 1);
    reference_pub_ = this->create_publisher<nav_msgs::msg::Path>("roadmap/reference", 1);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/roadmap/goal", 1);

    // Then convert the read waypoints to splines
    is_reversed_ = false;

    timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        rclcpp::Duration::from_seconds(1.0 / config_->update_frequency_),
        std::bind(&Roadmap::Poll, this));

    ROADMAP_WARN(this->get_logger(), "Initialization completed");
}

// Have two maps (input / output)
void Roadmap::ReadFromFile()
{
    ROADMAP_INFO(this->get_logger(), "Reading map from file");

    reader_->Read(shared_from_this());
    ConvertMap();
}

void Roadmap::WaypointCallback(const nav_msgs::msg::Path &msg)
{
    ROADMAP_INFO(this->get_logger(), "Received waypoints");

    reader_->WaypointCallback(msg);
    ConvertMap();
    reference_pub_->publish(ref_msg_); // publish
}

void Roadmap::OffsetCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
    reader_->OffsetCallback(msg);
    ReadFromFile();
}

void Roadmap::ReverseCallback(const std_msgs::msg::Empty &msg)
{

    is_reversed_ = !is_reversed_;

    config_.reset(new RoadmapConfig());
    config_->initialize(shared_from_this());

    reader_.reset(new Reader(config_.get()));

    // Then convert the read waypoints to splines

    spline_converter_.reset(new SplineConverter());
    spline_converter_->Initialize(this, config_.get());

    reader_->Read(shared_from_this());

    if (is_reversed_)
        reader_->GetMap().Reverse(); // Reverse the map

    ConvertMap();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.pose.position.x = spline_converter_->converted_map_.ways[0].nodes.back().x;
    pose_msg.pose.position.y = spline_converter_->converted_map_.ways[0].nodes.back().y;
    pose_msg.header.stamp = this->get_clock()->now();
    pose_msg.header.frame_id = "map";
    goal_pub_->publish(pose_msg);

    ROADMAP_INFO(this->get_logger(), "Roadmap reverse completed");
}

void Roadmap::ResetCallback(const std_msgs::msg::Empty &msg)
{
    /* // Initialize the configuration
     config_.reset(new RoadmapConfig());
     config_->initialize();

     reader_.reset(new Reader(config_.get()));

     // Then convert the read waypoints to splines
     spline_converter_->Initialize(nh_, config_.get());

     ReadFromFile();

     ROADMAP_WARN(logger_, "Roadmap reset completed");*/
}

void Roadmap::ConvertMap()
{
    spline_converter_->ConvertMap(reader_->GetMap());

    road_msg_.road_polylines.clear();
    ref_msg_.poses.clear();
    reader_->GetMap().ToMsg(road_msg_);                // Load map data into the message
    spline_converter_->converted_map_.ToMsg(ref_msg_); // Load reference path data into the message
}

void Roadmap::Poll()
{
    ROADMAP_INFO(this->get_logger(), "====== START LOOP ======");

    runs_++;

    // if (runs_ < 10)
    // {
    // Should happen by request?

    map_pub_->publish(road_msg_);      // publish
    reference_pub_->publish(ref_msg_); // publish

    // }
    ROADMAP_INFO(this->get_logger(), "Published polylines and reference");

    // Visualize the map
    // Two functions here
    spline_converter_->VisualizeInputData(reader_->GetMap());

    spline_converter_->VisualizeMap();

    ROADMAP_INFO(this->get_logger(), "======= END LOOP =======");
}

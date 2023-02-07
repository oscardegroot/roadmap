#include "roadmap.h"

Roadmap::Roadmap()
{
    // Initialize the configuration
    config_.reset(new RoadmapConfig());
    config_->initialize();

    reader_.reset(new Reader(config_.get()));

    // Subscribers
    waypoints_sub_ = nh_.subscribe(config_->external_waypoint_topic_, 1, &Roadmap::WaypointCallback, this); // Subscriber for waypoints (forwarded to the reader)

    // Debug: listens to rviz point for translating the map
    offset_sub_ = nh_.subscribe("roadmap/offset", 1, &Roadmap::OffsetCallback, this); // Subscriber for waypoints (forwarded to the reader)

    // Publishers
    map_pub_ = nh_.advertise<roadmap_msgs::RoadPolylineArray>("roadmap/polylines", 1);
    reference_pub_ = nh_.advertise<nav_msgs::Path>("roadmap/reference", 1);

    // Then convert the read waypoints to splines
    spline_converter_.Initialize(nh_, config_.get());

    ReadFromFile();

    timer_ = nh_.createTimer(ros::Duration(1.0 / config_->update_frequency_), &Roadmap::Poll, this);

    ROADMAP_WARN("Initialization completed");
}

// Have two maps (input / output)
void Roadmap::ReadFromFile()
{
    ROADMAP_INFO("Reading map from file")

    reader_->Read();
    ConvertMap();
}

void Roadmap::WaypointCallback(const nav_msgs::Path &msg)
{
    ROADMAP_INFO("Received waypoints")

    reader_->WaypointCallback(msg);
    ConvertMap();
    reference_pub_.publish(ref_msg_); // publish
}

void Roadmap::OffsetCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    reader_->OffsetCallback(msg);
    ReadFromFile();
}

void Roadmap::ConvertMap()
{
    spline_converter_.ConvertMap(reader_->GetMap());

    road_msg_.road_polylines.clear();
    ref_msg_.poses.clear();
    reader_->GetMap().ToMsg(road_msg_); // Load map data into the message
    reader_->GetMap().ToMsg(ref_msg_);  // Load reference path data into the message
}

void Roadmap::Poll(const ros::TimerEvent &event)
{
    ROADMAP_INFO("====== START LOOP ======")
    runs_++;
    // if (runs_ < 10)
    // {
    // Should happen by request?

    // }
    map_pub_.publish(road_msg_); // publish
    reference_pub_.publish(ref_msg_); // publish

    // Visualize the map
    // Two functions here
    spline_converter_.VisualizeInputData(reader_->GetMap());
    spline_converter_.VisualizeMap();

    ROADMAP_INFO("======= END LOOP =======")
}

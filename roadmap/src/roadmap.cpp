#include "roadmap.h"

Roadmap::Roadmap()
{
    // Initialize the configuration
    config_.reset(new RoadmapConfig());
    config_->initialize();

    reader_.reset(new Reader(config_.get()));

    // Subscribers
    waypoints_sub_ = nh_.subscribe(config_->external_waypoint_topic_, 1, &Reader::WaypointCallback, reader_.get()); // Subscriber for waypoints (forwarded to the reader)

    // Publishers
    map_pub_ = nh_.advertise<roadmap_msgs::RoadPolylineArray>("roadmap/polylines", 1);
    reference_pub_ = nh_.advertise<nav_msgs::Path>("roadmap/reference", 1);

    // Then convert the read waypoints to splines
    spline_converter_.Initialize(nh_, config_.get());

    timer_ = nh_.createTimer(ros::Duration(1.0 / config_->update_frequency_), &Roadmap::Poll, this);

    ROADMAP_WARN("Initialization completed");
}

// Need a better flow here...
void Roadmap::Poll(const ros::TimerEvent &event)
{
    ROADMAP_INFO("====== START LOOP ======")
    runs_++;
    if (runs_ < 10)
    {
        reader_->Read();
        spline_converter_.ConvertMap(reader_->GetMap());

        roadmap_msgs::RoadPolylineArray msg; // Setup the message
        reader_->GetMap().ToMsg(msg);        // Load map data into the message
        map_pub_.publish(msg);               // publish

        nav_msgs::Path ref_msg;           // Setup the message
        reader_->GetMap().ToMsg(ref_msg); // Load reference path data into the message
        reference_pub_.publish(ref_msg);  // publish
    }
    // spline_converter_.VisualizeMap(map);

    ROADMAP_INFO("======= END LOOP =======")
}

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

    // Then convert the read waypoints to splines
    spline_converter_.Initialize(nh_, config_.get());

    timer_ = nh_.createTimer(ros::Duration(1.0 / config_->update_frequency_), &Roadmap::Poll, this);

    ROADMAP_WARN("Initialization completed");
}

void Roadmap::Poll(const ros::TimerEvent &event)
{
    ROADMAP_INFO("====== START LOOP ======")
    reader_->Read();
    Map &map = reader_->GetMap();

    spline_converter_.ConvertMap(map);

    roadmap_msgs::RoadPolylineArray msg; // Setup the message
    map.ToMsg(msg);                      // Load map data into the message
    map_pub_.publish(msg);               // publish
    ROADMAP_INFO("======= END LOOP =======")
}

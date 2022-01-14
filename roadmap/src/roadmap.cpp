#include "roadmap.h"

Roadmap::Roadmap()
{
    // Read the configuration
    config_.initialize();

    ros::NodeHandle nh;

    // Subscribers

    //Publishers
    map_pub_ = nh.advertise<roadmap_msgs::RoadPolylineArray>("roadmap/polylines", 1);

    // Then convert the read waypoints to splines
    spline_converter_.Initialize(nh, &config_);

    timer_ = nh.createTimer(ros::Duration(1.0 / 10.0), &Roadmap::Poll, this);
}

void Roadmap::Poll(const ros::TimerEvent &event)
{
    reader_ = Reader();
    Map &map = reader_.GetMap();

    spline_converter_.ConvertMap(map);

    roadmap_msgs::RoadPolylineArray msg; // Setup the message
    map.ToMsg(msg);                      // Load map data into the message
    map_pub_.publish(msg);               // publish
}

#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include <ros/ros.h>
#include <ros/console.h>
#include <string>

#include "roadmap_msgs/RoadPolylineArray.h"
#include "roadmap_msgs/RoadPolyline.h"

#include "reader.h"
#include "types.h"
#include "spline_converter.h"

class Roadmap
{

    /**
     * @brief Main class for the roadmap node
     * 
     */
public:
    Roadmap();

public:
    /**
     * @brief Callback for external waypoints
     * 
     * @param msg the reference as a nav_msgs::Path
     */
    void WaypointCallback(const nav_msgs::Path &msg);

private:
    ros::NodeHandle nh_;

    std::unique_ptr<RoadmapConfig> config_; /** Parameters */

    SplineConverter spline_converter_; /** Spline convertion class */
    std::unique_ptr<Reader> reader_;   /** File reader class */

    ros::Subscriber waypoints_sub_; /** Subscriber for external waypoints */
    ros::Publisher map_pub_;        /** Publisher for road polyline output */
    ros::Publisher reference_pub_;  /** Publisher for road polyline output */

    roadmap_msgs::RoadPolylineArray road_msg_; // Setup the message
    nav_msgs::Path ref_msg_;                   // Setup the message

    ros::Timer timer_;

    int runs_ = 0;

    /**
     * @brief Function that reads the map, currently repeated on a frequency
     */
    void Poll(const ros::TimerEvent &event);

    /**
     * @brief Read the map from file and visualize the input
     */
    void ReadFromFile();

    /**
     * @brief Convert the map and create ros messages
     */
    void ConvertMap();
};

#endif // __ROADMAP_H__
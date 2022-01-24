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
private:
    ros::NodeHandle nh_;

    std::unique_ptr<RoadmapConfig> config_; /** Parameters */

    SplineConverter spline_converter_; /** Spline convertion class */
    std::unique_ptr<Reader> reader_;   /** File reader class */

    ros::Subscriber waypoints_sub_; /** Subscriber for external waypoints */
    ros::Publisher map_pub_;        /** Publisher for road polyline output */
    ros::Publisher reference_pub_;  /** Publisher for road polyline output */

    ros::Timer timer_;

    int runs_ = 0;

    /**
     * @brief Function that reads the map, currently repeated on a frequency
     */
    void Poll(const ros::TimerEvent &event);
};

#endif // __ROADMAP_H__
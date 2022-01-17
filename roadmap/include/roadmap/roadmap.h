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

public:
    Roadmap();

public:
private:
    ros::NodeHandle nh_;

    std::unique_ptr<RoadmapConfig> config_;

    SplineConverter spline_converter_;
    std::unique_ptr<Reader> reader_; // Should have a "read" function

    ros::Publisher map_pub_;

    ros::Timer timer_;

    void Poll(const ros::TimerEvent &event);

    // Debug
    int i = 0;
};

#endif // __ROADMAP_H__
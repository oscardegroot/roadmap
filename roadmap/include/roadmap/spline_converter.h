#ifndef SPLINE_CONVERTER_H
#define SPLINE_CONVERTER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Eigen>

#include <lmpcc_tools/ros_visuals.h>

// spline libraries
#include <spline.h>
#include <Clothoid.h>

#include "configuration.h"
#include "types.h"
#include "helpers.h"
// #include <lmpcc_msgs/halfspace_array.h>
// #include <lmpcc_msgs/halfspace.h>

struct Waypoint
{
    double x, y, theta, s;

    Waypoint()
    {
    }

    Waypoint(double _x, double _y, double _theta, double _s)
        : x(_x), y(_y), theta(_theta), s(_s)
    {
    }
    Waypoint(double _x, double _y, double _theta)
        : x(_x), y(_y), theta(_theta), s(-1.)
    {
    }
};

//Whens earching for the closest point on the path, this variable indicates the distance that the algorithm searches behind the current spline point.
#define MAX_STEP_BACK_TOLERANCE 0.1f

class SplineConverter
{

public:
    SplineConverter(){};

    void Initialize(ros::NodeHandle &nh, RoadmapConfig *config);

public:
    void ConvertMap(Map &map);

    void VisualizeMap(Map &map, bool converted_map = false);

private:
    RoadmapConfig *config_;

    std::unique_ptr<ROSMarkerPublisher> input_map_markers_;
    std::unique_ptr<ROSMarkerPublisher> output_map_markers_;

    void FitSplineOnLane(Lane &lane);

    void FitSplineOnWaypoints(const std::vector<Waypoint> &waypoints, std::vector<Waypoint> &waypoints_out, Lane &lane);

    /**
     * @brief Convert a set of input waypoints to the format for fitting the cubic spline
     * 
     * @param waypoints the input waypoints
     * @param x output x vector
     * @param y output y vector
     * @param s output s vector
     */
    void ConvertWaypointsToVectors(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s);

    /**
     * @brief Fit a Clothoid over the given waypoints
     * 
     * @param waypoints the input waypoints
     * @param x output x vector
     * @param y output y vector
     * @param s output s vector
     */
    void FitClothoid(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s);

    /**
     * @brief Fit a cubic spline over a set of waypoints (x, y, s)
     * 
     * @param lane The lane object that contains the spline
     * @param waypoints_out waypoints sampled over the fitted spline
     */
    void FitCubicSpline(Lane &lane, std::vector<Waypoint> &waypoints_out);

    void VisualizeLaneSpline(const Lane &lane, std::map<int, Node> &nodes);
};
#endif
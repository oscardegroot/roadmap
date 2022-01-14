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

    void FitSplineOnWay(Way &way, std::map<int, Node> &nodes);

    void FitSplineOnWaypoints(const std::vector<Waypoint> &waypoints, std::vector<Waypoint> &waypoints_out, Way &way);

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
     * @param way The way object that contains the spline
     * @param waypoints_out waypoints sampled over the fitted spline
     */
    void FitCubicSpline(Way &way, std::vector<Waypoint> &waypoints_out);

    void VisualizeWaySpline(const Way &way, std::map<int, Node> &nodes);

    // void ConstructReferencePathWithClothoid(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &theta);
    // void ConstructReferencePathWithoutClothoid(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &theta);

    // predictive_configuration *config_;

    // ros::Publisher spline_pub_, road_pub_, marker_pub_;

    // std::unique_ptr<ROSMarkerPublisher> ros_markers_refpath;
    // std::unique_ptr<ROSMarkerPublisher> ros_markers_splineindex;
    // std::unique_ptr<ROSMarkerPublisher> ros_markers_linearboundaries;
    // std::unique_ptr<ROSMarkerPublisher> ros_markers_road_limits;

    // void ReadReferencePath();
    // void ConstructReferencePath(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &theta); // Selects which method to use

    // int RecursiveClosestPointSearch(BaseModel *solver_interface_ptr, unsigned int cur_traj_i, double &s_guess, double window, int n_tries);

    // PUBLIC OLD

    // // Current spline index
    // unsigned int spline_index_;
    // unsigned int waypoints_size_;

    // // Waypoints x, y, theta
    // std::vector<double> x_, y_, theta_;

    // // Output splines
    // tk::spline ref_path_x_, ref_path_y_;

    // // Spline s, x, y
    // std::vector<double> ss_, xx_, yy_;

    // double dist_spline_pts_;

    // nav_msgs::Path spline_msg_;

    // void Init(ros::NodeHandle &nh, predictive_configuration *config);
    // void InitPath();
    // void InitPath(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &theta);

    // void SetParameters(BaseModel *solver_interface, int N_iter, int &param_idx); // Set solver parameters
    // void UpdateClosestPoint(BaseModel *solver_interface_ptr, double &s_guess, double window, int n_tries);

    // void InitializeClosestPoint(BaseModel *solver_interface_ptr);

    // bool EndOfCurrentSpline(double index);

    // void UpdateWaypoint(BaseModel *solver_interface_ptr);

    // bool ReachedEnd();

    // void ConstructRoadConstraints(BaseModel *solver_interface, std::vector<lmpcc_msgs::halfspace_array> &halfspaces_out);

    // void PublishReferencePath();
    // void PublishSpline();
    // void PublishCurrentSplineIndex();

    // void PublishRoadBoundaries(BaseModel *solver_interface_ptr);
    // void PublishLinearRoadBoundaries(BaseModel *solver_interface_ptr, const lmpcc_msgs::halfspace_array &halfspaces_out);
    // void VisualizeRoadLocally();
    // void VisualizeRoad();
};
#endif
#ifndef __LANELETS_TO_PATH_TYPES_H__
#define __LANELETS_TO_PATH_TYPES_H__

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <spline/spline.h>

#include <Eigen/Dense>
#include <vector>

// Radius of Earth
#define GLOBE_RADIUS 6371.0e3

// Map -> Ways -> Lanes -> Node

/** Struct for a waypoint with distance */
struct Waypoint
{
    double x, y, theta, s;

    Waypoint();

    Waypoint(double _x, double _y, double _theta, double _s);
    Waypoint(double _x, double _y, double _theta);
    Waypoint(const Eigen::Vector2d &point);

    Eigen::Vector2d asVector2d() const;
};

struct Line
{
    Eigen::Vector2d A;
    double b;

    Line(const Eigen::Vector2d &_A, const double _b);

    double distanceTo(const Waypoint &waypoint) const;

    double distanceTo(const Eigen::Vector2d &waypoint) const;
};

void WaypointVectorToGeometryPointVector(const std::vector<Waypoint> &waypoints,
                                                std::vector<geometry_msgs::msg::Point> &out);

/**
 * @brief Struct describing a waypoint node (x, y, theta)
 */
struct RoadNode
{

    double x, y, theta;

    RoadNode();

    RoadNode(double _x, double _y, double _theta);

    // Is skewed!
    RoadNode(double _lat, double _long, double base_latitude, double base_longitude, double _theta);
};

/**
 * @brief Struct describing a lane
 *
 */
struct Lane
{
    int id, type;                /** ID and the type of lane as int */
    std::vector<RoadNode> nodes; /** Nodes that construct this lane */

    // Variables after spline fitting
    bool spline_fit;               /** Has a spline been fitted over this lane? */
    double length;                 /** Length of the lane */
    tk::spline spline_x, spline_y; /** Cubic spline objects */

    Lane();

    /**
     * @brief Construct a new Lane object
     *
     * @param _nodes nodes of the centerlane
     * @param offset offset with respect to the centerlane for this lane
     * @param _type type of lane (see road_msgs/RoadPolyline)
     * @param _id current ID to add this lane on (and is increased after usage)
     */
    Lane(const std::vector<RoadNode> &_nodes, double offset, int _type, int &_id);

    void Reverse();
    /**
     * @brief Assign a spline to this lane
     */
    void AssignSpline(const tk::spline &_spline_x, const tk::spline &_spline_y);

    double distanceTo(const double s, const Waypoint &point) const;
    double distanceToLine(const double s, const Line &line) const;

    Eigen::Vector2d at(const double s) const;
};

/**
 * @brief A way struct that may contain multiple lanes (e.g., road, sidewalk, etc.)
 *
 */
struct Way
{
    std::vector<Lane> lanes;          /** Lanes on this way */
    std::vector<RoadNode> nodes;      /** Nodes defining the center lane */
    double plus_offset, minus_offset; /** Current aggregated offset in lane construction */

    Way();

    void Reverse();

    /**
     * @brief Add a node to the center line of this way
     *
     * @param node the node to add
     */
    void AddNode(const RoadNode &node);

    /**
     * @brief Add a road by copying the current data and offsetting it.
     * Can only be called before the spline is fit!
     *
     * @param _type type of lane to add
     * @param width width of the lane
     * @param two_way is it a two way lane?
     * @param id ID to start at
     */
    void AddLane(const std::string &_type, double width, bool two_way, int &id);
};

/**
 * @brief Struct to define the map.
 */
struct Map
{
    std::vector<Way> ways;

    Map();

    /**  @brief Clear the map  */
    void Clear();

    /** @brief Reverse the direction of the reference path */
    void Reverse();
};

#endif // __TYPES_H__
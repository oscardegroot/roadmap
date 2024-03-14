#ifndef __SPLINE_FITTER_H__
#define __SPLINE_FITTER_H__

#include <rclcpp/rclcpp.hpp>

#include <lanelets_to_path/types.h>
#include <lanelets_to_path/configuration.h>

#include <ros_tools/ros_visuals.h>

/** @brief Base class for fitting splines */
class SplineFitter
{
public:
    SplineFitter();
    void Initialize(RoadmapConfig *config);
    /**
     * @brief Fit a spline to a particular lane
     * @see types.h
     * @param lane A lane (road, sidewalk, etc. )
     */
    void FitSplineOnLane(Lane &lane);

protected:
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

    void VisualizeLaneSpline(RosTools::ROSMarkerPublisher &markers, const Lane &lane);
    void VisualizeLaneSpline(RosTools::ROSMarkerPublisher &markers, const Lane &lane, int color_idx);

    void VisualizePoints(RosTools::ROSMarkerPublisher &markers, const std::vector<Waypoint> &points,
                         int r = 0., int g = 0., int b = 0.);

    Waypoint FindPointOnSplineClosestTo(const Lane &lane, const Waypoint &point) const;
    double FindPointOnSplineClosestTo(const Lane &lane, const Waypoint &point,
                                      double low, double high, int num_recursions) const;

    double FindPointOnSplineClosestToLine(const Lane &lane, const Waypoint &point,
                                          double low, double high, int num_recursions,
                                          const Line &line) const;

    Waypoint FindPointClosestToSplineOrthogonal(const Lane &center_lane, const Lane &boundary, const Waypoint &point);

    void RemoveCornerPoints(std::vector<Waypoint> &points) const;
    void RemoveCloseTogetherPoints(std::vector<Waypoint> &points) const;

protected:
    rclcpp::Logger logger_;
    RoadmapConfig *config_;
};

#endif
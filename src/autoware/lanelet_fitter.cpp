#include <roadmap/autoware/lanelet_fitter.h>

#include <ros_tools/visuals.h>

LaneletFitter::LaneletFitter(RoadmapConfig *config, PathWithLaneId &path_with_lane_id)
{
    // Bounds are in std::vector<geometry_msgs::Point> format
    // Centerline is in (Autoware) Path format

    // Initialize the spline fitter base class
    Initialize(config);

    // Centerline
    // std::vector<Waypoint> center_lane_points;
    for (auto &point : path_with_lane_id.points)
        input_center_lane_.emplace_back(point.point.pose.position.x, point.point.pose.position.y, 0.);
    FitSplineOnWaypoints(input_center_lane_, output_center_lane_, center_lane);

    path_with_lane_id.points.clear();
    PathPointWithLaneId new_point;
    for (auto &waypoint : output_center_lane_)
    {
        new_point.point.pose.position.x = waypoint.x;
        new_point.point.pose.position.y = waypoint.y;
        path_with_lane_id.points.emplace_back(new_point);
    }

    // The boundaries are more complicated as we need each set of points (left and right)
    // to correspond with the centerline points

    // 1) Fit splines
    // Right Boundary
    for (auto &point : path_with_lane_id.right_bound)
        input_right_boundary_.emplace_back(point.x, point.y, 0.);

    std::vector<Waypoint> right_boundary_points = input_right_boundary_;
    // RemoveCloseTogetherPoints(right_boundary_points);
    RemoveCornerPoints(right_boundary_points);
    FitSplineOnWaypoints(right_boundary_points, output_right_boundary_, right_boundary);

    // Left Boundary
    for (auto &point : path_with_lane_id.left_bound)
        input_left_boundary_.emplace_back(point.x, point.y, 0.);

    std::vector<Waypoint> left_boundary_points = input_left_boundary_;
    // RemoveCloseTogetherPoints(left_boundary_points);
    RemoveCornerPoints(left_boundary_points);
    FitSplineOnWaypoints(left_boundary_points, output_left_boundary_, left_boundary);

    // 2) Find out for each centerline waypoint, the closest point on the boundary
    output_left_boundary_.clear();
    for (auto &waypoint : output_center_lane_)
    {
        // output_left_boundary_.emplace_back(FindPointOnSplineClosestTo(left_boundary, waypoint));
        output_left_boundary_.emplace_back(FindPointClosestToSplineOrthogonal(center_lane, left_boundary, waypoint));
    }

    output_right_boundary_.clear();
    for (auto &waypoint : output_center_lane_)
    {
        // output_right_boundary_.emplace_back(
        // FindPointOnSplineClosestTo(right_boundary, waypoint));
        output_right_boundary_.emplace_back(FindPointClosestToSplineOrthogonal(center_lane, right_boundary, waypoint));
    }

    WaypointVectorToGeometryPointVector(output_right_boundary_, path_with_lane_id.right_bound);
    WaypointVectorToGeometryPointVector(output_left_boundary_, path_with_lane_id.left_bound);

    // Set its values
    center_lane.type = 2;
    right_boundary.type = 3;
    left_boundary.type = 3;
}

void LaneletFitter::Visualize()
{
    auto &markers = VISUALS.getPublisher("lanelet_fitter");
    VisualizeLaneSpline(markers, center_lane, 15);
    VisualizeLaneSpline(markers, right_boundary, 0);
    VisualizeLaneSpline(markers, left_boundary, 0);

    if (config_->debug_output_)
    {
        int r = 0, g = 1., b = 0.;
        VisualizePoints(markers, input_center_lane_, r, g, b);
        VisualizePoints(markers, input_right_boundary_, r, g, b);
        VisualizePoints(markers, input_left_boundary_, r, g, b);
    }
    VisualizePoints(markers, output_center_lane_);
    VisualizePoints(markers, output_right_boundary_);
    VisualizePoints(markers, output_left_boundary_);

    markers.publish();
}

#include <roadmap/autoware/lanelet_fitter.h>

LaneletFitter::LaneletFitter(RoadmapConfig *config, PathWithLaneId &path_with_lane_id)
{
    // Bounds are in std::vector<geometry_msgs::Point> format
    // Centerline is in (Autoware) Path format

    // Initialize the spline fitter base class
    Initialize(config);

    // Centerline
    std::vector<Waypoint> center_lane_points;
    for (auto &point : path_with_lane_id.points)
        center_lane_points.emplace_back(point.point.pose.position.x, point.point.pose.position.y, 0.);
    FitSplineOnWaypoints(center_lane_points, output_center_lane_, center_lane);

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
    std::vector<Waypoint> right_boundary_points;
    for (auto &point : path_with_lane_id.right_bound)
        right_boundary_points.emplace_back(point.x, point.y, 0.);
    FitSplineOnWaypoints(right_boundary_points, output_right_boundary_, right_boundary);

    // Left Boundary
    std::vector<Waypoint> left_boundary_points;
    for (auto &point : path_with_lane_id.left_bound)
        left_boundary_points.emplace_back(point.x, point.y, 0.);
    FitSplineOnWaypoints(left_boundary_points, output_left_boundary_, left_boundary);

    // 2) Find out for each centerline waypoint, the closest point on the boundary
    output_left_boundary_.clear();
    for (auto &waypoint : output_center_lane_)
    {
        output_left_boundary_.emplace_back(
            FindPointOnSplineClosestTo(left_boundary, waypoint));
    }

    output_right_boundary_.clear();
    for (auto &waypoint : output_center_lane_)
    {
        output_right_boundary_.emplace_back(
            FindPointOnSplineClosestTo(right_boundary, waypoint));
    }

    WaypointVectorToGeometryPointVector(output_right_boundary_, path_with_lane_id.right_bound);
    WaypointVectorToGeometryPointVector(output_left_boundary_, path_with_lane_id.left_bound);

    // Set its values
    center_lane.type = 2;
    right_boundary.type = 3;
    left_boundary.type = 3;
}

void LaneletFitter::Visualize(RosTools::ROSMarkerPublisher &markers)
{
    VisualizeLaneSpline(markers, center_lane, 15);
    VisualizeLaneSpline(markers, right_boundary, 0);
    VisualizeLaneSpline(markers, left_boundary, 0);

    VisualizePoints(markers, output_center_lane_);
    VisualizePoints(markers, output_right_boundary_);
    VisualizePoints(markers, output_left_boundary_);

    markers.publish();
}

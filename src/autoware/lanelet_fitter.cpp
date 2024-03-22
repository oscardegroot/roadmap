#include <lanelets_to_path/autoware/lanelet_fitter.h>
#include <lanelets_to_path/configuration.h>

#include <ros_tools/math.h>
#include <ros_tools/visuals.h>

LaneletFitter::LaneletFitter(RoadmapConfig *config, PathWithLaneId &path_with_lane_id, const Eigen::Vector2d &goal)
{
    // Bounds are in std::vector<geometry_msgs::Point> format
    // Centerline is in (Autoware) Path format

    // Initialize the spline fitter base class
    Initialize(config);

    // Centerline
    // std::vector<Waypoint> center_lane_points;

    double min_dist = 1e9;
    int closest_index = -1;

    for (auto &point : path_with_lane_id.points)
    {
        input_center_lane_.emplace_back(point.point.pose.position.x, point.point.pose.position.y, 0.);

        double dist_to_goal =
            RosTools::distance(Eigen::Vector2d(point.point.pose.position.x, point.point.pose.position.y), goal);
        if (dist_to_goal < min_dist)
        {
            min_dist = dist_to_goal;
            closest_index = input_center_lane_.size() - 1;
        }
    }

    // Stop the reference path at the goal
    input_center_lane_.erase(input_center_lane_.begin() + closest_index, input_center_lane_.end());
    input_center_lane_.emplace_back(goal(0), goal(1), 0.);

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

    ReadBoundary(path_with_lane_id.right_bound, input_right_boundary_);
    std::vector<Waypoint> right_boundary_points = input_right_boundary_;
    // RemoveCloseTogetherPoints(right_boundary_points);
    RemoveCornerPoints(right_boundary_points);
    FitSplineOnWaypoints(right_boundary_points,
                         output_right_boundary_,
                         right_boundary,
                         true); // Fit a clothoid first!

    ReadBoundary(path_with_lane_id.left_bound, input_left_boundary_);
    std::vector<Waypoint> left_boundary_points = input_left_boundary_;
    // RemoveCloseTogetherPoints(left_boundary_points);
    RemoveCornerPoints(left_boundary_points);
    FitSplineOnWaypoints(left_boundary_points,
                         output_left_boundary_,
                         left_boundary,
                         true);

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

void LaneletFitter::ReadBoundary(const std::vector<geometry_msgs::msg::Point> &bound,
                                 std::vector<Waypoint> &output)
{
    std::vector<geometry_msgs::msg::Point> interpolated_bound;

    // Interpolate linear segments
    geometry_msgs::msg::Point point;
    for (size_t p = 0; p < bound.size() - 1; p++)
    {
        Eigen::Vector2d start(bound[p].x, bound[p].y);
        Eigen::Vector2d end(bound[p + 1].x, bound[p + 1].y);

        double dist = RosTools::distance(start, end);

        // This 10 is to give the clothoid fitter something to do (could be a parameter)
        int num_points = std::ceil(dist / (config_->clothoid_point_per_xm_ * 10.));
        double step = 1. / num_points;

        for (int i = 0; i < num_points; i++)
        {
            if (p > 0 && i == 0)
                continue; // Only add the first point for the first segment

            Eigen::Vector2d res = start + i * step * (end - start);
            point.x = res(0);
            point.y = res(1);

            interpolated_bound.emplace_back(point);
        }
        point.x = end(0);
        point.y = end(1);
        interpolated_bound.emplace_back(point);
    }

    double angle_prev, angle_next;
    for (size_t p = 0; p < interpolated_bound.size(); p++)
    {
        auto &point = interpolated_bound[p];

        if (p == 0 && interpolated_bound.size() > 1)
        {
            angle_next = std::atan2(interpolated_bound[p + 1].y - point.y,
                                    interpolated_bound[p + 1].x - point.x);
            output.emplace_back(point.x, point.y, angle_next);
        }
        else if (p == interpolated_bound.size() - 1 && interpolated_bound.size() > 1)
        {
            angle_prev = std::atan2(point.y - interpolated_bound[p - 1].y,
                                    point.x - interpolated_bound[p - 1].x);
            output.emplace_back(point.x, point.y, angle_prev);
        }
        else
        {
            angle_next = std::atan2(interpolated_bound[p + 1].y - point.y,
                                    interpolated_bound[p + 1].x - point.x);
            angle_prev = std::atan2(point.y - interpolated_bound[p - 1].y,
                                    point.x - interpolated_bound[p - 1].x);

            if (angle_next < 0.)
                angle_next += 2 * M_PI;
            if (angle_prev < 0.)
                angle_prev += 2 * M_PI;

            output.emplace_back(point.x, point.y, (angle_next + angle_prev) / 2.);
            // output.emplace_back(point.x, point.y, angle_next); // (angle_next + angle_prev) / 2.);
        }
    }
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

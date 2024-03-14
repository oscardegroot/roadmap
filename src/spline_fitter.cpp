#include <lanelets_to_path/spline_fitter.h>
#include <spline/Clothoid.h>

#include <ros_tools/logging.h>

SplineFitter::SplineFitter() : logger_(rclcpp::get_logger("roadmap.spline_fitter"))
{
}

void SplineFitter::Initialize(RoadmapConfig *config)
{
    config_ = config;
}

void SplineFitter::FitSplineOnLane(Lane &lane)
{

    // Construct x, y, theta vectors
    std::vector<Waypoint> waypoints;
    waypoints.reserve(lane.nodes.size());

    for (RoadNode &node : lane.nodes)
    {
        // Retrieve waypoints
        waypoints.emplace_back(node.x,
                               node.y,
                               node.theta);
    }

    // Fit a spline over the waypoints
    std::vector<Waypoint> spline_waypoints;
    FitSplineOnWaypoints(waypoints, spline_waypoints, lane);

    // Modify the way object
    lane.nodes.clear();
    for (Waypoint &waypoint : spline_waypoints)
        lane.nodes.push_back(RoadNode(waypoint.x, waypoint.y, waypoint.theta));
}

void SplineFitter::FitSplineOnWaypoints(const std::vector<Waypoint> &waypoints, std::vector<Waypoint> &waypoints_out, Lane &lane)
{
    // First we obtain a set of waypoints (either directly or by fitting a clothoid)
    std::vector<double> x, y, s;
    if (config_->fit_clothoid_)
        FitClothoid(waypoints, x, y, s);
    else
        ConvertWaypointsToVectors(waypoints, x, y, s);

    // Define the splines
    tk::spline ref_path_x, ref_path_y;
    ref_path_x.set_points(s, x);
    ref_path_y.set_points(s, y);
    lane.length = s.back();
    lane.AssignSpline(ref_path_x, ref_path_y); // Assign this spline to the way object

    // Then we fit cubic splines on these waypoints
    FitCubicSpline(lane, waypoints_out);
}

void SplineFitter::FitClothoid(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s)
{
    LOG_INFO("Generating path with clothoid interpolation...");

    double length = 0;
    double k, dk, L;
    int n_clothoid = 0;
    double last_s = 0.;

    std::vector<double> X, Y;
    int max_clothoid_size = std::ceil(500. / config_->clothoid_point_per_xm_);
    X.reserve(max_clothoid_size);
    Y.reserve(max_clothoid_size);

    s.push_back(0);

    // For each set of waypoints in the trajectory
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
        // Build a clothoid and get the X and Y waypoints
        Clothoid::buildClothoid(waypoints[i].x, waypoints[i].y, waypoints[i].theta,
                                waypoints[i + 1].x, waypoints[i + 1].y, waypoints[i + 1].theta,
                                k, dk, L);

        n_clothoid = std::max((int)std::ceil(L / config_->clothoid_point_per_xm_), 2);
        X.resize(n_clothoid);
        Y.resize(n_clothoid);
        Clothoid::pointsOnClothoid(waypoints[i].x, waypoints[i].y, waypoints[i].theta, k, dk, L, n_clothoid, X, Y);

        if (i == 0)
        {
            // For the first one insert the full clothoid
            x.insert(x.end(), X.begin(), X.end()); // Only take the first n_clothoid points!
            y.insert(y.end(), Y.begin(), Y.end());
        }
        else
        {
            // Afterwards, insert all except for the duplicate initial point
            X.erase(X.begin() + 0);
            Y.erase(Y.begin() + 0);
            x.insert(x.end(), X.begin(), X.end());
            y.insert(y.end(), Y.begin(), Y.end());
        }

        length += L;

        // For each point in the clothoid
        for (int j = 1; j < n_clothoid; j++)
        {
            s.push_back(last_s + L / (n_clothoid - 1) * j);
        }
        last_s = s.back();
    }
}

void SplineFitter::ConvertWaypointsToVectors(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s)
{
    LOG_DEBUG("Generating spline without clothoid interpolation");
    assert(waypoints.size() > 1);

    double length = 0.;
    double added_length = 0.;
    double L;

    s.push_back(0);
    x.push_back(waypoints[0].x);
    y.push_back(waypoints[0].y);

    for (size_t i = 1; i < waypoints.size(); i++)
    {
        // Compute the distance between the current waypoint and the last used waypoint
        L = std::sqrt(std::pow(waypoints[i].x - waypoints[i - 1].x, 2.) + std::pow(waypoints[i].y - waypoints[i - 1].y, 2.));

        added_length += L;
        length += L;

        // If the distance is sufficiently large, add the waypoint
        if (added_length > config_->minimum_waypoint_distance_)
        {
            added_length = 0.;
            x.push_back(waypoints[i].x);
            y.push_back(waypoints[i].y);
            s.push_back(length);
        }
    }

    // In the case of just 2 waypoints, we need to add an additional waypoint to have a spline representation
    if (x.size() == 2)
    {
        // Copy into the 2nd waypoint
        x.push_back(x.back());
        y.push_back(y.back());
        s.push_back(s.back());

        // Average for the middle waypoint
        x[1] = (x[0] + x[2]) / 2.;
        y[1] = (y[0] + y[2]) / 2.;
        s[1] = (s[0] + s[2]) / 2.;
    }
}

void SplineFitter::FitCubicSpline(Lane &lane, std::vector<Waypoint> &waypoints_out)
{
    assert(lane.spline_fit);
    double length = lane.length;

    double spline_sample_dist = std::min(config_->spline_sample_distance_, length); // length = 70
    int n_spline_pts = ceil(length / spline_sample_dist);                           // 70  / 10 = 7
    waypoints_out.resize(n_spline_pts);
    double s_cur = 0;

    for (int i = 0; i < n_spline_pts; i++)
    {
        waypoints_out[i].s = s_cur;
        waypoints_out[i].x = lane.spline_x(s_cur);
        waypoints_out[i].y = lane.spline_y(s_cur);
        waypoints_out[i].theta = std::atan2(lane.spline_y.deriv(1, s_cur), lane.spline_x.deriv(1, s_cur));

        s_cur += spline_sample_dist;
    }

    // Check if we are not at our destination yet
    double error = std::sqrt(std::pow(waypoints_out.back().x - lane.spline_x(length), 2) + std::pow(waypoints_out.back().y - lane.spline_y(length), 2));
    if (error > 0.01)
    {
        // Add a final waypoint
        waypoints_out.emplace_back(
            lane.spline_x(length),
            lane.spline_y(length),
            std::atan2(lane.spline_y.deriv(1, length), lane.spline_x.deriv(1, length)));
    }

    LOG_DEBUG("Cubic Spline Fitted");
}

void SplineFitter::VisualizeLaneSpline(RosTools::ROSMarkerPublisher &markers,
                                       const Lane &lane)
{
    VisualizeLaneSpline(markers, lane, lane.type);
}

void SplineFitter::VisualizeLaneSpline(RosTools::ROSMarkerPublisher &markers,
                                       const Lane &lane, int color_idx)
{
    RosTools::ROSLine &line = markers.getNewLine();
    line.setColorInt(color_idx % 20, 20, RosTools::Colormap::VIRIDIS);
    line.setScale(0.1);
    line.setOrientation(0.0);

    geometry_msgs::msg::Point cur, prev;
    cur.z = 21.; // Above autoware map

    for (double s = 0.; s < lane.length; s += 0.1) // Step through the spline
    {
        // Do not plot the middle of the roads (just the edges)
        if (lane.type == 1)
            continue;

        cur.x = lane.spline_x(s);
        cur.y = lane.spline_y(s);

        if (s > 0)
        {
            line.addLine(prev, cur);
        }

        prev = cur;
    }
}

void SplineFitter::VisualizePoints(RosTools::ROSMarkerPublisher &markers,
                                   const std::vector<Waypoint> &points, int r, int g, int b)
{
    RosTools::ROSPointMarker &cylinder = markers.getNewPointMarker("CYLINDER");
    cylinder.setColor(r, g, b);
    cylinder.setScale(0.25, 0.25, 0.01);

    for (auto &point : points)
        cylinder.addPointMarker(Eigen::Vector3d(point.x, point.y, 24.));
}

Waypoint SplineFitter::FindPointClosestToSplineOrthogonal(const Lane &center_lane, const Lane &boundary,
                                                          const Waypoint &waypoint)
{
    // Find the orthogonal to `center_lane` at `waypoint`
    double s_center = FindPointOnSplineClosestTo(center_lane, waypoint, 0., center_lane.length, 0);

    // Construct the line Ax = b
    Eigen::Vector2d A(center_lane.spline_x.deriv(1, s_center), center_lane.spline_y.deriv(1, s_center));
    A.normalize();
    double b = A.transpose() * waypoint.asVector2d();
    Line orthogonal(A, b);

    // For each section of the boundary, find out if it intersects the orthogonal
    const double step_size = 1.;
    std::vector<double> proposal_s;
    for (double s = step_size; s < boundary.length - step_size; s += 2 * step_size)
    {
        if (std::abs(orthogonal.distanceTo(boundary.at(s))) < 4 * step_size) // If sufficiently close, run a bisection
        {
            // std::cout << "Searching between: " << std::max(0., s - step_size) << " and " << std::min(boundary.length, s + step_size) << std::endl;
            double new_s = FindPointOnSplineClosestToLine(boundary, waypoint,
                                                          std::max(0., s - step_size),
                                                          std::min(boundary.length, s + step_size),
                                                          0, orthogonal); // Find the actual s

            // If it intersects
            if (boundary.distanceToLine(new_s, orthogonal) < 1e-4)
                proposal_s.push_back(new_s);
        }
    }

    if (proposal_s.size() == 0)
    {
        // std::cout << "Warning: No proposals found for centerline s: " << s_center << std::endl;
        double best_s = FindPointOnSplineClosestToLine(boundary, waypoint, 0., boundary.length, 0, orthogonal); // Find the actual s
        return Waypoint(boundary.at(best_s));
    }

    double result_s = proposal_s[0];

    if (proposal_s.size() > 1)
    {
        // Find the closest intersection
        double distance = RosTools::distance(boundary.at(result_s), waypoint.asVector2d());
        for (size_t i = 1; i < proposal_s.size(); i++)
        {
            double cur_s = proposal_s[i];

            // Distance from the initial centerline point
            double new_dist = RosTools::distance(boundary.at(cur_s), waypoint.asVector2d());
            if (new_dist < distance)
            {
                distance = new_dist;
                result_s = cur_s;
            }
        }
    }

    return Waypoint(boundary.at(result_s)); // Return the waypoint at that s
}

Waypoint SplineFitter::FindPointOnSplineClosestTo(const Lane &lane, const Waypoint &point) const
{

    // For all waypoints that define the lane, find the closest two

    double s = FindPointOnSplineClosestTo(lane, point, 0., lane.length, 0);
    return Waypoint(lane.spline_x(s), lane.spline_y(s), 0.);
}

double SplineFitter::FindPointOnSplineClosestTo(const Lane &lane, const Waypoint &point,
                                                double low, double high, int num_recursions) const
{
    // Stop after x recursions
    if (num_recursions > 20)
        return (low + high) / 2.;

    // Compute a middle s value
    double mid = (low + high) / 2.;

    // Compute the distance to the spline for high/low
    double value_low = lane.distanceTo(low, point);
    double value_high = lane.distanceTo(high, point);

    // Check the next closest value
    if (value_low < value_high)
        return FindPointOnSplineClosestTo(lane, point, low, mid, num_recursions + 1);
    else
        return FindPointOnSplineClosestTo(lane, point, mid, high, num_recursions + 1);
}

double SplineFitter::FindPointOnSplineClosestToLine(const Lane &lane, const Waypoint &point,
                                                    double low, double high, int num_recursions,
                                                    const Line &line) const
{
    // Stop after x recursions
    if (num_recursions > 20)
        return (low + high) / 2.;

    // Compute a middle s value
    double mid = (low + high) / 2.;

    // Compute the distance to the spline for high/low
    double value_low = lane.distanceToLine(low, line);
    double value_high = lane.distanceToLine(high, line);

    // Check the next closest value
    if (value_low < value_high)
        return FindPointOnSplineClosestToLine(lane, point, low, mid, num_recursions + 1, line);
    else
        return FindPointOnSplineClosestToLine(lane, point, mid, high, num_recursions + 1, line);
}

void SplineFitter::RemoveCornerPoints(std::vector<Waypoint> &points) const
{
    // size_t length = points.size();
    // for (size_t i = length - 2; i > 1; i--) // Iterate backwards and skip first and last
    // {
    //     // Detect a corner
    //     const auto prev_point = points[i - 1].asVector2d();
    //     const auto cur_point = points[i].asVector2d();
    //     const auto next_point = points[i + 1].asVector2d();

    //     Eigen::Vector2d a = cur_point - prev_point;
    //     Eigen::Vector2d b = next_point - cur_point;

    //     double angle = std::acos(a.dot(b) / (a.norm() * b.norm()));
    //     std::cout << angle << std::endl;
    //     if (angle != angle) // If nan, continue
    //         continue;

    //     if (std::abs(angle) > 0.8 * M_PI_2) // || a.norm() < 1e-7)
    //     {
    //         std::cout << "REMOVED CORNER\n";
    //         points.erase(points.begin() + i);
    //     }
    // }
    std::vector<Waypoint> out;
    out.reserve(points.size());
    out.push_back(points[0]);
    for (size_t i = 1; i < points.size() - 1; i++)
    {
        // Detect a corner
        const auto prev_point = points[i - 1].asVector2d();
        const auto cur_point = points[i].asVector2d();
        const auto next_point = points[i + 1].asVector2d();

        Eigen::Vector2d a = cur_point - prev_point;
        Eigen::Vector2d b = next_point - cur_point;

        double angle = std::acos(a.dot(b) / (a.norm() * b.norm()));

        if (angle == angle) // If not nan
        {
            if (std::abs(angle) > 0.8 * M_PI_2) // || a.norm() < 1e-7)
                continue;
        }

        out.push_back(points[i]);
    }
    out.push_back(points.back());
    points = out;
}

void SplineFitter::RemoveCloseTogetherPoints(std::vector<Waypoint> &points) const
{
    size_t length = points.size();
    for (size_t i = length - 2; i > 0; i--) // Iterate backwards and skip first and last
    {
        // Detect a corner
        const auto prev_point = points[i - 1].asVector2d();
        const auto cur_point = points[i].asVector2d();

        if ((cur_point - prev_point).norm() < config_->minimum_waypoint_distance_)
        {
            std::cout << "removed points close together\n";

            points.erase(points.begin() + i);
        }
    }
}

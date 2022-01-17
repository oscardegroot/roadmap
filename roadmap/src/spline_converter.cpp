#include "spline_converter.h"

void SplineConverter::Initialize(ros::NodeHandle &nh, RoadmapConfig *config)
{
    input_map_markers_.reset(new ROSMarkerPublisher(nh, "roadmap/input_map", "map", 500));
    output_map_markers_.reset(new ROSMarkerPublisher(nh, "roadmap/output_map", "map", 2000));

    config_ = config;

    ROADMAP_WARN("Initialized Spline Converter");
}

void SplineConverter::ConvertMap(Map &map)
{
    ROADMAP_INFO("Fitting splines");
    VisualizeMap(map);

    // First fit splines on all defined ways
    for (Way &way : map.ways)
    {
        for (auto &lane : way.lanes)
            FitSplineOnLane(lane);
    }

    // Then define additional ways by translating (i.e., for a sidewalk)

    VisualizeMap(map, true);
}

void SplineConverter::VisualizeMap(Map &map, bool converted_map)
{
    ROADMAP_INFO("Visualizing the map");

    std::unique_ptr<ROSMarkerPublisher> &markers = converted_map ? output_map_markers_ : input_map_markers_;
    ROSPointMarker &cube = markers->getNewPointMarker("Cube");

    double scale;

    for (auto &way : map.ways)
    {
        for (size_t i = 0; i < way.lanes.size(); i++)
        {
            const Lane &lane = way.lanes[i];
            for (const Node &node : lane.nodes)
            {
                if (!converted_map && i == 0)
                    scale = 0.75 * config_->scale_;
                else
                    scale = converted_map ? 0.2 * config_->scale_ : 0.5 * config_->scale_;

                cube.setScale(scale, scale, scale);

                cube.setColor((double)lane.type / (double)20);
                cube.addPointMarker(Eigen::Vector3d(
                    node.x,
                    node.y,
                    0.1));
            }

            if (converted_map)
                VisualizeLaneSpline(lane, map.nodes);
        }
    }

    markers->publish();
}

void SplineConverter::VisualizeLaneSpline(const Lane &lane, std::map<int, Node> &nodes)
{
    ROSLine &line = output_map_markers_->getNewLine();
    line.setColor(0.5, 0.0, 0.0);
    line.setScale(0.1);
    line.setOrientation(0.0);

    geometry_msgs::Point cur, prev;

    for (size_t i = 0; i < lane.nodes.size(); i++)
    {
        const Node &node = lane.nodes[i];
        cur.x = node.x;
        cur.y = node.y;

        if (i > 0)
        {
            line.addLine(prev, cur);
        }

        prev = cur;
    }
}

void SplineConverter::FitSplineOnLane(Lane &lane)
{

    // Construct x, y, theta vectors
    std::vector<Waypoint> waypoints;
    waypoints.reserve(lane.nodes.size());

    for (Node &node : lane.nodes)
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
        lane.nodes.push_back(Node(waypoint.x, waypoint.y, 0.0));
}

void SplineConverter::FitSplineOnWaypoints(const std::vector<Waypoint> &waypoints, std::vector<Waypoint> &waypoints_out, Lane &lane)
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

void SplineConverter::FitClothoid(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s)
{
    ROADMAP_INFO("Reference Path: Generating path with clothoid interpolation...");

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

void SplineConverter::ConvertWaypointsToVectors(const std::vector<Waypoint> &waypoints, std::vector<double> &x, std::vector<double> &y, std::vector<double> &s)
{
    ROADMAP_INFO("Generating spline without clothoid interpolation");
    assert(waypoints.size() > 1);

    double length = 0;
    double L;
    int j = 0;

    s.push_back(0);
    x.push_back(waypoints[0].x);
    y.push_back(waypoints[0].y);

    for (size_t i = 1; i < waypoints.size(); i++)
    {
        // Compute the distance between points
        L = std::sqrt(std::pow(waypoints[i].x - waypoints[j].x, 2) + std::pow(waypoints[i].y - waypoints[j].y, 2));

        // If the distance is sufficiently large, add the waypoint
        if (L > config_->minimum_waypoint_distance_)
        {
            j = i;
            length += L;
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

void SplineConverter::FitCubicSpline(Lane &lane, std::vector<Waypoint> &waypoints_out)
{
    assert(lane.spline_fit);
    double length = lane.length;

    double spline_sample_dist = std::min(config_->spline_sample_distance_, length);
    int n_spline_pts = ceil(length / spline_sample_dist);
    waypoints_out.resize(n_spline_pts);

    double s_cur = 0;
    for (int i = 0; i < n_spline_pts; i++)
    {
        waypoints_out[i].s = s_cur;
        waypoints_out[i].x = lane.spline_x(s_cur);
        waypoints_out[i].y = lane.spline_y(s_cur);

        s_cur += spline_sample_dist;
    }

    // Check if we are not at our destination yet
    double error = std::sqrt(std::pow(waypoints_out.back().x - lane.spline_x.m_y_.back(), 2) + std::pow(waypoints_out.back().y - lane.spline_y.m_y_.back(), 2));
    if (error > 0.01)
    {
        // Add a final waypoint
        waypoints_out.emplace_back(
            lane.spline_x.m_y_.back(),
            lane.spline_y.m_y_.back(),
            lane.spline_x.m_x_.back());
    }

    ROADMAP_INFO("Cubic Spline Fitted");
}
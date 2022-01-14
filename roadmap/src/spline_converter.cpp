#include "spline_converter.h"

void SplineConverter::Initialize(ros::NodeHandle &nh, RoadmapConfig *config)
{
    input_map_markers_.reset(new ROSMarkerPublisher(nh, "roadmap/input_map", "map", 100));
    output_map_markers_.reset(new ROSMarkerPublisher(nh, "roadmap/output_map", "map", 500));

    config_ = config;

    ROS_WARN("Initialized!");
}

void SplineConverter::ConvertMap(Map &map)
{
    VisualizeMap(map);

    // First fit splines on all defined ways
    for (Way &way : map.ways)
        FitSplineOnWay(way, map.nodes);

    // Then define additional ways by translating (i.e., for a sidewalk)

    VisualizeMap(map, true);
}

void SplineConverter::VisualizeMap(Map &map, bool converted_map)
{
    ROS_WARN("Visualizing Map");

    std::unique_ptr<ROSMarkerPublisher> &markers = converted_map ? output_map_markers_ : input_map_markers_;
    ROSPointMarker &cube = markers->getNewPointMarker("Cube");

    double scale = converted_map ? 0.2 : 0.5;
    cube.setScale(scale, scale, scale);

    for (auto &way : map.ways)
    {
        for (int node_idx : way.nodes)
        {
            // Retrieve the current node from the nodes mapping
            const Node &node = map.nodes[node_idx];

            cube.setColor((double)way.type / (double)20);
            cube.addPointMarker(Eigen::Vector3d(
                node.x,
                node.y,
                0.1));
        }

        if (converted_map)
            VisualizeWaySpline(way, map.nodes);
    }

    markers->publish();
}

void SplineConverter::VisualizeWaySpline(const Way &way, std::map<int, Node> &nodes)
{
    ROSLine &line = output_map_markers_->getNewLine();
    line.setColor(0.5, 0.0, 0.0);
    line.setScale(0.1);
    line.setOrientation(0.0);

    geometry_msgs::Point cur, prev;

    for (size_t i = 0; i < way.nodes.size(); i++)
    {
        const Node &node = nodes[way.nodes[i]];
        cur.x = node.x;
        cur.y = node.y;

        if (i > 0)
        {
            line.addLine(prev, cur);
        }

        prev = cur;
    }
}

void SplineConverter::FitSplineOnWay(Way &way, std::map<int, Node> &nodes)
{

    // Construct x, y, theta vectors
    std::vector<Waypoint> waypoints;
    waypoints.reserve(way.nodes.size());

    for (int node_idx : way.nodes)
    {
        // Retrieve the current node from the nodes mapping
        Node &node = nodes[node_idx];

        // Retrieve waypoints
        waypoints.emplace_back(node.x,
                               node.y,
                               node.theta);
    }

    // Fit a spline over the waypoints
    std::vector<Waypoint> spline_waypoints;
    FitSplineOnWaypoints(waypoints, spline_waypoints, way);

    // Modify the way object
    way.nodes.clear();
    for (Waypoint &waypoint : spline_waypoints)
    {
        int node_id = nodes.size();
        nodes[node_id] = Node(node_id, waypoint.x, waypoint.y, 0.0);
        way.nodes.push_back(node_id);
    }
}

void SplineConverter::FitSplineOnWaypoints(const std::vector<Waypoint> &waypoints, std::vector<Waypoint> &waypoints_out, Way &way)
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
    way.length = s.back();
    way.AssignSpline(ref_path_x, ref_path_y); // Assign this spline to the way object

    // Then we fit cubic splines on these waypoints
    FitCubicSpline(way, waypoints_out);
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
}

void SplineConverter::FitCubicSpline(Way &way, std::vector<Waypoint> &waypoints_out)
{
    assert(way.spline_fit);
    double length = way.length;

    double spline_sample_dist = std::min(config_->spline_sample_distance_, length);
    int n_spline_pts = ceil(length / spline_sample_dist);
    waypoints_out.resize(n_spline_pts);

    double s_cur = 0;
    for (int i = 0; i < n_spline_pts; i++)
    {
        waypoints_out[i].s = s_cur;
        waypoints_out[i].x = way.spline_x(s_cur);
        waypoints_out[i].y = way.spline_y(s_cur);

        s_cur += spline_sample_dist;
    }

    ROADMAP_INFO("Spline generated");
}

/////////////////////////////////////////

// void SplineConverter::Init(ros::NodeHandle &nh, predictive_configuration *config)
// {

//     ROS_WARN("Initializing Reference Path");

//     // Save the config
//     config_ = config;

//     // Initialize publishers
//     // Spline
//     spline_pub_ = nh.advertise<nav_msgs::Path>(config_->reference_path_topic_, 1);

//     // Reference Path
//     ros_markers_refpath.reset(new ROSMarkerPublisher(nh, config_->reference_arrows_topic_.c_str(), config_->target_frame_, 100));
//     ros_markers_splineindex.reset(new ROSMarkerPublisher(nh, config_->spline_index_topic_.c_str(), config_->target_frame_, 5));
//     ros_markers_linearboundaries.reset(new ROSMarkerPublisher(nh, "lmpcc/linear_road_constraints", config_->target_frame_, 100));
//     ros_markers_road_limits.reset(new ROSMarkerPublisher(nh, "road_limits", config_->target_frame_, 300));

//     //Road
//     road_pub_ = nh.advertise<visualization_msgs::MarkerArray>("road", 10);

//     // marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("road_limits", 10);

//     // MPCC path variables
//     x_.resize(config_->ref_x_.size());
//     y_.resize(config_->ref_y_.size());
//     theta_.resize(config_->ref_theta_.size());
//     waypoints_size_ = config_->ref_x_.size();

//     // Read the reference from the config
//     ReadSplineConverter();

//     // Initialize the path from file
//     InitPath();

//     ROS_WARN("Reference Path Initialized");
// }

// /* Initialize a path from x,y,theta in the class */
// void SplineConverter::InitPath()
// {

//     spline_index_ = 0;

//     // Construct a spline through these points
//     ConstructSplineConverter(x_, y_, theta_);

//     // Visualizes the given reference points (for debug mostly)
//     PublishSplineConverter();

//     // Visualize the fitted spline
//     PublishSpline();
// }

// /* Initialize a path from x,y,theta given */
// void SplineConverter::InitPath(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &theta)
// {
//     ROS_WARN("Received Reference Path");

//     // Save x,y, theta
//     x_ = x;
//     y_ = y;
//     theta_ = theta;
//     waypoints_size_ = x_.size();

//     // Initialize using these
//     InitPath();
// }

// void SplineConverter::SetParameters(BaseModel *solver_interface, int N_iter, int &param_idx)
// {

//     solver_interface->setParameter(N_iter, param_idx, ref_path_x_.m_a[spline_index_]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 1, ref_path_x_.m_b[spline_index_]);
//     solver_interface->setParameter(N_iter, param_idx + 2, ref_path_x_.m_c[spline_index_]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 3, ref_path_x_.m_d[spline_index_]);
//     solver_interface->setParameter(N_iter, param_idx + 4, ref_path_y_.m_a[spline_index_]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 5, ref_path_y_.m_b[spline_index_]);
//     solver_interface->setParameter(N_iter, param_idx + 6, ref_path_y_.m_c[spline_index_]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 7, ref_path_y_.m_d[spline_index_]);

//     solver_interface->setParameter(N_iter, param_idx + 8, ref_path_x_.m_a[spline_index_ + 1]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 9, ref_path_x_.m_b[spline_index_ + 1]);
//     solver_interface->setParameter(N_iter, param_idx + 10, ref_path_x_.m_c[spline_index_ + 1]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 11, ref_path_x_.m_d[spline_index_ + 1]);
//     solver_interface->setParameter(N_iter, param_idx + 12, ref_path_y_.m_a[spline_index_ + 1]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 13, ref_path_y_.m_b[spline_index_ + 1]);
//     solver_interface->setParameter(N_iter, param_idx + 14, ref_path_y_.m_c[spline_index_ + 1]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 15, ref_path_y_.m_d[spline_index_ + 1]);

//     solver_interface->setParameter(N_iter, param_idx + 16, ref_path_x_.m_a[spline_index_ + 2]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 17, ref_path_x_.m_b[spline_index_ + 2]);
//     solver_interface->setParameter(N_iter, param_idx + 18, ref_path_x_.m_c[spline_index_ + 2]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 19, ref_path_x_.m_d[spline_index_ + 2]);
//     solver_interface->setParameter(N_iter, param_idx + 20, ref_path_y_.m_a[spline_index_ + 2]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 21, ref_path_y_.m_b[spline_index_ + 2]);
//     solver_interface->setParameter(N_iter, param_idx + 22, ref_path_y_.m_c[spline_index_ + 2]); // spline coefficients
//     solver_interface->setParameter(N_iter, param_idx + 23, ref_path_y_.m_d[spline_index_ + 2]);

//     solver_interface->setParameter(N_iter, param_idx + 24, ss_[spline_index_]);            // s1
//     solver_interface->setParameter(N_iter, param_idx + 25, ss_[spline_index_ + 1]);        //s2
//     solver_interface->setParameter(N_iter, param_idx + 26, ss_[spline_index_ + 2]);        //s2
//     solver_interface->setParameter(N_iter, param_idx + 27, ss_[spline_index_ + 1] + 0.02); // d

//     param_idx += 28;
// }

// // Restructure initpath to incorporate a callback!
// // Was ConstructRefPath()
// void SplineConverter::ReadSplineConverter()
// {
//     ROS_WARN("Reading Reference Path");

//     geometry_msgs::Pose pose;
//     tf2::Quaternion myQuaternion;

//     // Iterate over the reference points given
//     for (size_t ref_point_it = 0; ref_point_it < config_->ref_x_.size(); ref_point_it++)
//     {
//         // Create a pose at each position
//         pose.position.x = config_->ref_x_.at(ref_point_it);
//         pose.position.y = config_->ref_y_.at(ref_point_it);

//         // Find the orientation as quaternion
//         tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, (double)config_->ref_theta_[ref_point_it]);
//         pose.orientation.x = q.x();
//         pose.orientation.y = q.y();
//         pose.orientation.z = q.z();
//         pose.orientation.w = q.w();

//         // // Convert from global_frame to planning frame
//         // if (config_->global_path_frame_.compare(config_->target_frame_) != 0)
//         //     transformPose(config_->global_path_frame_, config_->target_frame_, pose);

//         x_[ref_point_it] = pose.position.x;
//         y_[ref_point_it] = pose.position.y;
//         theta_[ref_point_it] = Helpers::quaternionToAngle(pose.orientation);
//     }
// }

// // Recursive clashes with no return type now...
// int SplineConverter::RecursiveClosestPointSearch(BaseModel *solver_interface_ptr, unsigned int cur_traj_i, double &s_guess, double window, int n_tries)
// {

//     if (ss_.size() > 0)
//     {

//         double s_min = ss_[cur_traj_i] - MAX_STEP_BACK_TOLERANCE;
//         double s_max = ss_[cur_traj_i + 1] + MAX_STEP_BACK_TOLERANCE;

//         double lower = std::max(s_min, s_guess - window);
//         double upper = std::min(s_max, s_guess + window);

//         double s_i = upper;
//         double spline_pos_x_i, spline_pos_y_i;
//         double dist_i, min_dist;

//         //First, try the furthest point in our search window. This is the reference that must be beat.
//         double s_best = s_i;
//         spline_pos_x_i = ref_path_x_(s_i);
//         spline_pos_y_i = ref_path_y_(s_i);

//         min_dist = std::sqrt((spline_pos_x_i - solver_interface_ptr->getState()->get_x()) *
//                                  (spline_pos_x_i - solver_interface_ptr->getState()->get_x()) +
//                              (spline_pos_y_i - solver_interface_ptr->getState()->get_y()) *
//                                  (spline_pos_y_i - solver_interface_ptr->getState()->get_y()));

//         //Compute the step size.
//         //Divide by minus one. If you want to go from 1 to 3 (distance two) with three steps, step size must be (3-1)/2=1 to go 1,2,3.
//         double step_size = (upper - lower) / (n_tries - 1);
//         for (s_i = lower; s_i < upper; s_i += step_size)
//         {
//             // Get the current spline position
//             spline_pos_x_i = ref_path_x_(s_i);
//             spline_pos_y_i = ref_path_y_(s_i);

//             // Compute the distance
//             dist_i = std::sqrt((spline_pos_x_i - solver_interface_ptr->getState()->get_x()) *
//                                    (spline_pos_x_i - solver_interface_ptr->getState()->get_x()) +
//                                (spline_pos_y_i - solver_interface_ptr->getState()->get_y()) *
//                                    (spline_pos_y_i - solver_interface_ptr->getState()->get_y()));

//             // Save it if it is the smallest
//             if (dist_i < min_dist)
//             {
//                 min_dist = dist_i;
//                 s_best = s_i;
//             }
//         }

//         // Save the previous best s
//         double previous_guess = s_guess;
//         s_guess = s_best;

//         int next_traj = cur_traj_i;

//         // If the smallest distance is the lower bound of the window
//         if (s_best == lower && lower != previous_guess)
//         {
//             //If we hit the low point of the window, and that low point was the end of this spline segment, try one segment higher!
//             if (lower == s_min && cur_traj_i > 0)
//             {
//                 next_traj--;
//             }
//             return RecursiveClosestPointSearch(solver_interface_ptr, next_traj, s_guess, window, n_tries);
//         }

//         if (s_best == upper && upper != previous_guess)
//         {
//             //If we hit the high point of the window, and that high point was the end of this spline segment, try one segment higher!
//             if (upper == s_max && cur_traj_i < ss_.size() - 2)
//             {
//                 next_traj++;
//             }
//             return RecursiveClosestPointSearch(solver_interface_ptr, next_traj, s_guess, window, n_tries);
//         }
//     }

//     return cur_traj_i;
// }

// void SplineConverter::UpdateClosestPoint(BaseModel *solver_interface_ptr, double &s_guess, double window, int n_tries)
// {

//     spline_index_ = RecursiveClosestPointSearch(solver_interface_ptr, spline_index_, s_guess, window, n_tries);
//     PublishCurrentSplineIndex();
// }

// void SplineConverter::InitializeClosestPoint(BaseModel *solver_interface_ptr)
// {

//     Eigen::Vector2d current_pose(solver_interface_ptr->getState()->get_x(), solver_interface_ptr->getState()->get_y());
//     Eigen::Vector2d trajectory_pose;

//     // Print the distance to the current trajectory index for feedback
//     // trajectory_pose = Eigen::Vector2d(ref_path_x_(ss_[spline_index_]), ref_path_y_(ss_[spline_index_]));
//     // std::cout << "Distance to current spline point: " << Helpers::dist(current_pose, trajectory_pose) << std::endl;
//     // ROS_INFO_STREAM("Current Pose: " << current_pose);

//     double smallest_dist = 9999999.0;
//     double current_dist;
//     int best_i = -1;
//     for (int i = 0; i < (int)ss_.size(); i++)
//     {
//         trajectory_pose = Eigen::Vector2d(ref_path_x_(ss_[i]), ref_path_y_(ss_[i]));
//         // ROS_INFO_STREAM("trajectory_pose: " << current_pose);
//         current_dist = Helpers::dist(current_pose, trajectory_pose);

//         if (current_dist < smallest_dist)
//         {
//             smallest_dist = current_dist;
//             best_i = i;
//         }
//     }

//     if (best_i == -1)
//         ROS_ERROR("Initial spline search failed: No point was found!");

//     // If it succeeded return our best index
//     if (best_i == -1)
//         spline_index_ = std::max(0, int(ss_.size() - 1));
//     else
//         spline_index_ = best_i;

//     // Visualizes the given reference points (for debug mostly)
//     PublishSplineConverter();

//     // Visualize the fitted spline
//     PublishSpline();

//     // Visualize the road limits
//     VisualizeRoad();
// }

// bool SplineConverter::EndOfCurrentSpline(double index)
// {
//     if (ss_.size() > spline_index_ + 1)
//         return index > ss_[spline_index_ + 1];
//     else
//         return true;
// }

// void SplineConverter::UpdateWaypoint(BaseModel *solver_interface_ptr)
// {
//     Eigen::Vector2d current_pose(solver_interface_ptr->getState()->get_x(), solver_interface_ptr->getState()->get_y());
//     Eigen::Vector2d current_goal(x_[spline_index_], y_[spline_index_]);

//     if ((current_pose - current_goal).norm() < config_->epsilon_)
//     {
//         spline_index_ += 1;
//     }

//     unsigned int s = ss_.size() - 1;
//     spline_index_ = std::min(spline_index_, s);
// }

// bool SplineConverter::ReachedEnd()
// {
//     return spline_index_ + 3 >= ss_.size(); // 1 for size, 2 for extra spline parts
// }

// void SplineConverter::ConstructRoadConstraints(BaseModel *solver_interface, std::vector<lmpcc_msgs::halfspace_array> &halfspaces_out)
// {
//     LMPCC_INFO("Reference Path: Constructing linear road constraints.");

//     // For all stages

//     ROSPointMarker &cube = ros_markers_linearboundaries->getNewPointMarker("Cube");
//     cube.setScale(0.5, 0.5, 0.5);

//     geometry_msgs::Point p;
//     p.z = 0.2;

//     // For each stage
//     for (size_t k = 0; k < solver_interface->FORCES_N; k++)
//     {
//         // halfspaces_out[k].halfspaces.clear();

//         double x1_path, y1_path, dx1_path, dy1_path;
//         double x2_path, y2_path, dx2_path, dy2_path;
//         double x3_path, y3_path, dx3_path, dy3_path;
//         double x_path, y_path, dx_path, dy_path;

//         double cur_s = solver_interface->spline(k);
//         // double cur_x = forces_params.x0[3 + (i) * FORCES_TOTAL_V];
//         // double cur_y = forces_params.x0[4 + (i) * FORCES_TOTAL_V];

//         double d = ss_[spline_index_ + 1] + 0.02;
//         double d2 = ss_[spline_index_ + 2] + 0.02;
//         double lambda = 1.0 / (1.0 + std::exp((cur_s - d) / 0.1)); // +0.01?
//         double lambda2 = 1.0 / (1.0 + std::exp((cur_s - d2) / 0.1));
//         // // Cubic spline cost on x^th stage
//         x1_path = ref_path_x_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 3.0) +
//                   ref_path_x_.m_b[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                   ref_path_x_.m_c[spline_index_] * (cur_s - ss_[spline_index_]) +
//                   ref_path_x_.m_d[spline_index_];

//         y1_path = ref_path_y_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 3.0) +
//                   ref_path_y_.m_b[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                   ref_path_y_.m_c[spline_index_] * (cur_s - ss_[spline_index_]) +
//                   ref_path_y_.m_d[spline_index_];

//         // Derivatives
//         dx1_path = 3 * ref_path_x_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                    2 * ref_path_x_.m_b[spline_index_] * (cur_s - ss_[spline_index_]) +
//                    ref_path_x_.m_c[spline_index_];

//         dy1_path = 3 * ref_path_y_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                    2 * ref_path_y_.m_b[spline_index_] * (cur_s - ss_[spline_index_]) +
//                    ref_path_y_.m_c[spline_index_];

//         // Cubic spline cost on x^th stage
//         x2_path = ref_path_x_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 3.0) +
//                   ref_path_x_.m_b[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                   ref_path_x_.m_c[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                   ref_path_x_.m_d[spline_index_ + 1];

//         y2_path = ref_path_y_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 3.0) +
//                   ref_path_y_.m_b[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                   ref_path_y_.m_c[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                   ref_path_y_.m_d[spline_index_ + 1];

//         // Derivatives
//         dx2_path = 3 * ref_path_x_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                    2 * ref_path_x_.m_b[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                    ref_path_x_.m_c[spline_index_ + 1];

//         dy2_path = 3 * ref_path_y_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                    2 * ref_path_y_.m_b[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                    ref_path_y_.m_c[spline_index_ + 1];

//         x3_path = ref_path_x_.m_a[spline_index_ + 2] * std::pow((cur_s - ss_[spline_index_ + 2]), 3.0) +
//                   ref_path_x_.m_b[spline_index_ + 2] * std::pow((cur_s - ss_[spline_index_ + 2]), 2.0) +
//                   ref_path_x_.m_c[spline_index_ + 2] * (cur_s - ss_[spline_index_ + 2]) +
//                   ref_path_x_.m_d[spline_index_ + 2];

//         y3_path = ref_path_y_.m_a[spline_index_ + 2] * std::pow((cur_s - ss_[spline_index_ + 2]), 3.0) +
//                   ref_path_y_.m_b[spline_index_ + 2] * std::pow((cur_s - ss_[spline_index_ + 2]), 2.0) +
//                   ref_path_y_.m_c[spline_index_ + 2] * (cur_s - ss_[spline_index_ + 2]) +
//                   ref_path_y_.m_d[spline_index_ + 2];

//         // Derivatives
//         dx3_path = 3 * ref_path_x_.m_a[spline_index_ + 2] * std::pow((cur_s - ss_[spline_index_ + 2]), 2.0) +
//                    2 * ref_path_x_.m_b[spline_index_ + 2] * (cur_s - ss_[spline_index_ + 2]) +
//                    ref_path_x_.m_c[spline_index_ + 2];

//         dy3_path = 3 * ref_path_y_.m_a[spline_index_ + 2] * std::pow((cur_s - ss_[spline_index_ + 2]), 2.0) +
//                    2 * ref_path_y_.m_b[spline_index_ + 2] * (cur_s - ss_[spline_index_ + 2]) +
//                    ref_path_y_.m_c[spline_index_ + 2];

//         x_path = lambda * x1_path + lambda2 * (1. - lambda) * x2_path + (1 - lambda2) * x3_path;
//         y_path = lambda * y1_path + lambda2 * (1. - lambda) * y2_path + (1 - lambda2) * y3_path;
//         dx_path = lambda * dx1_path + lambda2 * (1. - lambda) * dx2_path + (1 - lambda2) * dx3_path;
//         dy_path = lambda * dy1_path + lambda2 * (1. - lambda) * dy2_path + (1 - lambda2) * dy3_path;

//         Eigen::Vector2d path_point = Eigen::Vector2d(x_path, y_path);
//         Eigen::Vector2d dpath = Eigen::Vector2d(-dy_path, dx_path);
//         Eigen::Vector2d boundary_left = path_point + dpath * config_->road_width_left_;
//         Eigen::Vector2d boundary_right = path_point - dpath * config_->road_width_right_;

//         // Use the derivative to construct linear constraints
//         lmpcc_msgs::halfspace new_halfspace;
//         new_halfspace.A.resize(2);

//         // Left Halfspace
//         {
//             //dx, -dy
//             Eigen::Vector2d A(-dy_path, dx_path);                                                                        // line is parallel to the spline
//             Eigen::Vector2d boundary_left = path_point + dpath * (3 * config_->road_width_left_ - config_->ego_w_ / 2.); // Incorporate the left road side

//             double b = A.transpose() * boundary_left; // And lies on the boundary point

//             new_halfspace.A[0] = A(0);
//             new_halfspace.A[1] = A(1);
//             new_halfspace.b = b;
//             halfspaces_out[k].halfspaces[0] = new_halfspace;
//             // std::cout << "A: " << new_halfspace.A[1] << ", b: " << new_halfspace.b << std::endl;
//         }

//         // Right Halfspace
//         {
//             Eigen::Vector2d A(-dy_path, dx_path); // line is parallel to the spline
//             Eigen::Vector2d boundary_right = path_point - dpath * (config_->road_width_right_ - config_->ego_w_ / 2.);

//             double b = A.transpose() * boundary_right; // And lies on the boundary point

//             new_halfspace.A[0] = -A(0);
//             new_halfspace.A[1] = -A(1);
//             new_halfspace.b = -b;
//             halfspaces_out[k].halfspaces[1] = new_halfspace;
//             // std::cout << "A: " << new_halfspace.A[1] << ", b: " << new_halfspace.b << std::endl;
//         }

//         // p.x = boundary_right(0);
//         // p.y = boundary_right(1);
//         // cube.addPointMarker(p);
//         // p.x = boundary_left(0);
//         // p.y = boundary_left(1);
//         // cube.addPointMarker(p);
//     }

//     // PublishLinearRoadBoundaries(solver_interface, halfspaces_out[10]);
// }

// void SplineConverter::PublishSplineConverter()
// {
//     ROSPointMarker &arrow = ros_markers_refpath->getNewPointMarker("ARROW");
//     ROSLine &line = ros_markers_refpath->getNewLine();
//     line.setColor(0.0, 1.0, 0.0);
//     line.setScale(0.05);
//     line.setOrientation(0.0);
//     arrow.setScale(1.5, 0.3, 0.3);

//     geometry_msgs::Point prev_point;

//     for (unsigned int i = 0; i < config_->ref_x_.size(); i++)
//     {
//         // Draw the constraint as a line
//         geometry_msgs::Point p;
//         p.x = config_->ref_x_[i];
//         p.y = config_->ref_y_[i];
//         p.z = 0.2;

//         if (i > 0)
//             line.addLine(prev_point, p);

//         prev_point = p;

//         arrow.setOrientation(config_->ref_theta_[i]);
//         arrow.addPointMarker(p);
//     }

//     ros_markers_refpath->publish();
// }

// void SplineConverter::PublishSpline(void)
// {
//     // Plot 100 points
//     spline_msg_.poses.resize(200);

//     spline_msg_.header.stamp = ros::Time::now();
//     spline_msg_.header.frame_id = config_->target_frame_;
//     if (spline_msg_.poses.size() != (unsigned int)config_->n_points_spline_)
//         spline_msg_.poses.resize(config_->n_points_spline_);

//     for (unsigned int i = 0; i < spline_msg_.poses.size(); i++)
//     {
//         spline_msg_.poses[i].pose.position.x = ref_path_x_(i * dist_spline_pts_);
//         spline_msg_.poses[i].pose.position.y = ref_path_y_(i * dist_spline_pts_);
//         spline_msg_.poses[i].pose.position.z = 0.15;
//     }

//     spline_pub_.publish(spline_msg_);
// }

// void SplineConverter::PublishCurrentSplineIndex()
// {

//     // Use to debug spline init
//     ROSPointMarker &cube = ros_markers_splineindex->getNewPointMarker("Cube");
//     cube.setScale(0.5, 0.5, 0.5);
//     cube.setOrientation(config_->ref_theta_[spline_index_]);

//     geometry_msgs::Point p;
//     p.z = 0.2;

//     if (xx_.size() > 0)
//     {

//         p.x = xx_[spline_index_];
//         p.y = yy_[spline_index_];
//         cube.addPointMarker(p);

//         cube.setColor(0, 0.8, 0);
//         p.x = xx_[spline_index_ + 1];
//         p.y = yy_[spline_index_ + 1];
//         cube.addPointMarker(p);

//         ros_markers_splineindex->publish();
//     }
// }

// void SplineConverter::PublishLinearRoadBoundaries(BaseModel *solver_interface_ptr, const lmpcc_msgs::halfspace_array &halfspaces_out)
// {
//     LMPCC_INFO("Reference Path: Visualizing linear road constraints.");

//     Helpers::drawLinearConstraints(*ros_markers_linearboundaries, halfspaces_out);
//     ros_markers_linearboundaries->publish();
// }

// void SplineConverter::PublishRoadBoundaries(BaseModel *solver_interface_ptr)
// {

//     visualization_msgs::Marker line_strip;
//     visualization_msgs::MarkerArray line_list;
//     line_strip.header.frame_id = config_->target_frame_;
//     line_strip.id = 1;

//     line_strip.type = visualization_msgs::Marker::LINE_STRIP;

//     // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
//     line_strip.scale.x = 0.2;
//     line_strip.scale.y = 0.2;

//     // Line strip is blue
//     line_strip.color.r = 1.0;
//     line_strip.color.a = 1.0;

//     // Compute contour and lag error to publish
//     geometry_msgs::Point prev_left, prev_right;

//     for (size_t i = 0; i < solver_interface_ptr->FORCES_N; i++)
//     {
//         double x1_path, y1_path, dx1_path, dy1_path;
//         double x2_path, y2_path, dx2_path, dy2_path;
//         double x_path, y_path, dx_path, dy_path;

//         double cur_s = solver_interface_ptr->spline(i);
//         // double cur_x = forces_params.x0[3 + (i) * FORCES_TOTAL_V];
//         // double cur_y = forces_params.x0[4 + (i) * FORCES_TOTAL_V];

//         double d = ss_[spline_index_ + 1] + 0.02;
//         double lambda = 1.0 / (1.0 + std::exp((cur_s - d) / 0.1));

//         // // Cubic spline cost on x^th stage
//         x1_path = ref_path_x_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 3.0) +
//                   ref_path_x_.m_b[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                   ref_path_x_.m_c[spline_index_] * (cur_s - ss_[spline_index_]) +
//                   ref_path_x_.m_d[spline_index_];

//         y1_path = ref_path_y_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 3.0) +
//                   ref_path_y_.m_b[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                   ref_path_y_.m_c[spline_index_] * (cur_s - ss_[spline_index_]) +
//                   ref_path_y_.m_d[spline_index_];

//         // Derivatives
//         dx1_path = 3 * ref_path_x_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                    2 * ref_path_x_.m_b[spline_index_] * (cur_s - ss_[spline_index_]) +
//                    ref_path_x_.m_c[spline_index_];

//         dy1_path = 3 * ref_path_y_.m_a[spline_index_] * std::pow((cur_s - ss_[spline_index_]), 2.0) +
//                    2 * ref_path_y_.m_b[spline_index_] * (cur_s - ss_[spline_index_]) +
//                    ref_path_y_.m_c[spline_index_];

//         // Cubic spline cost on x^th stage
//         x2_path = ref_path_x_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 3.0) +
//                   ref_path_x_.m_b[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                   ref_path_x_.m_c[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                   ref_path_x_.m_d[spline_index_ + 1];

//         y2_path = ref_path_y_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 3.0) +
//                   ref_path_y_.m_b[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                   ref_path_y_.m_c[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                   ref_path_y_.m_d[spline_index_ + 1];

//         // Derivatives
//         dx2_path = 3 * ref_path_x_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                    2 * ref_path_x_.m_b[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                    ref_path_x_.m_c[spline_index_ + 1];

//         dy2_path = 3 * ref_path_y_.m_a[spline_index_ + 1] * std::pow((cur_s - ss_[spline_index_ + 1]), 2.0) +
//                    2 * ref_path_y_.m_b[spline_index_ + 1] * (cur_s - ss_[spline_index_ + 1]) +
//                    ref_path_y_.m_c[spline_index_ + 1];

//         x_path = lambda * x1_path + (1.0 - lambda) * x2_path;
//         y_path = lambda * y1_path + (1.0 - lambda) * y2_path;
//         dx_path = lambda * dx1_path + (1.0 - lambda) * dx2_path;
//         dy_path = lambda * dy1_path + (1.0 - lambda) * dy2_path;

//         Eigen::Vector2d path_point = Eigen::Vector2d(x_path, y_path);
//         Eigen::Vector2d dpath = Eigen::Vector2d(-dy_path, dx_path);
//         Eigen::Vector2d boundary_left = path_point + dpath * config_->road_width_left_;
//         Eigen::Vector2d boundary_right = path_point - dpath * config_->road_width_right_;

//         geometry_msgs::Point left, right;
//         left.x = boundary_left(0);
//         left.y = boundary_left(1);
//         left.z = 0.2;

//         right.x = boundary_right(0);
//         right.y = boundary_right(1);
//         right.z = 0.2;

//         if (i > 0)
//         {
//             line_strip.points.push_back(prev_left);
//             line_strip.points.push_back(left);
//             line_list.markers.push_back(line_strip);
//             line_strip.points.pop_back();
//             line_strip.points.pop_back();
//             line_strip.id++;

//             line_strip.points.push_back(prev_right);
//             line_strip.points.push_back(right);
//             line_list.markers.push_back(line_strip);
//             line_strip.points.pop_back();
//             line_strip.points.pop_back();
//             line_strip.id++;
//         }

//         prev_left = left;
//         prev_right = right;
//     }

//     road_pub_.publish(line_list);
// }

// void SplineConverter::VisualizeRoadLocally(void)
// {
//     visualization_msgs::MarkerArray line_list;
//     double theta, dx, dy;

//     for (size_t i = 1; i < spline_msg_.poses.size() - 1; i++) // 100 points
//     {
//         visualization_msgs::Marker line_strip_left, line_strip_right;

//         line_strip_left.header.frame_id = config_->target_frame_;
//         line_strip_left.id = 2 * i;

//         line_strip_left.type = visualization_msgs::Marker::LINE_STRIP;

//         // Left Road Limit
//         line_strip_left.scale.x = 0.5;
//         line_strip_left.scale.y = 0.5;

//         // Line strip is blue
//         line_strip_left.color.b = 1.0;
//         line_strip_left.color.a = 1.0;

//         dx = spline_msg_.poses[i].pose.position.x - spline_msg_.poses[i - 1].pose.position.x;
//         dy = spline_msg_.poses[i].pose.position.y - spline_msg_.poses[i - 1].pose.position.y;

//         theta = std::atan2(dy, dx);

//         geometry_msgs::Pose pose;

//         pose.position.x = (config_->road_width_left_ + line_strip_left.scale.x / 2.0) * -sin(theta);
//         pose.position.y = (config_->road_width_left_ + line_strip_left.scale.x / 2.0) * cos(theta);

//         geometry_msgs::Point p;
//         p.x = spline_msg_.poses[i - 1].pose.position.x + pose.position.x;
//         p.y = spline_msg_.poses[i - 1].pose.position.y + pose.position.y;
//         p.z = 0.2; //z a little bit above ground to draw it above the pointcloud.

//         line_strip_left.points.push_back(p);

//         p.x = spline_msg_.poses[i].pose.position.x + pose.position.x;
//         p.y = spline_msg_.poses[i].pose.position.y + pose.position.y;
//         p.z = 0.2; //z a little bit above ground to draw it above the pointcloud.

//         line_strip_left.points.push_back(p);

//         line_list.markers.push_back(line_strip_left);

//         // Right Road Limit
//         // Left Road Limit
//         line_strip_right.scale.x = 0.5;
//         line_strip_right.scale.y = 0.5;
//         line_strip_right.color.b = 1.0;
//         line_strip_right.color.a = 1.0;
//         line_strip_right.id = 2 * i + 1;
//         line_strip_right.header.frame_id = config_->target_frame_;
//         line_strip_right.type = visualization_msgs::Marker::LINE_STRIP;

//         pose.position.x = (-config_->road_width_right_ - line_strip_right.scale.x / 2.0) * -sin(theta);
//         pose.position.y = (-config_->road_width_right_ - line_strip_right.scale.x / 2.0) * cos(theta);

//         p.x = spline_msg_.poses[i - 1].pose.position.x + pose.position.x;
//         p.y = spline_msg_.poses[i - 1].pose.position.y + pose.position.y;
//         p.z = 0.2; //z a little bit above ground to draw it above the pointcloud.

//         line_strip_right.points.push_back(p);

//         p.x = spline_msg_.poses[i].pose.position.x + pose.position.x;
//         p.y = spline_msg_.poses[i].pose.position.y + pose.position.y;
//         p.z = 0.2; //z a little bit above ground to draw it above the pointcloud.

//         line_strip_right.points.push_back(p);

//         line_list.markers.push_back(line_strip_right);
//     }

//     marker_pub_.publish(line_list);
// }

// void SplineConverter::VisualizeRoad()
// {
//     ROSLine &line = ros_markers_road_limits->getNewLine();
//     line.setColor(0.5, 0.0, 0.0);
//     line.setScale(0.1);
//     line.setOrientation(0.0);

//     geometry_msgs::Point prev_l, cur_l, prev_r, cur_r, prev_m, cur_m;
//     cur_l.z = -0.5;
//     prev_l.z = -0.5;
//     cur_r.z = -0.5;
//     prev_r.z = -0.5;

//     for (size_t i = 0; i < ss_.size(); i++)
//     {
//         Eigen::Vector2d pose(xx_[i], yy_[i]);

//         // Vector orthogonal to the derivative of the spline
//         Eigen::Vector2d direction(-ref_path_y_.deriv(1, ss_[i]), ref_path_x_.deriv(1, ss_[i]));

//         // Construct the road limits using the orthogonal vector
//         Eigen::Vector2d right = pose - direction * config_->road_width_right_;
//         Eigen::Vector2d mid = pose + direction * config_->road_width_left_;
//         Eigen::Vector2d left = pose + direction * config_->road_width_left_ * 3.;

//         cur_l.x = left(0);
//         cur_l.y = left(1);

//         cur_m.x = mid(0);
//         cur_m.y = mid(1);

//         cur_r.x = right(0);
//         cur_r.y = right(1);

//         if (i > 0)
//         {
//             line.setColor(0.1, 0.1, 0.1); //64./256., 224. / 256., 208 / 256.);
//             line.addLine(prev_l, cur_l);
//             line.addLine(prev_r, cur_r);

//             if (i % 2 == 1)
//             {
//                 line.setColor(249. / 256., 215. / 256., 28 / 256.);
//                 line.addLine(prev_m, cur_m);
//             }
//         }

//         prev_l = cur_l;
//         prev_m = cur_m;
//         prev_r = cur_r;
//     }

//     ros_markers_road_limits->publish();
// }
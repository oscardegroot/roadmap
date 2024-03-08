#include <roadmap/spline_converter.h>

#include <ros_tools/logging.h>

SplineConverter::SplineConverter()
{
}

void SplineConverter::Initialize(rclcpp::Node *node, RoadmapConfig *config)
{
    logger_ = rclcpp::get_logger("roadmap.spline_converter");
    input_map_markers_.reset(new RosTools::ROSMarkerPublisher(node, "roadmap/input_map", "map", 500));
    output_map_markers_.reset(new RosTools::ROSMarkerPublisher(node, "roadmap/output_map", "map", 2000));
    arrow_markers_.reset(new RosTools::ROSMarkerPublisher(node, "roadmap/output_map_arrows", "map", 2000));
    reference_markers_.reset(new RosTools::ROSMarkerPublisher(node, "roadmap/reference_visual", "map", 300));

    config_ = config;

    LOG_WARN("Initialized Spline Converter");
}

Map &SplineConverter::ConvertMap(Map &map)
{
    if (map.ways.size() == 0)
    {
        LOG_WARN("Tried to convert an empty map (returning)");
        return converted_map_; // Needs to also give an error
    }

    LOG_INFO("Fitting splines over " << map.ways.size() << " ways");
    // VisualizeMap(map);

    // First fit splines on all defined ways
    converted_map_ = map;
    for (Way &way : converted_map_.ways)
    {
        for (auto &lane : way.lanes)
            FitSplineOnLane(lane);
    }

    // Then define additional ways by translating (i.e., for a sidewalk)

    // VisualizeMap(map, true);

    if (config_->debug_output_)
    {
        int total_waypoints = 0;
        for (auto &way : converted_map_.ways)
        {
            for (auto &lane : way.lanes)
            {
                for (size_t n = 0; n < lane.nodes.size(); n++)
                    total_waypoints++;
            }
        }
        LOG_INFO("Done converting map (" << total_waypoints << " waypoints)");
    }

    return converted_map_;
}

void SplineConverter::VisualizeReference(const nav_msgs::msg::Path &ref_msg)
{
    auto &lines = reference_markers_->getNewLine();
    lines.setColorInt(3, 5, 0.7);
    lines.setScale(0.1, 0.1);

    auto &points = reference_markers_->getNewPointMarker("CYLINDER");
    points.setColor(0., 0., 0., 1.);   // Black
    points.setScale(0.15, 0.15, 0.15); // Large then the lines

    geometry_msgs::msg::Point cur_point, prev_point;
    bool first = true;
    for (auto &pose : ref_msg.poses)
    {
        points.addPointMarker(pose.pose);

        cur_point.x = pose.pose.position.x;
        cur_point.y = pose.pose.position.y;

        if (!first)
            lines.addLine(prev_point, cur_point);

        prev_point = cur_point;
        first = false;
    }
    reference_markers_->publish();
}

void SplineConverter::VisualizeMap()
{
    LOG_INFO("Visualizing the map");
    bool plot_arrows = false;
    bool plot_cubes = true;

    RosTools::ROSPointMarker &arrow = arrow_markers_->getNewPointMarker("ARROW"); // Note: is expensive to DRAW, only show when debugging.

    double scale = 0.2 * config_->scale_;

    for (auto &way : converted_map_.ways)
    {
        for (size_t i = 0; i < way.lanes.size(); i++) // For all lanes in this way
        {
            const Lane &lane = way.lanes[i];

            if (lane.type == 1)
                continue;

            RosTools::ROSMultiplePointMarker &cube = output_map_markers_->getNewMultiplePointMarker("CUBE"); // Batch rendering (same color and scale)

            if (plot_cubes)
                cube.setScale(scale, scale, scale);

            const RoadNode *prev_node = nullptr;
            for (const RoadNode &node : lane.nodes)
            {
                if (plot_cubes)
                    cube.setColorInt(lane.type, 20, RosTools::Colormap::VIRIDIS);

                if (prev_node && plot_arrows)
                {

                    // For fitted points we plot arrows to indicate the direction
                    arrow.setScale(3 * scale, scale);

                    arrow.setColor((double)lane.type / (double)20);

                    double orientation = std::atan2(node.y - prev_node->y, node.x - prev_node->x);
                    arrow.setOrientation(orientation);
                    arrow.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                }
                if (plot_cubes)
                {
                    cube.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                }
                prev_node = &node;
            }

            VisualizeLaneSpline(*output_map_markers_, lane);

            if (plot_cubes)
                cube.finishPoints();
        }
    }

    // For multiple point markers we need to enforce the draw call

    output_map_markers_->publish();
    if (plot_arrows)
        arrow_markers_->publish();
}

void SplineConverter::VisualizeInputData(Map &map)
{

    LOG_INFO("Visualizing input data");

    RosTools::ROSPointMarker &unique_cubes = input_map_markers_->getNewPointMarker("CUBE"); // Unique per color

    double scale;

    for (auto &way : map.ways)
    {
        for (size_t i = 0; i < way.lanes.size(); i++) // For all lanes in this way
        {
            const Lane &lane = way.lanes[i];

            RosTools::ROSMultiplePointMarker &cube = input_map_markers_->getNewMultiplePointMarker("CUBE"); // Batch rendering (same color and scale)

            bool is_first_node = true;
            for (const RoadNode &node : lane.nodes)
            {
                scale = 0.75 * config_->scale_;

                if (is_first_node) // Color the first node black (with the unique cubes)
                {
                    unique_cubes.setColor(0, 0, 0);

                    scale *= 1.5;
                    unique_cubes.setScale(scale, scale, scale);
                    unique_cubes.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                    is_first_node = false;
                    continue;
                }
                else
                {
                    cube.setColor((double)lane.type / (double)20);
                    cube.setScale(scale, scale, scale);

                    cube.addPointMarker(Eigen::Vector3d(
                        node.x,
                        node.y,
                        0.1));
                }
            }

            cube.finishPoints();
        }
    }

    // For multiple point markers we need to enforce the draw call

    input_map_markers_->publish();
}
#ifndef SPLINE_CONVERTER_H
#define SPLINE_CONVERTER_H

#include <lanelets_to_path/spline_fitter.h>
#include <lanelets_to_path/configuration.h>
#include <lanelets_to_path/types.h>

#include <ros_tools/ros_visuals.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Eigen>

// spline libraries
#include <spline/spline.h>
#include <spline/Clothoid.h>

// Whens earching for the closest point on the path, this variable indicates the distance that the algorithm searches behind the current spline point.
#define MAX_STEP_BACK_TOLERANCE 0.1f

class SplineConverter : public SplineFitter
{

    /**
     * @brief Fits splines on a map and puts the result in `converted_map_`
     */

public:
    SplineConverter();

    void Initialize(rclcpp::Node *node, RoadmapConfig *config);

public:
    Map converted_map_;

    /** @brief Convert the waypoints of a map object into spline representations */
    Map &ConvertMap(Map &map);

    /**
     * @brief Visualize a map
     *
     * @param map the map object
     * @param converted_map is this an output map? Will change the topic and style of visualizations.
     */
    void VisualizeInputData(Map &map);

    void VisualizeMap();
    void VisualizeReference(const nav_msgs::msg::Path &ref_msg);

private:
    /** Two classes for visualization of the map */
    std::unique_ptr<RosTools::ROSMarkerPublisher> input_map_markers_;
    std::unique_ptr<RosTools::ROSMarkerPublisher> output_map_markers_;
    std::unique_ptr<RosTools::ROSMarkerPublisher> arrow_markers_;
    std::unique_ptr<RosTools::ROSMarkerPublisher> reference_markers_;
};
#endif
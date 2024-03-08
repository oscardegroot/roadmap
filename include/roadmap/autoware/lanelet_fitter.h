#ifndef __LANELET_FITTER_H__
#define __LANELET_FITTER_H__

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <roadmap/spline_fitter.h>
#include <roadmap/types.h>

using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;

class LaneletFitter : public SplineFitter
{

public:
    /** @brief Fit splines on just this path with its boundaries */
    LaneletFitter(RoadmapConfig *config, PathWithLaneId &path_with_lane_id);

public:
    void Visualize();

private:
    Lane center_lane, right_boundary, left_boundary;

    std::vector<Waypoint> output_center_lane_, output_right_boundary_, output_left_boundary_;
    std::vector<Waypoint> input_center_lane_, input_right_boundary_, input_left_boundary_;
};

#endif
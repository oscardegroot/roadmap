#ifndef __TYPES_H__
#define __TYPES_H__

#include "roadmap_msgs/RoadPolylineArray.h"
#include "roadmap_msgs/RoadPolyline.h"

#include "spline.h"

struct Node
{

    double x, y, theta;

    Node() {}

    Node(double _x, double _y, double _theta)
        : x(_x), y(_y), theta(_theta)
    {
    }
};

struct Lane
{
    int id, type;
    std::vector<Node> nodes;

    // Variables after spline fitting
    bool spline_fit;
    double length;
    tk::spline spline_x, spline_y; // Cubic spline objects

    Lane(const std::vector<Node> &_nodes, double offset, int _type, int &_id)
        : type(_type)
    {
        spline_fit = false;
        id = _id;
        _id++;

        // Offset the nodes
        nodes.reserve(_nodes.size());
        for (auto &node : _nodes)
        {
            // Create a node at the offset from the center node
            nodes.emplace_back(
                node.x + offset * std::cos(node.theta - M_PI_2),
                node.y + offset * std::sin(node.theta - M_PI_2),
                node.theta);
        }
    }

    // Copies for now
    void AssignSpline(const tk::spline &_spline_x, const tk::spline &_spline_y)
    {
        spline_x = _spline_x;
        spline_y = _spline_y;
        spline_fit = true;
    }
};

struct Way
{
    std::vector<Lane> lanes;
    std::vector<Node> nodes;
    double plus_offset, minus_offset;

    Way()
    {
        plus_offset = 0.;
        minus_offset = 0.;
    }

    void AddNode(const Node &node)
    {
        nodes.emplace_back(node);
    }

    // Add a road by copying the current data and offsetting it
    // Can only be called before the spline is fit!
    void AddLane(const std::string &_type, double width, bool two_way, int &id)
    {
        // The first lane we add needs to offset at the end, the rest at the start
        if (plus_offset != 0.)
        {
            plus_offset += width / 2.;

            if (two_way)
                minus_offset -= width / 2.;
        }

        if (!_type.compare("road"))
        {
            if (two_way)
            {
                lanes.emplace_back(nodes, plus_offset + width / 2., roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY, id);
                lanes.emplace_back(nodes, minus_offset - width / 2., roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY, id);

                lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::ROADLINE_BROKENSINGLEWHITE, id);

                lanes.emplace_back(nodes, minus_offset - width, roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
                lanes.emplace_back(nodes, plus_offset + width, roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
            }
            else
            {
                lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY, id);

                lanes.emplace_back(nodes, minus_offset - width / 2., roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
                lanes.emplace_back(nodes, plus_offset + width / 2., roadmap_msgs::RoadPolyline::ROADEDGEBOUNDARY, id);
            }
        }
        else if (!_type.compare("crosswalk"))
        {
            lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::CROSSWALK, id);
        }
        else if (!_type.compare("sidewalk")) // In some cases, we want a new way object, offset from the current one
        {
            lanes.emplace_back(nodes, plus_offset, roadmap_msgs::RoadPolyline::LANECENTER_SURFACESTREET, id);
            if (two_way)
                lanes.emplace_back(nodes, minus_offset, roadmap_msgs::RoadPolyline::LANECENTER_SURFACESTREET, id);
        }

        // For the first added lane
        if (plus_offset == 0.)
        {
            if (two_way)
            {
                plus_offset += width;
                minus_offset -= width;
            }
            else
            {
                plus_offset += width / 2.;
                minus_offset -= width / 2.;
            }
        }
    }
};

struct Map
{
    std::map<int, Node> nodes;
    std::vector<Way> ways;

    Map()
    {
    }

    void Clear()
    {
        nodes.clear();
        ways.clear();
    }

    // Load the data of this map object into a ros message
    void ToMsg(roadmap_msgs::RoadPolylineArray &msg)
    {
        msg.road_polylines.reserve(ways.size());

        // Load the ways one by one
        for (Way &way : ways)
        {
            for (Lane &lane : way.lanes)
            {
                roadmap_msgs::RoadPolyline line_msg;
                line_msg.type = lane.type;
                line_msg.id = lane.id;

                geometry_msgs::Point point;
                for (Node &node : lane.nodes)
                {
                    point.x = node.x;
                    point.y = node.y;
                    line_msg.coords.push_back(point);
                }

                msg.road_polylines.push_back(line_msg);
            }
        }

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
    }
};

#endif // __TYPES_H__
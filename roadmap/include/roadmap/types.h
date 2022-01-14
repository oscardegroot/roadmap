#ifndef __TYPES_H__
#define __TYPES_H__

#include "roadmap_msgs/RoadPolylineArray.h"
#include "roadmap_msgs/RoadPolyline.h"

#include "spline.h"

struct Node
{

    int id;
    double x, y, theta;

    Node() {}

    Node(int _id, double _x, double _y, double _theta)
        : id(_id), x(_x), y(_y), theta(_theta)
    {
    }
};

struct Way
{
    int id, type;
    std::vector<int> nodes;

    // Variables after spline fitting
    bool spline_fit;
    double length;
    tk::spline spline_x, spline_y; // Cubic spline objects

    Way(int _id)
        : id(_id), length(0.)
    {
        spline_fit = false;
    }

    Way(int _id, int _type, const std::vector<int> &_nodes)
        : id(_id), nodes(_nodes), length(0.)
    {
        spline_fit = false;
    }

    void AddNode(const int &node_idx)
    {
        nodes.push_back(node_idx);
    }

    // Add a road by copying the current data and offsetting it
    // Can only be called before the spline is fit!
    void AddRoad(const char *_type, double width, Way &new_object)
    {
        assert(!spline_fit);

        new_object = false;
        if (_type == "road")
        {
            types.push_back(roadmap_msgs::RoadPolyline::LANECENTER_FREEWAY);
        }
        else if (_type == "crosswalk")
        {
            types.push_back(roadmap_msgs::RoadPolyline::CROSSWALK);
        }
        else if (_type == "sidewalk") // In some cases, we want a new way object, offset from the current one
            types.push_back(roadmap_msgs::RoadPolyline::LANECENTER_SURFACESTREET);

        CopyWithOffset(new_object, offset);
    }

    void CopyWithOffset(Way &new_object, double offset)
    {
    }

    // Copies for now
    void AssignSpline(const tk::spline &_spline_x, const tk::spline &_spline_y)
    {
        spline_x = _spline_x;
        spline_y = _spline_y;
        spline_fit = true;
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
            roadmap_msgs::RoadPolyline line_msg;
            line_msg.type = way.type;
            line_msg.id = way.id;

            geometry_msgs::Point point;
            for (int node_idx : way.nodes)
            {
                point.x = nodes[node_idx].x;
                point.y = nodes[node_idx].y;
                line_msg.coords.push_back(point);
            }

            msg.road_polylines.push_back(line_msg);
        }

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
    }
};

#endif // __TYPES_H__
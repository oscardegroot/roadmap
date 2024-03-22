#include <lanelets_to_path/types.h>

#include <ros_tools/math.h>
#include <ros_tools/convertions.h>

Waypoint::Waypoint()
{
}

Waypoint::Waypoint(double _x, double _y, double _theta, double _s)
    : x(_x), y(_y), theta(_theta), s(_s)
{
}
Waypoint::Waypoint(double _x, double _y, double _theta)
    : x(_x), y(_y), theta(_theta), s(-1.)
{
}

Waypoint::Waypoint(const Eigen::Vector2d &point)
{
        x = point(0);
        y = point(1);
}

Eigen::Vector2d Waypoint::asVector2d() const { return Eigen::Vector2d(x, y); }

Line::Line(const Eigen::Vector2d &_A, const double _b) : A(_A), b(_b)
{
}

double Line::distanceTo(const Waypoint &waypoint) const
{
        return std::abs(A.transpose() * waypoint.asVector2d() - b);
}

double Line::distanceTo(const Eigen::Vector2d &waypoint) const
{
        return std::abs(A.transpose() * waypoint - b);
}

void WaypointVectorToGeometryPointVector(const std::vector<Waypoint> &waypoints,
                                                std::vector<geometry_msgs::msg::Point> &out)
{
        out.clear();
        geometry_msgs::msg::Point p;
        for (auto &waypoint : waypoints)
        {
                p.x = waypoint.x;
                p.y = waypoint.y;
                out.emplace_back(p);
        }
}

RoadNode::RoadNode() {}

RoadNode::RoadNode(double _x, double _y, double _theta)
    : x(_x), y(_y), theta(_theta)
{
}

RoadNode::RoadNode(double _lat, double _long, double base_latitude, double base_longitude, double _theta)
{
        x = GLOBE_RADIUS * (_long - base_longitude) * std::cos(0.);
        y = GLOBE_RADIUS * (_lat - base_latitude);
        theta = _theta;
}

Lane::Lane() { spline_fit = false; };

/**
 * @brief Construct a new Lane object
 *
 * @param _nodes nodes of the centerlane
 * @param offset offset with respect to the centerlane for this lane
 * @param _type type of lane (see road_msgs/RoadPolyline)
 * @param _id current ID to add this lane on (and is increased after usage)
 */
Lane::Lane(const std::vector<RoadNode> &_nodes, double offset, int _type, int &_id)
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

void Lane::Reverse()
{
        std::reverse(nodes.begin(), nodes.end());

        for (size_t n = 0; n < nodes.size(); n++)
                nodes[n].theta += M_PI; // Reverse the direction
}

/**
 * @brief Assign a spline to this lane
 */
void Lane::AssignSpline(const tk::spline &_spline_x, const tk::spline &_spline_y)
{
        spline_x = _spline_x;
        spline_y = _spline_y;
        spline_fit = true;
}

double Lane::distanceTo(const double s, const Waypoint &point) const
{
        return RosTools::distance(at(s), Eigen::Vector2d(point.x, point.y));
}

double Lane::distanceToLine(const double s, const Line &line) const
{
        return line.distanceTo(at(s));
}

Eigen::Vector2d Lane::at(const double s) const
{
        return Eigen::Vector2d(spline_x(s), spline_y(s));
}

Way::Way()
{
        plus_offset = 0.;
        minus_offset = 0.;
}

void Way::Reverse()
{

        std::reverse(nodes.begin(), nodes.end());
        for (size_t n = 0; n < nodes.size(); n++)
                nodes[n].theta += M_PI;

        for (size_t l = 0; l < lanes.size(); l++)
                lanes[l].Reverse();
}

/**
 * @brief Add a node to the center line of this way
 *
 * @param node the node to add
 */
void Way::AddNode(const RoadNode &node)
{
        nodes.emplace_back(node);
}

/**
 * @brief Add a road by copying the current data and offsetting it.
 * Can only be called before the spline is fit!
 *
 * @param _type type of lane to add
 * @param width width of the lane
 * @param two_way is it a two way lane?
 * @param id ID to start at
 */
void Way::AddLane(const std::string &_type, double width, bool two_way, int &id)
{
        // The first lane we add needs to offset at the end, the rest at the start
        if (plus_offset != 0.)
        {
                plus_offset += width / 2.;

                if (two_way)
                        minus_offset -= width / 2.;
        }

        lanes.emplace_back(nodes, plus_offset, 1, id); // Main road is ON the reference

        // For the first added lane
        if (plus_offset == 0.)
        {
                if (two_way)
                {
                        plus_offset += width / 2.;
                        minus_offset -= 3. * width / 2.;
                }
                else
                {
                        plus_offset += width / 2.;
                        minus_offset -= width / 2.;
                }
        }
}

Map::Map()
{
}

/**  @brief Clear the map  */
void Map::Clear() { ways.clear(); }

/** @brief Reverse the direction of the reference path */
void Map::Reverse()
{
        for (size_t w = 0; w < ways.size(); w++)
        {
                // std::cout << ways[w].nodes.back().y << std::endl;
                ways[w].Reverse();
                // std::cout << ways[w].nodes.back().y << std::endl;
        }
}
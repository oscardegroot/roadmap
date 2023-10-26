#ifndef __READER_H__
#define __READER_H__

#include <roadmap/types.h>
#include <roadmap/configuration.h>
#include <ros_tools/helpers.h>

#include <rclcpp/rclcpp.hpp>
// #include <ros/package.h>
#include <string>
#include <vector>
#include <stdlib.h> /* atoi */
#include <map>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <asr_rapidxml/rapidxml_utils.hpp>

class Map;
static const rclcpp::Logger READ_LOGGER = rclcpp::get_logger("roadmap.reader");
class Reader
{

    /**
     * @brief Class for reading map files and converting it into necessary formats
     * @see types.h
     */
public:
    Reader(RoadmapConfig *config)
        : config_(config)
    {
        from_callback_ = false;
        offset_ = geometry_msgs::msg::Pose();
        offset_.position.x = 0.;
        offset_.position.y = 0.;

        offset_.orientation.x = 0.;
        offset_.orientation.y = 0.;
        offset_.orientation.z = 0.;
        offset_.orientation.w = 1.;
    }

public:
    /**
     * @brief Read a map file
     *
     * @param file_name file name relative to roadmap package (either xml or yaml files)
     */
    void Read(rclcpp::Node::SharedPtr node, const std::string &file_name); // With custom name

    /**
     * @brief Read the file as defined in the config
     */
    void Read(rclcpp::Node::SharedPtr node);

    /**
     * @brief Read wrapper for r values
     */
    void Read(rclcpp::Node::SharedPtr node, const std::string &&file_name);

    /**
     * @brief Get a reference to the loaded map
     * @see types.h
     * @return const Map
     */
    Map &GetMap() { return map_; };

    /**
     * @brief Callback for external waypoints instead of reading a file
     *
     * @param msg the reference as a nav_msgs::Path
     */
    void WaypointCallback(const nav_msgs::msg::Path &msg);

    /**
     * @brief Debug callback for translating the map with a Pose
     *
     * @param msg Translation
     */
    void OffsetCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);

private:
    Map map_;               /** The read map */
    RoadmapConfig *config_; /** Parameters */

    geometry_msgs::msg::Pose offset_;

    bool from_callback_; /** Are we reading a callback instead of a file? */

    /**
     * @brief Read an XML file with map data
     *
     * @param file the file path to read from
     */
    void ReadXML(const std::string &file);

    /**
     * @brief Read a yaml file with map data
     *
     * @param file the file path to read from
     */
    void ReadYAML(rclcpp::Node::SharedPtr node, const std::string &file);

    /**
     * @brief Read an OSM file with map data
     *
     * @param doc file to read from
     */
    void ReadOSM(const std::string &file);
};

#endif // __READER_H__
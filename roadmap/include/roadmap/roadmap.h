#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include <roadmap/reader.h>
#include <roadmap/types.h>
#include <roadmap/spline_converter.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <roadmap_msgs/msg/road_polyline_array.hpp>

#include <rclcpp/rclcpp.hpp>
#include <string>

class Roadmap : public rclcpp::Node
{

    /**
     * @brief Main class for the roadmap node
     *
     */
public:
    Roadmap();

    void Initialize();

public:
    /**
     * @brief Callback for external waypoints
     *
     * @param msg the reference as a nav_msgs::Path
     */
    void WaypointCallback(const nav_msgs::msg::Path &msg);

    void OffsetCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg);

    void ResetCallback(const std_msgs::msg::Empty &msg);

    void ReverseCallback(const std_msgs::msg::Empty &msg);

private:
    std::unique_ptr<RoadmapConfig> config_; /** Parameters */

    std::unique_ptr<SplineConverter> spline_converter_; /** Spline convertion class */
    std::unique_ptr<Reader> reader_;                    /** File reader class */

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoints_sub_;                        /** Subscriber for external waypoints */
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr offset_sub_; /** Subscriber for external waypoints */
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;                           /** Subscriber for resetting the map */
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reverse_sub_;                         /** Subscriber for reversing the map */
    rclcpp::Publisher<roadmap_msgs::msg::RoadPolylineArray>::SharedPtr map_pub_;                /** Publisher for road polyline output */
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_pub_;                           /** Publisher for road polyline output */
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;                    /** Publisher for road polyline output */

    roadmap_msgs::msg::RoadPolylineArray road_msg_; // Setup the message
    nav_msgs::msg::Path ref_msg_;                   // Setup the message

    rclcpp::TimerBase::SharedPtr timer_;
    bool is_reversed_;

    int runs_ = 0;

    /**
     * @brief Function that reads the map, currently repeated on a frequency
     */
    void Poll();

    /**
     * @brief Read the map from file and visualize the input
     */
    void ReadFromFile();

    /**
     * @brief Convert the map and create ros messages
     */
    void ConvertMap();

    // /**
    //  * @brief Reset to load and apply changed map
    //  */
    // void Reset();
};

#endif // __ROADMAP_H__
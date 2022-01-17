#ifndef HELPERS_H
#define HELPERS_H

/** Logging Macros */
#define ROADMAP_INFO(msg)                      \
    if (config_->debug_output_)                \
    {                                          \
        ROS_INFO_STREAM("[Roadmap]: " << msg); \
    }

#define ROADMAP_WARN(msg)                      \
    if (config_->debug_output_)                \
    {                                          \
        ROS_WARN_STREAM("[Roadmap]: " << msg); \
    }

#define ROADMAP_ERROR(msg) ROS_ERROR_STREAM("[Roadmap]: " << msg)

#define ROADMAP_INFO_STREAM(msg)               \
    if (config_->debug_output_)                \
    {                                          \
        ROS_INFO_STREAM("[Roadmap]: " << msg); \
    }

#define ROADMAP_WARN_STREAM(msg)               \
    if (config_->debug_output_)                \
    {                                          \
        ROS_WARN_STREAM("[Roadmap]: " << msg); \
    }

#define ROADMAP_ERROR_STREAM(msg) ROS_ERROR_STREAM("[Roadmap]: " << msg)

#define ROADMAP_INFO_ALWAYS(msg) ROS_INFO_STREAM("[Roadmap]: " << msg)
#define ROADMAP_WARN_ALWAYS(msg) ROS_WARN_STREAM("[Roadmap]: " << msg)

#endif
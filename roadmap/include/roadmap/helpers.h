#ifndef __HELPERS_H__
#define __HELPERS_H__

/** Logging Macros */
#define ROADMAP_INFO(logger, msg)        \
    if (config_->debug_output_)          \
    {                                    \
        RCLCPP_INFO_STREAM(logger, msg); \
    }

#define ROADMAP_WARN(logger, msg)        \
    if (config_->debug_output_)          \
    {                                    \
        RCLCPP_WARN_STREAM(logger, msg); \
    }

#define ROADMAP_ERROR(logger, msg) RCLCPP_ERROR_STREAM(logger, msg)

#define ROADMAP_INFO_STREAM(logger, msg) \
    if (config_->debug_output_)          \
    {                                    \
        RCLCPP_INFO_STREAM(logger, msg); \
    }

#define ROADMAP_WARN_STREAM(logger, msg) \
    if (config_->debug_output_)          \
    {                                    \
        RCLCPP_WARN_STREAM(logger, msg); \
    }

#define ROADMAP_ERROR_STREAM(logger, msg) RCLCPP_ERROR_STREAM(logger, msg)

#define ROADMAP_INFO_ALWAYS(logger, msg) RCLCPP_INFO_STREAM(logger, msg)
#define ROADMAP_WARN_ALWAYS(logger, msg) RCLCPP_WARN_STREAM(logger, msg)

#endif // __HELPERS_H__
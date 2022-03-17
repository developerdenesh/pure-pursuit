#include "pure_pursuit_local_planner/pure_pursuit_local_planner.h"

// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

// PLUGINLIB_DECLARE_CLASS has been changed to PLUGINLIB_EXPORT_CLASS in ROS Noetic
// Changing all tf::TransformListener* to tf2_ros::Buffer*
PLUGINLIB_EXPORT_CLASS(pure_pursuit::PurePursuitPlannerROS, nav_core::BaseLocalPlanner)

namespace pure_pursuit_local_planner
{

    PurePursuitPlannerROS::PurePursuitPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

    PurePursuitPlannerROS::PurePursuitPlannerROS(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        // initialize planner
        initialize(name, tf, costmap_ros);
    }

    PurePursuitPlannerROS::~PurePursuitPlannerROS() {}

    void PurePursuitPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {

    }

    bool PurePursuitPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
    {
        return true;
    }

    bool PurePursuitPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        return true;
    }

    bool PurePursuitPlannerROS::isGoalReached()
    {
        return true;
    }

}
#ifndef PURE_PURSUIT_LOCAL_PLANNER_ROS_H_
#define PURE_PURSUIT_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

using namespace std;


namespace pure_pursuit_local_planner{
  class PurePursuitPlannerROS : public nav_core::BaseLocalPlanner{

    public:

      PurePursuitPlannerROS();
  
      PurePursuitPlannerROS(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      ~PurePursuitPlannerROS();

      void initialize(std::string name, tf2_ros::Buffer* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      bool isGoalReached();

    private:

  };
};

#endif
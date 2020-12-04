#include "ros/ros.h"
#include "move_class.hpp"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "space_exploration");
    ros::NodeHandle n;

    Move ObjectGoal(n);

    while(ros::ok()){
     ObjectGoal.robot_goal();
     ros::spinOnce();
   }

    return 0;
}

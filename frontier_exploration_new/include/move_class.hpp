#ifndef MOVE_CLASS_H
#define MOVE_CLASS_H

#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <math.h>
#include "frontier_class.hpp"
#include "geometry_msgs/Twist.h"

class FrontierGoals{
public:
  virtual void robot_goal()=0;


};
class Move: public FrontierGoals{
private:
  geometry_msgs::Twist velMsg;
  ros::Publisher velPub;


public:
  Move(ros::NodeHandle& nodehandle);

   ImageFilter ObjectFrontier;
   float x_short;
   float y_short;


   typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

   float distance(float x1, float y1, float x2, float y2);

   void rotate();

   void robot_goal();

}; //end of Move class.
#endif

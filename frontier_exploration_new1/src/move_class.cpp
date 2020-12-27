#include "ros/ros.h"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <math.h>
#include "move_class.hpp"
#include "geometry_msgs/Twist.h"


Move::Move(ros::NodeHandle& nodehandle):ObjectFrontier(nodehandle){
  velPub = nodehandle.advertise <geometry_msgs::Twist> ("cmd_vel", 100);        //amr/cmd_vel1
}

float Move::distance(float x1, float y1, float x2, float y2){
     dist = pow((x1-x2),2) + pow((y1-y2),2);
     return dist;
   }

void Move::rotate(){


  velMsg.linear.x = 0.0;
  velMsg.linear.y = 0.0;
  velMsg.linear.z = 0.0;
  velMsg.angular.x = 0.0;
  velMsg.angular.y = 0.0;
  velMsg.angular.z = 0.5;

  std::cout << "start rotating" << "\n";
  begin = ros::Time::now();
  waitTime = ros::Duration(14);
  end = begin + waitTime;
  std::cout << "[ Start time:]" << begin << std::endl;

  ros::Rate loop_rate(10);
  while (ros::Time::now() < end && ros::ok()) {
    // Publish the velocity
    velPub.publish(velMsg);
    loop_rate.sleep();
  }
  ROS_INFO_STREAM("[ End time:]" << ros::Time::now());
}


void Move::robot_goal(){


     ObjectFrontier.detection();

     lis.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));             //amr/base_link
     lis.lookupTransform("/map", "/base_link", ros::Time(0), transform);                        //amr//base_link

     std::cout <<"Robot position w.r.t map frame: "<<transform.getOrigin().x() << " "<<transform.getOrigin().y() << "\n";

     frontier_vector.reserve(ObjectFrontier.frontier_vec.size());
     frontier_vector = ObjectFrontier.frontier_vec;

     std::cout << frontier_vector.size() << " frontiers point active.\n";

     if(frontier_vector.size()==0){
       std::cout << "Map of the environment is ready." << "\n";
       ros::shutdown();
     }

     else{
     float short_distance = 1000;

     for(int i=0;i<frontier_vector.size();i++){
         float x = frontier_vector[i][0];
         float y = frontier_vector[i][1];


       float dist = distance(x,y,transform.getOrigin().x(),transform.getOrigin().y());
       if (dist < short_distance){
         short_distance = dist;
         x_short = x;
         y_short = y;
       }

     }
      std::cout << "Shortest goal coordinates: "<< x_short << " " << y_short << "\n";

      MoveBaseClient ac("move_base", true);                                             //amr/move_base

      while(!ac.waitForServer(ros::Duration(5.0))){
         ROS_INFO("Waiting for the move_base action server to come up");
       }

       goal.target_pose.header.frame_id = "map";
       goal.target_pose.header.stamp = ros::Time::now();

       goal.target_pose.pose.position.x = x_short;
       goal.target_pose.pose.position.y = y_short;
       goal.target_pose.pose.position.z = 0;
       goal.target_pose.pose.orientation.x = 0.0;
       goal.target_pose.pose.orientation.y = 0.0;
       goal.target_pose.pose.orientation.z = 0.0;
       goal.target_pose.pose.orientation.w = 1.0;



       ROS_INFO("Sending goal");
       ac.sendGoal(goal);

       ac.waitForResult();

       if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
         ROS_INFO("Reached");}
       else{
         ROS_INFO("Fail");}

       }

   }

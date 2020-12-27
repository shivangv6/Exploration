#ifndef FRONTIER_CLASS_H
#define FRONTIER_CLASS_H

#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include "exploration.hpp"


class ImageFilter: public Exploration
{
private:
    ros::Publisher poseArrayPub;
    geometry_msgs::PoseArray poseArray;
    geometry_msgs::Pose pose;
    nav_msgs::MapMetaData info;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> msg;
    boost::shared_ptr<nav_msgs::OccupancyGrid const> costmap_msg;

    int map_width;
    int map_height;
    float map_reso;
    float map_origin_x;
    float map_origin_y;
    int n_by_two_points_arr[2];

    int product;
    cv::Mat final_map_image;

    int nLabels;
    int index;
    int temp;
    int number_of_connected_components;
    float x;
    float y;
    int* n_by_two_point;
    cv::Mat map_image;
    cv::Mat global_costmap_image;
    std::vector<int> all_frontier_points_vector;
    // cv::Mat labelImage;

public:

    ImageFilter(ros::NodeHandle& n);

    int count_components(int number,std::vector<int> all_frontier_points_vector);

    int * n_by_two(int number,std::vector<int> all_frontier_points_vector, cv::Mat labelImage);

    void map_information();

    void goal_points(cv::Mat labelImage);

    void detection();

}; //end of class ImageFilter.
#endif

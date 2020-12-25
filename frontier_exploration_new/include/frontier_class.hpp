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



class DetectFrontiers{
public:
  virtual void detection()=0;
};


class ImageFilter: public DetectFrontiers
{
private:
    ros::Publisher poseArrayPub;
    // ros::Subscriber costmapSub;
    // ros::NodeHandle n;
    int map_width;
    int map_height;
    float map_reso;
    float map_origin_x;
    float map_origin_y;
    int n_by_two_points_arr[2];



public:
    cv::Mat A;
    cv::Mat B;

    std::vector<std::vector<float>> frontier_vec;


      ImageFilter(ros::NodeHandle& n);

      void useful_pixel(cv::Mat temp);


      int count_components(int number,std::vector<int> vec);

      int * n_by_two(int number,std::vector<int> vec, cv::Mat labelImage);

      void detection();

}; //end of class ImageFilter.
#endif

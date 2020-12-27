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
#include "frontier_class.hpp"



ImageFilter::ImageFilter(ros::NodeHandle& n)
{
      poseArrayPub = n.advertise<geometry_msgs::PoseArray>("array", 10000);
}


int ImageFilter::count_components(int number,std::vector<int> all_frontier_points_vector){
        return count(all_frontier_points_vector.begin(),all_frontier_points_vector.end(),number);
      }

int * ImageFilter::n_by_two(int number,std::vector<int> all_frontier_points_vector, cv::Mat labelImage){
        index = 0;
        number_of_connected_components = count_components(number,all_frontier_points_vector);
        if (number_of_connected_components % 2 == 0){
           temp = number_of_connected_components/2;
        }
        else{
           temp = (number_of_connected_components+1)/2;
        }
        for(int i=0;i<map_height;i++){
          for(int j=0;j<map_width;j++){
            if(labelImage.at<int>(i,j)==number){
              index = index + 1;
              if(index == temp){
                 n_by_two_points_arr[0] = i;
                 n_by_two_points_arr[1] = j;
                return n_by_two_points_arr;
              }
            }
          }
        }
      }

void ImageFilter::map_information(){
  msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
  info = msg->info;
  map_width = info.width;
  map_height = info.height;
  map_reso = info.resolution;
  map_origin_x = info.origin.position.x;
  map_origin_y = info.origin.position.y;
  product = map_width * map_height;
  ROS_INFO_STREAM("Map Width: " << map_width << ", Map Height: " << map_height
                  << ", Map Resolution: " << map_reso
                  << ", Map Origin: [" << map_origin_x << "," << map_origin_y << "]");

}

void ImageFilter::goal_points(cv::Mat labelImage){
  poseArray.poses.clear();     // Clear last block perception result
  poseArray.header.stamp = ros::Time::now();
  poseArray.header.frame_id = "/map";

  for(int i=1; i<nLabels; i++){
    if(count_components(i,all_frontier_points_vector) > 35){
      n_by_two_point = n_by_two(i,all_frontier_points_vector,labelImage);
      pose.position.x = (n_by_two_point[1]*map_reso)+map_origin_x;
      pose.position.y = ((map_height-n_by_two_point[0])*map_reso)+map_origin_y;
      x = (n_by_two_point[1]*map_reso)+map_origin_x;
      y = ((map_height-n_by_two_point[0])*map_reso)+map_origin_y;
      std::cout << x << " " << y << "\n";
      frontier_vec.push_back({x,y});
      pose.position.z = 0;
      pose.orientation.x = 0;
      pose.orientation.y = 0;
      pose.orientation.z = 0;
      pose.orientation.w = 1;
      poseArray.poses.push_back(pose);

    }
  }

  poseArrayPub.publish(poseArray);

 }

void ImageFilter::detection()
    {   frontier_vec.clear();
        all_frontier_points_vector.clear();

        map_information();

        short int arr[product];

        for(int x = 0; x < product; x++){
          arr[x] = msg->data[x];
        }

        // converting to gray image form.
        for(int i = 0; i < product;i++){
          if(arr[i] == -1){
            arr[i] = 205;
          }
          else if(arr[i] == 100){
            arr[i] = 0;
          }
          else {
            arr[i] = 254;
              }
            }

          map_image = cv::Mat(map_height,map_width,CV_16U,arr);
          cv::flip(map_image,map_image,0);

          // cv::imwrite("map_image.jpg",map_image);

          final_map_image = cv::Mat::zeros(map_height,map_width,CV_16U);


          costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap");    //amr/move_base/global_costmap/costmap

          short int arr_cost[product];

          for(int x = 0; x < product; x++){
            arr_cost[x] = costmap_msg->data[x];
          }

          global_costmap_image = cv::Mat(map_height,map_width,CV_16U,arr_cost);
          cv::flip(global_costmap_image,global_costmap_image,0);

          for(int i=0;i<map_height;i++){
            for(int j=0;j<map_width;j++){
              if(map_image.at<short>(i,j) == 254 &&  0 <= global_costmap_image.at<short>(i,j) <=80){
                if(map_image.at<short>(i-1,j-1) == 205 || map_image.at<short>(i-1,j) == 205 || map_image.at<short>(i-1,j+1) == 205 || map_image.at<short>(i,j-1) == 205 || map_image.at<short>(i,j+1) == 205 || map_image.at<short>(i+1,j-1) == 205 || map_image.at<short>(i+1,j) == 205 || map_image.at<short>(i+1,j+1) == 205){
                  final_map_image.at<short>(i,j) = 255;
                }
              }
            }
          }

          // cv::imwrite("final_image_short.jpg",final_map_image);

          final_map_image.convertTo(final_map_image,CV_8U);

          cv::Mat labelImage(final_map_image.size(),CV_8U);
          nLabels = connectedComponents(final_map_image,labelImage, 8);

          std::cout << "I am here" << "\n";

          for(int i=0;i<map_height;i++){
            for(int j=0;j<map_width;j++){
              all_frontier_points_vector.push_back(labelImage.at<int>(i,j));
            }
          }

          goal_points(labelImage);

    }

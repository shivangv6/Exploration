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
      // costmapSub = n.subscribe("/move_base/global_costmap/costmap", 1000, &Listener::callback, this)
}

void ImageFilter::useful_pixel(cv::Mat temp){
        for (int i=0;i<map_height;i++){
          for (int j=0;j<map_width;j++){
            if(temp.at<short>(i,j)!=49){
              temp.at<short>(i,j) = 0;
            }
          }
        }
      }

int ImageFilter::count_components(int number,std::vector<int> vec){
        return count(vec.begin(),vec.end(),number);
      }

int * ImageFilter::n_by_two(int number,std::vector<int> vec, cv::Mat labelImage){
        int cou = 0;
        int l;
        int number_unique = count_components(number,vec);
        if (number_unique % 2 == 0){
           l = number_unique/2;
        }
        else{
           l = (number_unique+1)/2;
        }
        for(int i=0;i<map_height;i++){
          for(int j=0;j<map_width;j++){
            if(labelImage.at<int>(i,j)==number){
              cou = cou + 1;
              if(cou == l){
                 n_by_two_points_arr[0] = i;
                 n_by_two_points_arr[1] = j;
                return n_by_two_points_arr;
              }
            }
          }
        }
      }


void ImageFilter::detection()
    {   frontier_vec.clear();
        boost::shared_ptr<nav_msgs::OccupancyGrid const> msg;
        msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
        nav_msgs::MapMetaData info = msg->info;
        map_width = info.width;
        map_height = info.height;
        map_reso = info.resolution;
        map_origin_x = info.origin.position.x;
        map_origin_y = info.origin.position.y;
        int product = map_width * map_height;
        ROS_INFO_STREAM("Map Width: " << map_width << ", Map Height: " << map_height
                        << ", Map Resolution: " << map_reso
                        << ", Map Origin: [" << map_origin_x << "," << map_origin_y << "]");

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

          A = cv::Mat(map_height,map_width,CV_16U,arr);
          cv::flip(A,A,0);

          cv::imwrite("A.jpg",A);

          cv::Mat final_image = cv::Mat::zeros(map_height,map_width,CV_16U);

          boost::shared_ptr<nav_msgs::OccupancyGrid const> costmap_msg;
          costmap_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("amr/move_base/global_costmap/costmap");
          nav_msgs::MapMetaData cost_info = costmap_msg->info;
          int cost_map_width = info.width;
          int cost_map_height = info.height;
          int cost_map_reso = info.resolution;
          int cost_map_origin_x = info.origin.position.x;
          int cost_map_origin_y = info.origin.position.y;
          ROS_INFO_STREAM("Cost Map Width: " << cost_map_width << ", Map Height: " << cost_map_height
                          << ", Map Resolution: " << cost_map_reso
                          << ", Map Origin: [" << cost_map_origin_x << "," << cost_map_origin_y << "]");

          short int arr_cost[product];

          for(int x = 0; x < product; x++){
            arr_cost[x] = costmap_msg->data[x];
          }

          B = cv::Mat(map_height,map_width,CV_16U,arr_cost);
          cv::flip(B,B,0);

          for(int i=0;i<map_height;i++){
            for(int j=0;j<map_width;j++){
              if(A.at<short>(i,j) == 254 &&  0 <= B.at<short>(i,j) <=80){
                if(A.at<short>(i-1,j-1) == 205 || A.at<short>(i-1,j) == 205 || A.at<short>(i-1,j+1) == 205 || A.at<short>(i,j-1) == 205 || A.at<short>(i,j+1) == 205 || A.at<short>(i+1,j-1) == 205 || A.at<short>(i+1,j) == 205 || A.at<short>(i+1,j+1) == 205){
                  final_image.at<short>(i,j) = 255;
                }
              }
            }
          }

          cv::imwrite("final_image_short.jpg",final_image);

          final_image.convertTo(final_image,CV_8U);

          cv::Mat labelImage(final_image.size(),CV_8U);
          int nLabels = connectedComponents(final_image,labelImage, 8);

          std::vector<int> vec;

          for(int i=0;i<map_height;i++){
            for(int j=0;j<map_width;j++){
              vec.push_back(labelImage.at<int>(i,j));
            }
          }

          geometry_msgs::PoseArray poseArray;
          poseArray.poses.clear();     // Clear last block perception result
          poseArray.header.stamp = ros::Time::now();
          poseArray.header.frame_id = "/map";

        for(int i=1; i<nLabels; i++){
          if(count_components(i,vec) > 35){
            int* ptr = n_by_two(i,vec,labelImage);
            geometry_msgs::Pose pose;
            pose.position.x = (ptr[1]*map_reso)+map_origin_x;
            pose.position.y = ((map_height-ptr[0])*map_reso)+map_origin_y;
            // pose.position.y = ((ptr[0])*map_reso)+map_origin_y;
            float x = (ptr[1]*map_reso)+map_origin_x;
            float y = ((map_height-ptr[0])*map_reso)+map_origin_y;
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






    /*
              cv::Mat  A_1 = cv::Mat::zeros(map_height,map_width, CV_16U);
              cv::Mat  A_2 = cv::Mat::zeros(map_height,map_width, CV_16U);
              cv::Mat  A_3 = cv::Mat::zeros(map_height,map_width, CV_16U);
              cv::Mat  A_4 = cv::Mat::zeros(map_height,map_width, CV_16U);


              for(int i=0;i<map_height-1;i++){
                for(int j=0;j<map_width;j++){
                  A_1.at<short>(i+1,j) = A.at<short>(i,j);
                  // array1[i+1][j] = A.at<short>(i,j);
                }
              }

              for(int i=0;i<map_height;i++){
                for(int j=0;j<map_width-1;j++){
                  A_2.at<short>(i,j+1) = A.at<short>(i,j);
                  // array2[i][j+1] = A.at<short>(i,j);
                }
              }



              for(int i=1;i<map_height-1;i++){
                for(int j=0;j<map_width;j++){
                  A_3.at<short>(i-1,j) = A.at<short>(i,j);
                  // array3[i-1][j] = A.at<short>(i,j);
                }
              }



               for(int i=0;i<map_height;i++){
                for(int j=1;j<map_width-1;j++){
                  A_4.at<short>(i,j-1) = A.at<short>(i,j);
                  // array4[i][j-1] = A.at<short>(i,j);
                }
              }

              A_1 = A_1 - A;
              A_2 = A_2 - A;
              A_3 = A_3 - A;
              A_4 = A_4 - A;

              useful_pixel(A_1);
              useful_pixel(A_2);
              useful_pixel(A_3);
              useful_pixel(A_4);

              cv::Mat final_image =  A_1 + A_2 + A_3 + A_4;
    */

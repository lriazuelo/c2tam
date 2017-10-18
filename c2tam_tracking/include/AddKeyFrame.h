/*
*
* This file is part of the c2tam project
*
* c2tam is free software: you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by 
* the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
*
* c2tam is distributed in the hope that it will be useful, 
* but WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with c2tam. If not, see http://www.gnu.org/licenses/.
*
*/

#ifndef __ADDKEYFRAME_H
#define __ADDKEYFRAME_H

#include <ros/ros.h>

#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <fstream>
#include <stdio.h>
#include <fstream>
#include <sys/time.h>

#include <cvd/thread.h>

#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>

//#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include "c2tam_msgs/AddKeyFrame.h"
#include "c2tam_msgs/RgbdInfo.h"
#include "c2tam_msgs/MapInfo.h"
#include "c2tam_msgs/DenseKf.h"
#include "c2tam_msgs/DenseInfo.h"


using namespace std;

typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;


class AddKeyFrame : protected CVD::Thread
{
public:

  AddKeyFrame(ros::NodeHandle nh);
  ~AddKeyFrame();
  void NewKeyFrame(); 

  pcl::PointCloud<pcl::PointXYZRGB>* createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, 
                                         const sensor_msgs::ImageConstPtr& rgb_msg);

  cv_bridge::CvImagePtr color_ptr;
  cv_bridge::CvImagePtr depth_ptr;

  c2tam_msgs::AddKeyFrame * msg;

protected:

  virtual void run(); //here lives the thread code
  pthread_mutex_t addMutex;

  ros::NodeHandle n;

  ros::Publisher *addKeyFrame_pub;
  ros::Publisher *pcl_pub;
  ros::Publisher *denseInfo_pub;

  std::ofstream timeSend;

  int countAddKF;
  bool sendRgb;
  bool tracking_visualizer;

  sensor_msgs::CvBridge m_bridge;

  double cam_cx;
  double cam_cy;
  double cam_fx;
  double cam_fy;


};


#endif //__ADDKEYFRAME_H

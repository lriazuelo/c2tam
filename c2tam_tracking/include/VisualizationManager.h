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


#ifndef __VISUALIZATION_MANAGER__
#define __VISUALIZATION_MANAGER__

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>

class VisualizationManager
{
public:
  /** 
   * 
   */
  VisualizationManager(ros::Publisher &pub);
  
  ~VisualizationManager();
  
  /** 
   * 
   */
  void show(cv::Mat &image, std::string msg);

protected:

protected:
  ros::Publisher m_publisher;
  sensor_msgs::CvBridge m_bridge;
  
};

#endif

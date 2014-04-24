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

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include "VisualizationManager.h"


// --------------------------------------------------------------------------

VisualizationManager::VisualizationManager(ros::Publisher &pub):
  m_publisher(pub)
{
}

// --------------------------------------------------------------------------

VisualizationManager::~VisualizationManager()
{
}

// --------------------------------------------------------------------------

void VisualizationManager::show(cv::Mat &image, std::string msg)
{
  int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
  double fontScale = 0.5;
  int thickness = 1.5;

  cv::Point textOrg(10,image.rows - 8);

  cv::putText(image, msg, textOrg, fontFace, fontScale,
         cvScalar(255,255,0), thickness, 8);

  IplImage iplimg(image);
  try{
    sensor_msgs::Image::Ptr ros_img_ptr = 
    m_bridge.cvToImgMsg(&iplimg, "rgb8");
    sensor_msgs::Image msg = sensor_msgs::Image(*ros_img_ptr);
    m_publisher.publish(msg);
  }catch (sensor_msgs::CvBridgeException error){
    ROS_WARN("Error sending image for visualization");
  }
}


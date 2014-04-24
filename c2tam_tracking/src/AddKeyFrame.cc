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

#include "AddKeyFrame.h"



AddKeyFrame::AddKeyFrame(ros::NodeHandle nh):
n(nh)
{

  start(); // This CVD::thread func starts the map-maker thread with function run()

  msg = new c2tam_msgs::AddKeyFrame();

  addMutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_lock(&addMutex);

  addKeyFrame_pub = new  ros::Publisher(n.advertise<c2tam_msgs::AddKeyFrame>("/addKeyFrame", 1000));

  pcl_pub = new  ros::Publisher(n.advertise<sensor_msgs::PointCloud2>("vslam/rgb/points", 100));

  denseInfo_pub = new  ros::Publisher(n.advertise<c2tam_msgs::DenseInfo>("/c2tam_mapping/dense_info", 1000));

  n.param("C2TAMtracking/sendRgb", sendRgb, true);
  n.param("C2TAMtracking/visualizer", tracking_visualizer, false);

  n.param("C2TAMtracking/camera_cx", cam_cx, 311.592488);  //Kinect Default calibration
  n.param("C2TAMtracking/camera_cy", cam_cy, 250.148823);
  n.param("C2TAMtracking/camera_fx", cam_fx, 525.776310);
  n.param("C2TAMtracking/camera_fy", cam_fy, 526.555874);

  countAddKF = 1;

  std::string path_node;
  std::string path_file;
  FILE * find_file = popen("rospack find c2tam_tracking", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;
  path_node = command_find;

  std::string path_file_send;
  char filenameSend[255];
  sprintf(filenameSend,"/.times-send-2.dat");
  path_file_send = path_node + filenameSend;

}



AddKeyFrame::~AddKeyFrame()
{
  cerr << "Waiting for AddKeyFrame to die.." << endl;
  stop(); // makes shouldStop() return true
  cerr << "Waiting for AddKeyFrame to die.." << endl;
  join();
  cerr << " .. AddKeyFrame has died." << endl;
}


pcl::PointCloud<pcl::PointXYZRGB>* AddKeyFrame::createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, 
                                         const sensor_msgs::ImageConstPtr& rgb_msg) 
{

  pcl::PointCloud<pcl::PointXYZRGB>* cloud (new pcl::PointCloud<pcl::PointXYZRGB>() );
  cloud->header.stamp     = depth_msg->header.stamp;
  cloud->header.frame_id  = rgb_msg->header.frame_id;
  cloud->is_dense         = true; //single point of view, 2d rasterized

  float cx, cy, fx, fy;//principal point and focal lengths
  unsigned color_step, color_skip;

  cloud->height = depth_msg->height;
  cloud->width = depth_msg->width;

  cx = cam_cx;
  cy = cam_cy;
  fx = cam_fx;
  fy = cam_fy;

  int pixel_data_size = 3;
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(rgb_msg->encoding.compare("mono8") == 0) pixel_data_size = 1;
  if(rgb_msg->encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }


  ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", rgb_msg->encoding.c_str());
  color_step = pixel_data_size * rgb_msg->width / cloud->width;
  color_skip = pixel_data_size * (rgb_msg->height / cloud->height - 1) * rgb_msg->width;

  cloud->points.resize (cloud->height * cloud->width);

  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg->data[0]);
  const uint8_t* rgb_buffer = &rgb_msg->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  int color_idx = 0, depth_idx = 0;
  double depth_scaling = 1;// = ParameterServer::instance()->get<double>("depth_scaling_factor");

  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud->begin ();
  for (int v = 0; v < (int)cloud->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)cloud->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
    {
      pcl::PointXYZRGB& pt = *pt_iter;
      float Z = depth_buffer[depth_idx] * depth_scaling;

      // Check for invalid measurements
      if (std::isnan (Z))
      {
        pt.x = pt.y = pt.z = Z;
      }
      else // Fill in XYZ
      {
        pt.x = (u - cx) * Z / fx;
        pt.y = (v - cy) * Z / fy;
        pt.z = Z;
      }

      // Fill in color
      RGBValue color;
      if(pixel_data_size == 3){
        color.Red   = rgb_buffer[color_idx + red_idx];
        color.Green = rgb_buffer[color_idx + green_idx];
        color.Blue  = rgb_buffer[color_idx + blue_idx];
      } else {
        color.Red   = color.Green = color.Blue  = rgb_buffer[color_idx];
      }
      color.Alpha = 0;
      pt.rgb = color.float_value;
    }
  }

  return cloud;
}



void AddKeyFrame::NewKeyFrame()
{
  pthread_mutex_unlock(&addMutex);

}

void AddKeyFrame::run()
{

  while(!shouldStop()){  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
    pthread_mutex_lock(&addMutex);

    msg->KeyFrame.rgbd.clear();

    sensor_msgs::ImagePtr color_img_ptr;
    sensor_msgs::ImagePtr depth_img_ptr;

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_col(new pcl::PointCloud<pcl::PointXYZRGB>() );

    color_img_ptr = color_ptr->toImageMsg();
    depth_img_ptr = depth_ptr->toImageMsg();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_col(AddKeyFrame::createXYZRGBPointCloud(depth_img_ptr, color_img_ptr));

    msg->KeyFrame.imgDepth = sensor_msgs::Image(*depth_img_ptr);
    if(sendRgb){
      if (tracking_visualizer){
        cv::Mat color =  cv_bridge::toCvCopy(color_img_ptr)->image;
        cv::Mat gray =  cv_bridge::toCvCopy(color_img_ptr)->image;
        cv::cvtColor(color, gray, CV_RGB2GRAY);

        IplImage iplimg(gray);
        sensor_msgs::Image::Ptr ros_img_ptr = m_bridge.cvToImgMsg(&iplimg, "mono8");
        msg->KeyFrame.imgColor = sensor_msgs::Image(*ros_img_ptr);
      }
      else
        msg->KeyFrame.imgColor = sensor_msgs::Image(*color_img_ptr);
    }

    if (tracking_visualizer){

      boost::shared_ptr<const sensor_msgs::Image> *ros_img_depth_ptr;
      boost::shared_ptr<const sensor_msgs::Image> *ros_img_color_ptr;

      int img_height;
      int img_width;

      c2tam_msgs::DenseInfo dInf;
      c2tam_msgs::DenseKf dkf;
      dkf.idKf = countAddKF ;
      countAddKF++;

      ros_img_depth_ptr = new boost::shared_ptr<const sensor_msgs::Image>(new sensor_msgs::Image(*depth_img_ptr));    
      ros_img_color_ptr = new boost::shared_ptr<const sensor_msgs::Image>(new sensor_msgs::Image(*color_img_ptr));    
      unsigned color_step, color_skip;
      int pixel_data_size = 3;
      char red_idx = 0, green_idx = 1, blue_idx = 2;

      if(color_img_ptr->encoding.compare("mono8") == 0) pixel_data_size = 1;
      if(color_img_ptr->encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }
      color_step = pixel_data_size * color_img_ptr->width / depth_img_ptr->width;
      color_skip = pixel_data_size * (color_img_ptr->height / depth_img_ptr->height - 1) * color_img_ptr->width;

      const float* depth_buffer = reinterpret_cast<const float*>(&depth_img_ptr->data[0]);

      const uint8_t* img_buffer;
      img_buffer = &color_img_ptr->data[0];
      img_height = (int)color_img_ptr->height;
      img_width = (int)color_img_ptr->width;
   
      int color_idx = 0, depth_idx = 0;
      double depth_scaling = 1;// = ParameterServer::instance()->get<double>("depth_scaling_factor");
      int img_idx;
      for (int v = 0; v < img_height; ++v, color_idx += color_skip)
      {
        for (int u = 0; u < img_width; ++u, color_idx += color_step, ++depth_idx)
        {
          c2tam_msgs::RgbdInfo rgbdI;
          float Z = depth_buffer[depth_idx] * depth_scaling;
          if (std::isnan (Z))
          {
            rgbdI.z = Z;
          }
          else // Fill in XYZ
          {
            rgbdI.z = Z;
          }
          // Fill in color

          if(sendRgb) 
            img_idx = color_idx;
          else
            img_idx = depth_idx;


          if(pixel_data_size == 3){
            rgbdI.r =  img_buffer[img_idx + red_idx];// rgb_buffer[color_idx + red_idx];
            rgbdI.g = img_buffer[img_idx + green_idx];//rgb_buffer[color_idx + green_idx];
            rgbdI.b  = img_buffer[img_idx + blue_idx];//rgb_buffer[color_idx + blue_idx];
          } else {
            rgbdI.r = rgbdI.g = rgbdI.b = img_buffer[img_idx];//rgb_buffer[color_idx];
          }

          c2tam_msgs::RgbdInfo rgbdData;

          rgbdData.r = rgbdI.r;
          rgbdData.g = rgbdI.g;
          rgbdData.b = rgbdI.b;
          rgbdData.z = rgbdI.z;
          dkf.denseCloud.push_back(rgbdData);
        }
      }
      dInf.denseKF.push_back(dkf);
      denseInfo_pub->publish(dInf);
    }

    addKeyFrame_pub->publish(*msg);

    sensor_msgs::PointCloud2 cloud_filtered;

    try{
      pcl::toROSMsg(*pc_col, cloud_filtered);
    }
    catch(std::runtime_error e){
      ROS_ERROR_STREAM("Error in converting cloud to image message: "<< e.what());
    }

    pcl_pub->publish (cloud_filtered); //OCTOMAP

  }

}



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


#ifndef __SYSTEM_H
#define __SYSTEM_H
#include "VideoSource.h"
#include "GLWindow2.h"

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include "c2tam_msgs/DataPoints.h"
#include "c2tam_msgs/SemanticInfo.h"
#include "c2tam_msgs/MeasPoint.h"
#include "c2tam_srvs/SearchForMap.h"
#include "c2tam_srvs/LoadIdMap.h"
#include "c2tam_msgs/ObjectTransform.h"
#include "c2tam_msgs/ObjectTransformArray.h"
#include "c2tam_srvs/GetMapObjects.h"
#include "c2tam_srvs/RunVslam.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



#include <sensor_msgs/image_encodings.h>

#include "VisualizationManager.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>


// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <time.h>
#include <sys/time.h>
//#include "scoped_timer.h"

#include <ostream>
#include <iostream>
#include <fstream>

//typedef pcl::PointXYZRGB point_type;
//typedef pcl::PointCloud<point_type> pointcloud_type;

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::PointCloud2> KinectSyncPolicy;


using namespace TooN;
#include <TooN/se3.h>

//class ObjectDetector;
class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class MapViewer;
class Updater;
/*
typedef union
{
  struct //anonymous
  {
    unsigned char Blue;
    unsigned char Green;
    unsigned char Red;
    unsigned char Alpha;
  };
  float float_value;
  long long_value;
} RGBValue;
*/
class System
{
public:
  #ifdef _SEQUENCE_
  System(std::string path, double fps = 25.0 );
  #else
  System();
  #endif
  void Run();

private:
  VideoSource *mVideoSource;

  GLWindow2 *mGLWindow, *mGLWindowVirtual, *mGLWindowUpdater, *EKF, *EKFVW;
  CVD::Image<CVD::Rgb<CVD::byte> > mimFrameRGB;
  CVD::Image<CVD::byte> mimFrameBW;

  Map *mpMap;
  Tracker *mpTracker;
  ATANCamera *mpCamera;
  MapViewer *mpMapViewer;
  Updater *mpMapUpdater;
//  ObjectDetector *mpObjectDetector;

  bool mbDone;
  bool drawVirtualDense;

  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);


  void drawVirtual(GLWindow2*);
  void drawUpdater(GLWindow2*);
  void sendReal(cv::Mat imgMat);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_updater_;
  ros::CallbackQueue updater_queue_;

  ros::Subscriber data_points_sub;
//  ros::Subscriber semantic_info_sub;

  ros::Subscriber image_sub;

  void updaterCallback(const c2tam_msgs::DataPoints::ConstPtr& msg);
//  void semanticInfoCallback(const c2tam_msgs::SemanticInfo::ConstPtr& msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr);
//  void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::PointCloud2ConstPtr& pcl); //DCTAM

void printTransform(const char* name, const tf::Transform t);


  void rgbdCallback(const sensor_msgs::ImageConstPtr& image_ptr, const sensor_msgs::PointCloud2ConstPtr& depth_ptr); //RGBD CTAM
  void imageDCallback(const sensor_msgs::ImageConstPtr& image_ptr, const sensor_msgs::ImageConstPtr& depth_ptr); //RGBDepth CTAM
  void imageDCallback2(const sensor_msgs::ImageConstPtr& image_ptr, const sensor_msgs::ImageConstPtr& depth_ptr); //RGBDepth CTAM
//  void imageDCallback(const sensor_msgs::ImageConstPtr& image_ptr, const sensor_msgs::PointCloud2ConstPtr& depth_ptr);


  bool GetMapObjectsCb(c2tam_srvs::GetMapObjects::Request  &req, 
			   c2tam_srvs::GetMapObjects::Response &res );

    void kinectCallback (const sensor_msgs::ImageConstPtr& visual_img,
                         const sensor_msgs::PointCloud2ConstPtr& point_cloud);


  bool startCB(c2tam_srvs::RunVslam::Request  &req, c2tam_srvs::RunVslam::Response &res );
  bool stopCB(c2tam_srvs::RunVslam::Request  &req, c2tam_srvs::RunVslam::Response &res );

  void InitLoadMode(int id);

  // Don't start until map is loaded
  bool firstUnlock;
  pthread_mutex_t loadMutex;
  int modeTracking;
  int printStat;
  int idTracker;
  std::ofstream timeStat;

  c2tam_msgs::DataPoints::ConstPtr msgUp;

  sensor_msgs::CvBridge bridge_;
  IplImage *cv_image;

  CVD::ImageRef *imageSize;

  SE3<>  current_cam;
  geometry_msgs::PoseStamped cam_pose;
  geometry_msgs::PoseStamped cam_loc;

  c2tam_msgs::ObjectTransformArray objT_list;
  ros::Publisher camera_pub;
  ros::Publisher camera_loc_pub;
  tf::TransformBroadcaster br_tf;

  ros::ServiceClient *clientSearchMap;
  ros::ServiceClient *clientLoadIdMap;

  // visualization tracking
  VisualizationManager *m_visualizer;
  ros::Publisher visualization_pub;

  int freqVisual;
  int cntImg;
  int freqObj;
  int cntImgO;

  bool detectObject;
  bool frameAdded;
  bool initMap;

  bool start_vslam;

  ros::Publisher pcl_pub;

  ros::ServiceServer get_map_objects_ ;

  ros::ServiceServer srvStart;
  ros::ServiceServer srvStop;

int sec;

    message_filters::Synchronizer<KinectSyncPolicy>* kinect_sync_;
    message_filters::Subscriber<sensor_msgs::Image> *visua_sub_;      
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;  


int numSec;

  std::ofstream timeSend;

  tf::TransformListener* tflistener_; 

  std::ofstream odometry_tf;
  std::ofstream slam_tf;
  std::ofstream kf_fd;

  bool initialOdom;
  tf::StampedTransform initial_camera_odom_transform;

};



#endif

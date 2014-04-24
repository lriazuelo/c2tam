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


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "c2tam_srvs/RequestReset.h"
#include "c2tam_srvs/InitFromStereoEKF.h"
#include "c2tam_srvs/AttemptRecovery.h"
#include "c2tam_msgs/KeyFrame.h"
#include "c2tam_msgs/AddKeyFrame.h"
#include "c2tam_msgs/Measurement.h"
#include "c2tam_srvs/InitFromStereoEKF.h"
#include "c2tam_srvs/InitFromRGBD.h"
#include "c2tam_srvs/GetId.h"
#include "c2tam_srvs/RequestReset.h"
#include "c2tam_srvs/SaveMap.h"
#include "c2tam_srvs/LoadMap.h"
#include "c2tam_srvs/LoadIdMap.h"
#include "c2tam_srvs/SearchForMap.h"
#include "c2tam_msgs/RgbdInfo.h"
#include "c2tam_msgs/CreateOctomap.h"
#include "c2tam_srvs/GetOctomap.h"

#include "c2tam_msgs/SemanticKeyFrame.h"
#include "c2tam_msgs/Candidate.h"
#include "c2tam_msgs/VCandidates.h"
#include "c2tam_msgs/VSurf.h"

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>

#include <iostream>
#include <fstream>

#include <cvd/image_io.h>

#include <pthread.h>

#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>

#include <c2tam_msgs/File.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>


class ATANCamera;
class Map;
class MapMaker;
class MapViewer;

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


class System
{
public:
//  #ifdef _SEQUENCE_
//  System(std::string path, double fps = 25.0 );
//  #else
  System();
//  #endif
  void Run();

void RequestResetProcessCb(c2tam_srvs::RequestReset::Request  &req,
		           c2tam_srvs::RequestReset::Response &res );

bool RequestResetCb(c2tam_srvs::RequestReset::Request  &req,
		           c2tam_srvs::RequestReset::Response &res );

bool ResetDoneCb(c2tam_srvs::RequestReset::Request  &req,
		           c2tam_srvs::RequestReset::Response &res );

bool InitFromStereoEkfCb(c2tam_srvs::InitFromStereoEKF::Request  &req,
		           c2tam_srvs::InitFromStereoEKF::Response &res );

bool InitFromRGBDCb(c2tam_srvs::InitFromRGBD::Request  &req,
		           c2tam_srvs::InitFromRGBD::Response &res );

bool AttemptRecoveryCb(c2tam_srvs::AttemptRecovery::Request  &req,
		           c2tam_srvs::AttemptRecovery::Response &res );

int RecoveryMultiMap(int id, sensor_msgs::Image img, SE3<> &poseInMap);

int SearchMultiMap(int id, sensor_msgs::Image img);

bool SaveMapCb(c2tam_srvs::SaveMap::Request  &req,
		           c2tam_srvs::SaveMap::Response &res );

bool LoadMapCb(c2tam_srvs::LoadMap::Request  &req,
			   c2tam_srvs::LoadMap::Response &res );

bool LoadIdCb(c2tam_srvs::LoadIdMap::Request  &req, 
			   c2tam_srvs::LoadIdMap::Response &res );

bool printInfoCallback(c2tam_srvs::LoadIdMap::Request  &req, 
			   c2tam_srvs::LoadIdMap::Response &res );

bool SearchForMapCb(c2tam_srvs::SearchForMap::Request  &req, 
			c2tam_srvs::SearchForMap::Response &res );

int SearchMultiMap(sensor_msgs::Image img);

void addKeyFrameCallback(const c2tam_msgs::AddKeyFrame::ConstPtr& msg);

bool GetId(c2tam_srvs::GetId::Request  &req, c2tam_srvs::GetId::Response &res);

bool FreeId(c2tam_srvs::GetId::Request  &req, c2tam_srvs::GetId::Response &res);

bool ServiceTest(c2tam_srvs::GetId::Request  &req, c2tam_srvs::GetId::Response &res);
bool ServiceTest2(c2tam_srvs::GetId::Request  &req, c2tam_srvs::GetId::Response &res);

void drawVirtual(GLWindow2* w);

  double KeyFrameLinearDist(SE3<> k1,SE3<> k2);


private:
  GLWindow2 *mGLWindowVirtual;

  std::vector<Map *>mpMap;
  std::vector<MapMaker *>mpMapMaker;
  std::vector<ATANCamera *>mpCamera;
//  std::vector<MapViewer *>mpMapViewer; 

//ROS, added by Luis 28/06/2011
//
  ros::NodeHandle nh_;

//ROS, added by Luis 19/07/2011
  // Services
  ros::ServiceServer request_reset_service_;
  ros::ServiceServer reset_done_service_;
  ros::ServiceServer init_from_stereo_ekf_service_ ;
  ros::ServiceServer init_from_rgbd_service_ ;
  ros::ServiceServer save_map_ ;

  ros::ServiceServer load_map_ ;
  ros::ServiceServer load_id_map_ ;
  ros::ServiceServer search_for_map_ ;
  ros::ServiceServer info_kf_sub_;



  ros::ServiceServer attempt_recovery_ ;
  ros::Subscriber add_key_frame_sub_;

  ros::ServiceServer get_id_;
  ros::ServiceServer free_id_;

  sensor_msgs::CvBridge bridge_;

  CVD::ImageRef *sizeVideo;

  pthread_mutex_t loadSaveMutex;

  ros::ServiceServer s1_ ;
  ros::ServiceServer s2_ ;

  ros::Publisher pcl_pub;

//  GLWindow2 *mGLWindowVirtual;
  MapViewer *mpMapViewer;
//  void drawVirtual(GLWindow2* w);

  std::ofstream timeSend;

  std::ofstream kf_fd;

  double cam_cx;
  double cam_cy;
  double cam_fx;
  double cam_fy;

};



#endif

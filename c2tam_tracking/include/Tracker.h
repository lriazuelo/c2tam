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

#ifndef __TRACKER_H
#define __TRACKER_H

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include "c2tam_srvs/QueueSize.h"
#include "c2tam_srvs/AttemptRecovery.h"
#include "c2tam_msgs/KeyFrame.h"
#include "c2tam_msgs/AddKeyFrame.h"
#include "c2tam_msgs/Measurement.h"
#include "c2tam_srvs/InitFromStereoEKF.h"
#include "c2tam_srvs/InitFromRGBD.h"
#include "c2tam_srvs/GetId.h"
#include "c2tam_srvs/RequestReset.h"
#include "c2tam_msgs/Point.h"
#include "c2tam_msgs/LoadPoint.h"
#include "c2tam_msgs/MapPoints.h"
#include "c2tam_msgs/Index.h"
#include "c2tam_msgs/PoseKeyFrame.h"
#include "c2tam_msgs/KeyFramePoseList.h"
#include "c2tam_msgs/NewInfo.h"
#include "c2tam_msgs/LoadInfo.h"
#include "c2tam_srvs/LoadVslamMap.h"

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "c2tam_msgs/ModPoint.h"
#include "c2tam_msgs/ListModPoints.h"
#include "c2tam_srvs/LoadMap.h"
#include "c2tam_msgs/DataPoints.h"
#include "c2tam_msgs/RgbdInfo.h"


#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"

#include "TrackerData.h"
#include "AddKeyFrame.h"

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sstream>
#include <vector>
#include <list>
#include <time.h>
#include <sys/time.h>

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include <math.h>

class TrackerData;
class AddKeyFrame;

struct Trail    // This struct is used for initial correspondences of the first stereo pair.
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

struct denseVertex
{
  SVertex actualKFdensePoints[640*480];
  double actualKFdensePointsStamp;
};

class Tracker
{
public:
  Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, Map &m,ros::NodeHandle nh, int *idTracker, bool *detectObj);

  // TrackFrame is the main working part of the tracker: call this every frame.
  SE3<> TrackFrame(CVD::Image<CVD::byte> &imFrame, bool *addKF, bool *initMap);

void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe

  // Luis Riazuelo: 20/Dic/2012
  void TrackForInitialMap_RGBD(CVD::Image<CVD::byte> &imFrame);

  inline SE3<> GetCurrentPose() { return mse3CamFromWorld;}

  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();

  int getPointSize(){ return mMap.vpPoints.size();}
 int getKFSize(){ return mMap.vpKeyFrames.size();}

  KeyFrame mFirstKF;              // First of the stereo pair
  KeyFrame mCurrentKF;            // The current working frame as a keyframe struct
  sKeyFrame serviceCurrentKF;	//Roboearth
  sKeyFrame serviceFirstKF;	//Roboearth
  sKeyFrame servicePreviousKF;	//Roboearth
  CVD::Image<CVD::byte> imFrameFirst; //Roboearth

  std::vector<TrackerData*> vTrackedFeatures;

  SVertex densePoints[640*480];
  SVertex actualKFdensePoints[640*480];
  pthread_mutex_t denseMutex;
  pthread_mutex_t denseScaleMutex;
  double densePointsStamp;

  bool firstPoints; //DCTAM  

  void dataPointsCallbackCb(const c2tam_msgs::DataPoints::ConstPtr& msg);
  void dataPointsCallback(const c2tam_msgs::DataPoints::ConstPtr& msg);
  bool LoadUIDCb(c2tam_srvs::LoadVslamMap::Request  &req, c2tam_srvs::LoadVslamMap::Response &res );


// DCTAM
  void copyDense(KeyFrame *k);
  double computeScale();
//

  void LoadIdMapClient(double ScaleDepth);

  bool notPose;

  AddKeyFrame *mpAddKeyFrame;

  cv_bridge::CvImagePtr initColor_ptr;
  cv_bridge::CvImagePtr initDepth_ptr;

protected:

  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  ATANCamera mCamera;             // Projection model
  Relocaliser mRelocaliser;       // Relocalisation module

  CVD::ImageRef mirSize;          // Image size of whole image

  void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.
  //void RenderGrid();              // Draws the reference grid

  ros::NodeHandle n;    //Roboearth for ROS TRACKING NODE


  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
   enum {TRAIL_TRACKING_NOT_STARTED,
        TRAIL_TRACKING_STARTED,
        TRAIL_TRACKING_COMPLETE
  } mnInitialStage;  // How far are we towards making the initial map?

  std::list<Trail> mlTrails;      // Used by trail tracking

  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches


  // Methods for tracking the map once it has been made:
  void TrackMap(CVD::Image<CVD::byte> &imFrame);                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<TrackerData*> &vTD,
            int nRange,
            int nFineIts);  // Finds points in the image
  Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD,
            double dOverrideSigma = 0.0,
            bool bMarkOutliers = false); // Updates pose from found points.
  SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
  SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
  Vector<6> mv6CameraVelocity;    // Motion model
  double mdVelocityMagnitude;     // Used to decide on coarse tracking
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?


  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.

  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;

  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  Vector<6> mv6SBIRot;
  bool mbUseSBIInit;

  // User interaction for initial tracking:
  bool mbUserPressedSpacebar;
  std::ostringstream mMessageForUser;

  // GUI interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;


  double delta_t;

  FILE * fd_index;
  
  bool initRGBD;

  //ROBOEARTH ROS NODE
  ros::ServiceClient *clientQueueSize;
  ros::ServiceClient *clientAttemptRecovery;
  ros::ServiceClient *clientInitFromStereo_EKF;
  ros::ServiceClient *clientInitFromRGBD;
  ros::ServiceClient *clientGetId;
  ros::ServiceClient *clientFreeId;
  ros::ServiceClient *clientRequestReset;
  ros::ServiceClient *clientResetDone;
  ros::Publisher *addKeyFrame_pub;

  ros::ServiceServer load_uid_map_;

  ros::ServiceClient *clientLoadMap;

  sensor_msgs::Image::Ptr ros_img_ptr;
  sensor_msgs::CvBridge bridge_;

  int QueuseSizeClient();
  bool AttemptRecoveryClient(CVD::Image<CVD::byte> &imFrame);
  bool AddServiceKeyFrame(sKeyFrame k);
  bool AddServiceKeyFrameOld(sKeyFrame k);
  double InitFromStereo_EKFClient( sKeyFrame skF, sKeyFrame skS, SE3<> &se3);
  double InitFromRGBD_Client( sKeyFrame skF, SE3<> &se3);
  int GetIdClient(int mode);
  int FreeIdClient(int mode);
  bool RequestResetClient();
  bool ResetDoneClient();
  int LoadMapClient(std::string nameMap);

  bool NeedNewKeyFrame(SE3<> kCurrent, double dSceneDepthMean);
  double KeyFrameLinearDist(SE3<> k1,SE3<> k2);
  double KeyFrameAngularDist(SE3<> k1,SE3<> k2);
  SE3<> ClosestKeyFrame(SE3<> k);
  double DistToNearestKeyFrame(SE3<>  kCurrent);
  bool IsDistanceToNearestKeyFrameExcessive(SE3<> kCurrent);

  double mdWiggleScaleDepthNormalized;
  GVars3::gvar3<double> mgvdWiggleScale;
  double mdWiggleScale;

  ros::Subscriber data_points_sub;

  int queueSizeInfo;

  int cntKeyFrame;

  int id;

  int mode;
  int printStat;
  std::ofstream timeStat;
  std::ofstream timeDataFlow;
  std::ofstream timeRecov;

  bool startTracker;
  bool loadMapServer;

  CVD::Image<CVD::byte> imgkf1;
  CVD::Image<CVD::byte> imgkf2;
  CVD::Image<CVD::byte> imgkf;

  FILE * denseMFile; //DCTAM  

  pthread_mutex_t pendingKFMutex;
  pthread_mutex_t addKFMutex;
  std::vector< SE3<> > pendingKF; 
  std::vector< denseVertex >pendingDense;

  pthread_mutex_t mutexLoadRE;
  bool waitingForLoadMap;

  bool *detectObject;

  bool addVariable;

  double cam_cx;
  double cam_cy;
  double cam_fx;
  double cam_fy;

  double needKFCoefficient;

};

#endif

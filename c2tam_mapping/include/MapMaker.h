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

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>

#include <TooN/LU.h>

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include <queue>

#include "TrackerData.h"
#include "Relocaliser.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "MapViewer.h"

#include <TooN/se2.h>
#include <TooN/Cholesky.h>
#include <TooN/wls.h>

#include <ostream>
#include <iostream>
#include <fstream>

#include "c2tam_msgs/Point.h"
#include "c2tam_msgs/LoadPoint.h"
#include "c2tam_msgs/MapPoints.h"
#include "c2tam_msgs/Index.h"
#include "c2tam_msgs/PoseKeyFrame.h"
#include "c2tam_msgs/KeyFramePoseList.h"
#include "c2tam_msgs/NewInfo.h"
#include "c2tam_msgs/LoadInfo.h"
#include "c2tam_msgs/DataPoints.h"
#include "c2tam_msgs/MeasPoint.h"
#include "c2tam_msgs/SemanticInfo.h"
#include "c2tam_msgs/RgbdInfo.h"
#include "c2tam_msgs/MapInfo.h"
#include "c2tam_msgs/DenseKf.h"
#include "c2tam_msgs/ObjectTransform.h"
#include "c2tam_msgs/ObjectTransformArray.h"
#include "c2tam_srvs/GetMapObjects.h"

#include "c2tam_msgs/ModPoint.h"
#include "c2tam_msgs/ListModPoints.h"

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

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <pcl_ros/transforms.h>


// Needed includes to g2o
#include <Eigen/StdVector>
#include <Eigen/SVD>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
#include <tr1/unordered_set>

#include "../EXTERNAL/g2o/g2o/core/sparse_optimizer.h"
#include "../EXTERNAL/g2o/g2o/core/block_solver.h"
#include "../EXTERNAL/g2o/g2o/core/solver.h"
#include "../EXTERNAL/g2o/g2o/core/robust_kernel_impl.h"
#include "../EXTERNAL/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../EXTERNAL/g2o/g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "../EXTERNAL/g2o/g2o/solvers/dense/linear_solver_dense.h"
#include "../EXTERNAL/g2o/g2o/types/sba/types_six_dof_expmap.h"
#include "../EXTERNAL/g2o/g2o/solvers/structure_only/structure_only_solver.h"

/////////////////////////////////////




#include <cvd/image_io.h>

#include <pthread.h>

#include "ros/ros.h"


// Type definition for the optimization
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;


struct RecoveryMap
{
  bool firstKF;
  bool secondKF;
  int  idFirstKF;
  int  idSecondKF;
  SE3 <> recoveryFirstKF;
  SE3 <> recoverySecondKF;
  double recoveryScaleFirstKF;
  double recoveryScaleSecondKF;
  SE3 <> actualFirstKF;
  SE3 <> actualSecondKF;
  double actualScaleFirstKF;
  double actualScaleSecondKF;

  bool recovery;
};

struct MapMakerData
{
  std::set<KeyFrame*> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<KeyFrame*> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount() {  return sMeasurementKFs.size(); }
};

class ATANCamera;

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker : protected CVD::Thread
{
public:
  MapMaker(Map &m, ros::NodeHandle n,int id,int mode,double cx, double cy, double fx, double fy,bool start=true);
  ~MapMaker();

  // Make a map from scratch. Called by the tracker.
  bool InitFromStereo_EKF(KeyFrame &kF, KeyFrame &kS, SE3<> &se3TrackerPose);

  bool InitServiceFromStereo_EKF(  KeyFrame &kF, KeyFrame &kS,sKeyFrame skF,sKeyFrame skS,SE3<> &se3);
  double InitServFromStereo_EKF( sKeyFrame skF,sKeyFrame skS,SE3<> &se3);
  double InitServFromRGBD(sKeyFrame skF,  SE3<> &se3);

  void TrailTracking_Start_EKF(CVD::Image<CVD::byte> &imFrame);
  int  TrailTracking_Advance_EKF(CVD::Image<CVD::byte> &imFrame);
  void  TrackForInitialMap_EKF(CVD::Image<CVD::byte> &imFrame,SE3<> &mse3CFW);

  void AddKeyFrame(KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(KeyFrame &kCurrent);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)

  bool AttemptServiceRecovery(CVD::Image<CVD::byte> &imFrame, bool &mbJustRecoveredSoUseCoarse_r,Vector<6> &mv6CameraVelocity_r, SE3<> &mse3StartPos_r, SE3<> &mse3CamFromWorld_r, int *indexKF);

  void AddServiceKeyFrame(sKeyFrame k,int color=0);

bool GetMapObjectsCb(c2tam_srvs::GetMapObjects::Request  &req, 
			   c2tam_srvs::GetMapObjects::Response &res );

  void printInfo();

  // OSC: Funcion de debug
  std::ofstream os;
  std::string id;
  friend std::ostream& operator<<(std::ostream& os, const MapMaker& mapa);

  bool pubLoadPoints();
  bool pubDataPoints();
  bool pubSeenObjs();
  bool pubSemanticInfo(bool success);
  bool pubEKFPoints();
  bool pubRGBPoints();
  bool pubMergePoints(int firstKF);
  double computeScale();

  std::vector<MapPoint*> pubPoints;
  ros::Publisher *dataPoints_pub;
  ros::Publisher *semanticInfo_pub;
  ros::Publisher *mapInfo_pub;
  ros::NodeHandle nh;
  std::vector<int> indexModPoints;
  std::vector<int> indexBadPoints;

  tf::TransformListener m_tfListener;

  double getmdWiggleScaleDepthNormalized(){return mdWiggleScaleDepthNormalized;}
  void setmdWiggleScaleDepthNormalized(double val){mdWiggleScaleDepthNormalized = val;}

  pthread_mutex_t loadSaveMutex;
  pthread_mutex_t ekfMutex;
  pthread_mutex_t mergeMutex;
  bool pubLoadData; // Publish initial data when a map is loaded

  int mapMode;


  bool mbJustRecoveredSoUseCoarse;
  void TrackMap(CVD::Image<CVD::byte> &imFrame);
  bool AssessTrackingQuality();
  KeyFrame mCurrentKF;

  void ApplyGlobalTransformationToMap(SE3<> se3NewFromOld);
  void ApplyGlobalTransformationScaleToMap(SE3<> se3NewFromOld, double dScale);
  void ApplyGlobalScaleToMap(double dScale);
  void ApplyCivera(SE3<> se3NewFromOld, double dScale);
  void ApplyExportMap(SE3<> se3OldMap, SE3<> se3ActualMap, double dScaleActualMap, double scaleMonocular);
  void MergeMaps(Map& mNew, double dSceneDepthMean, double dSceneDepthSigma,std::vector<int> iOld, std::vector<int> iACtual);

  bool AddPointEpipolarSemantic(KeyFrame &kSrc,KeyFrame &kTarget, int nCandidate);
  double DistPointLine(Vector<2> P1, Vector<2> P2, double x, double y);
  KeyFrame* KeyFrameWithParallax(KeyFrame &k);
  double MaxSceneDepth(KeyFrame *pKF);
  double AngleBetweenVectors(Vector<3> &v1, Vector<3> &v2 );

  int  ObjectQueueSize() { return mvpObjectQueue.size() ;}


  Map &mMap;               // The map

  bool recoveryActive;

  RecoveryMap vctRecovery[20];
  SE3<> mse3CamFromWorld;

  double cam_cx;
  double cam_cy;
  double cam_fx;
  double cam_fy;

protected:


  KeyFrame servicekf; //Roboearth
  KeyFrame servicekf_ekf; //Roboearth
  KeyFrame serviceks_ekf; //Roboearth
  KeyFrame aux_KF; //Roboearth

  ATANCamera *mCamera;      // Roboearth
  Relocaliser *mRelocaliser;       // Roboearth

  virtual void run();      // The MapMaker thread code lives here

  // Functions for starting the map from scratch:
  SE3<> CalcPlaneAligner();

  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();
  void ThinCandidates(KeyFrame &k, int nLevel);
  void AddSomeMapPoints(int nLevel);
  bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate,bool bSemantic=false);
  void AddObjecttoMap();
  void addRGBDPoints(KeyFrame &kSrc,int nLevel,	int nCandidate,const Vector <3> &v3pose);

  // Returns point in ref frame B
  Vector<3> ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);

  // Bundle adjustment functions:
  void BundleAdjustM(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>,std::set<MapObject*>, bool, bool check = false);


  void BundleAdjustAll(bool check = false);
  void BundleAdjustRecentM();

  bool checkIndexModPoints(int index);

  int SearchForPoints(std::vector<TrackerData*> &vTD, int nRange, int nSubPixIts);
  Vector<6> CalcPoseUpdate(std::vector<TrackerData*> vTD,
            double dOverrideSigma = 0.0,
            bool bMarkOutliers = false);

  bool mbDidCoarse;
  double mdMSDScaledVelocityMagnitude;

  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];

  // Data association functions:
  int ReFindInSingleKeyFrame(KeyFrame &k);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(KeyFrame &k, MapPoint &p);
  void SubPixelRefineMatches(KeyFrame &k, int nLevel);

  // General Maintenance/Utility:
  void Reset();
  void HandleBadPoints();
  double DistToNearestKeyFrame(KeyFrame &kCurrent);
  double KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2);
  double KeyFrameAngularDist(KeyFrame &k1, KeyFrame &k2);
  KeyFrame* ClosestKeyFrame(KeyFrame &k);
  //KeyFrame* BestEpipolarKeyFrame(KeyFrame &k);
  std::vector< std::vector<KeyFrame*> > BestEpipolarKeyFrame(KeyFrame &k);
  std::vector<KeyFrame*> NClosestKeyFramesM(KeyFrame &k, unsigned int N);
  void RefreshSceneDepth(KeyFrame *pKF);

  void cvd2opencv(const CVD::Image<CVD::byte> &cvd_Im, IplImage & openCv_Im);

  double CalculateMedianScale(Vector<3> & v1, Vector<3> &v2);
  void checkSemanticPoints();

  bool parallaxCameras(MapPoint* p);
  double computeDistError(double dist);

  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;
  bool pubObjectInfo();

  // Member variables:
  std::vector<KeyFrame*> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
  std::vector<MapObject*> mvpObjectQueue;
  std::vector<std::pair<KeyFrame*, MapPoint*> > mvFailureQueue; // Queue of failed observations to re-find
  std::queue<MapPoint*> mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames


  double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
                         // This sets the scale of the map
  GVars3::gvar3<double> mgvdWiggleScale;   // GVar for above
  double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth,
                                        // this controls keyframe separation

  bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
  bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?

  // Thread interaction signalling stuff
  bool mbResetRequested;   // A reset has been requested
  bool mbResetDone;        // The reset was done.
  bool mbBundleAbortRequested;      // We should stop bundle adjustment
  bool mbBundleRunning;             // Bundle adjustment is running
  bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.

  pthread_mutex_t pointsMutex;
  bool	pointsBool;

  sensor_msgs::CvBridge bridge_;

  int mapId;


  bool newKeyFrameInfo;
  bool modKeyFrameInfo;

  bool pub1;
  bool pub2;

  MapObject *lastObject;


  bool unlockSemantic;

  ros::ServiceServer get_map_objects_ ;


  c2tam_msgs::ObjectTransformArray objT_list;

  std::string robotId; // base of the robot for ground plane filtering




//  CVD::ImageRef *imageSizemapping_visualizer;
//  GLWindow2 *mGLWindowVirtual;
//  MapViewer *mpMapViewer;
//  void drawVirtual(GLWindow2* w);


private:
  std::ofstream timeBAG;
  std::ofstream timeBAL;
  std::ofstream timeT;

  int numKeyFrames;

  std::ofstream timeSend;
  bool mapping_visualizer;

};

#endif

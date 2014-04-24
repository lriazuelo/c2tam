// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include <queue>

#include <Scene/settings.h>
#include <MonoSLAM/monoslaminterface.h>
#include <MonoSLAM/robot.h>
#include <Scene/scene_single.h>
#include <MonoSLAM/model_creators.h>
#include <VW/Image/imagemono.h>

#include "Relocaliser.h"

#include <ostream>
#include <iostream>
#include <fstream>

#include <cvd/image_io.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

// Each MapPoint has an associated MapMakerData class
// Where the mapmaker can store extra information

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

  MapMaker(Map &m, Map &mTracker);
  ~MapMaker();

  // Make a map from scratch. Called by the tracker.
  bool InitFromStereo_EKF(KeyFrame &kF, KeyFrame &kS, SE3<> &se3TrackerPose);

  bool InitServiceFromStereo_EKF(  KeyFrame &kF, KeyFrame &kS,sKeyFrame skF,sKeyFrame skS,SE3<> &se3);
  double InitServFromStereo_EKF( sKeyFrame skF,sKeyFrame skS,SE3<> &se3);

  void TrailTracking_Start_EKF(CVD::Image<CVD::byte> &imFrame);
  int  TrailTracking_Advance_EKF(CVD::Image<CVD::byte> &imFrame);
  void  TrackForInitialMap_EKF(CVD::Image<CVD::byte> &imFrame,SE3<> &mse3CFW);


  void AddKeyFrame(KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(KeyFrame &kCurrent);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)

  bool AttemptServiceRecovery(CVD::Image<CVD::byte> &imFrame, bool &mbJustRecoveredSoUseCoarse_r,Vector<6> &mv6CameraVelocity_r, SE3<> &mse3StartPos_r, SE3<> &mse3CamFromWorld_r);


  //Roboearth
  void AddServiceKeyFrame(sKeyFrame k);

  // OSC: Funcion de debug
  std::ofstream os;
  std::string id;
  friend std::ostream& operator<<(std::ostream& os, const MapMaker& mapa);

protected:

  Map &mMap;               // The map
  Map &mMapTracker; //Roboearth
  KeyFrame servicekf; //Roboearth
  KeyFrame servicekf_ekf; //Roboearth
  KeyFrame serviceks_ekf; //Roboearth
  KeyFrame aux_KF; //Roboearth

//  ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!
  ATANCamera *mCamera;      // Roboearth
  Relocaliser *mRelocaliser;       // Roboearth

  virtual void run();      // The MapMaker thread code lives here

  // Functions for starting the map from scratch:
  SE3<> CalcPlaneAligner();
  void ApplyGlobalTransformationToMap(SE3<> se3NewFromOld);
  void ApplyGlobalScaleToMap(double dScale);

  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();
  void ThinCandidates(KeyFrame &k, int nLevel);
  void AddSomeMapPoints(int nLevel);
  bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate);
  // Returns point in ref frame B
  Vector<3> ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);

  // Bundle adjustment functions:
  void BundleAdjust(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>, bool);
  void BundleAdjustM(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>, bool);
  void BundleAdjustT(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>, bool);
  void BundleAdjustAll();
  void BundleAdjustRecentM();
  void BundleAdjustRecentT();

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
  KeyFrame* ClosestKeyFrameT(KeyFrame &k);

  std::vector<KeyFrame*> NClosestKeyFramesM(KeyFrame &k, unsigned int N);
  std::vector<KeyFrame*> NClosestKeyFramesT(KeyFrame &k, unsigned int N);
  void RefreshSceneDepth(KeyFrame *pKF);


  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  struct Command {std::string sCommand; std::string sParams; };
  std::vector<Command> mvQueuedCommands;


  // Member variables:
  std::vector<KeyFrame*> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
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

private:
  std::ofstream timeBAG;
  std::ofstream timeBAL;
  std::ofstream timeT;

};

#endif

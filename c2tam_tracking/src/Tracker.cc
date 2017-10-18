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

#include "OpenGL.h"
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <cvd/image_io.h>
#include <TooN/wls.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <fstream>
#include <fcntl.h>


#include <iomanip>
#include <unistd.h>


#define DEFAULT_NAME ""

using namespace CVD;
using namespace std;
using namespace GVars3;

// The constructor mostly sets up interal reference variables
// to the other classes..
Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, Map &m, ros::NodeHandle nh, int *idTracker,bool *detectObj) :
  mMap(m),
  mCamera(c),
  mRelocaliser(mMap, &mCamera),
  mirSize(irVideoSize),
  n(nh),
  detectObject(detectObj)
{
  mCurrentKF.bFixed = false;
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
  GV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.1, SILENT); // Default to 10cm between keyframes

  TrackerData::irImageSize = mirSize;

  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;

  queueSizeInfo = 0;
  cntKeyFrame = 1;


  // Most of the initialisation is done in Reset()

  clientQueueSize = new ros::ServiceClient(n.serviceClient<c2tam_srvs::QueueSize>("/queue_size_service"));
  clientAttemptRecovery = new ros::ServiceClient(n.serviceClient<c2tam_srvs::AttemptRecovery>("/attempt_recovery_service"));

  clientInitFromStereo_EKF = new ros::ServiceClient(n.serviceClient<c2tam_srvs::InitFromStereoEKF>("/init_from_stereo_ekf_service"));
  clientInitFromRGBD = new ros::ServiceClient(n.serviceClient<c2tam_srvs::InitFromRGBD>("/init_from_rgbd_service"));


  clientGetId = new ros::ServiceClient(n.serviceClient<c2tam_srvs::GetId>("/get_id_service"));
  clientFreeId = new ros::ServiceClient(n.serviceClient<c2tam_srvs::GetId>("/vslam/free_id_service"));
  clientLoadMap = new ros::ServiceClient(n.serviceClient<c2tam_srvs::LoadMap>("/load_map_service"));
  clientRequestReset = new ros::ServiceClient(n.serviceClient<c2tam_srvs::RequestReset>("/reset_service"));
  clientResetDone = new ros::ServiceClient(n.serviceClient<c2tam_srvs::RequestReset>("/reset_done"));
  addKeyFrame_pub = new  ros::Publisher(n.advertise<c2tam_msgs::AddKeyFrame>("/addKeyFrame", 1000));

  load_uid_map_ = n.advertiseService("/vslam/load_uid_vmap", &Tracker::LoadUIDCb, this);

  n.param("C2TAMtracking/camera_cx", cam_cx, 311.592488);  //Kinect Default calibration
  n.param("C2TAMtracking/camera_cy", cam_cy, 250.148823);
  n.param("C2TAMtracking/camera_fx", cam_fx, 525.776310);
  n.param("C2TAMtracking/camera_fy", cam_fy, 526.555874);

  std::string nameMap;
  std::string def_nameMap = DEFAULT_NAME;
//  n.param( "C2TAMtracking/name_map", nameMap, def_nameMap );
  n.param("C2TAMtracking/mode", mode, 0); 
  n.param("C2TAMtracking/stat", printStat, 0); 
  n.param("C2TAMtracking/needKFCoefficient", needKFCoefficient, 3.0); 
  n.param("C2TAMtracking/needAngKFCoefficient", needAngKFCoefficient, 0.2); 


  pendingKFMutex = PTHREAD_MUTEX_INITIALIZER;
  addKFMutex = PTHREAD_MUTEX_INITIALIZER;
  mutexLoadRE = PTHREAD_MUTEX_INITIALIZER;
  denseScaleMutex = PTHREAD_MUTEX_INITIALIZER;

//  pthread_mutex_lock(&mutexLoadRE);
  waitingForLoadMap = false;

  initRGBD = true; 

  startTracker = true;
  loadMapServer = false;
  if (mode == 0){
    id = GetIdClient(mode);
    *idTracker = id;
  }
  if (mode == 1){
//    pthread_mutex_lock(&mutexLoadRE);
//    id = LoadMapClient(nameMap);
//    *idTracker = id;
    waitingForLoadMap = true;
    id = FreeIdClient(mode);
    *idTracker = id;
//    *idTracker = 0;
  }
  if (mode == 2){
    id = GetIdClient(mode);
    *idTracker = id;
  }

  if (mode == 3)
    id = *idTracker;

  ostringstream nameServiceDP;
  nameServiceDP << "/map" << id << "/dataPoints";

  Reset();

  if (mode == 1 || mode == 3){
    mnLostFrames = 3;
    cntKeyFrame = 3;
    mnLastKeyFrameDropped = mnFrame;
  }

 //*idTracker = id;


 //denseMFile = fopen("/home/luis/densePoints.m","w"); //DCTAM 
 firstPoints = true; //DCTAM  

  if(printStat == 1){
    std::string path_node;
    std::string path_file;
    std::string path_file_data;
    std::string path_file_recov;
    FILE * find_file = popen("rospack find c2tam_tracking", "r");
    char command_find[1000];
    int numChar = fread(command_find, 1, 1000, find_file);
    command_find[numChar-1] = 0;
    path_node = command_find;

    time_t now = time(0);
    struct tm* tm = localtime(&now);
    char filename[255];
    char filename_data[255];
    char filename_recov[255];
    sprintf(filename,"/.times-tracker-%04d-%02d-%02d-%02d-%02d-%02d.dat",
		tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour,tm->tm_min,tm->tm_sec);
    path_file = path_node + filename;
    sprintf(filename_data,"/.times-data-%04d-%02d-%02d-%02d-%02d-%02d.dat",
		tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour,tm->tm_min,tm->tm_sec);
    sprintf(filename_recov,"/.times-recov-%04d-%02d-%02d-%02d-%02d-%02d.dat",
		tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour,tm->tm_min,tm->tm_sec);
    path_file_data = path_node + filename_data;
    path_file_recov = path_node + filename_recov;
    /* NEW luis 01082014 */    
    timeStat.open(path_file.c_str(),ios_base::out|ios_base::trunc);
    timeStat.precision (15);

    //timeRecov.open(path_file_recov.c_str(),ios_base::out|ios_base::trunc);
    //timeRecov.precision (15);
    //timeDataFlow.open(path_file_data.c_str(),ios_base::out|ios_base::trunc);
    //timeDataFlow.precision (15);

  }
  else{
    std::string path_node;
    std::string path_file;
    std::string path_file_data;
    FILE * find_file = popen("rospack find c2tam_tracking", "r");
    char command_find[1000];
    int numChar = fread(command_find, 1, 1000, find_file);
    command_find[numChar-1] = 0;
    path_node = command_find;

    time_t now = time(0);
    struct tm* tm = localtime(&now);
    char filename[255];
    char filename_data[255];
    sprintf(filename,"/.times-tracker-%04d-%02d-%02d-%02d-%02d-%02d.dat",
		tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday, tm->tm_hour,tm->tm_min,tm->tm_sec);
    path_file = path_node + filename;
    //timeStat.open(path_file.c_str(),ios_base::out|ios_base::trunc);
    //timeStat.precision (15);
  }

  pendingDense.reserve(30);
  notPose = true;
  addVariable = true;

  mpAddKeyFrame = new AddKeyFrame(n);

}


bool Tracker::LoadUIDCb(c2tam_srvs::LoadVslamMap::Request  &req, c2tam_srvs::LoadVslamMap::Response &res ){

   id = LoadMapClient(req.mapUID);
   id = 0;
   waitingForLoadMap = false;
   *detectObject = true;
   res.success = true;
}


// Resets the tracker, wipes the map.
// This is the main Reset-handler-entry-point of the program! Other classes' resets propagate from here.
// It's always called in the Tracker's thread, often as a GUI command.
void Tracker::Reset()
{
  mbDidCoarse = false;
  mbUserPressedSpacebar = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  mCurrentKF.dSceneDepthMean = 1.0;
  mCurrentKF.dSceneDepthSigma = 1.0;
  serviceCurrentKF.dSceneDepthMean = 1.0;  //Roboearth
  serviceCurrentKF.dSceneDepthSigma = 1.0; //Roboearth
  mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mCamera.SetImageSize(mirSize);
  mCurrentKF.mMeasurements.clear();
  serviceCurrentKF.sMeasurements.clear(); //Roboearth
  mnLastKeyFrameDropped = -20;
  mnFrame=0;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = false;
  queueSizeInfo = 0;

  cntKeyFrame = 1;

  // Tell the MapMaker to reset itself..
  // this may take some time, since the mapmaker thread may have to wait
  // for an abort-check during calculation, so sleep while waiting.
  // MapMaker will also clear the map.

  if (mode == 0){
    RequestResetClient();  //Roboearth

    while(!ResetDoneClient())
    #ifndef WIN32
      usleep(10);
    #else
      Sleep(1);
    #endif
    }
  }



// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions.

SE3<> Tracker::TrackFrame(Image<CVD::byte> &imFrame, bool *addKF, bool *initMap)
{
  struct timeval	tc;
  double t1,t2;
  bool addData = false;
  gettimeofday(&tc, NULL);
  t1=tc.tv_sec+tc.tv_usec/1e6;

  *addKF = false;
  *initMap = false;

  if (startTracker && !waitingForLoadMap){

    mMessageForUser.str("");   // Wipe the user message clean

    // Take the input video image, and convert it into the tracker's keyframe struct
    // This does things like generate the image pyramid and find FAST corners
    mCurrentKF.mMeasurements.clear();
    serviceCurrentKF.sMeasurements.clear(); //Roboearth

    // Crea una piramide de imagenes (binning 2x2) y busca caracteristicas
    // FAST-10 en cada nivel de la piramide
    mCurrentKF.MakeKeyFrame_Lite(imFrame);
    serviceCurrentKF.imFrame = imFrame; //LUIS

    // Update the small images for the rotation estimator
    static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
    static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
    mbUseSBIInit = *gvnUseSBI;
    if(!mpSBIThisFrame)
    {
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
      mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    } 
    else
    {
      delete  mpSBILastFrame;
      mpSBILastFrame = mpSBIThisFrame;
      mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    }

    // From now on we only use the keyframe struct!
    mnFrame++;

    // Decide what to do - if there is a map, try to track the map ...
    if(mMap.IsGood())
    {
      //ROS_INFO("mMap.IsGood()");
      // Tenemos un mapa inicial => lo intentamos usar
      if(mnLostFrames < 3)  // .. but only if we're not lost!
      {
//ROS_INFO("TF_1");
        notPose = false;

        if(mbUseSBIInit){
          CalcSBIRotation();
	}
//ROS_INFO("TF_2");
        ApplyMotionModel();       //
ROS_INFO("2");
        TrackMap(imFrame);               //  These three lines do the main tracking work.
ROS_INFO("18");
        UpdateMotionModel();      //
//ROS_INFO("TF_5");
        AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.
//ROS_INFO("TF_6");
        { // Provide some feedback for the user:
          mMessageForUser << "Tracking Map, quality ";
          if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
          if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
          if(mTrackingQuality == BAD)   mMessageForUser << "bad.";
          mMessageForUser << " Found:";
          for(int i=0; i<LEVELS; i++) mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
          // mMessageForUser << " Found " << mnMeasFound << " of " << mnMeasAttempted <<". (";
          mMessageForUser << " Map: " << mMap.vpPoints.size() << "P, " << mMap.vpKeyFrames.size() << "KF";
        }
//ROS_INFO("TF_7");
        //***********************************
        // Heuristics to check if a key-frame should be added to the map:
        //if(mTrackingQuality == GOOD &&
        //     NeedNewKeyFrame(mCurrentKF.se3CfromW, mCurrentKF.dSceneDepthMean) &&
        //     mnFrame - mnLastKeyFrameDropped > 20  &&
        //     queueSizeInfo < 3)
        //{
          if(NeedNewKeyFrame(mCurrentKF.se3CfromW, mCurrentKF.dSceneDepthMean)){
            // fprintf(stderr,"add new kf\n");
            //cerr << mCurrentKF.se3CfromW << endl;
   
            //fprintf(stderr,"vpKeyFrames %d\n",mMap.vpKeyFrames.size());

          pthread_mutex_lock(&pendingKFMutex);
          pthread_mutex_lock(&addKFMutex); 

          //for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
          //{
          //  cerr << i << endl;
          //  cerr << mMap.vpKeyFrames[i]->se3CfromW << endl;
          //}

         //fprintf(stderr,"pendingKF %d\n",pendingKF.size());
         //for(unsigned int i=0; i<pendingKF.size(); i++)
         //{
         //   cerr << i << endl;
         //   cerr << pendingKF[i] << endl;
         //}
         pthread_mutex_unlock(&pendingKFMutex);
         pthread_mutex_unlock(&addKFMutex); 
//ROS_INFO("TF_8");


            mMessageForUser << " Adding key-frame.";
            addData = true;
            gettimeofday(&tc, NULL);
            t2=tc.tv_sec+tc.tv_usec/1e6;

          //  if (mMap.vpKeyFrames.size() < 120){
            //AddNewKeyFrame();
            *addKF = true;
          //  }
	    /* New Luis 01082014 */
            if(printStat == 1){
              timeStat << 0  << "\t" << 0  << "\t" << t2-t1  << "\t" << 0  << "\t" << 0  << "\t" 
              << mMap.vpKeyFrames.size() << "\t" << mMap.vpPoints.size() << "\t" << 1 << "\t" << 1 << "\t" << t2 << endl;
              timeStat.flush();
            }
         }
	 else{
           /* New Luis 01082014 */
           gettimeofday(&tc, NULL);
           t2=tc.tv_sec+tc.tv_usec/1e6;
           if(printStat == 1){
             timeStat << 0  << "\t" << 0  << "\t" << t2-t1  << "\t" << 0  << "\t" << 0  << "\t" 
             << mMap.vpKeyFrames.size() << "\t" << mMap.vpPoints.size() << "\t" << 0 << "\t" << 1 << "\t" << t2 << endl;
             timeStat.flush();
           }
         }
//ROS_INFO("TF_9");
       }
       else  // what if there is a map, but tracking has been lost?
       {
ROS_INFO("2");
         notPose = true;
         mMessageForUser << "** Attempting recovery **.";
         if(AttemptRecoveryClient(imFrame))
         {
           TrackMap(imFrame);
           AssessTrackingQuality();
         }
ROS_INFO("18");
         /* New Luis 01082014 */
         gettimeofday(&tc, NULL);
         t2=tc.tv_sec+tc.tv_usec/1e6;
         if(printStat == 1){
           timeStat << 0  << "\t" << 0  << "\t" << t2-t1  << "\t" << 0  << "\t" << 0  << "\t" 
           << mMap.vpKeyFrames.size() << "\t" << mMap.vpPoints.size() << "\t" << 0 << "\t" << 0 << "\t" << t2 << endl;
           timeStat.flush();
         }
       }
     }
     else // If there is no map, try to make one.
     {
       //TrackForInitialMap_EKF(imFrame);
       *initMap = true;
       //TrackForInitialMap_RGBD(imFrame); // -> RGBD initialization
     }

     // GUI interface
     while(!mvQueuedCommands.empty())
     {
       GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
       mvQueuedCommands.erase(mvQueuedCommands.begin());
     }
   }
   if (!addData){
     gettimeofday(&tc, NULL);
     //t2=tc.tv_sec+tc.tv_usec/1e6;
     /* New Luis 01082014 */
     //if(printStat == 1){
     //  timeStat << 0  << "\t" << 0  << "\t" << t2-t1  << "\t" << 0  << "\t" << 0  << "\t" 
     //  << mMap.vpKeyFrames.size() << "\t" << mMap.vpPoints.size() << "\t" << 0 << "\t" << t2 << endl;
     //  timeStat.flush();
     //}
   }
ROS_INFO("19");
   return mCurrentKF.se3CfromW;
};



// Try to relocalise in case tracking was lost.
// Returns success or failure as a bool.
// Actually, the SBI relocaliser will almost always return true, even if
// it has no idea where it is, so graphics will go a bit
// crazy when lost. Could use a tighter SSD threshold and return more false,
// but the way it is now gives a snappier response and I prefer it.
bool Tracker::AttemptRecovery()
{
  bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
  if(!bRelocGood)
    return false;

  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;
  return true;
}



// GUI interface. Stuff commands onto the back of a queue so the tracker handles
// them in its own thread at the end of each frame. Note the charming lack of
// any thread safety (no lock on mvQueuedCommands).
void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}




// This is called in the tracker's own thread.
void Tracker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="Reset")
  {
      Reset();
      return;
  }

  // KeyPress commands are issued by GLWindow
  if(sCommand=="KeyPress")
  {
      if(sParams == "Space")
      {
          mbUserPressedSpacebar = true;
      }
      else if(sParams == "r")
      {
          Reset();
      }
      else if(sParams == "q" || sParams == "Escape")
      {
          GUI.ParseLine("quit");
      }
      return;
  }
  if((sCommand=="PokeTracker"))
  {
      mbUserPressedSpacebar = true;
      return;
  }

  cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
};




// TrackMap is the main purpose of the Tracker.
// It first projects all map points into the image to find a potentially-visible-set (PVS);
// Then it tries to find some points of the PVS in the image;
// Then it updates camera pose according to any points found.
// Above may happen twice if a coarse tracking stage is performed.
// Finally it updates the tracker's current-frame-KeyFrame struct with any
// measurements made.
// A lot of low-level functionality is split into helper classes:
// class TrackerData handles the projection of a MapPoint and stores intermediate results;
// class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
void Tracker::TrackMap(Image<CVD::byte> &imFrame)
{
ROS_INFO("3");
  // Some accounting which will be used for tracking quality assessment:
  for(int i=0; i<LEVELS; i++)
      manMeasAttempted[i] = manMeasFound[i] = 0;
ROS_INFO("4");

  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  vector<TrackerData*> avPVS[LEVELS];
  for(int i=0; i<LEVELS; i++)
      avPVS[i].reserve(500);
ROS_INFO("5");

  // For all points in the map..
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
  {
      MapPoint &p= *(mMap.vpPoints[i]);
    
      // Ensure that this map point has an associated TrackerData struct.
      if(!p.pTData){
        p.pTData = new TrackerData(&p);
      }
      TrackerData &TData = *p.pTData;
      
      TData.Project(mse3CamFromWorld, mCamera);
      if(!TData.bInImage)
          continue;

      // Calculate camera projection derivatives of this point.
      TData.GetDerivsUnsafe(mCamera);

      // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
      TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
      if(TData.nSearchLevel == -1)
          continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

      // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
      TData.bSearched = false;
      TData.bFound = false;
      avPVS[TData.nSearchLevel].push_back(&TData);
  }
ROS_INFO("6");

  // Next: A large degree of faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for(int i=0; i<LEVELS; i++)
      random_shuffle(avPVS[i].begin(), avPVS[i].end());
ROS_INFO("7");

  // The next two data structs contain the list of points which will next
  // be searched for in the image, and then used in pose update.
  vector<TrackerData*> vNextToSearch;
  vector<TrackerData*> vIterationSet;

  // Tunable parameters to do with the coarse tracking stage:
  static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
  static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
  static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
  static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.

  unsigned int nCoarseMax = *gvnCoarseMax;
  unsigned int nCoarseRange = *gvnCoarseRange;

  mbDidCoarse = false;
ROS_INFO("8");

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool bTryCoarse = true;
  if(*gvnCoarseDisabled ||
     mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
     nCoarseMax == 0)
      bTryCoarse = false;
  if(mbJustRecoveredSoUseCoarse)
  {
      //ROS_INFO("mbJustRecoveredSoUseCoarse\n");
      bTryCoarse = true;
      nCoarseMax *=2;
      nCoarseRange *=2;
      mbJustRecoveredSoUseCoarse = false;
  };
ROS_INFO("9");

  // If we do want to do a coarse stage, also check that there's enough high-level
  // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
  // with preference to LEVELS-1.
  if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin )
  {
      // Now, fill the vNextToSearch struct with an appropriate number of
      // TrackerDatas corresponding to coarse map points! This depends on how many
      // there are in different pyramid levels compared to CoarseMin and CoarseMax.

      if(avPVS[LEVELS-1].size() <= nCoarseMax)
      { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
          vNextToSearch = avPVS[LEVELS-1];
          avPVS[LEVELS-1].clear();
      }
      else
      { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
          for(unsigned int i=0; i<nCoarseMax; i++)
              vNextToSearch.push_back(avPVS[LEVELS-1][i]);
          avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
      }

      // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
      if(vNextToSearch.size() < nCoarseMax)
      {
          unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
          if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
          {
              vNextToSearch = avPVS[LEVELS-2];
              avPVS[LEVELS-2].clear();
          }
          else
          {
              for(unsigned int i=0; i<nMoreCoarseNeeded; i++)
                  vNextToSearch.push_back(avPVS[LEVELS-2][i]);
              avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
          }
      }
      // Now go and attempt to find these points in the image!
      unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
      vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
      if(nFound >= *gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
      {
          mbDidCoarse = true;
          for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
          {
              if(iter != 0)
              { // Re-project the points on all but the first iteration.
                  for(unsigned int i=0; i<vIterationSet.size(); i++)
                     if(vIterationSet[i]->bFound)
                        vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
              }
              for(unsigned int i=0; i<vIterationSet.size(); i++)
                  if(vIterationSet[i]->bFound)
                      vIterationSet[i]->CalcJacobian();
                  double dOverrideSigma = 0.0;
                 // Hack: force the MEstimator to be pretty brutal
                 // with outliers beyond the fifth iteration.
                 if(iter > 5)
                     dOverrideSigma = 1.0;

                 // Calculate and apply the pose update...
                 Vector<6> v6Update =
                   CalcPoseUpdate(vIterationSet, dOverrideSigma);
                 mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
          }
      }
  }
ROS_INFO("10");

  // So, at this stage, we may or may not have done a coarse tracking stage.
  // Now do the fine tracking stage. This needs many more points!

  int nFineRange = 10;  // Pixel search range for the fine stage.
  if(mbDidCoarse)       // Can use a tighter search if the coarse stage was already done.
      nFineRange = 5;

  // What patches shall we use this time? The high-level ones are quite important,
  // so do all of these, with sub-pixel refinement.
  {
      int l = LEVELS - 1;
      for(unsigned int i=0; i<avPVS[l].size(); i++){
          avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
      }
      SearchForPoints(avPVS[l], nFineRange, 8);
      for(unsigned int i=0; i<avPVS[l].size(); i++)
          vIterationSet.push_back(avPVS[l][i]); // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
  };
ROS_INFO("11");

  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  vNextToSearch.clear();
  for(int l=LEVELS - 2; l>=0; l--)
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      vNextToSearch.push_back(avPVS[l][i]);
ROS_INFO("12");

  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
  // ourselves to 1000, and choose these randomly.
  static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
  int nFinePatchesToUse = *gvnMaxPatchesPerFrame - vIterationSet.size();
  if((int) vNextToSearch.size() > nFinePatchesToUse)
  {
      random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
      vNextToSearch.resize(nFinePatchesToUse); // Chop!
  };
ROS_INFO("13");

  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(mbDidCoarse)
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
        vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
ROS_INFO("14");

  // Find fine points in image:
  SearchForPoints(vNextToSearch, nFineRange, 0);
  // And attach them all to the end of the optimisation-set.
  for(unsigned int i=0; i<vNextToSearch.size(); i++)
      vIterationSet.push_back(vNextToSearch[i]);
ROS_INFO("15");

  // Again, ten gauss-newton pose update iterations.
  Vector<6> v6LastUpdate;
  v6LastUpdate = Zeros;
  for(int iter = 0; iter<10; iter++)
  {
//ROS_INFO("TM_13_1");
      bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
                                // reprojection at every iteration - it really isn't necessary!
//ROS_INFO("TM_13_2");
      if(iter == 0 || iter == 4 || iter == 9)
          bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
      else                            // iterations is for M-Estimator convergence rather than
          bNonLinearIteration = false;  // linearisation effects.
//ROS_INFO("TM_13_3");

      if(iter != 0)   // Either way: first iteration doesn't need projection update.
      {
          if(bNonLinearIteration)
          {
              for(unsigned int i=0; i<vIterationSet.size(); i++)
                  if(vIterationSet[i]->bFound)
                      vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
          }
          else
          {
              for(unsigned int i=0; i<vIterationSet.size(); i++)
                  if(vIterationSet[i]->bFound)
                      vIterationSet[i]->LinearUpdate(v6LastUpdate);
          };
      }
//ROS_INFO("TM_13_4");

      if(bNonLinearIteration)
          for(unsigned int i=0; i<vIterationSet.size(); i++)
              if(vIterationSet[i]->bFound)
                  vIterationSet[i]->CalcJacobian();
//ROS_INFO("TM_13_5");

      // Again, an M-Estimator hack beyond the fifth iteration.
      double dOverrideSigma = 0.0;
      if(iter > 5)
      dOverrideSigma = 16.0;
//ROS_INFO("TM_13_6");

      // Calculate and update pose; also store update vector for linear iteration updates.
      Vector<6> v6Update =
        CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      v6LastUpdate = v6Update;
//ROS_INFO("TM_13_7");

  };
ROS_INFO("16");


  // Update the current keyframe with info on what was found in the frame.
  // Strictly speaking this is unnecessary to do every frame, it'll only be
  // needed if the KF gets added to MapMaker. Do it anyway.
  // Export pose to current keyframe:
  mCurrentKF.se3CfromW = mse3CamFromWorld;
  serviceCurrentKF.se3CfromW = mse3CamFromWorld; 
  // Record successful measurements. Use the KeyFrame-Measurement struct for this.
  mCurrentKF.mMeasurements.clear();
  serviceCurrentKF.sMeasurements.clear(); 
  for(vector<TrackerData*>::iterator it = vIterationSet.begin();
      it!= vIterationSet.end();
      it++)
  {
      if(! (*it)->bFound)
          continue;
      Measurement m;
      sMeasurement sm; 
      m.v2RootPos = (*it)->v2Found;
      m.nLevel = (*it)->nSearchLevel;
      m.bSubPix = (*it)->bDidSubPix;
      mCurrentKF.mMeasurements[& ((*it)->Point)] = m;

      sm.m = m; //LUIS
      sm.indexPointMap =  (*it)->Point.indexPointMap; //Roboearth
      serviceCurrentKF.sMeasurements.push_back(sm); //Roboearth
  }
//ROS_INFO("TM_15");

  // Finally, find the mean scene depth from tracked features
  {
    double dSum = 0;
    double dSumSq = 0;
    int nNum = 0;
    //vector<double> depths;                  //Added by Marta
    for(vector<TrackerData*>::iterator it = vIterationSet.begin();
        it!= vIterationSet.end(); it++)
        if((*it)->bFound)
        {
            double z = (*it)->v3Cam[2];
            dSum+= z;
            dSumSq+= z*z;
            nNum++;
    //        depths.push_back(z);            //Added by Marta
        };
  //  sort(depths.begin(), depths.end());    //Added by marta
    if(nNum > 20)
    {
        mCurrentKF.dSceneDepthMean = dSum/nNum; //depths[depths.size()/2];//trying to use the median instead of mean(dSum/nNum);
        mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
  	serviceCurrentKF.dSceneDepthMean = mCurrentKF.dSceneDepthMean; //Roboearth
  	serviceCurrentKF.dSceneDepthSigma = mCurrentKF.dSceneDepthSigma; //Roboearth
    }
  }
  vTrackedFeatures.clear();
  vTrackedFeatures = vIterationSet;
ROS_INFO("17");

}



// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{

  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
  {
      // First, attempt a search at pixel locations which are FAST corners.
      // (PatchFinder::FindPatchCoarse)
      TrackerData &TD = *vTD[i];

      PatchFinder &Finder = TD.Finder;
      Finder.MakeTemplateCoarseCont(TD.Point);
      if(Finder.TemplateBad())
      {
          TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
          continue;
      }

      manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessment

      bool bFound =
        Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange);
      TD.bSearched = true;
      if(!bFound)
      {
          TD.bFound = false;
          continue;
      }

      TD.bFound = true;
      TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());

      nFound++;
      manMeasFound[Finder.GetLevel()]++;

      // Found the patch in coarse search - are Sub-pixel iterations wanted too?
      if(nSubPixIts > 0)
      {
          TD.bDidSubPix = true;
          Finder.MakeSubPixTemplate();
          bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF, nSubPixIts);
          if(!bSubPixConverges)
          { // If subpix doesn't converge, the patch location is probably very dubious!
              TD.bFound = false;
              nFound--;
              manMeasFound[Finder.GetLevel()]--;
              continue;
          }
          TD.v2Found = Finder.GetSubPixPos();
      }
      else
      {
          TD.v2Found = Finder.GetCoarsePosAsVector();
          TD.bDidSubPix = false;
      }
  }
  return nFound;
};




//Calculate a pose update 6-vector from a bunch of image measurements.
//User-selectable M-Estimator.
//Normally this robustly estimates a sigma-squared for all the measurements
//to reduce outlier influence, but this can be overridden if
//dOverrideSigma is positive. Also, bMarkOutliers set to true
//records any instances of a point being marked an outlier measurement
//by the Tukey MEstimator.
Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
//ROS_INFO("CalcPoseUpdate_1");
  // Which M-estimator are we using?
  int nEstimator = 0;
  static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
  if(*gvsEstimator == "Tukey")
      nEstimator = 0;
  else if(*gvsEstimator == "Cauchy")
      nEstimator = 1;
  else if(*gvsEstimator == "Huber")
      nEstimator = 2;
  else
  {
      cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
      nEstimator = 0;
      *gvsEstimator = "Tukey";
  };
//ROS_INFO("CalcPoseUpdate_2");
  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  vector<double> vdErrorSquared;
  for(unsigned int f=0; f<vTD.size(); f++)
  {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
          continue;
      TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
      vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
  };
//ROS_INFO("CalcPoseUpdate_3");
  // No valid measurements? Return null update.
  if(vdErrorSquared.size() == 0)
      return makeVector( 0,0,0,0,0,0);
//ROS_INFO("CalcPoseUpdate_4");
  // What is the distribution of errors?
  double dSigmaSquared;
  if(dOverrideSigma > 0)
      dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
  else
  {
      if (nEstimator == 0)
          dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      else if(nEstimator == 1)
          dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
      else
          dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
  }
//ROS_INFO("CalcPoseUpdate_5 %d ",vTD.size());
  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior
//ROS_INFO("CalcPoseUpdate_5b %d ",vTD.size());

  for(unsigned int f=0; f<vTD.size(); f++)
  {
//ROS_INFO("CalcPoseUpdate_51");
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
          continue;
      Vector<2> &v2 = TD.v2Error_CovScaled;
      double dErrorSq = v2 * v2;
      double dWeight;
//ROS_INFO("CalcPoseUpdate_52");
      if(nEstimator == 0)
          dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
          dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else
          dWeight= Huber::Weight(dErrorSq, dSigmaSquared);
//ROS_INFO("CalcPoseUpdate_53");
      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      if(dWeight == 0.0)
      {
          if(bMarkOutliers)
              TD.Point.nMEstimatorOutlierCount++;
              continue;
      }
      else if(bMarkOutliers)
          TD.Point.nMEstimatorInlierCount++;
//ROS_INFO("CalcPoseUpdate_54");
      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
//ROS_INFO("CalcPoseUpdate_55");
  }
//ROS_INFO("CalcPoseUpdate_6");
  wls.compute();
//ROS_INFO("CalcPoseUpdate_7");
  return wls.get_mu();
}


// Just add the current velocity to the current pose.
// N.b. this doesn't actually use time in any way, i.e. it assumes
// a one-frame-per-second camera. Skipped frames etc
// are not handled properly here.
void Tracker::ApplyMotionModel()
{
  mse3StartPos = mse3CamFromWorld;
  Vector<6> v6Velocity = mv6CameraVelocity;
  if(mbUseSBIInit)
  {
      v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
      v6Velocity[0] = 0.0;
      v6Velocity[1] = 0.0;
  }
  mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
};




// The motion model is entirely the tracker's, and is kept as a decaying
// constant velocity model.
void Tracker::UpdateMotionModel()
{
  SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
  Vector<6> v6Motion = SE3<>::ln(se3NewFromOld);
  Vector<6> v6OldVel = mv6CameraVelocity;

  mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5 * v6OldVel);
  mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);

  // Also make an estimate of this which has been scaled by the mean scene depth.
  // This is used to decide if we should use a coarse tracking stage.
  // We can tolerate more translational vel when far away from scene!
  Vector<6> v6 = mv6CameraVelocity;
  v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}




// Time to add a new keyframe? The MapMaker handles most of this.
void Tracker::AddNewKeyFrame()
{
  AddServiceKeyFrame(serviceCurrentKF); //Roboearth
//  AddServiceKeyFrameOld(serviceCurrentKF); //Roboearth
  mnLastKeyFrameDropped = mnFrame;
}




// Some heuristics to decide if tracking is any good, for this frame.
// This influences decisions to add key-frames, and eventually
// causes the tracker to attempt relocalisation.
void Tracker::AssessTrackingQuality()
{
//DDD fprintf(stderr,"AssessTrackingQuality\n");
  int nTotalAttempted = 0;
  int nTotalFound = 0;
  int nLargeAttempted = 0;
  int nLargeFound = 0;

  for(int i=0; i<LEVELS; i++)
  {
      nTotalAttempted += manMeasAttempted[i];
      nTotalFound += manMeasFound[i];
      if(i>=2) nLargeAttempted += manMeasAttempted[i];
      if(i>=2) nLargeFound += manMeasFound[i];

  }

  if(nTotalFound == 0 || nTotalAttempted == 0){
      mTrackingQuality = BAD;
  }else
  {
      double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
      double dLargeFracFound;
      if(nLargeAttempted > 10)
          dLargeFracFound = (double) nLargeFound / nLargeAttempted;
      else
          dLargeFracFound = dTotalFracFound;

      static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
      static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);


      if(dTotalFracFound > *gvdQualityGood)
          mTrackingQuality = GOOD;
      else if(dLargeFracFound < *gvdQualityLost)
          mTrackingQuality = BAD;
      else
          mTrackingQuality = DODGY;

  }

  if(mTrackingQuality == DODGY)
  {
      // Further heuristics to see if it's actually bad, not just dodgy...
      // If the camera pose estimate has miles away, it's probably bad.
      if(IsDistanceToNearestKeyFrameExcessive(mCurrentKF.se3CfromW)){
          mTrackingQuality = BAD;
      }
  }

  if(mTrackingQuality==BAD)
      mnLostFrames++;
  else
      mnLostFrames = 0;
}



string Tracker::GetMessageForUser()
{
  return mMessageForUser.str();
}


void Tracker::CalcSBIRotation()
{
  mpSBILastFrame->MakeJacs();
  pair<SE2<>, double> result_pair;
  result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
  SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
  mv6SBIRot = se3Adjust.ln();
}



ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here


// Luis Riazuelo: 20/Dic/2012

void Tracker::TrackForInitialMap_RGBD(CVD::Image<CVD::byte> &imFrame)
{
  if(initRGBD){
    //ROS_INFO("TrackForInitialMap_RGBD");

    //serviceCurrentKF.dSceneDepthMean = 
    //serviceCurrentKF.dSceneDepthSigma = 

    mdWiggleScaleDepthNormalized = InitFromRGBD_Client(serviceCurrentKF, mse3CamFromWorld);
    //mdWiggleScaleDepthNormalized = 0.1;//InitFromRGBD_Client(serviceCurrentKF, mse3CamFromWorld);
    mdWiggleScale = *mgvdWiggleScale;
    //ROS_INFO("mdWiggleScaleDepthNormalized %f",mdWiggleScaleDepthNormalized);
//ROS_INFO("Initial Camera %f %f %f",mse3CamFromWorld.get_translation()[0],mse3CamFromWorld.get_translation()[1], mse3CamFromWorld.get_translation()[2] ); //LL
    initRGBD = false;
  }
}



////////////////////////////////
// ROS CLIENTS
//

int Tracker::QueuseSizeClient()
{
  c2tam_srvs::QueueSize srv;
  srv.request.id = id;
  if (clientQueueSize->call(srv))
  {
    //ROS_INFO("QueuseSize: %d", srv.response.size);
  }
  else
  {
    ROS_ERROR("Failed to call service queue_size_service");
    return -1;
  }
  return srv.response.size;

}

bool Tracker::AttemptRecoveryClient(CVD::Image<CVD::byte> &imFrame)
{
  struct timeval	tc;
  double t1, t2;

  gettimeofday(&tc, NULL);
  t1 = tc.tv_sec+tc.tv_usec/1e6; 

  mCurrentKF.MakeKeyFrame_Lite(imFrame);

  bool bRelocGood = mRelocaliser.AttemptRecovery(mCurrentKF);
  if(!bRelocGood)
    return false;

  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;

  gettimeofday(&tc, NULL);
  t2 = tc.tv_sec+tc.tv_usec/1e6;


  //timeRecov << t2-t1  << "\t" << mMap.vpKeyFrames.size() << "\t" << mMap.vpPoints.size() << endl;
  //timeRecov.flush(); 

  return true;
}


bool Tracker::AddServiceKeyFrameOld(sKeyFrame k)
{
  struct timeval	tc;
  double t1, t2, t3, t4, t5, t6, t7;

  gettimeofday(&tc, NULL);
  t1 = tc.tv_sec+tc.tv_usec/1e6; 


  sensor_msgs::ImagePtr color_ptr;
  sensor_msgs::ImagePtr depth_ptr;

  c2tam_msgs::AddKeyFrame msg;

  msg.KeyFrame.rgbd.reserve(640*480);

  gettimeofday(&tc, NULL);
  t2 = tc.tv_sec+tc.tv_usec/1e6; 


  //fprintf(stderr,"Add New KeyFrame\n");
  Vector<2> dim = vec(k.imFrame.size());
  IplImage* greyImage     = cvCreateImage(cvSize((int)dim[0],(int)dim[1]) , IPL_DEPTH_8U, 1);
  cvSetImageData(greyImage,  k.imFrame.data(),(int)dim[0]);
  ros_img_ptr = bridge_.cvToImgMsg(greyImage, "mono8");

  imgkf.copy_from(k.imFrame); //IMG

  msg.id = id;
  msg.KeyFrame.img = sensor_msgs::Image(*ros_img_ptr);
  msg.type = false;


  gettimeofday(&tc, NULL);
  t3 = tc.tv_sec+tc.tv_usec/1e6; 


  for(unsigned int i=0; i<k.sMeasurements.size(); i++)
  {
    c2tam_msgs::Measurement m;
    m.indexPointMap = k.sMeasurements[i].indexPointMap;
    m.nLevel = k.sMeasurements[i].m.nLevel;
    m.bSubPix = k.sMeasurements[i].m.bSubPix;
    m.v2RootPos.push_back(k.sMeasurements[i].m.v2RootPos[0]);
    m.v2RootPos.push_back(k.sMeasurements[i].m.v2RootPos[1]);
    m.indexPointMap = k.sMeasurements[i].indexPointMap;
    m.Source = k.sMeasurements[i].m.Source;
    msg.KeyFrame.sMeasurements.push_back(m);
  }


  msg.KeyFrame.se3CfromW.push_back( k.se3CfromW.get_translation()[0]);
  msg.KeyFrame.se3CfromW.push_back( k.se3CfromW.get_translation()[1]);
  msg.KeyFrame.se3CfromW.push_back( k.se3CfromW.get_translation()[2]);

  Matrix< 3 > mAux;
  mAux = k.se3CfromW.get_rotation().get_matrix();

  msg.KeyFrame.se3CfromW.push_back(mAux(0,0));
  msg.KeyFrame.se3CfromW.push_back(mAux(0,1));
  msg.KeyFrame.se3CfromW.push_back(mAux(0,2));
  msg.KeyFrame.se3CfromW.push_back(mAux(1,0));
  msg.KeyFrame.se3CfromW.push_back(mAux(1,1));
  msg.KeyFrame.se3CfromW.push_back(mAux(1,2));
  msg.KeyFrame.se3CfromW.push_back(mAux(2,0));
  msg.KeyFrame.se3CfromW.push_back(mAux(2,1));
  msg.KeyFrame.se3CfromW.push_back(mAux(2,2));

  msg.KeyFrame.bFixed =  k.bFixed;
  msg.KeyFrame.dSceneDepthMean =  k.dSceneDepthMean;
  msg.KeyFrame.dSceneDepthSigma = k.dSceneDepthSigma;

  pthread_mutex_lock(&denseScaleMutex);
  msg.mapDenseScale =  mMap.denseScale;
  pthread_mutex_unlock(&denseScaleMutex);

  gettimeofday(&tc, NULL);
  t4 = tc.tv_sec+tc.tv_usec/1e6; 

/*
//DCTAM
  for (int i=0; i < 640*480;i++){
    c2tam_msgs::RgbdInfo rgbd;
    rgbd.r = (int) (pendingDense[pendingDense.size()-1].actualKFdensePoints[i].r * 255);
    rgbd.g = (int) (pendingDense[pendingDense.size()-1].actualKFdensePoints[i].g * 255);
    rgbd.b = (int) (pendingDense[pendingDense.size()-1].actualKFdensePoints[i].b * 255);
    rgbd.z = pendingDense[pendingDense.size()-1].actualKFdensePoints[i].z;
    msg.KeyFrame.rgbd.push_back(rgbd);
  }
///
*/

mpAddKeyFrame->NewKeyFrame();



  color_ptr = mpAddKeyFrame->color_ptr->toImageMsg();
  depth_ptr = mpAddKeyFrame->depth_ptr->toImageMsg();

  unsigned color_step, color_skip;

  int pixel_data_size = 3;
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(color_ptr->encoding.compare("mono8") == 0) pixel_data_size = 1;
  if(color_ptr->encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }

  ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", color_ptr->encoding.c_str());
  color_step = pixel_data_size * color_ptr->width / depth_ptr->width;
  color_skip = pixel_data_size * (color_ptr->height / depth_ptr->height - 1) * color_ptr->width;

  const float* depth_buffer = reinterpret_cast<const float*>(&depth_ptr->data[0]);
  const uint8_t* rgb_buffer = &color_ptr->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  int color_idx = 0, depth_idx = 0;
  double depth_scaling = 1;// = ParameterServer::instance()->get<double>("depth_scaling_factor");

  for (int v = 0; v < (int)color_ptr->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)color_ptr->width; ++u, color_idx += color_step, ++depth_idx)
    {
      c2tam_msgs::RgbdInfo rgbd;
      rgbd.z = depth_buffer[depth_idx] * depth_scaling;

      // Fill in color
      if(pixel_data_size == 3){
        rgbd.r   = rgb_buffer[color_idx + red_idx];
        rgbd.g = rgb_buffer[color_idx + green_idx];
        rgbd.b  = rgb_buffer[color_idx + blue_idx];
      } else {
        rgbd.r   = rgbd.g = rgbd.b  = rgb_buffer[color_idx];
      }
      msg.KeyFrame.rgbd.push_back(rgbd);   
//      mpAddKeyFrame->msg.KeyFrame.rgbd.push_back(rgbd);   
    }
  }

  gettimeofday(&tc, NULL);
  t5 = tc.tv_sec+tc.tv_usec/1e6; 


/*
  for (int i=0; i < 640*480;i++){
    c2tam_msgs::RgbdInfo rgbd;
    rgbd.z = densePoints[i].z;
    rgbd.r = densePoints[i].r;
    rgbd.g = densePoints[i].g;
    rgbd.b = densePoints[i].b;
    msg.KeyFrame.rgbd.push_back(rgbd); 
  }
*/


 addKeyFrame_pub->publish(msg);
//  addKeyFrame_pub->publish(mpAddKeyFrame->msg);

  gettimeofday(&tc, NULL);
  t6 = tc.tv_sec+tc.tv_usec/1e6; 


//  fprintf(stderr,"AddServiceKeyFrame\n");

  // ADD ON PENDING QUEUE
  //fprintf(stderr,"ADD ON PENDING QUEUE BEFORE %d\n",pendingKF.size());
  pthread_mutex_lock(&pendingKFMutex); 
  pendingKF.push_back(k.se3CfromW);
  pthread_mutex_unlock(&pendingKFMutex);
  //fprintf(stderr,"ADD ON PENDING QUEUE AFTER %d\n",pendingKF.size());

  gettimeofday(&tc, NULL);
  t7 = tc.tv_sec+tc.tv_usec/1e6; 


  //timeStat << t1  << "\t" << t2  << "\t" << t3  << "\t" << t4 << "\t" << t5 << "\t" << t6 << "\t" << t7 <<"\n";
  //timeStat.flush();


  return true;
}


bool Tracker::AddServiceKeyFrame(sKeyFrame k)
{
  struct timeval	tc;
  double t1, t2, t3, t4, t5, t6, t7;

  // INIT AddServiceKeyFrame
  gettimeofday(&tc, NULL);
  t1 = tc.tv_sec+tc.tv_usec/1e6; 

  delete mpAddKeyFrame->msg;
  mpAddKeyFrame->msg = new c2tam_msgs::AddKeyFrame();
  //mpAddKeyFrame->msg->KeyFrame.rgbd.reserve(640*480);

  // Create and reserve memory
  gettimeofday(&tc, NULL);
  t2 = tc.tv_sec+tc.tv_usec/1e6; 

  Vector<2> dim = vec(k.imFrame.size());
  IplImage* greyImage     = cvCreateImage(cvSize((int)dim[0],(int)dim[1]) , IPL_DEPTH_8U, 1);
  cvSetImageData(greyImage,  k.imFrame.data(),(int)dim[0]);
  ros_img_ptr = bridge_.cvToImgMsg(greyImage, "mono8");

  imgkf.copy_from(k.imFrame); //IMG

  mpAddKeyFrame->msg->id = id;
  mpAddKeyFrame->msg->KeyFrame.img = sensor_msgs::Image(*ros_img_ptr);
  mpAddKeyFrame->msg->type = false;

  // Convert and add image to msg
  gettimeofday(&tc, NULL);
  t3 = tc.tv_sec+tc.tv_usec/1e6; 


  for(unsigned int i=0; i<k.sMeasurements.size(); i++)
  {
    c2tam_msgs::Measurement m;
    m.indexPointMap = k.sMeasurements[i].indexPointMap;
    m.nLevel = k.sMeasurements[i].m.nLevel;
    m.bSubPix = k.sMeasurements[i].m.bSubPix;
    m.v2RootPos.push_back(k.sMeasurements[i].m.v2RootPos[0]);
    m.v2RootPos.push_back(k.sMeasurements[i].m.v2RootPos[1]);
    m.indexPointMap = k.sMeasurements[i].indexPointMap;
    m.Source = k.sMeasurements[i].m.Source;
    mpAddKeyFrame->msg->KeyFrame.sMeasurements.push_back(m);
  }

  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back( k.se3CfromW.get_translation()[0]);
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back( k.se3CfromW.get_translation()[1]);
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back( k.se3CfromW.get_translation()[2]);

  Matrix< 3 > mAux;
  mAux = k.se3CfromW.get_rotation().get_matrix();

  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(0,0));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(0,1));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(0,2));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(1,0));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(1,1));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(1,2));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(2,0));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(2,1));
  mpAddKeyFrame->msg->KeyFrame.se3CfromW.push_back(mAux(2,2));

  mpAddKeyFrame->msg->KeyFrame.bFixed =  k.bFixed;
  mpAddKeyFrame->msg->KeyFrame.dSceneDepthMean =  k.dSceneDepthMean;
  mpAddKeyFrame->msg->KeyFrame.dSceneDepthSigma = k.dSceneDepthSigma;


  pthread_mutex_lock(&denseScaleMutex);
  mpAddKeyFrame->msg->mapDenseScale =  mMap.denseScale;
  pthread_mutex_unlock(&denseScaleMutex);

  // Copy measurements and fill msg fields
  gettimeofday(&tc, NULL);
  t4 = tc.tv_sec+tc.tv_usec/1e6; 


  mpAddKeyFrame->NewKeyFrame();


  // NewKeyFrame trigger
  gettimeofday(&tc, NULL);
  t5 = tc.tv_sec+tc.tv_usec/1e6; 


  // ADD ON PENDING QUEUE
  pthread_mutex_lock(&pendingKFMutex); 
  pendingKF.push_back(k.se3CfromW);
  pthread_mutex_unlock(&pendingKFMutex);

  // ADD ON PENDING QUEUE
  gettimeofday(&tc, NULL);
  t6 = tc.tv_sec+tc.tv_usec/1e6; 


  //timeStat << t1  << "\t" << t2  << "\t" << t3  << "\t" << t4 << "\t" << t5 << "\t" << t6 <<"\n";
  //timeStat.flush();


  return true;
}

double Tracker::InitFromRGBD_Client( sKeyFrame skF, SE3<> &se3)
{
  sensor_msgs::ImagePtr color_ptr;
  sensor_msgs::ImagePtr depth_ptr;

  denseVertex infoD;

// DCTAM
  //fprintf(stderr,"// Copy the last pcl\n");
/*
  pthread_mutex_lock(&denseMutex); 
  for (int i=0; i < 640*480;i++){
    infoD.actualKFdensePoints[i].x = densePoints[i].x;
    infoD.actualKFdensePoints[i].y = densePoints[i].y;
    infoD.actualKFdensePoints[i].z = densePoints[i].z;
    infoD.actualKFdensePoints[i].r = densePoints[i].r;
    infoD.actualKFdensePoints[i].g = densePoints[i].g;
    infoD.actualKFdensePoints[i].b = densePoints[i].b;
  }
  infoD.actualKFdensePointsStamp = densePointsStamp;
  pendingDense.push_back(infoD);
  pthread_mutex_unlock(&denseMutex);
*/
  color_ptr = initColor_ptr->toImageMsg();
  depth_ptr = initDepth_ptr->toImageMsg();



  double result = 0.0;

  c2tam_srvs::InitFromRGBD srv;
  srv.request.id = id;

  c2tam_msgs::KeyFrame msg;

  Vector<2> dim = vec(skF.imFrame.size());
  IplImage* greyImage     = cvCreateImage(cvSize((int)dim[0],(int)dim[1]) , IPL_DEPTH_8U, 1);
  cvSetImageData(greyImage,  skF.imFrame.data(),(int)dim[0]);
  ros_img_ptr = bridge_.cvToImgMsg(greyImage, "mono8");

  srv.request.First.img = sensor_msgs::Image(*ros_img_ptr);

  imgkf1.copy_from(skF.imFrame); //IMG1

  for(unsigned int i=0; i<skF.sMeasurements.size(); i++)
  {
    c2tam_msgs::Measurement m;
    m.indexPointMap = skF.sMeasurements[i].indexPointMap;
    m.nLevel = skF.sMeasurements[i].m.nLevel;
    m.bSubPix = skF.sMeasurements[i].m.bSubPix;
    m.v2RootPos.push_back(skF.sMeasurements[i].m.v2RootPos[0]);
    m.v2RootPos.push_back(skF.sMeasurements[i].m.v2RootPos[1]);
    m.indexPointMap = skF.sMeasurements[i].indexPointMap;
    m.Source = skF.sMeasurements[i].m.Source;
    srv.request.First.sMeasurements.push_back(m);
  }

  srv.request.First.se3CfromW.push_back( skF.se3CfromW.get_translation()[0]);
  srv.request.First.se3CfromW.push_back( skF.se3CfromW.get_translation()[1]);
  srv.request.First.se3CfromW.push_back( skF.se3CfromW.get_translation()[2]);

  Matrix< 3 > mAux;
  mAux = skF.se3CfromW.get_rotation().get_matrix();

  srv.request.First.se3CfromW.push_back(mAux(0,0));
  srv.request.First.se3CfromW.push_back(mAux(0,1));
  srv.request.First.se3CfromW.push_back(mAux(0,2));
  srv.request.First.se3CfromW.push_back(mAux(1,0));
  srv.request.First.se3CfromW.push_back(mAux(1,1));
  srv.request.First.se3CfromW.push_back(mAux(1,2));
  srv.request.First.se3CfromW.push_back(mAux(2,0));
  srv.request.First.se3CfromW.push_back(mAux(2,1));
  srv.request.First.se3CfromW.push_back(mAux(2,2));

  srv.request.First.bFixed =  skF.bFixed;
  srv.request.First.dSceneDepthMean =  skF.dSceneDepthMean;
  srv.request.First.dSceneDepthSigma = skF.dSceneDepthSigma;

  //DCTAM

/*
  for (int i=0; i < 640*480;i++){
    c2tam_msgs::RgbdInfo rgbd;
    rgbd.r = (int) (pendingDense[pendingDense.size()-1].actualKFdensePoints[i].r * 255);
    rgbd.g = (int) (pendingDense[pendingDense.size()-1].actualKFdensePoints[i].g * 255);
    rgbd.b = (int) (pendingDense[pendingDense.size()-1].actualKFdensePoints[i].b * 255);
    rgbd.z = pendingDense[pendingDense.size()-1].actualKFdensePoints[i].z;
    srv.request.First.rgbd.push_back(rgbd);
  }
*/

  unsigned color_step, color_skip;

  int pixel_data_size = 3;
  char red_idx = 0, green_idx = 1, blue_idx = 2;
  if(color_ptr->encoding.compare("mono8") == 0) pixel_data_size = 1;
  if(color_ptr->encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }

  ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", color_ptr->encoding.c_str());
  color_step = pixel_data_size * color_ptr->width / depth_ptr->width;
  color_skip = pixel_data_size * (color_ptr->height / depth_ptr->height - 1) * color_ptr->width;

  const float* depth_buffer = reinterpret_cast<const float*>(&depth_ptr->data[0]);
  const uint8_t* rgb_buffer = &color_ptr->data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  int color_idx = 0, depth_idx = 0;
  double depth_scaling = 1;// = ParameterServer::instance()->get<double>("depth_scaling_factor");

  int i = 0;

  for (int v = 0; v < (int)color_ptr->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)color_ptr->width; ++u, color_idx += color_step, ++depth_idx,i++)
    {
      c2tam_msgs::RgbdInfo rgbd;
      rgbd.z = depth_buffer[depth_idx] * depth_scaling;

      // Fill in color
      if(pixel_data_size == 3){
        rgbd.r   = rgb_buffer[color_idx + red_idx];
        rgbd.g = rgb_buffer[color_idx + green_idx];
        rgbd.b  = rgb_buffer[color_idx + blue_idx];
      } else {
        rgbd.r   = rgbd.g = rgbd.b  = rgb_buffer[color_idx];
      }
      srv.request.First.rgbd.push_back(rgbd);   
    }
  }

  if (clientInitFromRGBD->call(srv))
  {
    se3.get_translation()[0] = srv.response.mse3CamFromWorld[0];
    se3.get_translation()[1] = srv.response.mse3CamFromWorld[1];
    se3.get_translation()[2] = srv.response.mse3CamFromWorld[2];

    Matrix< 3 > mAux_res;

    mAux_res(0,0) = srv.response.mse3CamFromWorld[3];
    mAux_res(0,1) = srv.response.mse3CamFromWorld[4];
    mAux_res(0,2) = srv.response.mse3CamFromWorld[5];

    mAux_res(1,0) = srv.response.mse3CamFromWorld[6];
    mAux_res(1,1) = srv.response.mse3CamFromWorld[7];
    mAux_res(1,2) = srv.response.mse3CamFromWorld[8];

    mAux_res(2,0) = srv.response.mse3CamFromWorld[9];
    mAux_res(2,1) = srv.response.mse3CamFromWorld[10];
    mAux_res(2,2) = srv.response.mse3CamFromWorld[11];

    se3.get_rotation() = mAux_res;
    result = srv.response.mdWiggleScaleDepthNormalized;
  }
  else
  {
    ROS_ERROR("Failed to call service init_from_rgbd_service");
    return result;
  }

  //mMap.bGood = true; //PUT it on dataPointsCallback
  return result;

}


int Tracker::GetIdClient(int mode)
{
  bool readyService = false;
  c2tam_srvs::GetId srv;
  srv.request.num = mode;
  srv.request.cx = cam_cx;
  srv.request.cy = cam_cy;
  srv.request.fx = cam_fx;
  srv.request.fy = cam_fy;

  while (!readyService){
    if (clientGetId->call(srv))
    {
      //ROS_INFO("ID: %d", srv.response.id);
      readyService = true;
    }
    else
    {
      ROS_ERROR("Failed to call service get_id_service, Try it again");
      //return -1;
      sleep(1);
    }
  }
  return srv.response.id;

}

int Tracker::FreeIdClient(int mode)
{
  //ROS_INFO("FreeIdClient");

  bool readyService = false;
  c2tam_srvs::GetId srv;
  srv.request.num = mode;
  while (!readyService){
    if (clientFreeId->call(srv))
    {
      ROS_INFO("ID: %d", srv.response.id);
      readyService = true;
    }
    else
    {
      ROS_ERROR("Failed to call service /vslam/free_id_service, Try it again");
      sleep(1);
      //return -1;
    }
  }
  return srv.response.id;

}

int Tracker::LoadMapClient(std::string nameMap)
{

  c2tam_srvs::LoadMap srv;
  srv.request.mode = mode;
  srv.request.name = nameMap.c_str();;
  if (clientLoadMap->call(srv))
  {
    ROS_INFO("ID: %d", srv.response.id);
    loadMapServer = true;
    queueSizeInfo = srv.response.queueSize;
    mdWiggleScaleDepthNormalized = srv.response.mdWiggleScaleDepthNormalized;
    mMap.bGood = true;
    startTracker = false;
  }
  else
  {
    ROS_ERROR("Failed to call service Load_map_client");
    return -1;
  }
  return srv.response.id;
}

void Tracker::LoadIdMapClient(double ScaleDepth)
{
    loadMapServer = true;
    queueSizeInfo = 0;
    mdWiggleScaleDepthNormalized = ScaleDepth;
    mMap.bGood = true;
    startTracker = false;
}


bool Tracker::RequestResetClient()
{
  c2tam_srvs::RequestReset srv;
  srv.request.id = id;
  if (clientRequestReset->call(srv))
  {
    //ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service request_reset_service");
    return false;
  }
  return srv.response.reset;

}

bool Tracker::ResetDoneClient()
{

  c2tam_srvs::RequestReset srv;
  srv.request.id = id;
  if (clientResetDone->call(srv))
  {
    //ROS_INFO("OK");
  }
  else
  {
    ROS_ERROR("Failed to call service reset_done_service");
    return false;
  }
  return srv.response.reset;

}


double Tracker::KeyFrameLinearDist(SE3<> k1,SE3<> k2)
{
  Vector<3> v3KF1_CamPos = k1.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dDist = sqrt(v3Diff * v3Diff);
  return dDist;
}

double Tracker::KeyFrameAngularDist(SE3<> k1,SE3<> k2)
{
  double roll, yaw;
  tf::Quaternion q;

  geometry_msgs::Quaternion orientation1;
  double pitch1;
  geometry_msgs::Quaternion orientation2;
  double pitch2;

  Vector<3> cam_loc_rot_1 = k1.get_rotation().ln();
  double theta_1 = norm(cam_loc_rot_1);
  normalize(cam_loc_rot_1);
  if (std::isnan(cam_loc_rot_1[0]))
    orientation1.x = 0;
  else
    orientation1.x = cam_loc_rot_1[0] * sin(theta_1/2);

  if (std::isnan(cam_loc_rot_1[1]))
    orientation1.y = 0;
  else
    orientation1.y = cam_loc_rot_1[1] * sin(theta_1/2);

  if (std::isnan(cam_loc_rot_1[2]))
    orientation1.z = 0;
  else
    orientation1.z = cam_loc_rot_1[2] * sin(theta_1/2);

  orientation1.w = cos(theta_1/2);

 
  tf::quaternionMsgToTF(orientation1, q);
  tf::Matrix3x3(q).getRPY(roll, pitch1, yaw);


  Vector<3> cam_loc_rot_2 = k2.get_rotation().ln();
  double theta_2 = norm(cam_loc_rot_2);
  normalize(cam_loc_rot_2);
  if (std::isnan(cam_loc_rot_2[0]))
    orientation2.x = 0;
  else
    orientation2.x = cam_loc_rot_2[0] * sin(theta_2/2);

  if (std::isnan(cam_loc_rot_2[1]))
    orientation2.y = 0;
  else
    orientation2.y = cam_loc_rot_2[1] * sin(theta_2/2);

  if (std::isnan(cam_loc_rot_2[2]))
    orientation2.z = 0;
  else
    orientation2.z = cam_loc_rot_2[2] * sin(theta_2/2);

  orientation2.w = cos(theta_2/2);


  tf::quaternionMsgToTF(orientation2, q);
  tf::Matrix3x3(q).getRPY(roll, pitch2, yaw);


 return fabs(pitch1-pitch2);
}

SE3<> Tracker::ClosestKeyFrame(SE3<> k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  bool pendingClosest = false;
  int pendingnClosest = -1;
  SE3<> pose;


  pthread_mutex_lock(&addKFMutex); 

  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
  {
     double dDist = KeyFrameLinearDist(k, mMap.vpKeyFrames[i]->se3CfromW);
     if(dDist < dClosestDist)
     {
        dClosestDist = dDist;
        nClosest = i;
     }
   }
   assert(nClosest != -1);

   pthread_mutex_lock(&pendingKFMutex); 

   // Queue of pending KF. Waiting for Mapping ACK
   for(unsigned int i=0; i<pendingKF.size(); i++)
   {
     double pendingdDist = KeyFrameLinearDist(k, pendingKF[i]);
     if(pendingdDist < dClosestDist)
     {
        dClosestDist = pendingdDist;
        pendingnClosest = i;
        pendingClosest = true;
     }
   }
   if (pendingClosest){
     pose = pendingKF[pendingnClosest];
   }
   else{
     pose = mMap.vpKeyFrames[nClosest]->se3CfromW;
   }
   pthread_mutex_unlock(&pendingKFMutex);
   pthread_mutex_unlock(&addKFMutex); 

   return pose;
}


bool Tracker::NeedNewKeyFrame(SE3<> kCurrent, double dSceneDepthMean)
{
  SE3<>  pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, pClosest);
  double dDist1 = dDist;


  double aDist = KeyFrameAngularDist(kCurrent, pClosest);
  dDist *= (1.0 / dSceneDepthMean);

//  if((dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",1.0,SILENT) * mdWiggleScaleDepthNormalized) ||
/* if((dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",needKFCoefficient,SILENT) * mdWiggleScaleDepthNormalized) ||
    ((dDist < GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",(double) needKFCoefficient/ (double) 3,SILENT) * mdWiggleScaleDepthNormalized) && aDist > needAngKFCoefficient)){ //0.5*/

if(dDist1 > needKFCoefficient ||  aDist > needAngKFCoefficient){
 //  ROS_INFO("%f\t%f\t%fTRUE%f",dDist1,dDist,aDist,needKFCoefficient);

   return true;
  }
else{
 //  ROS_INFO("%f\t%f\t%fFALSE%f",dDist1,dDist,aDist,needKFCoefficient);

}
  return false;
   
/* 
  if((dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",0.8,SILENT) * mdWiggleScaleDepthNormalized) ||
    ((dDist < GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",(double) 0.8/ (double) 3,SILENT) * mdWiggleScaleDepthNormalized) && aDist > 0.2)){
   return true;
  }
*/

}

double Tracker::DistToNearestKeyFrame(SE3<>  kCurrent)
{
  SE3<>  pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, pClosest);
  return dDist;
}

bool Tracker::IsDistanceToNearestKeyFrameExcessive(SE3<> kCurrent)
{
  return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}


void Tracker::dataPointsCallbackCb(const c2tam_msgs::DataPoints::ConstPtr& msg)
{
  boost::thread thrdDataPointsCallback(boost::bind(&Tracker::dataPointsCallback, this, msg));
  thrdDataPointsCallback.detach();
}





// DCTAM
void Tracker::copyDense(KeyFrame *k){

    k->denseModel = new SVertex[640*480];
    for (int i=0; i< 640*480; i++){
/*      k->denseModel[i].x = actualKFdensePoints[i].x;
      k->denseModel[i].y = actualKFdensePoints[i].y;
      k->denseModel[i].z = actualKFdensePoints[i].z;
      k->denseModel[i].r = actualKFdensePoints[i].r;
      k->denseModel[i].g = actualKFdensePoints[i].g;
      k->denseModel[i].b = actualKFdensePoints[i].b;
*/      k->denseModel[i].x = pendingDense[0].actualKFdensePoints[i].x;
      k->denseModel[i].y = pendingDense[0].actualKFdensePoints[i].y;
      k->denseModel[i].z = pendingDense[0].actualKFdensePoints[i].z;
      k->denseModel[i].r = pendingDense[0].actualKFdensePoints[i].r;
      k->denseModel[i].g = pendingDense[0].actualKFdensePoints[i].g;
      k->denseModel[i].b = pendingDense[0].actualKFdensePoints[i].b;
    }
    pendingDense.erase(pendingDense.begin()); 
    k->denseModelSize = 640*480;

}
//

// DCTAM
double Tracker::computeScale(){

  Vector<3> v3KinectCam;
  Vector<3> v3WorldPos;
  Vector<3> v3CamPos;
  std::vector<double> scaleVector;
  double kinectDst = 0;
  double monoDst = 0;    

  scaleVector.clear();


  for(int i=0; i< mMap.vpPoints.size(); i++){

   if (mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->denseModelSize > 0){

    if(mMap.vpPoints[i]->nSourceLevel == 0 && !std::isnan(mMap.vpPoints[i]->v3WorldPos[0]) &&
	  !std::isnan(mMap.vpPoints[i]->v3WorldPos[1]) && !std::isnan(mMap.vpPoints[i]->v3WorldPos[2]) &&
          !std::isnan(mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->denseModel[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].x) &&
          !std::isnan(mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->denseModel[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].y) &&
          !std::isnan(mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->denseModel[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].z)){

      v3WorldPos[0] = mMap.vpPoints[i]->v3WorldPos[0];
      v3WorldPos[1] = mMap.vpPoints[i]->v3WorldPos[1];
      v3WorldPos[2] = mMap.vpPoints[i]->v3WorldPos[2];
      SE3<> se3CfromW = mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->se3CfromW;

      v3KinectCam[0] = mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->denseModel[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].x;  
    
      v3KinectCam[1] = mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->denseModel[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].y; 

      v3KinectCam[2] = mMap.vpKeyFrames[mMap.vpPoints[i]->indexKF]->denseModel[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].z; 
  
      v3CamPos = se3CfromW * v3WorldPos;

      kinectDst = sqrt((v3KinectCam[0] * v3KinectCam[0]) + 
				(v3KinectCam[1] * v3KinectCam[1]) + 
 				(v3KinectCam[2] * v3KinectCam[2]));
 
      monoDst = sqrt((v3CamPos[0] * v3CamPos[0]) + 
				(v3CamPos[1] * v3CamPos[1]) + 
 				(v3CamPos[2] * v3CamPos[2]));

 
      scaleVector.push_back(monoDst / kinectDst);
    }
   }
  }

  if (scaleVector.size() > 0){
    sort(scaleVector.begin(),scaleVector.end());
    return scaleVector[scaleVector.size()/2];
  }
  else
   return 1;
}


void Tracker::dataPointsCallback(const c2tam_msgs::DataPoints::ConstPtr& msg){

bool bmap = false;

// PARA no añadir una vez se han fusionado mapas!!!
if (addVariable){
//ROS_INFO("addVariable");
  struct timeval	tc;
  double timeP;

  gettimeofday(&tc, NULL);
  timeP=tc.tv_sec+tc.tv_usec/1e6;

  pthread_mutex_lock(&addKFMutex); 

  for(unsigned int i=0; i< msg->newKeyframes.size(); i++)
  {
    //fprintf(stderr,"New KF %d size %d\n",i,msg->newKeyframes.size());
    KeyFrame *k = new KeyFrame();


    k->se3CfromW.get_translation()[0] = msg->newKeyframes[i].pose[0];
    k->se3CfromW.get_translation()[1] = msg->newKeyframes[i].pose[1];
    k->se3CfromW.get_translation()[2] = msg->newKeyframes[i].pose[2];

    Matrix< 3 > mAux_res;

    mAux_res(0,0) = msg->newKeyframes[i].pose[3];
    mAux_res(0,1) = msg->newKeyframes[i].pose[4];
    mAux_res(0,2) = msg->newKeyframes[i].pose[5];

    mAux_res(1,0) = msg->newKeyframes[i].pose[6];
    mAux_res(1,1) = msg->newKeyframes[i].pose[7];
    mAux_res(1,2) = msg->newKeyframes[i].pose[8];

    mAux_res(2,0) = msg->newKeyframes[i].pose[9];
    mAux_res(2,1) = msg->newKeyframes[i].pose[10];
    mAux_res(2,2) = msg->newKeyframes[i].pose[11];

    k->se3CfromW.get_rotation() = mAux_res;

    k->color = msg->newKeyframes[i].color;

    k->denseModelSize = 0;

    if (cntKeyFrame == 1){
      k->MakeKeyFrame_Lite(imgkf1);
      k->pSBI = new SmallBlurryImage(*k);  // RELOC
      k->pSBI->MakeJacs(); // RELOC
      cntKeyFrame++;
      cntKeyFrame++;  // -> RGBD initialization
      bmap = true;    // -> RGBD initialization
    }
    else if(cntKeyFrame == 2){
      k->MakeKeyFrame_Lite(imgkf2);
      k->pSBI = new SmallBlurryImage(*k); // RELOC
      k->pSBI->MakeJacs(); // RELOC
      cntKeyFrame++;
    }
    else{
      if (loadMapServer){
       boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr(new sensor_msgs::Image(msg->imgkf[i]));
       IplImage *im;
       im = bridge_.imgMsgToCv(ros_img_ptr, "mono8");
       ImageRef imSize(im->width,im->height);
       Image<CVD::byte> imFrame(imSize);
       memcpy(imFrame.data(),im->imageData,im->width*im->height*sizeof(uchar));


       if (msg->newKeyframes[i].rgbd.size() > 0){
         k->denseModel = new SVertex[640*480];
         k->denseModelSize = 640*480;
         mMap.enableDense = true;
         int indexImage = 0;
         for (int u=0; u< 480; u++){
           for (int v=0; v< 640; v++){
             indexImage = (u*640) + v;
               k->denseModel[indexImage].z = msg->newKeyframes[i].rgbd[indexImage].z;
               k->denseModel[indexImage].x = (double) ((double) (((double) v - 319.5) * msg->newKeyframes[i].rgbd[indexImage].z) / (double) 525.0);
               k->denseModel[indexImage].y = (double) ((double) (((double) u - 239.5) * msg->newKeyframes[i].rgbd[indexImage].z) / (double) 525.0);
               k->denseModel[indexImage].r = (double) msg->newKeyframes[i].rgbd[indexImage].r /(double) 255;
               k->denseModel[indexImage].g = (double) msg->newKeyframes[i].rgbd[indexImage].g /(double) 255;
               k->denseModel[indexImage].b = (double) msg->newKeyframes[i].rgbd[indexImage].b /(double) 255;

           }
         }

       }
       else {
         k->denseModelSize = 0;
       }
             
       k->MakeKeyFrame_Lite(imFrame);
       k->pSBI = new SmallBlurryImage(*k); // RELOC
       k->pSBI->MakeJacs(); // RELOC
      }
      else{
        if (msg->newKeyframes[0].color != 10 && msg->newKeyframes[0].color != 12 ){
          k->MakeKeyFrame_Lite(imgkf);
          k->pSBI = new SmallBlurryImage(*k); // RELOC
          k->pSBI->MakeJacs();  // RELOC
        }
        else{  
         if ( msg->newKeyframes[0].color != 12 ){ 
           boost::shared_ptr<const sensor_msgs::Image> ros_img_merge_ptr(new sensor_msgs::Image(msg->imgkf[i]));
           IplImage *im_merge;
           im_merge = bridge_.imgMsgToCv(ros_img_merge_ptr, "mono8");
           ImageRef im_mergeSize(im_merge->width,im_merge->height);
           Image<CVD::byte> im_mergeFrame(im_mergeSize);
           memcpy(im_mergeFrame.data(),im_merge->imageData,im_merge->width*im_merge->height*sizeof(uchar));

           k->MakeKeyFrame_Lite(im_mergeFrame);
           k->pSBI = new SmallBlurryImage(*k); // RELOC
           k->pSBI->MakeJacs(); // RELOC 

           if (msg->newKeyframes[i].rgbd.size() > 0){
             k->denseModel = new SVertex[640*480];
             k->denseModelSize = 640*480;
             int indexImage = 0;
             for (int u=0; u< 480; u++){
               for (int v=0; v< 640; v++){
                 indexImage = (u*640) + v;
                   k->denseModel[indexImage].z = msg->newKeyframes[i].rgbd[indexImage].z;
                   k->denseModel[indexImage].x = (double) ((double) (((double) v - 319.5) * msg->newKeyframes[i].rgbd[indexImage].z) / (double) 525.0);
                   k->denseModel[indexImage].y = (double) ((double) (((double) u - 239.5) * msg->newKeyframes[i].rgbd[indexImage].z) / (double) 525.0);
                   k->denseModel[indexImage].r = (double) msg->newKeyframes[i].rgbd[indexImage].r /(double) 255;
                   k->denseModel[indexImage].g = (double) msg->newKeyframes[i].rgbd[indexImage].g /(double) 255;
                   k->denseModel[indexImage].b = (double) msg->newKeyframes[i].rgbd[indexImage].b /(double) 255;
               }
             }
           }
         }
        }
      }
    }
    mMap.vpKeyFrames.push_back(k);
  }


  // REMOVE OF PENDING QUEUE

  pthread_mutex_lock(&pendingKFMutex); 

  if(msg->newKeyframes.size() == 1){
    if (msg->newKeyframes[0].color != 10 || msg->newKeyframes[0].color != 11 ){
      if (pendingKF.size() > 0){
       pendingKF.erase(pendingKF.begin()); 
      }
    }
  }
  pthread_mutex_unlock(&pendingKFMutex);
  pthread_mutex_unlock(&addKFMutex); 


// DCTAM
  if (!loadMapServer){
    if (msg->newKeyframes.size() > 0){
      if (msg->newKeyframes[0].color != 10 && msg->newKeyframes[0].color != 12){
      //  copyDense(mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]);
        mMap.enableDense = true;
      }
    }
  }

  for(unsigned int i=0; i< msg->newPoints.size(); i++)
  {
    Vector<2> irPoints;

    MapPoint *p = new MapPoint;

    p->bBad = msg->newPoints[i].bBad;
    p->nSourceLevel = msg->newPoints[i].nSourceLevel;
    p->nMEstimatorOutlierCount = msg->newPoints[i].nMEstimatorOutlierCount;
    p->nMEstimatorInlierCount = msg->newPoints[i].nMEstimatorInlierCount;
    p->indexPointMap = msg->newPoints[i].indexPointMap;

    p->v3WorldPos[0] = msg->newPoints[i].v3[0];
    p->v3WorldPos[1] = msg->newPoints[i].v3[1];
    p->v3WorldPos[2] = msg->newPoints[i].v3[2];

    p->v3PixelDown_W[0] = msg->newPoints[i].v3[3];
    p->v3PixelDown_W[1] = msg->newPoints[i].v3[4];
    p->v3PixelDown_W[2] = msg->newPoints[i].v3[5];

    p->v3PixelRight_W[0] = msg->newPoints[i].v3[6];
    p->v3PixelRight_W[1] = msg->newPoints[i].v3[7];
    p->v3PixelRight_W[2] = msg->newPoints[i].v3[8];

    p->pPatchSourceKF =  mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1];
 	ImageRef ir(msg->newPoints[i].irCenter[0],msg->newPoints[i].irCenter[1]);
    p->irCenter = ir;

    p->update = true;
    p->indexKF = mMap.vpKeyFrames.size()-1;

    mMap.vpPoints.push_back(p);

  }

  queueSizeInfo = msg->queueSize;

  if (loadMapServer){
    loadMapServer = false;
    for(unsigned int i=0; i< msg->loadPoints.size(); i++)
    {
      MapPoint *p = new MapPoint;

      p->bBad = msg->loadPoints[i].p.bBad;
      p->nSourceLevel = msg->loadPoints[i].p.nSourceLevel;
      p->nMEstimatorOutlierCount = msg->loadPoints[i].p.nMEstimatorOutlierCount;
      p->nMEstimatorInlierCount = msg->loadPoints[i].p.nMEstimatorInlierCount;
      p->indexPointMap = msg->loadPoints[i].p.indexPointMap;

      p->v3WorldPos[0] = msg->loadPoints[i].p.v3[0];
      p->v3WorldPos[1] = msg->loadPoints[i].p.v3[1];
      p->v3WorldPos[2] = msg->loadPoints[i].p.v3[2];

      p->v3PixelDown_W[0] = msg->loadPoints[i].p.v3[3];
      p->v3PixelDown_W[1] = msg->loadPoints[i].p.v3[4];
      p->v3PixelDown_W[2] = msg->loadPoints[i].p.v3[5];

      p->v3PixelRight_W[0] = msg->loadPoints[i].p.v3[6];
      p->v3PixelRight_W[1] = msg->loadPoints[i].p.v3[7];
      p->v3PixelRight_W[2] = msg->loadPoints[i].p.v3[8];

      p->pPatchSourceKF =  mMap.vpKeyFrames[msg->loadPoints[i].indexKeyFrame];
      p->indexKF = msg->loadPoints[i].indexKeyFrame;
 	ImageRef ir(msg->loadPoints[i].p.irCenter[0],msg->loadPoints[i].p.irCenter[1]);
      p->irCenter = ir;

      mMap.vpPoints.push_back(p);
    }   
    startTracker = true; 
  }else{
    for(unsigned int i=0; i< msg->loadPoints.size(); i++)
    {
      MapPoint *p = new MapPoint;

      p->bBad = msg->loadPoints[i].p.bBad;
      p->nSourceLevel = msg->loadPoints[i].p.nSourceLevel;
      p->nMEstimatorOutlierCount = msg->loadPoints[i].p.nMEstimatorOutlierCount;
      p->nMEstimatorInlierCount = msg->loadPoints[i].p.nMEstimatorInlierCount;
      p->indexPointMap = msg->loadPoints[i].p.indexPointMap;

      p->v3WorldPos[0] = msg->loadPoints[i].p.v3[0];
      p->v3WorldPos[1] = msg->loadPoints[i].p.v3[1];
      p->v3WorldPos[2] = msg->loadPoints[i].p.v3[2];

      p->v3PixelDown_W[0] = msg->loadPoints[i].p.v3[3];
      p->v3PixelDown_W[1] = msg->loadPoints[i].p.v3[4];
      p->v3PixelDown_W[2] = msg->loadPoints[i].p.v3[5];

      p->v3PixelRight_W[0] = msg->loadPoints[i].p.v3[6];
      p->v3PixelRight_W[1] = msg->loadPoints[i].p.v3[7];
      p->v3PixelRight_W[2] = msg->loadPoints[i].p.v3[8];

      p->pPatchSourceKF =  mMap.vpKeyFrames[msg->loadPoints[i].indexKeyFrame];
      p->indexKF = msg->loadPoints[i].indexKeyFrame;
 	ImageRef ir(msg->loadPoints[i].p.irCenter[0],msg->loadPoints[i].p.irCenter[1]);
      p->irCenter = ir;

      mMap.vpPoints.push_back(p);
    } 



  }

//BAD POINTS

  if (msg->badPoints.size() > 0){
    for(unsigned int i=0; i< msg->badPoints.size(); i++)
    {
       mMap.vpPoints[msg->badPoints[i]]->bBad = true;
    }

    mMap.MoveBadPointsToTrash();
  }



//MOD POINTS
 
 for(int i=0; i < mMap.vpPoints.size(); i++){
   mMap.vpPoints[i]->update = false;
 }

 for(unsigned int i=0; i< msg->modPoints.size(); i++)
  {
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->bBad = msg->modPoints[i].bBad;
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->nSourceLevel = msg->modPoints[i].nSourceLevel;
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->nMEstimatorOutlierCount = msg->modPoints[i].nMEstimatorOutlierCount;
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->nMEstimatorInlierCount = msg->modPoints[i].nMEstimatorInlierCount;
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->indexPointMap = msg->modPoints[i].indexPointMap;

    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3WorldPos[0] = msg->modPoints[i].v3[0];
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3WorldPos[1] = msg->modPoints[i].v3[1];
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3WorldPos[2] = msg->modPoints[i].v3[2];

    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3PixelDown_W[0] = msg->modPoints[i].v3[3];
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3PixelDown_W[1] = msg->modPoints[i].v3[4];
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3PixelDown_W[2] = msg->modPoints[i].v3[5];
 
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3PixelRight_W[0] = msg->modPoints[i].v3[6];
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3PixelRight_W[1] = msg->modPoints[i].v3[7];
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->v3PixelRight_W[2] = msg->modPoints[i].v3[8];
    mMap.vpPoints[msg->modPoints[i].indexPointMap]->update = true;


  }


//MOD KF

  SE3<> se3;

  for(unsigned int i=0; i< msg->modKeyframes.size(); i++)
  {


    se3.get_translation()[0] = msg->modKeyframes[i].pose[0];
    se3.get_translation()[1] = msg->modKeyframes[i].pose[1];
    se3.get_translation()[2] = msg->modKeyframes[i].pose[2];

    Matrix< 3 > mAux_res;

    mAux_res(0,0) = msg->modKeyframes[i].pose[3];
    mAux_res(0,1) = msg->modKeyframes[i].pose[4];
    mAux_res(0,2) = msg->modKeyframes[i].pose[5];

    mAux_res(1,0) = msg->modKeyframes[i].pose[6];
    mAux_res(1,1) = msg->modKeyframes[i].pose[7];
    mAux_res(1,2) = msg->modKeyframes[i].pose[8];

    mAux_res(2,0) = msg->modKeyframes[i].pose[9];
    mAux_res(2,1) = msg->modKeyframes[i].pose[10];
    mAux_res(2,2) = msg->modKeyframes[i].pose[11];

    se3.get_rotation() = mAux_res;

    mMap.vpKeyFrames[i]->se3CfromW = se3;
  }

//BAD POINTS



  pthread_mutex_lock(&denseScaleMutex);
  mMap.denseScale = 1;// RGBD SLAM    // computeScale(); //DCTAM  
  pthread_mutex_unlock(&denseScaleMutex);


  if (bmap)
    mMap.bGood = true; //-> RGBD initialization


}
}


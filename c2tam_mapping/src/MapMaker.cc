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

#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "HomographyInit.h"

#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>
#include <cvd/image_io.h>

#include <cvd/draw.h>


#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

#include <gvars3/instances.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include <fstream>
#include <algorithm>   

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

using namespace CVD;
using namespace std;
using namespace GVars3;

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, ros::NodeHandle n,int id,int mode,double cx, double cy, double fx, double fy, bool startMapping)
 : mMap(m), nh(n), mapId(id), mapMode(mode), robotId("ari")
{
  mbResetRequested = false;
  pubLoadData =  false;
  loadSaveMutex = PTHREAD_MUTEX_INITIALIZER;
  ekfMutex = PTHREAD_MUTEX_INITIALIZER;
  mergeMutex = PTHREAD_MUTEX_INITIALIZER;

   if(!startMapping)
     pthread_mutex_lock(&loadSaveMutex);

  Reset();
  start(); // This CVD::thread func starts the map-maker thread with function run()
  GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);


  GV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.10, SILENT); // Default to 10cm between keyframes

  //NEW CAMERA FOR MAPPING
  mCamera = new ATANCamera("Camera");
  mRelocaliser =  new Relocaliser(mMap, mCamera);

  ostringstream nameServiceDP;
  nameServiceDP << "/map" << mapId << "/dataPoints";


  //ROS PUBLISHER TOPICS
  dataPoints_pub = new  ros::Publisher(nh.advertise<c2tam_msgs::DataPoints>(nameServiceDP.str().c_str(), 1000));

  mapInfo_pub = new  ros::Publisher(nh.advertise<c2tam_msgs::MapInfo>("/c2tam_mapping/map_info", 1000));

  ostringstream nameServiceSI;
  nameServiceSI << "/map" << mapId << "/semanticInfo";

  //ROS PUBLISHER TOPICS
  semanticInfo_pub = new  ros::Publisher(nh.advertise<c2tam_msgs::SemanticInfo>(nameServiceSI.str().c_str(), 1000));

  get_map_objects_ = nh.advertiseService("vslam/get_map_objects", &MapMaker::GetMapObjectsCb, this);

  ros::NodeHandle private_nh("~");
  private_nh.param("robot_id", robotId, robotId);



  //ROS PARAMS
  nh.param("C2TAMmapping/numKeyFrames", numKeyFrames, -1); 
  nh.param("C2TAMmapping/visualizer", mapping_visualizer, false);

  //MUTEX
  pointsMutex = PTHREAD_MUTEX_INITIALIZER;
 
  pointsBool = false;
  indexBadPoints.clear();

  newKeyFrameInfo =  false;
  modKeyFrameInfo =  false;


  mbDidCoarse = false;
  mdMSDScaledVelocityMagnitude = 0.0;
  mbJustRecoveredSoUseCoarse = false;

  TrackerData::irImageSize[0] = 640;
  TrackerData::irImageSize[1] = 480;

  if (mapMode==1 || mapMode==7 || mapMode==8)
    mdWiggleScale = *mgvdWiggleScale;

  if (mapMode==7 || mapMode==8)
    recoveryActive = true;
  else
    recoveryActive = false;

  for(int i=0; i< 20;i++){
    vctRecovery[i].firstKF = false;
    vctRecovery[i].secondKF = false;
    vctRecovery[i].recovery = false;
  }

  pub1 = false;
  pub2 = false;
  unlockSemantic = false;

  mMap.vpKeyFrames.reserve(200);

  std::string path_node;
  std::string path_file;
  FILE * find_file = popen("rospack find c2tam_mapping", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;
  path_node = command_find;

  std::string path_file_send;
  char filenameSend[255];
  sprintf(filenameSend,"/.times-send-4.dat");
  path_file_send = path_node + filenameSend;

  cam_cx = cx;
  cam_cy = cy;
  cam_fx = fx;
  cam_fy = fy;

};

ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here

void MapMaker::Reset()
{
  // This is only called from within the mapmaker thread...
  mMap.Reset();

  mvFailureQueue.clear();
  while(!mqNewQueue.empty()) mqNewQueue.pop();
  mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes
  mvpKeyFrameQueue.clear(); // TODO: actually erase old keyframes
  mbBundleRunning = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbResetDone = true;
  mbResetRequested = false;
  mbBundleAbortRequested = false;

  mbDidCoarse = false;
  mdMSDScaledVelocityMagnitude = 0.0;
  mbJustRecoveredSoUseCoarse = false;

  aux_KF.dSceneDepthMean = 1.0;
  aux_KF.dSceneDepthSigma = 1.0;

  //CLEAR POINTS VECTORS
  pubPoints.clear();
  indexModPoints.clear();
  indexBadPoints.clear();

  for(int i =0; i< 20;i++){
    vctRecovery[i].firstKF = false;
    vctRecovery[i].secondKF = false;
    vctRecovery[i].recovery = false;
  }
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); pthread_mutex_unlock(&loadSaveMutex); pthread_mutex_unlock(&ekfMutex); pthread_mutex_unlock(&mergeMutex); continue;};

// 

void MapMaker::run()
{

#ifdef WIN32
  // For some reason, I get tracker thread starvation on Win32 when
  // adding key-frames. Perhaps this will help:
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif

  while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
  {
     sleep(5);
     if(mapMode!= 7){

       pthread_mutex_lock(&loadSaveMutex);
       pthread_mutex_lock(&ekfMutex);
       pthread_mutex_lock(&mergeMutex);

       if(pubLoadData){
         //ROS_INFO("PubLoadData TRUE");
         pubLoadData = false;
         usleep(1000000);
         pubLoadPoints();
       }

       struct timeval	tc,Tt;
       double t1,t2,Tt1,Tt2;
       //gettimeofday(&Tt, NULL);  //DEBUG TIME
       //Tt1=Tt.tv_sec+Tt.tv_usec/1e6;  //DEBUG TIME

       CHECK_RESET;
       sleep(5); // Sleep not really necessary, especially if mapmaker is busy
       CHECK_RESET;

       // Handle any GUI commands encountered..
       while(!mvQueuedCommands.empty())
       {
         GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
         mvQueuedCommands.erase(mvQueuedCommands.begin());
       }

       if(!mMap.IsGood()){  // Nothing to do if there is no map yet!
         pthread_mutex_unlock(&loadSaveMutex); 
         pthread_mutex_unlock(&ekfMutex);
         pthread_mutex_unlock(&mergeMutex);
         continue;
       }

       // From here on, mapmaker does various map-maintenance jobs in a certain priority
       // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
       // then that takes high priority.

       CHECK_RESET;
       // Should we run local bundle adjustment?
       if(!mbBundleConverged_Recent && QueueSize() == 0)
       {
         BundleAdjustRecentM();

         pthread_mutex_lock(&pointsMutex);
         if (!pointsBool){
           pthread_mutex_unlock(&pointsMutex);
           modKeyFrameInfo =  true;
         }
         else 
           pthread_mutex_unlock(&pointsMutex);

       }
       CHECK_RESET;
       // Are there any newly-made map points which need more measurements from older key-frames?
       if(mbBundleConverged_Recent && QueueSize() == 0)
        ReFindNewlyMade();

       CHECK_RESET;
       // Run global bundle adjustment?
       // 25th May
       if(mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0)
       {
         BundleAdjustAll(true);
         pthread_mutex_lock(&pointsMutex);
         if (!pointsBool){
           pthread_mutex_unlock(&pointsMutex);
           modKeyFrameInfo =  true;
         }
         else 
          pthread_mutex_unlock(&pointsMutex);
       }
   

       CHECK_RESET;
       // Very low priorty: re-find measurements marked as outliers
       if(mbBundleConverged_Recent && mbBundleConverged_Full && rand()%20 == 0 && QueueSize() == 0)
         ReFindFromFailureQueue();

       CHECK_RESET;
       HandleBadPoints();

       CHECK_RESET;
       // Any new key-frames to be added?
       if(QueueSize() > 0){
         pubPoints.clear();
         //fprintf(stderr,"Run AddKeyFrameFromTopOfQueue\n");
         AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
         newKeyFrameInfo =  true;
       }

       if(ObjectQueueSize()>0){
          AddObjecttoMap();
          BundleAdjustAll(true);
       }
  
       pubDataPoints();
 
       pthread_mutex_unlock(&loadSaveMutex);
       pthread_mutex_unlock(&ekfMutex);
       pthread_mutex_unlock(&mergeMutex);

    }
     else{
       if(pubLoadData){
         pubLoadData = false;
         usleep(1000000);
         pubLoadPoints();
         mapMode = 1;
       }
       usleep(1000000);

     }
  }
}

// Tracker calls this to demand a reset
void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
}

bool MapMaker::ResetDone()
{
  return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints()
{
  // Did the tracker see this point as an outlier more often than as an inlier?
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      MapPoint &p = *mMap.vpPoints[i];
      if (p.bSURF) continue;
      if (p.bBad) continue;
      if(p.nMEstimatorOutlierCount > 20 && p.nMEstimatorOutlierCount > p.nMEstimatorInlierCount){
	p.bBad = true;
      }
    }

  // All points marked as bad will be erased - erase all records of them
  // from keyframes in which they might have been measured.

    int outlierCount=0;

  for(unsigned int i=0; i<mMap.vpPoints.size(); i++){
    if(mMap.vpPoints[i]->bBad)
      {
        indexBadPoints.push_back(mMap.vpPoints[i]->indexPointMap);
        //@hoyROS_INFO("Bad Points %d %d\n\n\n",i,mMap.vpPoints[i]->indexPointMap);
	MapPoint *p = mMap.vpPoints[i];
	for(unsigned int j=0; j<mMap.vpKeyFrames.size(); j++)
	  {
	    KeyFrame &k = *mMap.vpKeyFrames[j];
	    if(k.mMeasurements.count(p))
	      k.mMeasurements.erase(p);
	  }
      }
   }
  // Move bad points to the trash list.
  mMap.MoveBadPointsToTrash();

}

MapMaker::~MapMaker()
{
  mbBundleAbortRequested = true;
  cout << "Waiting for mapmaker to die.." << endl;
  stop(); // makes shouldStop() return true
  cout << "Waiting for mapmaker to die.." << endl;
  join();
  cout << " .. mapmaker has died." << endl;
}


// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> MapMaker::ReprojectPoint(SE3<> se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
  // El calculo que hace es:
  // PA(X,Y,Z): Coordenadas del pto en A (Z != 1)
  // PB(X,Y,Z): Coordenadas del pto en B (Z != 1)
  // PA = se3AfromB * PB
  // v2A(x) * PA(Z) - PA(X) = 0
  // v2A(y) * PA(Z) - PA(Y) = 0
  // v2B(x) * PB(Z) - PB(X) = 0
  // v2B(y) * PB(Z) - PB(Y) = 0
  // 3 incognitas PB(X,Y,Z) 4 ecs. => Minimos cuadrados mediante svd.

  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();

  Matrix<4> A;
  A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A[2] = v2A[0] * PDash[2] - PDash[0];
  A[3] = v2A[1] * PDash[2] - PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame &k, int nLevel)
{
  vector<Candidate> &vCSrc = k.aLevels[nLevel].vCandidates;
  vector<Candidate> vCGood;
  vector<ImageRef> irBusyLevelPos;
  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  // Rellena de una lista con las mediciones de las caracteristicas
  // en los niveles n y n+1
  //cerr << "**********************************\n";
  for(meas_it it = k.mMeasurements.begin(); it!=k.mMeasurements.end(); it++)
  {
     //cerr << "\n Level: " << it->second.nLevel
     //     << "  PTO: "    << it->second.v2RootPos / LevelScale(nLevel);
     if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
	     continue;
     //cerr << "   Aniadido";
     irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
  }
  //cerr << "\n";

  // Only keep those candidates further than 10 pixels away from busy positions.
  // Usa la lista de arriba para seleccionar las caracteristicas candidatas
  // que se encuentren a mas de 10 px de las medidas
  unsigned int nMinMagSquared = 20*20;
 // cerr << "Nivel: " << nLevel
 //      << "  Nº candidatos: " << vCSrc.size();
  for(unsigned int i=0; i<vCSrc.size(); i++)
  {
     ImageRef irC = vCSrc[i].irLevelPos;
     //cerr << "\n -- PTO: "    << irC;
     bool bGood = true;
     for(unsigned int j=0; j<irBusyLevelPos.size(); j++)
	  {
	     ImageRef irB = irBusyLevelPos[j];
	     if((irB - irC).mag_squared() < nMinMagSquared)
	     {
	        bGood = false;
	        break;
	     }
	  }
     if(bGood){
        //cerr << "   Aniadido";
        vCGood.push_back(vCSrc[i]);
     }
  }
  vCSrc = vCGood;
  //cerr << " Aniadidos: " << vCGood.size() << endl;
  //cerr << "**********************************\n";
}



KeyFrame* MapMaker::KeyFrameWithParallax(KeyFrame &k)
{
   Vector<3> Pmd;
   Pmd[0]= 0.0; Pmd[1]=0.0; Pmd[2] = MaxSceneDepth(mMap.vpKeyFrames[0]);
   Pmd = k.se3CfromW.inverse() * Pmd;
   Vector<3> v1 = k.se3CfromW.get_translation() - Pmd;

  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
  {
     if(mMap.vpKeyFrames[i] == &k) continue;

     Vector<3> v2 = mMap.vpKeyFrames[i]->se3CfromW.get_translation() - Pmd;
     double angle = AngleBetweenVectors(v1, v2);
     angle=abs(angle)-0.15;
     if(angle < dClosestDist)
     {
        dClosestDist = angle;
        nClosest = i;
     }
   }
   assert(nClosest != -1);
   return mMap.vpKeyFrames[nClosest];
}

double MapMaker::AngleBetweenVectors(Vector<3> &v1, Vector<3> &v2 )
{
  double cosalpha = ( v1 * v2 ) / (norm(v1) * norm(v2));
  return acos(cosalpha);
}


double MapMaker::MaxSceneDepth(KeyFrame *pKF)
{

  vector<double> depths;
  for(meas_it it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
      MapPoint &point = *it->first;
      Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
      depths.push_back(v3PosK[2]);
    }
  sort(depths.begin(), depths.end());
  return depths[depths.size()-1];
}


// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel)
{
  KeyFrame &kSrc = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The new keyframe
  KeyFrame &kTarget = *(ClosestKeyFrame(kSrc));
  Level &l = kSrc.aLevels[nLevel];

  // Selecciona todos los candidatos que se encuentren a mas de 10 px
  // de las caracteristicas medidas.
  // No mira que las candidatas esten juntas o no.
  //cerr << "Antes de entrar: " << kSrc.aLevels[nLevel].vCandidates.size() << endl;
  ThinCandidates(kSrc, nLevel);
  //cerr << "Despues de entrar: " << kSrc.aLevels[nLevel].vCandidates.size() << endl;
  if(kSrc.bSemantic)
  {
     if(nLevel==0){
        vector<int> indexDelete;
        vector<vector <KeyFrame*> > kTarget2 = BestEpipolarKeyFrame(kSrc);
        if( kTarget2.size()>0){
            for(unsigned int j=0; j<l.vSurfOD.size();j++){
                cerr<<"punto "<<j<<" #KF " << kTarget2[j].size()<<endl;
               if( kTarget2[j].size()==0) continue;
               for (unsigned int ikf=0; ikf<  kTarget2[j].size(); ikf++){
                  bool bTriangulated=AddPointEpipolarSemantic(kSrc, *kTarget2[j][ikf], j);
                  if(bTriangulated){

                      indexDelete.push_back(j);
                      break;
                  }//endif(bTriangulated)
               }//endfor(kTargets)
           }//endfor(vSurfOD)
       }// endif(kTarget2.size()>0)
     }//endif(nLvel==0)

      return;
  }//endif(bSemantic)

  for(unsigned int i = 0; i<l.vCandidates.size(); i++){

      Vector<2> v2RootPos = LevelZeroPos(l.vCandidates[i].irLevelPos, nLevel);

      Vector<3> xyzPose;

      xyzPose[0] = kSrc.rgbdData[int(640*rint(v2RootPos[1])+ rint(v2RootPos[0]))].x;
      xyzPose[1] = kSrc.rgbdData[int(640*rint(v2RootPos[1])+ rint(v2RootPos[0]))].y;
      xyzPose[2] = kSrc.rgbdData[int(640*rint(v2RootPos[1])+ rint(v2RootPos[0]))].z;

      xyzPose = kSrc.se3CfromW.inverse() * xyzPose;

      if (!std::isnan(xyzPose[0]) && !std::isnan(xyzPose[1]) && !std::isnan(xyzPose[2])){
        addRGBDPoints(kSrc, nLevel, i, xyzPose);
      }
    }

};

bool MapMaker::AddPointEpipolarSemantic(KeyFrame &kSrc,
				KeyFrame &kTarget,
				int nCandidate)
{

  static Image<Vector<2> > imUnProj;
  static bool bMadeCache = false;

  // Proyecta toda la imagen al plano Z=1
  if(!bMadeCache)
  {
     imUnProj.resize(kSrc.aLevels[0].im.size());
     ImageRef ir;
     do imUnProj[ir] = mCamera->UnProject(ir);
     while(ir.next(imUnProj.size()));
     bMadeCache = true;
  }

  // Posicion de la caracteristica en el nivel 0
  int nLevelScale = LevelScale(0);
  Surf &candidate = kSrc.aLevels[0].vSurfOD[nCandidate];

  ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, 0);

  // Calcula el rayo que une a la camara 2 con la caracteristica candidata
  // Lo calcula en la camara 2 y luego se lo lleva a la camara 1 (mundo)
  Vector<3> v3Ray_SC = unproject(mCamera->UnProject(v2RootPos));
  normalize(v3Ray_SC);
  Vector<3> v3LineDirn_TC =
     kTarget.se3CfromW.get_rotation() *
     (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);
  // En este pto sabemos en que direccion vive la candidata pero no sabemos a
  // que distancia de la camara (profundidad)

  // Restrict epipolar search to a relatively narrow depth range
  // to increase reliability
  // Se restringe la busqueda epipolar a un rango de profundidades estrecho
  // Previamente se ha calculado la profundidad media y su desviacion
  // => el rango será:
  //   [max(mdWiggleScale, media-desv) .. max(40*mdWiggleScale media+desv)]
  // con mdWiggleScale = 0.1 por defecto
  double dMean = kSrc.dSceneDepthMean;
  double dSigma = kSrc.dSceneDepthSigma;
  //cerr<<"dMean"<<dMean<<" dSigma "<<dSigma<<endl;
  double dStartDepth = max(mdWiggleScale, dMean -2*dSigma);
  double dEndDepth = min(40 * mdWiggleScale, dMean + 2*dSigma);

  // Calculo de la posicion de la camara 2 con respecto a la camara 1 (mundo)
  // La camara 1 debería ser Rot == I(3x3)  y Tras = 0(3x1)
  Vector<3> v3CamCenter_TC =
     kTarget.se3CfromW *
     kSrc.se3CfromW.inverse().get_translation(); // The camera end

  // Calculo de la 2a coordenada 3D del rango donde puede vivir la
  // caracteristica en el mundo
  Vector<3> v3RayEnd_TC =
     v3CamCenter_TC +
     dEndDepth * v3LineDirn_TC;                  // the far-away end

  // Calculo de la 1a coordenada 3D del rango donde puede vivir la
  // caracteristica en el mundo
  Vector<3> v3RayStart_TC =
     v3CamCenter_TC +
     dStartDepth * v3LineDirn_TC;                // the far-away end

  //debug
  Vector<3> v3RayCen=v3CamCenter_TC +
     dMean * v3LineDirn_TC;


  // Si la profundidad del inicio del rango es mayor que la del final
  // Algo raro ocurre, no es posible que la camara hay girado tanto
  if(v3RayEnd_TC[2] <= v3RayStart_TC[2])
    // it's highly unlikely that we'll manage to get anything out
    // if we're facing backwards wrt the other camera's view-ray
    return false;

  // Si la profundidad del final del rango es negativa => algo raro pasa
  if(v3RayEnd_TC[2] <= 0.0 )
       return false;


  // Establece todo el rango en valores Z positivos
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC +=
       v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);


  // Hasta aqui tenemos definido el rango 3D donde es mas probable que viva la
  // caracteristica

  // Proyecta el rango al plano Z=1 y calcula la linea dentro de ese plano
  Vector<2> v2A = project(v3RayStart_TC);
  Vector<2> v2B = project(v3RayEnd_TC);
  Vector<2> v2AlongProjectedLine = v2A-v2B;
  //debug
  Vector<2> v2Center = project(v3RayCen);


  // Se mira si la linea sobre la imagen es suficientemente larga
  if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
  {
     cerr << "v2AlongProjectedLine too small.\n";
     return false;
  }
  // Se calcula la normal a la linea
  normalize(v2AlongProjectedLine);
  Vector<2> v2Normal;
  v2Normal[0] = v2AlongProjectedLine[1];
  v2Normal[1] = -v2AlongProjectedLine[0];


  // Distancia del inicio del rango a la linea
  double dNormDist = v2A * v2Normal;
  if(fabs(dNormDist) > mCamera->LargestRadiusInImage() )
    return false;

  double dMinLen =
     min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
  double dMaxLen =
     max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;


  // Find current-frame corners which might match this
  // Se crea el patch y la suma de sus pixeles y la suma al cuadrado
  PatchFinder * Finder;
  Finder = new PatchFinder(candidate.size);

  // Create a mapPoint to warp the patch in order to improve the search
  MapPoint *pPatch = new MapPoint;
  Vector<3> v3=v3Ray_SC;
  normalize(v3);
  v3*=dMean;
  pPatch->v3WorldPos = kSrc.se3CfromW.inverse()*v3;
  pPatch->pMMData = new MapMakerData();
  pPatch->pPatchSourceKF = &kSrc;
  pPatch->nSourceLevel = 0;
  pPatch->v3Normal_NC = makeVector( 0,0,-1);
  pPatch->irCenter = irLevelPos;
  pPatch->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));
  pPatch->v3OneRightFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pPatch->v3OneDownFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));

  normalize(pPatch->v3Center_NC);
  normalize(pPatch->v3OneDownFromCenter_NC);
  normalize(pPatch->v3OneRightFromCenter_NC);

  pPatch->RefreshPixelVectors();


  Matrix<2> m2CamDerivs = mCamera->GetProjectionDerivs();
  Finder->MakeTemplateCoarse(*pPatch, kTarget.se3CfromW, m2CamDerivs);
  if(Finder->TemplateBad()){
      cerr<<"bad template\n";
      return false;
  }

  ImageRef ir2;
  Vector<2> vv2Corners;

  int nBest = -1;
  ImageRef irBest;
  int nBestZMSSD = Finder->mnMaxSSD + 1;
  double dMaxDistDiff = mCamera->OnePixelDist() * (4.0 + 1.0 * nLevelScale);

   // Project in the image plane the extreme points of the epipole line (v2A, v2B)
  Vector<2> ImSize =mCamera->GetImageSize();
  Vector<2> v2Apro = mCamera->Project(v2A);
  if( v2Apro[0] > ImSize[0]) v2Apro[0] = ImSize[0];
  if( v2Apro[1] > ImSize[1]) v2Apro[1] = ImSize[1];

  Vector<2> v2Bpro = mCamera->Project(v2B);
  if( v2Bpro[0] > ImSize[0]) v2Bpro[0] = ImSize[0];
  if( v2Bpro[1] > ImSize[1]) v2Bpro[1] = ImSize[1];

  Vector<2> v2Cpro =mCamera->Project(v2Center);
   // perpendicular vector to epipolar line
  Vector<2> v2AB = v2Apro -v2Bpro;
  normalize(v2AB);
  Vector<2> v2norm;
  v2norm[0] = v2AB[1];
  v2norm[1] = -v2AB[0];

  vector< Vector<2> > corners;
  corners.reserve(4);
  corners[0]=v2norm + v2Apro;
  corners[1]=v2norm + v2Bpro;
  corners[2]=-1*v2norm + v2Apro;
  corners[3]=-1*v2norm + v2Bpro;

  CVD::ImageRef sizeIm=kTarget.aLevels[0].im.size();

  double xMin,xMax,yMin,yMax;
  xMin=corners[0][0];
  xMax=corners[0][0];
  yMin=corners[0][1];
  yMax=corners[0][1];
  for (unsigned int i=1; i<4; i++)
  {
     xMin=min(xMin,corners[i][0]);
     yMin=min(yMin,corners[i][1]);
     xMax=max(xMax,corners[i][0]);
     yMax=max(yMax,corners[i][1]);
  }


  for (unsigned int i= (int) yMin; i<(int) yMax;i++) {
      for(unsigned int j= (int) xMin; j<(int) xMax;j++){
          ir2.x = j;
          ir2.y = i;
          if (DistPointLine(v2Apro,v2Bpro,j,i)> 1.0)continue;
          int nZMSSD = Finder->ZMSSDAtPoint(kTarget.aLevels[0].im, ir2);
          if(nZMSSD < nBestZMSSD)
          {
              nBest++;
              nBestZMSSD = nZMSSD;
              irBest= ir2;
          }
      }
  }


  if(nBest == -1) return false;   // Nothing found.
 //  Found a likely candidate along epipolar ray
  Finder->MakeSubPixTemplate();
  Finder->SetSubPixPos(LevelZeroPos(irBest,0));
  bool bSubPixConverges = Finder->IterateSubPixToConvergence(kTarget,10);

  if(!bSubPixConverges)return false;
  // Now triangulate the 3d point...
  Vector<3> v3New;
  v3New = kTarget.se3CfromW.inverse() *
    ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
		   mCamera->UnProject(v2RootPos),
		   mCamera->UnProject(Finder->GetSubPixPos()));//mCamera.UnProject(LevelZeroPos(irBest,0)));

  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3New;
  pNew->pMMData = new MapMakerData();


  // Patch source stuff:
  pNew->pPatchSourceKF = &kSrc;
  pNew->nSourceLevel = 0;
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNew->irCenter = irLevelPos;
  pNew->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));
  pNew->v3OneRightFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));

  normalize(pNew->v3Center_NC);
  normalize(pNew->v3OneDownFromCenter_NC);
  normalize(pNew->v3OneRightFromCenter_NC);
  pNew->bSURF=true;
  pNew->RefreshPixelVectors();

  pNew->indexPointMap = mMap.vpPoints.size();
  pNew->mod = false;

  mMap.vpPoints.push_back(pNew);
  pubPoints.push_back(pNew);  //return to tracker
  mqNewQueue.push(pNew);
  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2RootPos;
  m.nLevel = 0;
  m.bSubPix = true;
  kSrc.mMeasurementsOD[pNew] = candidate;
  kSrc.mMeasurements[pNew] = m; //should it be remove?

  kTarget.bEpipolar=true;

  m.Source = Measurement::SRC_EPIPOLAR;
  m.v2RootPos = Finder->GetSubPixPos();
  kTarget.mMeasurements[pNew] = m; //should it be remove?
  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  pNew->pMMData->sMeasurementKFs.insert(&kTarget);

  delete Finder;
  delete pPatch;

  return true;
}

double MapMaker::DistPointLine(Vector<2> P1, Vector<2> P2, double x, double y){

   // Line between P1 and P2 =>r: Ax + By + C= 0 ; calculate as: (x - P1x)    (y-P1y)
   //                                                            --------- =  ---------
   //                                                           (P2x -P1x)    (P2y -P1y)
   double A = P2[1] - P1[1];
   double B = -(P2[0] - P1[0]);
   double C = -A*P1[0] - B*P1[1];
   // Distance between line an point d((x,y),r)=|Ax+B*y+C| / sqrt(A^2+B^2)
   double num=abs(A*x + B*y + C);
   double den=sqrt(A*A + B*B);
   return num/den;
}

std::vector< std::vector<KeyFrame*> > MapMaker::BestEpipolarKeyFrame(KeyFrame &k)
{
   const double dEnoughParallax=0.09;

   std::vector<double> vX;
   std::vector<double> vY;
   std::vector<MapPoint*> PVP;
   std::vector<Vector<2> > Projected;
   std::vector<int> vIndexDist;
   vIndexDist.clear();
   std::vector<int> vIndexDistKF;
   vIndexDistKF.clear();

   std::vector< std::vector<KeyFrame*> > kTargets;
    //bounding box of the object points
   if(k.aLevels[0].vSurfOD.size()==0) return  kTargets;
   for(unsigned int i=0; i<k.aLevels[0].vSurfOD.size(); i++)
   {
      vX.push_back(k.aLevels[0].vSurfOD[i].irLevelPos.x);
      vY.push_back(k.aLevels[0].vSurfOD[i].irLevelPos.y);
   }

   sort(vX.begin(),vX.end());
   sort(vY.begin(),vY.end());
     //checking what map points are projected inside the bounding box
   for(unsigned int ip=0; ip<mMap.vpPoints.size(); ip++)
   {
      Vector<2> v2Point=project(k.se3CfromW*mMap.vpPoints[ip]->v3WorldPos);
      Vector<2> v2PointProjected=mCamera->Project(v2Point);

      if(v2PointProjected[0]<vX[vX.size()-1] &&
         v2PointProjected[0]>vX[0] &&
         v2PointProjected[1]<vY[vY.size()-1] &&
         v2PointProjected[1]>vY[0])
      {
            PVP.push_back(mMap.vpPoints[ip]);
            Projected.push_back(v2PointProjected);
      }
   }
   if (PVP.size()==0) return kTargets;
   //for every anchor point select the nearest point in the map
   for(unsigned int i=0; i<k.aLevels[0].vSurfOD.size(); i++)
   {
      vector<double> vdDist_aux;
      vdDist_aux.clear();
      for (unsigned int j=0; j<Projected.size(); j++)
      {
         Vector<2> Diff;
         Diff[0]=k.aLevels[0].vSurfOD[i].irLevelPos.x-Projected[j][0];
         Diff[1]=k.aLevels[0].vSurfOD[i].irLevelPos.y-Projected[j][1];
         double dDist=sqrt(Diff*Diff);
         vdDist_aux.push_back(dDist);
       }
       unsigned int index=min_element(vdDist_aux.begin(),vdDist_aux.end())-vdDist_aux.begin();
       vIndexDist.push_back(index);
   }
   for (unsigned int i=0; i<vIndexDist.size(); i++)
   {
      MapPoint* pto = PVP[vIndexDist[i]];
      set<KeyFrame*>::iterator itKF;
      vector<pair<double, KeyFrame*> > vdDist_aux;
      vdDist_aux.clear();
      Vector<3> vsem=k.se3CfromW.get_translation()-pto->v3WorldPos;
      for(itKF = pto->pMMData->sMeasurementKFs.begin(); itKF != pto->pMMData->sMeasurementKFs.end(); itKF++)
      {
          Vector<3> vkf= (*itKF)->se3CfromW.get_translation()-pto->v3WorldPos;
          double vsemDotvkf=vsem*vkf;
          vsemDotvkf=vsemDotvkf/(norm(vsem)*norm(vkf));
          double dDist= acos(vsemDotvkf);
          dDist=abs(dDist-dEnoughParallax);
          if (dDist>0.08 && dDist<=dEnoughParallax) continue;
          vdDist_aux.push_back(make_pair(dDist,*itKF));
       }
       sort(vdDist_aux.begin(),vdDist_aux.end());
       vector<KeyFrame*> aux;
       vector<pair<double, KeyFrame*> >::iterator ipkf;
       for(ipkf=vdDist_aux.begin(); ipkf!=vdDist_aux.end(); ipkf++)
          aux.push_back(ipkf->second);

        kTargets.push_back(aux);
   }
   return kTargets;
}


// Rotates/translates the whole map and all keyframes
void MapMaker::ApplyGlobalTransformationToMap(SE3<> se3NewFromOld)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();

  SO3<> so3Rot = se3NewFromOld.get_rotation();
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    {
      mMap.vpPoints[i]->v3WorldPos =
	se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      mMap.vpPoints[i]->RefreshPixelVectors();
    }
}


void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    mMap.vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;

  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
  {
      (*mMap.vpPoints[i]).v3WorldPos *= dScale;
      (*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
      (*mMap.vpPoints[i]).v3PixelDown_W *= dScale;
      (*mMap.vpPoints[i]).RefreshPixelVectors();
   }
  for(unsigned int i=0; i<mMap.vpObjects.size(); i++){
    //mMap.vpObjects[i]->relativeScale=1/dScale;
    mMap.vpObjects[i]->relativeScale=1.0;
  }

  //mMap.scale=dScale;
  mMap.scale=1.0;
  RefreshSceneDepth(mMap.vpKeyFrames[0]);
  double dist=KeyFrameLinearDist(*mMap.vpKeyFrames[0],*mMap.vpKeyFrames[1]);
  mdWiggleScaleDepthNormalized = 0.3*dist/ mMap.vpKeyFrames[0]->dSceneDepthMean;
}




void MapMaker::ApplyGlobalTransformationScaleToMap(SE3<> se3NewFromOld, double dScale)
{

  SE3<> se3NewFromNew;


     Matrix< 3 > mAux = se3NewFromOld.get_rotation().get_matrix();

     mAux(0,0) *= dScale;
     mAux(0,1) *= dScale;
     mAux(0,2) *= dScale;

     mAux(1,0) *= dScale;
     mAux(1,1) *= dScale;
     mAux(1,2) *= dScale;

     mAux(2,0) *= dScale;
     mAux(2,1) *= dScale;
     mAux(2,2) *= dScale;

     se3NewFromNew.get_rotation() = mAux;

     se3NewFromNew.get_rotation() = se3NewFromOld.get_rotation().get_matrix();


     for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
      mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();
     mMap.vpKeyFrames[i]->se3CfromW.ln() *= dScale;
     }


     SO3<> so3Rot = se3NewFromOld.get_rotation();
     for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
     {
        mMap.vpPoints[i]->v3WorldPos = se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      (*mMap.vpPoints[i]).v3WorldPos *= dScale;
       (*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
       (*mMap.vpPoints[i]).v3PixelDown_W *= dScale;
       mMap.vpPoints[i]->RefreshPixelVectors();
     }
}


void MapMaker::ApplyExportMap(SE3<> se3OldMap, SE3<> se3ActualMap, double dScaleActualMap, double scaleMonocular)
{
 double scaleKinect = dScaleActualMap/mMap.denseScale;

 for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){

    SE3<> kfWorld = mMap.vpKeyFrames[i]->se3CfromW;
      mMap.vpKeyFrames[i]->color = 10;


    SE3<> se3World = se3OldMap;

    SE3<> kfRelative = kfWorld * se3World.inverse();
    kfRelative.get_translation()[0] = scaleMonocular * kfRelative.get_translation()[0];
    kfRelative.get_translation()[1] = scaleMonocular * kfRelative.get_translation()[1];
    kfRelative.get_translation()[2] = scaleMonocular * kfRelative.get_translation()[2];

    SE3<> kfNewRelative = kfRelative;
    mMap.vpKeyFrames[i]->se3CfromW = kfNewRelative * se3ActualMap;


  }


  for(unsigned int i=0; i<mMap.vpPoints.size(); i++){

    Vector<3> pointWorld = mMap.vpPoints[i]->v3WorldPos;

    SE3<> se3World = se3OldMap;

    Vector<3> PointRelative = se3World * pointWorld;

    Vector<3> PointNewRelative = PointRelative;

    PointNewRelative[0] = scaleMonocular * PointNewRelative[0];
    PointNewRelative[1] = scaleMonocular * PointNewRelative[1];
    PointNewRelative[2] = scaleMonocular * PointNewRelative[2];

    mMap.vpPoints[i]->v3WorldPos = se3ActualMap.inverse() * PointNewRelative;

  }

}

void MapMaker::ApplyCivera(SE3<> se3NewFromOld, double dScale)
{

SE3<> se3NewFromNew;

     Matrix< 3 > rRA = se3NewFromOld.get_rotation().get_matrix();
     Vector<3> tRA = se3NewFromOld.get_translation();

     rRA(0,0) *= dScale;
     rRA(0,1) *= dScale;
     rRA(0,2) *= dScale;

     rRA(1,0) *= dScale;
     rRA(1,1) *= dScale;
     rRA(1,2) *= dScale;

     rRA(2,0) *= dScale;
     rRA(2,1) *= dScale;
     rRA(2,2) *= dScale;


     se3NewFromNew.get_rotation() = se3NewFromOld.get_rotation().get_matrix();


     SO3<> so3Rot = se3NewFromOld.get_rotation();
     for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
     {
 
       mMap.vpPoints[i]->v3WorldPos = se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
      (*mMap.vpPoints[i]).v3WorldPos *= dScale;

       (*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
      mMap.vpPoints[i]->RefreshPixelVectors();
     }


     for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
        
      mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();
       mMap.vpKeyFrames[i]->se3CfromW.ln() *= dScale;
     }
}



// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so
// just dumps it on the top of the mapmaker's queue to
// be dealt with later, and return.
void MapMaker::AddKeyFrame(KeyFrame &k)
{
  KeyFrame *pK = new KeyFrame;
  *pK = k;
  pK->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  mvpKeyFrameQueue.push_back(pK);
  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;
}

void MapMaker::AddServiceKeyFrame(sKeyFrame k,int color)
{

  if (!k.bSemantic){

    servicekf.mMeasurements.clear();
    servicekf.rgbdData.clear();
    servicekf.MakeKeyFrame_Lite(k.imFrame);
    servicekf.se3CfromW = k.se3CfromW;
    servicekf.bFixed = k.bFixed;
    servicekf.bSemantic = false;
    servicekf.dSceneDepthMean = k.dSceneDepthMean;
    servicekf.dSceneDepthSigma = k.dSceneDepthSigma;

    for(unsigned int i=0; i<k.sMeasurements.size(); i++)
    {
      servicekf.mMeasurements[mMap.vpPoints[k.sMeasurements[i].indexPointMap]] = k.sMeasurements[i].m;
    }

    for(unsigned int i=0; i<k.rgbdData.size(); i++)
    {
      servicekf.rgbdData.push_back(k.rgbdData[i]);
    }

  } else { 

    servicekf.mMeasurements.clear();
    servicekf.mMeasurementsOD.clear();
    servicekf.rgbdData.clear();
    servicekf.MakeKeyFrame_Lite(k.imFrame);
    servicekf.se3CfromW = k.se3CfromW;
    servicekf.bFixed = true;//k.bFixed;
    servicekf.bSemantic = k.bSemantic;
    servicekf.dSceneDepthMean = k.dSceneDepthMean;
    servicekf.dSceneDepthSigma = k.dSceneDepthSigma;

    for(unsigned int i=0; i<k.rgbdData.size(); i++)
    {
      servicekf.rgbdData.push_back(k.rgbdData[i]);
    }

  }

  KeyFrame *pK = new KeyFrame;
  *pK = servicekf;
  pK->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  pK->color = color;

  if (k.bSemantic){ 
    vector<Surf>::const_iterator it;
    for(int itC = 0; itC < k.sCandidates.size(); itC++){
      pK->aLevels[itC].vSurfOD.clear();
      for(it = k.sCandidates[itC].vCandidatesOD.begin(); it != k.sCandidates[itC].vCandidatesOD.end(); ++it){
        Surf cand;
        cand.irLevelPos[0] = (*it).irLevelPos[0];
        cand.irLevelPos[1] = (*it).irLevelPos[1];
        cand.size = (*it).size;
        cand.index = (*it).index;
        pK->aLevels[itC].vSurfOD.push_back(cand);
      }
    }

    lastObject = new MapObject;

    lastObject->Name = k.name;
    lastObject->Depth = k.depth;
    lastObject->Tco.get_translation() = k.Tco.get_translation();
    lastObject->Tco.get_rotation() = k.Tco.get_rotation();

    for(int itV = 0; itV < k.vp3DPoints.size(); itV++){
      Surf vSurf;
      Vector<3> v3d;

      vSurf.irLevelPos[0] = k.vp2DPoints[itV][0];
      vSurf.irLevelPos[1] = k.vp2DPoints[itV][1];
      vSurf.index = itV;
      v3d[0] = k.vp3DPoints[itV][0];
      v3d[1] = k.vp3DPoints[itV][1];
      v3d[2] = k.vp3DPoints[itV][2];

      lastObject->vp2DPoints.push_back(vSurf);
      lastObject->vp3DPoints.push_back(v3d);
    }
  }
  if(pK->bSemantic){


    cerr << pK->se3CfromW;
    cerr << servicekf.se3CfromW;
  }

  mvpKeyFrameQueue.push_back(pK);

  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;

}


// Mapmaker's code to handle incoming key-frames.
void MapMaker::AddKeyFrameFromTopOfQueue()
{
  if(mvpKeyFrameQueue.size() == 0)
    return;

  KeyFrame *pK = mvpKeyFrameQueue[0];
  mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
  pK->MakeKeyFrame_Rest();
  pK->index = mMap.vpKeyFrames.size();

  if (mMap.vpKeyFrames.size() == 1) //@roborgbd
    pK->bFixed = true;
  mMap.vpKeyFrames.push_back(pK);

  //Para la posicion de los keyframes en el tracking
  KeyFrame * newK = new KeyFrame();

  // Any measurements? Update the relevant point's measurement counter status map
  for(meas_it it = pK->mMeasurements.begin();
      it!=pK->mMeasurements.end();
      it++)
    {
      if(!(it->first->bSURF)) {
        it->first->pMMData->sMeasurementKFs.insert(pK);
        it->second.Source = Measurement::SRC_TRACKER;
      }
    }

  // And maybe we missed some - this now adds to the map itself, too.
  ReFindInSingleKeyFrame(*pK);
  if(pK->bSemantic){
      RefreshSceneDepth(pK);
      pK->color = 12; //pK->color = 10;
  }

  if(pK->bSemantic){
    unlockSemantic = true;
    cerr << pK->se3CfromW;
  }

  AddSomeMapPoints(0);       // .. and add more map points by epipolar search.

  AddSomeMapPoints(1);

  AddSomeMapPoints(2);

  AddSomeMapPoints(3);

  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;


  if (mMap.vpKeyFrames.size() == 2){ //@roborgbd
    mbBundleConverged_Full = true;
    mbBundleConverged_Recent = true;
  }


}


// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(KeyFrame &kSrc,
				KeyFrame &kTarget,
				int nLevel,
				int nCandidate,bool bSemantic)
{

  static Image<Vector<2> > imUnProj;
  static bool bMadeCache = false;

  if(!bMadeCache)
  {
     imUnProj.resize(kSrc.aLevels[0].im.size());
     ImageRef ir;
     do imUnProj[ir] = mCamera->UnProject(ir);
     while(ir.next(imUnProj.size()));
     bMadeCache = true;
  }


  // nLevelScale = 2^nLevel
  // Posicion de la caracteristica en el nivel 0
  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];

  //if(bSemantic)
  //  candidate = kSrc.aLevels[nLevel].vCandidatesOD[nCandidate];

  ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);

  // Calcula el rayo que une a la camara 2 con la caracteristica candidata
  // Lo calcula en la camara 2 y luego se lo lleva a la camara 1 (mundo)
  Vector<3> v3Ray_SC = unproject(mCamera->UnProject(v2RootPos));
  normalize(v3Ray_SC);
  Vector<3> v3LineDirn_TC =
     kTarget.se3CfromW.get_rotation() *
     (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);
  // En este pto sabemos en que direccion vive la candidata pero no sabemos a
  // que distancia de la camara (profundidad)

  // Restrict epipolar search to a relatively narrow depth range
  // to increase reliability
  // Se restringe la busqueda epipolar a un rango de profundidades estrecho
  // Previamente se ha calculado la profundidad media y su desviacion
  // => el rango será:
  //   [max(mdWiggleScale, media-desv) .. max(40*mdWiggleScale media+desv)]
  // con mdWiggleScale = 0.1 por defecto
  double dMean = kSrc.dSceneDepthMean;
  double dSigma = kSrc.dSceneDepthSigma;
  double dStartDepth = max(mdWiggleScale, dMean - 2*dSigma);
  double dEndDepth = min(40 * mdWiggleScale, dMean + 2*dSigma);
  //fprintf(stderr,"%f %f %f %f %f\n",dMean,dSigma,dStartDepth,dEndDepth,mdWiggleScale);

  // Calculo de la posicion de la camara 2 con respecto a la camara 1 (mundo)
  // La camara 1 debería ser Rot == I(3x3)  y Tras = 0(3x1)
  Vector<3> v3CamCenter_TC =
     kTarget.se3CfromW *
     kSrc.se3CfromW.inverse().get_translation(); // The camera end

  // Calculo de la 2a coordenada 3D del rango donde puede vivir la
  // caracteristica en el mundo
  Vector<3> v3RayEnd_TC =
     v3CamCenter_TC +
     dEndDepth * v3LineDirn_TC;                  // the far-away end

  // Calculo de la 1a coordenada 3D del rango donde puede vivir la
  // caracteristica en el mundo
  Vector<3> v3RayStart_TC =
     v3CamCenter_TC +
     dStartDepth * v3LineDirn_TC;                // the far-away end

  //cerr << v3RayStart_TC[0] << " | "
  //     << v3RayStart_TC / v3RayStart_TC[2]
  //     << endl;


  // Si la profundidad del inicio del rango es mayor que la del final
  // Algo raro ocurre, no es posible que la camara hay girado tanto
  if(v3RayEnd_TC[2] <= v3RayStart_TC[2])
    // it's highly unlikely that we'll manage to get anything out
    // if we're facing backwards wrt the other camera's view-ray
    return false;
  //fprintf(stderr,"v3RayEnd_TC[2] <= v3RayStart_TC[2]\n");

  //cerr << v3RayStart_TC[0] << " | "
  //     << v3RayStart_TC / v3RayStart_TC[2]
  //     << endl;

  // Si la profundidad del final del rango es negativa => algo raro pasa
  if(v3RayEnd_TC[2] <= 0.0 )  return false;
  //fprintf(stderr,"v3RayEnd_TC[2] <= 0.0 \n");
  /*
  cerr << "C " << nCandidate << ": "
     << v3RayStart_TC << " | ";
  */

  // Establece todo el rango en valores Z positivos
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC +=
       v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);


  // Hasta aqui tenemos definido el rango 3D donde es mas probable que viva la
  // caracteristica


  //cerr << "C " << nCandidate << ": "
  //     << v3RayStart_TC[0] << " | "
  //     << v3RayEnd_TC << "  "
  //     << v3RayStart_TC / v3RayStart_TC[2]
  //     << endl;

  // Proyecta el rango al plano Z=1 y calcula la linea dentro de ese plano
  Vector<2> v2A = project(v3RayStart_TC);
  Vector<2> v2B = project(v3RayEnd_TC);
  Vector<2> v2AlongProjectedLine = v2A-v2B;

  /*
  cerr << "C " << nCandidate << ": "  << v2AlongProjectedLine << " | "
       << v3RayStart_TC << " | "
       << v2A << " | "
       << v3RayEnd_TC << " | "
       << v2B << endl;
  */


  // Se mira si la linea sobre la imagen es suficientemente larga
  if(v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001)
  {
     cout << "v2AlongProjectedLine too small." << endl;
     return false;
  }
  //fprintf(stderr,"v2AlongProjectedLine * v2AlongProjectedLine < 0.00000001\n");

  // Se calcula la normal a la linea
  normalize(v2AlongProjectedLine);
  Vector<2> v2Normal;
  v2Normal[0] = v2AlongProjectedLine[1];
  v2Normal[1] = -v2AlongProjectedLine[0];


  // Distancia del inicio del rango a la linea
  double dNormDist = v2A * v2Normal;
  if(fabs(dNormDist) > mCamera->LargestRadiusInImage() )
    return false;
  //fprintf(stderr,"fabs(dNormDist) > mCamera->LargestRadiusInImage()\n");


  double dMinLen =
     min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
  double dMaxLen =
     max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;


  // Find current-frame corners which might match this
  // Se crea el patch y la suma de sus pixeles y la suma al cuadrado
  PatchFinder Finder;
  Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
  if(Finder.TemplateBad())  return false;
  //fprintf(stderr,"Finder.TemplateBad()\n");


  // Representa las coordenadas en 3D sobre el plano Z=1
  vector<Vector<2> > &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
  vector<ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
  if(!kTarget.aLevels[nLevel].bImplaneCornersCached)
  {
     for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
	     vv2Corners.push_back(imUnProj[ir(LevelZeroPos(vIR[i], nLevel))]);
     kTarget.aLevels[nLevel].bImplaneCornersCached = true;
  }

  int nBest = -1;
  int nBestZMSSD = Finder.mnMaxSSD + 1;
  double dMaxDistDiff = mCamera->OnePixelDist() * (4.0 + 1.0 * nLevelScale);
  double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;

  for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
  {
     Vector<2> v2Im = vv2Corners[i];
     double dDistDiff = dNormDist - v2Im * v2Normal;
     if(dDistDiff * dDistDiff > dMaxDistSq)	continue; // skip if not along epi line
     if(v2Im * v2AlongProjectedLine < dMinLen)	continue; // skip if not far enough along line
     if(v2Im * v2AlongProjectedLine > dMaxLen)	continue; // or too far
     int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
     if(nZMSSD < nBestZMSSD)
	  {
	     nBest = i;
	     nBestZMSSD = nZMSSD;
	  }
  }

  if(nBest == -1)   return false;   // Nothing found.
  //fprintf(stderr,"nBest == -1\n");

  //  Found a likely candidate along epipolar ray
  Finder.MakeSubPixTemplate();
  Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget,10);


  if(!bSubPixConverges) return false;
  //fprintf(stderr,"!bSubPixConverges\n");

  // Now triangulate the 3d point...
  Vector<3> v3New;
  v3New = kTarget.se3CfromW.inverse() *
    ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
		   mCamera->UnProject(v2RootPos),
		   mCamera->UnProject(Finder.GetSubPixPos()));

  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3New;
  pNew->pMMData = new MapMakerData();

  // Patch source stuff:
  pNew->pPatchSourceKF = &kSrc;
  pNew->nSourceLevel = nLevel;
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNew->irCenter = irLevelPos;
  pNew->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));
  pNew->v3OneRightFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));

  normalize(pNew->v3Center_NC);
  normalize(pNew->v3OneDownFromCenter_NC);
  normalize(pNew->v3OneRightFromCenter_NC);

  pNew->RefreshPixelVectors();

  pNew->indexPointMap = mMap.vpPoints.size();
  pNew->mod = false;

  mMap.vpPoints.push_back(pNew);
  pubPoints.push_back(pNew); 

  mqNewQueue.push(pNew);
  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2RootPos;
  m.nLevel = nLevel;
  m.bSubPix = true;
  kSrc.mMeasurements[pNew] = m;

  m.Source = Measurement::SRC_EPIPOLAR;
  m.v2RootPos = Finder.GetSubPixPos();
  kTarget.mMeasurements[pNew] = m;
  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  pNew->pMMData->sMeasurementKFs.insert(&kTarget);
  return true;

}


void MapMaker::addRGBDPoints(KeyFrame &kSrc,int nLevel,	int nCandidate,const Vector<3> &v3pose){


  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];

  ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);

  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3pose; 
  pNew->pMMData = new MapMakerData();

  // Patch source stuff:
  pNew->pPatchSourceKF = &kSrc;
  pNew->nSourceLevel = nLevel;
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNew->irCenter = irLevelPos;
  pNew->v3Center_NC = unproject(mCamera->UnProject(v2RootPos));
  pNew->v3OneRightFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromCenter_NC =
     unproject(mCamera->UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));

  normalize(pNew->v3Center_NC);
  normalize(pNew->v3OneDownFromCenter_NC);
  normalize(pNew->v3OneRightFromCenter_NC);

  pNew->RefreshPixelVectors();

  pNew->indexPointMap = mMap.vpPoints.size();
  pNew->mod = false;
  pNew->v3WorldPosKinect = pNew->v3WorldPos; // Criterio para el optimizador

  pNew->distanceKinect = fabs(kSrc.se3CfromW.inverse().get_translation()[2] - pNew->v3WorldPos[2]);

  mMap.vpPoints.push_back(pNew);
  pubPoints.push_back(pNew);  //return to tracker

  mqNewQueue.push(pNew);
  if (!kSrc.bSemantic){
    Measurement m;
    m.Source = Measurement::SRC_ROOT;
    m.v2RootPos = v2RootPos;
    m.nLevel = nLevel;
    m.bSubPix = true;
    kSrc.mMeasurements[pNew] = m;
  }
  else{
    kSrc.mMeasurementsOD[pNew] = kSrc.aLevels[0].vSurfOD[nCandidate];
  }
  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  
}


double MapMaker::KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2)
{
  Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dDist = sqrt(v3Diff * v3Diff);
  return dDist;
}

double MapMaker::KeyFrameAngularDist(KeyFrame &k1, KeyFrame &k2)
{
  Vector<3> v3KF2_CamPos = k1.se3CfromW*k2.se3CfromW.get_rotation().ln();
  double aDist = sin((sqrt(v3KF2_CamPos * v3KF2_CamPos))/2);
  return aDist;
}


vector<KeyFrame*> MapMaker::NClosestKeyFramesM(KeyFrame &k, unsigned int N)
{
  vector<pair<double, KeyFrame* > > vKFandScores;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    {
      if(mMap.vpKeyFrames[i] == &k)
	continue;
      double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
      vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFrames[i]));
    }
  if(N > vKFandScores.size())
    N = vKFandScores.size();
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

  vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  return vResult;
}

KeyFrame* MapMaker::ClosestKeyFrame(KeyFrame &k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
  {
     if(mMap.vpKeyFrames[i] == &k) continue;

     double dDist = KeyFrameLinearDist(k, *mMap.vpKeyFrames[i]);
     if(dDist < dClosestDist)
     {
        dClosestDist = dDist;
        nClosest = i;
     }
   }
   assert(nClosest != -1);
   return mMap.vpKeyFrames[nClosest];
}

double MapMaker::DistToNearestKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  return dDist;
}



bool MapMaker::NeedNewKeyFrame(KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
  double aDist = KeyFrameAngularDist(kCurrent, *pClosest);
  dDist *= (1.0 / kCurrent.dSceneDepthMean);

  if(dDist > GV2.GetDouble("MapMaker.MaxKFDistWiggleMult",1.0,SILENT) * mdWiggleScaleDepthNormalized)
    return true;
  return false;
}


// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll(bool check)
{

  // construct the sets of kfs/points to be adjusted:
  // in this case, all of them
  set<KeyFrame*> sAdj;
  set<KeyFrame*> sFixed;

  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
    if(mMap.vpKeyFrames[i]->bFixed)
      sFixed.insert(mMap.vpKeyFrames[i]);
    else
      sAdj.insert(mMap.vpKeyFrames[i]);

  set<MapPoint*> sMapPoints;
  for(unsigned int i=0; i<mMap.vpPoints.size();i++)
    sMapPoints.insert(mMap.vpPoints[i]);

  set<MapObject*> sObjects;
  //@optimizar for(unsigned int i=0; i<mMap.vpObjects.size();i++)
  //  sObjects.insert(mMap.vpObjects[i]);

  BundleAdjustM(sAdj, sFixed, sMapPoints, sObjects, false, check);

}


// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecentM()
{
  if(mMap.vpKeyFrames.size() < 2)  //RGBD
    { // Ignore this unless map is big enough
      return;
    }


  if(mMap.vpKeyFrames.size() < 8 )
    { // Ignore this unless map is big enough
      mbBundleConverged_Recent = true;
      return;
    }

  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = mMap.vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);
  vector<KeyFrame*> vClosest = NClosestKeyFramesM(*pkfNewest, 4);
  for(int i=0; i<4; i++)
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);

  // Now we find the set of features which they contain.
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter!=sAdjustSet.end();
      iter++)
    {
      map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
      for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); jiter++)
	sMapPoints.insert(jiter->first);
    };

  // Finally, add all keyframes which measure above points as fixed keyframes
  set<MapObject*> sObjects;
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = mMap.vpKeyFrames.begin(); it!=mMap.vpKeyFrames.end(); it++)
    {
      if(sAdjustSet.count(*it))
	continue;
      bool bInclude = false;
      for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); jiter++)
	if(sMapPoints.count(jiter->first))
	  {
	    bInclude = true;
	    break;
	  }
      if(bInclude)
	sFixedSet.insert(*it);
    }

  BundleAdjustM(sAdjustSet, sFixedSet, sMapPoints, sObjects, true,true);
}

double MapMaker::computeDistError(double dist)
{

 double sig;

 double tga = (0.03-0.001)/(3-0.3);

 sig = ((dist-0.3)*tga)+0.001;

 if (dist > 3.0)
   sig = 0.03;

 if (dist < 0.30)
   sig = 0.001;


 return(sig*sig);


}



bool MapMaker::parallaxCameras(MapPoint* p)
{
  std::set<KeyFrame *>::iterator it;
  std::set<KeyFrame *>::iterator it2;
  double constCos = cos(5*M_PI/180);
  double maxParallax = 1.0;

  for(it=p->pMMData->sMeasurementKFs.begin(); it!=p->pMMData->sMeasurementKFs.end(); it++){
    Vector<3> vdPtoCam1 = (*it)->se3CfromW.inverse().get_translation() - p->v3WorldPos;
    normalize(vdPtoCam1);
    it2 = it;
    for(++it2; it2!=p->pMMData->sMeasurementKFs.end(); it2++){
      Vector<3> vdPtoCam2 = (*it2)->se3CfromW.inverse().get_translation() - p->v3WorldPos;
      normalize(vdPtoCam2);
      double parallax = vdPtoCam1 * vdPtoCam2;
      if (parallax <= maxParallax)
        maxParallax = parallax;
    }
  }

  return (maxParallax<=constCos);
}


void MapMaker::BundleAdjustM(set<KeyFrame*> sAdjustSet, set<KeyFrame*> sFixedSet, set<MapPoint*> sMapPoints, set<MapObject*> sObjects, bool bRecent, bool check)
{

  int numFree = 0;
  int numFixed = 0;

   mbBundleRunning = true;
   mbBundleRunningIsRecent = bRecent;
   //should be define out of the if clause
   g2o::VertexScale * v_scale; //ObjD

   bool DENSE=false;
   g2o::SparseOptimizer optimizer;
   optimizer.setVerbose(false);
   //g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
   g2o::BlockSolverX::LinearSolverType * linearSolver;
   if (DENSE) {
    linearSolver= new g2o::LinearSolverDense<g2o
         ::BlockSolverX::PoseMatrixType>();
        //::BlockSolver_6_3::PoseMatrixType>();
   } else {
    linearSolver
        = new g2o::LinearSolverCholmod<g2o
        ::BlockSolverX::PoseMatrixType>();
        //::BlockSolver_6_3::PoseMatrixType>();
   }


   //g2o::BlockSolver_6_3 * solver_ptr
   //   = new g2o::BlockSolver_6_3(linearSolver);
   g2o::BlockSolverX * solver_ptr
      = new g2o::BlockSolverX(linearSolver);
   g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
   optimizer.setAlgorithm(solver);


  // The bundle adjuster does different accounting of keyframes and map points;
  // Translation maps are stored:
   map<MapPoint*, int> mPoint_G2OID;
   map<int, MapPoint*> mG2OID_Point;
   map<KeyFrame*, int> mView_G2OID;
   map<int, KeyFrame*> mG2OID_View;
   map<MapObject*, int> mObject_G2OID; //ObjD
   map<int, MapObject*> mG2OID_Object; //ObjD
   set<g2o::VertexSE3Expmap *> mFixedViews;


  // set the camera calibration as a parameter for the optimization.
   Vector<5> calibration = mCamera->GetCameraParameters();
   Eigen::Vector2d focal_length(calibration[0],calibration[1]);
   Eigen::Vector2d principal_point(calibration[2], calibration[3]);
   double distortion=calibration[4];

   g2o::CameraParameters * cam_params
      = new g2o::CameraParameters (focal_length, principal_point, distortion);
   cam_params->setId(0);


   if (!optimizer.addParameter(cam_params)) assert(false);

   // INITIALIZATION OF VERTEX

   int vertex_id = 0;
 
   //Vertex: Fixed set of Cameras
   for(set<KeyFrame*>::iterator
      it  = sFixedSet.begin();
      it != sFixedSet.end();
      it++)
   {

     Matrix3d R = Matrix3d::Identity();
     R(0,0) = (*it)->se3CfromW.get_rotation().get_matrix()[0][0];
     R(0,1) = (*it)->se3CfromW.get_rotation().get_matrix()[0][1];
     R(0,2) = (*it)->se3CfromW.get_rotation().get_matrix()[0][2];

     R(1,0) = (*it)->se3CfromW.get_rotation().get_matrix()[1][0];
     R(1,1) = (*it)->se3CfromW.get_rotation().get_matrix()[1][1];
     R(1,2) = (*it)->se3CfromW.get_rotation().get_matrix()[1][2];

     R(2,0) = (*it)->se3CfromW.get_rotation().get_matrix()[2][0];
     R(2,1) = (*it)->se3CfromW.get_rotation().get_matrix()[2][1];
     R(2,2) = (*it)->se3CfromW.get_rotation().get_matrix()[2][2];

     Eigen::Vector3d trans((*it)->se3CfromW.get_translation()[0],(*it)->se3CfromW.get_translation()[1],(*it)->se3CfromW.get_translation()[2]);

     g2o::SE3Quat pose(R,trans);
     g2o::VertexSE3Expmap * v_se3
        = new g2o::VertexSE3Expmap();
     v_se3->setId(vertex_id);
     v_se3->setFixed(true);
     v_se3->setEstimate(pose);
     optimizer.addVertex(v_se3);

     mView_G2OID[*it] = vertex_id;
     mG2OID_View[vertex_id] = *it;
     mFixedViews.insert(v_se3);

     vertex_id++;

   }


   //Vertex: Adjust set of Cameras
   for(set<KeyFrame*>::iterator
      it  = sAdjustSet.begin();
      it != sAdjustSet.end();
      it++)
   {


     Matrix3d R = Matrix3d::Identity();
     R(0,0) = (*it)->se3CfromW.get_rotation().get_matrix()[0][0];
     R(0,1) = (*it)->se3CfromW.get_rotation().get_matrix()[0][1];
     R(0,2) = (*it)->se3CfromW.get_rotation().get_matrix()[0][2];

     R(1,0) = (*it)->se3CfromW.get_rotation().get_matrix()[1][0];
     R(1,1) = (*it)->se3CfromW.get_rotation().get_matrix()[1][1];
     R(1,2) = (*it)->se3CfromW.get_rotation().get_matrix()[1][2];

     R(2,0) = (*it)->se3CfromW.get_rotation().get_matrix()[2][0];
     R(2,1) = (*it)->se3CfromW.get_rotation().get_matrix()[2][1];
     R(2,2) = (*it)->se3CfromW.get_rotation().get_matrix()[2][2];

     Eigen::Vector3d trans((*it)->se3CfromW.get_translation()[0],(*it)->se3CfromW.get_translation()[1],(*it)->se3CfromW.get_translation()[2]);

     g2o::SE3Quat pose(R,trans);
     g2o::VertexSE3Expmap * v_se3
        = new g2o::VertexSE3Expmap();

     v_se3->setId(vertex_id);
     v_se3->setFixed(false);
     v_se3->setEstimate(pose);
     optimizer.addVertex(v_se3);

     mView_G2OID[*it] = vertex_id;
     mG2OID_View[vertex_id] = *it;

     vertex_id++;

   }


   //Vertex 3D Points
   for(set<MapPoint*>::iterator
      it  = sMapPoints.begin();
      it != sMapPoints.end();
      it++)
   {
     Eigen::Vector3d pos((*it)->v3WorldPos[0], (*it)->v3WorldPos[1], (*it)->v3WorldPos[2]);
     g2o::VertexSBAPointXYZ * v_p
        = new g2o::VertexSBAPointXYZ();
     v_p->setId(vertex_id);
     v_p->setMarginalized(true);
     v_p->setFixed(false); //@roborgbd
     v_p->setEstimate(pos);

     optimizer.addVertex(v_p);

     mPoint_G2OID[*it] = vertex_id;
     mG2OID_Point[vertex_id] = *it;

     vertex_id++;


     g2o::EdgeKinectDistance * e
              = new g2o::EdgeKinectDistance();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                       (v_p));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                       (v_p));

        Eigen::Vector3d z((*it)->v3WorldPosKinect[0], (*it)->v3WorldPosKinect[1], (*it)->v3WorldPosKinect[2]);
        e->setMeasurement(z);

//        double noise;
//        noise = LevelScale(it->second.nLevel); // noise = 2^nLevel
       double errorSigma = computeDistError((*it)->distanceKinect);

        e->information() = Eigen::Matrix3d::Identity()/(errorSigma*errorSigma);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(7.8147); //noise
//        e->setParameterId(0, 0);

        optimizer.addEdge(e);

   }

   //INITIALIZATION OF EDGES

  // Edges: Measurements and camera-3d point relations
   double meas = 0;
   for(unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
   {
     KeyFrame* kFrame = mMap.vpKeyFrames[i];

     map<KeyFrame*, int>::iterator itrCam;

     itrCam = mView_G2OID.find(kFrame);
	 if(itrCam == mView_G2OID.end()) continue;

     for(meas_it
         it  = mMap.vpKeyFrames[i]->mMeasurements.begin();
	      it != mMap.vpKeyFrames[i]->mMeasurements.end();
	      it++)
	  {
	     MapPoint* point = (*it).first;
	     map<MapPoint*, int>::iterator itrPoint;

	     itrPoint = mPoint_G2OID.find(point);
	     if(itrPoint == mPoint_G2OID.end()) continue;

        g2o::EdgeProjectXYZ2UVdist * e
              = new g2o::EdgeProjectXYZ2UVdist();

        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                       (optimizer.vertices().find(itrPoint->second)->second));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                       (optimizer.vertices().find(itrCam->second)->second));

        Eigen::Vector2d z(it->second.v2RootPos[0],it->second.v2RootPos[1]);
        e->setMeasurement(z);

        double noise;
        noise = LevelScale(it->second.nLevel); // noise = 2^nLevel
        e->information() = Eigen::Matrix2d::Identity()/(noise*noise);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(5.991);
        e->setParameterId(0, 0);
        optimizer.addEdge(e);
        meas +=2;

	  }
   }

   optimizer.initializeOptimization();
   bool bAccepted = false;
   optimizer.setVerbose(false);


   optimizer.setForceStopFlag(&mbBundleAbortRequested);


     if (optimizer.optimize(10)) bAccepted = true;

   // DATA RECOVERING
   // Keyframe poses
   for(map<KeyFrame*,int>::iterator
         itr  = mView_G2OID.begin();
	      itr != mView_G2OID.end();
	      itr++)
   {
      g2o::VertexSE3Expmap *v_cam =
      dynamic_cast< g2o::VertexSE3Expmap * >
          (optimizer.vertices()[itr->second]);

      Eigen::Matrix4d Tcw = v_cam->estimate().to_homogeneous_matrix();
      TooN::Matrix<3,3> TcwT;

      TcwT[0][0] = Tcw(0,0);
      TcwT[0][1] = Tcw(0,1);
      TcwT[0][2] = Tcw(0,2);
      itr->first->se3CfromW.get_translation()[0] = Tcw(0,3);

      TcwT[1][0] = Tcw(1,0);
      TcwT[1][1] = Tcw(1,1);
      TcwT[1][2] = Tcw(1,2);
      itr->first->se3CfromW.get_translation()[1] = Tcw(1,3);

      TcwT[2][0] = Tcw(2,0);
      TcwT[2][1] = Tcw(2,1);
      TcwT[2][2] = Tcw(2,2);
      itr->first->se3CfromW.get_translation()[2] = Tcw(2,3);

      itr->first->se3CfromW.get_rotation() = TcwT;
   }

   // 3d point locations OUTPUT 
   for(map<MapPoint*,int>::iterator
         itr  = mPoint_G2OID.begin();
	      itr != mPoint_G2OID.end();
	      itr++)
   {

     g2o::VertexSBAPointXYZ *v_p =
       dynamic_cast< g2o::VertexSBAPointXYZ * >
          (optimizer.vertices()[itr->second]);

          itr->first->v3WorldPos[0] = v_p->estimate()[0];
           itr->first->v3WorldPos[1] = v_p->estimate()[1];
           itr->first->v3WorldPos[2] = v_p->estimate()[2];
           itr->first->mod = true;

   }

   if(bRecent)   mbBundleConverged_Recent = false;
   mbBundleConverged_Full = false;

   if(bAccepted)
   {
      mbBundleConverged_Recent = true;
      if(!bRecent)   mbBundleConverged_Full = true;
   }

   if(unlockSemantic){
       unlockSemantic = false;
       checkSemanticPoints();
     }

   mbBundleRunning = false;
   mbBundleAbortRequested = false;

}





void MapMaker::checkSemanticPoints(){

     lastObject->relativeScale = 1.0; //scale[scale.size()/2];
    mMap.scale=1.0;

     SE3<> TcwScaled(mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->se3CfromW);
     lastObject->Tow = lastObject->Tco.inverse() * TcwScaled;

      mvpObjectQueue.push_back(lastObject);
     pubSemanticInfo(true);      
}



double MapMaker::CalculateMedianScale(Vector<3> & v1, Vector<3> &v2)
{
    Matrix<3> M;
    M[0][0]=v1[0]; M[0][1]=0.0;   M[0][2]=0.0;
    M[1][0]=0.0;   M[1][1]=v1[1]; M[1][2]=0.0;
    M[2][0]=0.0;   M[2][1]=0.0;   M[2][2]=v1[2];

    LU<3> luM(M);
    Vector<3> x = luM.backsub(v2);
    std::vector<double> scale;
    for (int i =0; i<3; i++)
       scale.push_back(x[i]);
    sort(scale.begin(), scale.end());
    return scale[1];

}


// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in
// TrackerData.h.
bool MapMaker::ReFind_Common(KeyFrame &k, MapPoint &p)
{
  // abort if either a measurement is already in the map, or we've
  // decided that this point-kf combo is beyond redemption

  if(!p.bSURF){
    if(p.pMMData->sMeasurementKFs.count(&k)
       || p.pMMData->sNeverRetryKFs.count(&k))
      return false;

    static PatchFinder Finder;
    Vector<3> v3Cam = k.se3CfromW*p.v3WorldPos;
    if(v3Cam[2] < 0.001)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
    Vector<2> v2ImPlane = project(v3Cam);
    if(v2ImPlane* v2ImPlane > mCamera->LargestRadiusInImage() * mCamera->LargestRadiusInImage())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

    Vector<2> v2Image = mCamera->Project(v2ImPlane);
    if(mCamera->Invalid())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

    ImageRef irImageSize = k.aLevels[0].im.size();
    if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

    Matrix<2> m2CamDerivs = mCamera->GetProjectionDerivs();
    Finder.MakeTemplateCoarse(p, k.se3CfromW, m2CamDerivs);

    if(Finder.TemplateBad())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
    bool bFound;

    bFound = Finder.FindPatchCoarse(ir(v2Image), k, 4);  // Very tight search radius!

    if(!bFound)
    {

      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

  // If we found something, generate a measurement struct and put it in the map
    Measurement m;
    m.nLevel = Finder.GetLevel();
    m.Source = Measurement::SRC_REFIND;

    if(Finder.GetLevel() > 0)
    {
      Finder.MakeSubPixTemplate();
      Finder.IterateSubPixToConvergence(k,8);
      m.v2RootPos = Finder.GetSubPixPos();
      m.bSubPix = true;
    }
    else
    {
      m.v2RootPos = Finder.GetCoarsePosAsVector();
      m.bSubPix = false;
    };

    if(k.mMeasurements.count(&p))
    {
      assert(0); // This should never happen, we checked for this at the start.
    }
    k.mMeasurements[&p] = m;
    p.pMMData->sMeasurementKFs.insert(&k);
    return true;


   }else{ // If the point corresponds to a anchor point


    if(p.pMMData->sMeasurementKFs.count(&k)
     || p.pMMData->sNeverRetryKFs.count(&k))
        return false;
    Vector<3> v3Cam = k.se3CfromW*p.v3WorldPos;
    if(v3Cam[2] < 0.001)
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
    Vector<2> v2ImPlane = project(v3Cam);
    if(v2ImPlane* v2ImPlane > mCamera->LargestRadiusInImage() * mCamera->LargestRadiusInImage())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

    Vector<2> v2Image = mCamera->Project(v2ImPlane);
    if(mCamera->Invalid())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }

    ImageRef irImageSize = k.aLevels[0].im.size();
    if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      return false;
    }
    bool bFound;

    Matrix<2> m2CamDerivs = mCamera->GetProjectionDerivs();
    PatchFinder *Finder;
    Surf sCandidate = p.pPatchSourceKF->mMeasurementsOD[&p];
    Finder = new PatchFinder(sCandidate.size);
    //cerr<<"tamaño del patch "<<Finder->GetPatchSize()<<endl;

    Finder->MakeTemplateCoarse(p, k.se3CfromW, m2CamDerivs);

    if(Finder->TemplateBad())
    {
      p.pMMData->sNeverRetryKFs.insert(&k);
      delete Finder;
      return false;
    }

    bFound = Finder->FindPatchCoarseTemplateMatching(ir(v2Image), k, 4);
   //cerr<<"bFound=" <<bFound<<endl;

    if(!bFound)
    {

      p.pMMData->sNeverRetryKFs.insert(&k);
      delete Finder;
      return false;
    }

   // If we found something, generate a measurement struct and put it in the map
    Measurement m;
    m.nLevel = Finder->GetLevel();
    m.Source = Measurement::SRC_REFIND;

    if(Finder->GetLevel() > 0)
    {
      Finder->MakeSubPixTemplate();
      Finder->IterateSubPixToConvergence(k,8);
      m.v2RootPos = Finder->GetSubPixPos();
      m.bSubPix = true;
    }
    else
    {
      m.v2RootPos = Finder->GetCoarsePosAsVector();
      m.bSubPix = false;
    };

    if(k.mMeasurements.count(&p))
    {
      assert(0); // This should never happen, we checked for this at the start.
    }
    k.mMeasurements[&p] = m;
    p.pMMData->sMeasurementKFs.insert(&k);
    delete Finder;
    return true;
  }
}



// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(KeyFrame &k)
{
  vector<MapPoint*> vToFind;
  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
    vToFind.push_back(mMap.vpPoints[i]);

  int nFoundNow = 0;
  for(unsigned int i=0; i<vToFind.size(); i++)
    if(ReFind_Common(k,*vToFind[i]))
      nFoundNow++;

  return nFoundNow;
};

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade()
{
  //ROS_INFO("ReFindNewlyMade()");
  if(mqNewQueue.empty())
    return;
  int nFound = 0;
  int nBad = 0;
  while(!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0)
    {
      MapPoint* pNew = mqNewQueue.front();
      mqNewQueue.pop();
      if(pNew->bBad)
	{
	  nBad++;
	  continue;
	}
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	if(ReFind_Common(*mMap.vpKeyFrames[i], *pNew))
	  nFound++;
    }
};

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue()
{
  //ROS_INFO("ReFindFromFailureQueue()");
  if(mvFailureQueue.size() == 0)
    return;
  sort(mvFailureQueue.begin(), mvFailureQueue.end());
  vector<pair<KeyFrame*, MapPoint*> >::iterator it;
  int nFound=0;
  for(it = mvFailureQueue.begin(); it!=mvFailureQueue.end(); it++)
    if(ReFind_Common(*it->first, *it->second))
      nFound++;

  mvFailureQueue.erase(mvFailureQueue.begin(), it);
};

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent)
{
  return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> MapMaker::CalcPlaneAligner()
{
  unsigned int nPoints = mMap.vpPoints.size();
  if(nPoints < 10)
    {
      cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
      return SE3<>();
    };

  int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 100, HIDDEN|SILENT);
  Vector<3> v3BestMean;
  Vector<3> v3BestNormal;
  double dBestDistSquared = 9999999999999999.9;

  for(int i=0; i<nRansacs; i++)
    {
      int nA = rand()%nPoints;
      int nB = nA;
      int nC = nA;
      while(nB == nA)
	nB = rand()%nPoints;
      while(nC == nA || nC==nB)
	nC = rand()%nPoints;

      Vector<3> v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos +
				       mMap.vpPoints[nB]->v3WorldPos +
				       mMap.vpPoints[nC]->v3WorldPos);

      Vector<3> v3CA = mMap.vpPoints[nC]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3BA = mMap.vpPoints[nB]->v3WorldPos  - mMap.vpPoints[nA]->v3WorldPos;
      Vector<3> v3Normal = v3CA ^ v3BA;
      if(v3Normal * v3Normal  == 0)
	continue;
      normalize(v3Normal);

      double dSumError = 0.0;
      for(unsigned int i=0; i<nPoints; i++)
	{
	  Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
	  double dDistSq = v3Diff * v3Diff;
	  if(dDistSq == 0.0)
	    continue;
	  double dNormDist = fabs(v3Diff * v3Normal);

	  if(dNormDist > 0.05)
	    dNormDist = 0.05;
	  dSumError += dNormDist;
	}
      if(dSumError < dBestDistSquared)
	{
	  dBestDistSquared = dSumError;
	  v3BestMean = v3Mean;
	  v3BestNormal = v3Normal;
	}
    }

  // Done the ransacs, now collect the supposed inlier set
  vector<Vector<3> > vv3Inliers;
  for(unsigned int i=0; i<nPoints; i++)
    {
      Vector<3> v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
      double dDistSq = v3Diff * v3Diff;
      if(dDistSq == 0.0)
	continue;
      double dNormDist = fabs(v3Diff * v3BestNormal);
      if(dNormDist < 0.05)
	vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
    }

  // With these inliers, calculate mean and cov
  Vector<3> v3MeanOfInliers = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    v3MeanOfInliers+=vv3Inliers[i];
  v3MeanOfInliers *= (1.0 / vv3Inliers.size());

  Matrix<3> m3Cov = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    {
      Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
      m3Cov += v3Diff.as_col() * v3Diff.as_row();
    };

  // Find the principal component with the minimal variance: this is the plane normal
  SymEigen<3> sym(m3Cov);
  Vector<3> v3Normal = sym.get_evectors()[0];

  // Use the version of the normal which points towards the cam center
  if(v3Normal[2] > 0)
    v3Normal *= -1.0;

  Matrix<3> m3Rot = Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];

  SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;

  return se3Aligner;
}


// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void MapMaker::RefreshSceneDepth(KeyFrame *pKF)
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  //vector<double> depths;
  for(meas_it it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
      MapPoint &point = *it->first;
      Vector<3> v3PosK = pKF->se3CfromW * point.v3WorldPos;
      dSumDepth += v3PosK[2];
      dSumDepthSquared += v3PosK[2] * v3PosK[2];
      nMeas++;
    //  depths.push_back(v3PosK[2]);
    }
  //sort(depths.begin(), depths.end());
  assert(nMeas > 2); // If not then something is seriously wrong with this KF!!
  //assert(depths.size() > 2);
  pKF->dSceneDepthMean = dSumDepth / nMeas;//depths[depths.size()/2];//trying to use the median instead of mean(dSumDepth / nMeas;)
  pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}


void MapMaker::AddObjecttoMap()
{
    if(mvpObjectQueue.size() == 0)
    return;
    for(int i=mvpObjectQueue.size()-1; i>=0;i--)
    {
        MapObject *pO=mvpObjectQueue[i];
        mvpObjectQueue.erase(mvpObjectQueue.end()-1);
        mMap.vpObjects.push_back(pO);
    }
}


void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="SaveMap")
    {
      cout << "  MapMaker: Saving the map.... " << endl;
      ofstream ofs("map.dump");
      for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
	{
	  ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
	  ofs << mMap.vpPoints[i]->nSourceLevel << endl;
	}
      ofs.close();

      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++)
	{
	  ostringstream ost1;
	  ost1 << "keyframes/" << i << ".jpg";

	  ostringstream ost2;
	  ost2 << "keyframes/" << i << ".info";
	  ofstream ofs2;
	  ofs2.open(ost2.str().c_str());
	  ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
	  ofs2.close();
	}
      cout << "  ... done saving map." << endl;
      return;
    }

  cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
};


void MapMaker::printInfo()
{
}

// 
double MapMaker::InitServFromRGBD
(
 sKeyFrame skF,  SE3<> &se3)
{
int num;
  //ROS_INFO("InitServFromRGBD");
  pthread_mutex_lock(&ekfMutex);
  pthread_mutex_lock(&pointsMutex);

  pointsBool = true;
  pthread_mutex_unlock(&pointsMutex);

  pubPoints.clear();


  mdWiggleScale = *mgvdWiggleScale;

  //REBUILD THE sKeyFrame on a KeyFrame
  servicekf_ekf.mMeasurements.clear();
  servicekf_ekf.MakeKeyFrame_Lite(skF.imFrame);
  mCamera->SetImageSize(servicekf_ekf.aLevels[0].im.size());
  servicekf_ekf.se3CfromW = skF.se3CfromW;
  servicekf_ekf.bFixed = skF.bFixed;
  servicekf_ekf.bSemantic = skF.bSemantic;
  servicekf_ekf.dSceneDepthMean = skF.dSceneDepthMean;
  servicekf_ekf.dSceneDepthSigma = skF.dSceneDepthSigma;

  for(unsigned int i=0; i<skF.sMeasurements.size(); i++)
  {
    servicekf_ekf.mMeasurements[mMap.vpPoints[skF.sMeasurements[i].indexPointMap]] = skF.sMeasurements[i].m;
  }

  for(unsigned int i=0; i < skF.rgbdData.size(); i++)
  {
    servicekf_ekf.rgbdData.push_back(skF.rgbdData[i]);
  }

  // Use EKF estimates to populate new map:
  KeyFrame *pkFirst = new KeyFrame();


  *pkFirst = servicekf_ekf;


  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();

  pkFirst->MakeKeyFrame_Rest();

  pkFirst->index = mMap.vpKeyFrames.size();
  mMap.vpKeyFrames.push_back(pkFirst);

  for(int nLevel=0; nLevel<LEVELS; nLevel++){
    num = 0;
    Level &l = pkFirst->aLevels[nLevel];
    for(unsigned int i = 0; i<l.vCandidates.size(); i++){

      Vector<2> v2RootPos = LevelZeroPos(l.vCandidates[i].irLevelPos, nLevel);

      Vector<3> xyzPose;

      xyzPose[0] = servicekf_ekf.rgbdData[int(640*rint(v2RootPos[1])+ rint(v2RootPos[0]))].x;
      xyzPose[1] = servicekf_ekf.rgbdData[int(640*rint(v2RootPos[1])+ rint(v2RootPos[0]))].y;
      xyzPose[2] = servicekf_ekf.rgbdData[int(640*rint(v2RootPos[1])+ rint(v2RootPos[0]))].z;

      if (!std::isnan(xyzPose[0]) && !std::isnan(xyzPose[1]) && !std::isnan(xyzPose[2])){

        if (num == 0){
          num++;
        }
        addRGBDPoints(*pkFirst, nLevel, i, xyzPose);
      }
    }
  }


  mbBundleConverged_Full = true; //false;
  mbBundleConverged_Recent = true; //false;

  mMap.bGood = true;

  se3 = pkFirst->se3CfromW;
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;

  RefreshSceneDepth(pkFirst);
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;

  //pubEKFPoints();
  pubRGBPoints();

  pthread_mutex_lock(&pointsMutex);
  pointsBool = false;
  pthread_mutex_unlock(&pointsMutex);

  pthread_mutex_unlock(&ekfMutex);

  return mdWiggleScaleDepthNormalized;
}


// Oscar Garcia: 9/Mar/2011
double MapMaker::InitServFromStereo_EKF
(
 sKeyFrame skF,
  sKeyFrame skS,
  SE3<> &se3
)
{
  //ROS_INFO("InitServFromStereo_EKF");
  pthread_mutex_lock(&ekfMutex);
  pthread_mutex_lock(&pointsMutex);

  pointsBool = true;
  pthread_mutex_unlock(&pointsMutex);

  pubPoints.clear();


  mdWiggleScale = *mgvdWiggleScale;

  //REBUILD THE sKeyFrame on a KeyFrame
  servicekf_ekf.mMeasurements.clear();
  servicekf_ekf.MakeKeyFrame_Lite(skF.imFrame);
  mCamera->SetImageSize(servicekf_ekf.aLevels[0].im.size());
  servicekf_ekf.se3CfromW = skF.se3CfromW;
  servicekf_ekf.bFixed = skF.bFixed;
  servicekf_ekf.bSemantic = skF.bSemantic;
  servicekf_ekf.dSceneDepthMean = skF.dSceneDepthMean;
  servicekf_ekf.dSceneDepthSigma = skF.dSceneDepthSigma;

  for(unsigned int i=0; i<skF.sMeasurements.size(); i++)
  {
    servicekf_ekf.mMeasurements[mMap.vpPoints[skF.sMeasurements[i].indexPointMap]] = skF.sMeasurements[i].m;
  }

  //REBUILD THE sKeyFrame on a KeyFrame
  serviceks_ekf.mMeasurements.clear();
  serviceks_ekf.MakeKeyFrame_Lite(skS.imFrame);
  serviceks_ekf.se3CfromW = skS.se3CfromW;
  serviceks_ekf.bFixed = skS.bFixed;
  serviceks_ekf.bSemantic = skS.bSemantic;
  serviceks_ekf.dSceneDepthMean = skS.dSceneDepthMean;
  serviceks_ekf.dSceneDepthSigma = skS.dSceneDepthSigma;

  for(unsigned int i=0; i<skS.sMeasurements.size(); i++)
  {
    serviceks_ekf.mMeasurements[mMap.vpPoints[skS.sMeasurements[i].indexPointMap]] = skS.sMeasurements[i].m;
  }

  for(unsigned int i=0; i < skS.rgbdData.size(); i++)
  {
    serviceks_ekf.rgbdData.push_back(skS.rgbdData[i]);
  }


  // Use EKF estimates to populate new map:
  KeyFrame *pkFirst = new KeyFrame();
  KeyFrame *pkSecond = new KeyFrame();

  *pkFirst = servicekf_ekf;
  *pkSecond = serviceks_ekf;

  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();

  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
  if(dTransMagn == 0)
   {
     cout << "  Estimated zero baseline from stereo pair, try again." << endl;
     return false;
     pthread_mutex_unlock(&ekfMutex);
   }
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;

  pkSecond->bFixed = false;
  pkSecond->se3CfromW = se3;


  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();

  pkFirst->index = mMap.vpKeyFrames.size();
  mMap.vpKeyFrames.push_back(pkFirst);

  pkSecond->index = mMap.vpKeyFrames.size();
  mMap.vpKeyFrames.push_back(pkSecond);

  AddSomeMapPoints(0);
  BundleAdjustAll(true);
  AddSomeMapPoints(0);
  BundleAdjustAll(true);
  id = "nivel0";
  AddSomeMapPoints(3);
  BundleAdjustAll(true);
  id = "nivel3";
  AddSomeMapPoints(1);
  BundleAdjustAll(true);
  id = "nivel1";
  AddSomeMapPoints(2);

  id = "nivel2";

  mbBundleConverged_Full = false;
  mbBundleConverged_Recent = false;

  while(!mbBundleConverged_Full)
  {
     BundleAdjustAll(true);
     if(mbResetRequested){
       pubEKFPoints();

  pthread_mutex_lock(&pointsMutex);
  pointsBool = false;
  pthread_mutex_unlock(&pointsMutex);
       return false;
     pthread_mutex_unlock(&ekfMutex);
     }
  }
  id = "ba2";

  mMap.bGood = true;

  se3 = pkSecond->se3CfromW;
  cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << endl;

  RefreshSceneDepth(pkFirst);
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;

       //ROS_INFO("mdWiggleScaleDepthNormalized %f\n",mdWiggleScaleDepthNormalized);
   pubEKFPoints();

  pthread_mutex_lock(&pointsMutex);
  pointsBool = false;
  pthread_mutex_unlock(&pointsMutex);

  pthread_mutex_unlock(&ekfMutex);

  return mdWiggleScaleDepthNormalized;
}



bool MapMaker::AttemptServiceRecovery(CVD::Image<CVD::byte> &imFrame, bool &mbJustRecoveredSoUseCoarse_r,Vector<6> &mv6CameraVelocity_r, SE3<> &mse3StartPos_r, SE3<> &mse3CamFromWorld_r, int *indexKF)
{

  aux_KF.mMeasurements.clear();
  aux_KF.MakeKeyFrame_Lite(imFrame);


  bool bRelocGood = mRelocaliser->AttemptRecovery(aux_KF);
  if(!bRelocGood)
    return false;

  SE3<> se3Best = mRelocaliser->BestPose();
  cerr << se3Best << endl;
  *indexKF = mRelocaliser->BestIndexKF();
  mse3CamFromWorld_r = mse3StartPos_r = se3Best;
  mv6CameraVelocity_r = Zeros;
  mbJustRecoveredSoUseCoarse_r = true;

  mse3CamFromWorld = mse3CamFromWorld_r;
  return true;

}

bool MapMaker::pubRGBPoints()
{
  c2tam_msgs::DataPoints dp;
  c2tam_msgs::MapInfo mi;


  int cntModPoints = 0;
  indexModPoints.clear();

//  NEW KEYFRAMES

  dp.id = mapId;
  dp.queueSize = mvpKeyFrameQueue.size();
  dp.initialLoad = 0;

  for(int i=0 ;i < 1; i++){
    c2tam_msgs::PoseKeyFrame kf;
    Matrix< 3 > mAux;

    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

    mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

    kf.pose.push_back(mAux(0,0));
    kf.pose.push_back(mAux(0,1));
    kf.pose.push_back(mAux(0,2));
    kf.pose.push_back(mAux(1,0));
    kf.pose.push_back(mAux(1,1));
    kf.pose.push_back(mAux(1,2));
    kf.pose.push_back(mAux(2,0));
    kf.pose.push_back(mAux(2,1));
    kf.pose.push_back(mAux(2,2));

    mi.KFse3CfromW.push_back(mAux(0,0));
    mi.KFse3CfromW.push_back(mAux(0,1));
    mi.KFse3CfromW.push_back(mAux(0,2));
    mi.KFse3CfromW.push_back(mAux(1,0));
    mi.KFse3CfromW.push_back(mAux(1,1));
    mi.KFse3CfromW.push_back(mAux(1,2));
    mi.KFse3CfromW.push_back(mAux(2,0));
    mi.KFse3CfromW.push_back(mAux(2,1));
    mi.KFse3CfromW.push_back(mAux(2,2));

    kf.color = mMap.vpKeyFrames[i]->color;
    dp.newKeyframes.push_back(kf);
  }

  //  NEW POINTS
  for(unsigned int i=0; i<pubPoints.size(); i++){

    c2tam_msgs::Point p;

    p.bBad = pubPoints[i]->bBad;
    p.nSourceLevel = pubPoints[i]->nSourceLevel;
    p.nMEstimatorOutlierCount = pubPoints[i]->nMEstimatorOutlierCount;
    p.nMEstimatorInlierCount = pubPoints[i]->nMEstimatorInlierCount;
    p.indexPointMap = pubPoints[i]->indexPointMap;
    p.v3.push_back(pubPoints[i]->v3WorldPos[0]);
    p.v3.push_back(pubPoints[i]->v3WorldPos[1]);
    p.v3.push_back(pubPoints[i]->v3WorldPos[2]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[0]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[1]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[2]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[0]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[1]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[2]);
    p.irCenter.push_back(pubPoints[i]->irCenter.x);
    p.irCenter.push_back(pubPoints[i]->irCenter.y);

    mi.Pointv3WorldPos.push_back(pubPoints[i]->v3WorldPos[0]);
    mi.Pointv3WorldPos.push_back(pubPoints[i]->v3WorldPos[1]);
    mi.Pointv3WorldPos.push_back(pubPoints[i]->v3WorldPos[2]);
    mi.nSourceLevel.push_back(pubPoints[i]->nSourceLevel);

    dp.newPoints.push_back(p);
  }
  pubPoints.clear(); 


  KeyFrame &lastKf = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The last keyframe
  vector<KeyFrame*> vClosest = NClosestKeyFramesM(lastKf, numKeyFrames);
  vClosest.push_back(&lastKf);

  for(vector<MapPoint*>::iterator itr = mMap.vpPoints.begin(); itr!=mMap.vpPoints.end(); itr++){
    if ((*itr)->mod == true){
      cntModPoints++;

      if (numKeyFrames > 0){
        for(vector<KeyFrame*>::iterator itrKf = vClosest.begin(); itrKf!=vClosest.end(); itrKf++){
          if((*itr)->pPatchSourceKF->index == (*itrKf)->index){
            indexModPoints.push_back((*itr)->indexPointMap); 
            (*itr)->mod = false;
          }
        }
      }
      else if (numKeyFrames == -1){
          indexModPoints.push_back((*itr)->indexPointMap); 
          (*itr)->mod = false;
      }
    }
  }

// MOD POINTS

  if (indexModPoints.size() > 0){

    for(unsigned int i=0; i<indexModPoints.size(); i++)
    {
      c2tam_msgs::Point p;

      p.bBad = mMap.vpPoints[indexModPoints[i]]->bBad;
      p.nSourceLevel = mMap.vpPoints[indexModPoints[i]]->nSourceLevel;
      p.nMEstimatorOutlierCount = mMap.vpPoints[indexModPoints[i]]->nMEstimatorOutlierCount;
      p.nMEstimatorInlierCount = mMap.vpPoints[indexModPoints[i]]->nMEstimatorInlierCount;
      p.indexPointMap = mMap.vpPoints[indexModPoints[i]]->indexPointMap;
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3WorldPos[0]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3WorldPos[1]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3WorldPos[2]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelDown_W[0]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelDown_W[1]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelDown_W[2]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelRight_W[0]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelRight_W[1]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelRight_W[2]);

      dp.modPoints.push_back(p);
    }
    indexModPoints.clear();
  }

// MOD KF

    if (numKeyFrames > 0){
      for(unsigned int i=0; i< vClosest.size(); i++){
        c2tam_msgs::PoseKeyFrame kf;
 
        Matrix< 3 > mAux;

        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[0]);
        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[1]);
        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[2]);

        mAux = mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_rotation().get_matrix();

        kf.pose.push_back(mAux(0,0));
        kf.pose.push_back(mAux(0,1));
        kf.pose.push_back(mAux(0,2));
        kf.pose.push_back(mAux(1,0));
        kf.pose.push_back(mAux(1,1));
        kf.pose.push_back(mAux(1,2));
        kf.pose.push_back(mAux(2,0));
        kf.pose.push_back(mAux(2,1));
        kf.pose.push_back(mAux(2,2));
        kf.color = mMap.vpKeyFrames[vClosest[i]->index]->color;
        dp.modKeyframes.push_back(kf);
      }
    }
    else if (numKeyFrames == -1){
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
        c2tam_msgs::PoseKeyFrame kf;
 
        Matrix< 3 > mAux;

        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

        mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

        kf.pose.push_back(mAux(0,0));
        kf.pose.push_back(mAux(0,1));
        kf.pose.push_back(mAux(0,2));
        kf.pose.push_back(mAux(1,0));
        kf.pose.push_back(mAux(1,1));
        kf.pose.push_back(mAux(1,2));
        kf.pose.push_back(mAux(2,0));
        kf.pose.push_back(mAux(2,1));
        kf.pose.push_back(mAux(2,2));
        kf.color = mMap.vpKeyFrames[i]->color;
        dp.modKeyframes.push_back(kf);
      }
    }

  dataPoints_pub->publish(dp);

  mapInfo_pub->publish(mi);

}


bool MapMaker::pubEKFPoints()
{
  c2tam_msgs::DataPoints dp;
  c2tam_msgs::MapInfo mi;


  int cntModPoints = 0;
  indexModPoints.clear();

//  NEW KEYFRAMES

  dp.id = mapId;
  dp.queueSize = mvpKeyFrameQueue.size();
  dp.initialLoad = 0;

  for(int i=0 ;i < 2; i++){
    c2tam_msgs::PoseKeyFrame kf;
    Matrix< 3 > mAux;

    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

    mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

    kf.pose.push_back(mAux(0,0));
    kf.pose.push_back(mAux(0,1));
    kf.pose.push_back(mAux(0,2));
    kf.pose.push_back(mAux(1,0));
    kf.pose.push_back(mAux(1,1));
    kf.pose.push_back(mAux(1,2));
    kf.pose.push_back(mAux(2,0));
    kf.pose.push_back(mAux(2,1));
    kf.pose.push_back(mAux(2,2));

    mi.KFse3CfromW.push_back(mAux(0,0));
    mi.KFse3CfromW.push_back(mAux(0,1));
    mi.KFse3CfromW.push_back(mAux(0,2));
    mi.KFse3CfromW.push_back(mAux(1,0));
    mi.KFse3CfromW.push_back(mAux(1,1));
    mi.KFse3CfromW.push_back(mAux(1,2));
    mi.KFse3CfromW.push_back(mAux(2,0));
    mi.KFse3CfromW.push_back(mAux(2,1));
    mi.KFse3CfromW.push_back(mAux(2,2));

    kf.color = mMap.vpKeyFrames[i]->color;
    dp.newKeyframes.push_back(kf);
  }

  //  NEW POINTS
  for(unsigned int i=0; i<pubPoints.size(); i++){

    c2tam_msgs::Point p;

    p.bBad = pubPoints[i]->bBad;
    p.nSourceLevel = pubPoints[i]->nSourceLevel;
    p.nMEstimatorOutlierCount = pubPoints[i]->nMEstimatorOutlierCount;
    p.nMEstimatorInlierCount = pubPoints[i]->nMEstimatorInlierCount;
    p.indexPointMap = pubPoints[i]->indexPointMap;
    p.v3.push_back(pubPoints[i]->v3WorldPos[0]);
    p.v3.push_back(pubPoints[i]->v3WorldPos[1]);
    p.v3.push_back(pubPoints[i]->v3WorldPos[2]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[0]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[1]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[2]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[0]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[1]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[2]);
    p.irCenter.push_back(pubPoints[i]->irCenter.x);
    p.irCenter.push_back(pubPoints[i]->irCenter.y);

    mi.Pointv3WorldPos.push_back(pubPoints[i]->v3WorldPos[0]);
    mi.Pointv3WorldPos.push_back(pubPoints[i]->v3WorldPos[1]);
    mi.Pointv3WorldPos.push_back(pubPoints[i]->v3WorldPos[2]);
    mi.nSourceLevel.push_back(pubPoints[i]->nSourceLevel);

    dp.newPoints.push_back(p);
  }
  pubPoints.clear(); 


  KeyFrame &lastKf = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The last keyframe
  vector<KeyFrame*> vClosest = NClosestKeyFramesM(lastKf, numKeyFrames);
  vClosest.push_back(&lastKf);

  for(vector<MapPoint*>::iterator itr = mMap.vpPoints.begin(); itr!=mMap.vpPoints.end(); itr++){
    if ((*itr)->mod == true){
      cntModPoints++;

      if (numKeyFrames > 0){
        for(vector<KeyFrame*>::iterator itrKf = vClosest.begin(); itrKf!=vClosest.end(); itrKf++){
          if((*itr)->pPatchSourceKF->index == (*itrKf)->index){
            indexModPoints.push_back((*itr)->indexPointMap); 
            (*itr)->mod = false;
          }
        }
      }
      else if (numKeyFrames == -1){
          indexModPoints.push_back((*itr)->indexPointMap); 
          (*itr)->mod = false;
      }
    }
  }

 
// MOD POINTS

  if (indexModPoints.size() > 0){

    for(unsigned int i=0; i<indexModPoints.size(); i++)
    {
      c2tam_msgs::Point p;

      p.bBad = mMap.vpPoints[i]->bBad;
      p.nSourceLevel = mMap.vpPoints[i]->nSourceLevel;
      p.nMEstimatorOutlierCount = mMap.vpPoints[i]->nMEstimatorOutlierCount;
      p.nMEstimatorInlierCount = mMap.vpPoints[i]->nMEstimatorInlierCount;
      p.indexPointMap = mMap.vpPoints[i]->indexPointMap;
      p.v3.push_back(mMap.vpPoints[i]->v3WorldPos[0]);
      p.v3.push_back(mMap.vpPoints[i]->v3WorldPos[1]);
      p.v3.push_back(mMap.vpPoints[i]->v3WorldPos[2]);
      p.v3.push_back(mMap.vpPoints[i]->v3PixelDown_W[0]);
      p.v3.push_back(mMap.vpPoints[i]->v3PixelDown_W[1]);
      p.v3.push_back(mMap.vpPoints[i]->v3PixelDown_W[2]);
      p.v3.push_back(mMap.vpPoints[i]->v3PixelRight_W[0]);
      p.v3.push_back(mMap.vpPoints[i]->v3PixelRight_W[1]);
      p.v3.push_back(mMap.vpPoints[i]->v3PixelRight_W[2]);

      dp.modPoints.push_back(p);
    }
    indexModPoints.clear();
  }

// MOD KF

    if (numKeyFrames > 0){
      for(unsigned int i=0; i< vClosest.size(); i++){
        c2tam_msgs::PoseKeyFrame kf;
 
        Matrix< 3 > mAux;

        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[0]);
        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[1]);
        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[2]);

        mAux = mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_rotation().get_matrix();

        kf.pose.push_back(mAux(0,0));
        kf.pose.push_back(mAux(0,1));
        kf.pose.push_back(mAux(0,2));
        kf.pose.push_back(mAux(1,0));
        kf.pose.push_back(mAux(1,1));
        kf.pose.push_back(mAux(1,2));
        kf.pose.push_back(mAux(2,0));
        kf.pose.push_back(mAux(2,1));
        kf.pose.push_back(mAux(2,2));
        kf.color = mMap.vpKeyFrames[vClosest[i]->index]->color;
        dp.modKeyframes.push_back(kf);
      }
    }
    else if (numKeyFrames == -1){
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
        c2tam_msgs::PoseKeyFrame kf;
 
        Matrix< 3 > mAux;

        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

        mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

        kf.pose.push_back(mAux(0,0));
        kf.pose.push_back(mAux(0,1));
        kf.pose.push_back(mAux(0,2));
        kf.pose.push_back(mAux(1,0));
        kf.pose.push_back(mAux(1,1));
        kf.pose.push_back(mAux(1,2));
        kf.pose.push_back(mAux(2,0));
        kf.pose.push_back(mAux(2,1));
        kf.pose.push_back(mAux(2,2));
        kf.color = mMap.vpKeyFrames[i]->color;
        dp.modKeyframes.push_back(kf);
      }
    }

  dataPoints_pub->publish(dp);
  mapInfo_pub->publish(mi);

}


bool MapMaker::GetMapObjectsCb(c2tam_srvs::GetMapObjects::Request  &req, c2tam_srvs::GetMapObjects::Response &res ){


  for(int i = 0; i < mMap.vpObjects.size(); i++){

    c2tam_msgs::ObjectTransform objT;

    objT.name = mMap.vpObjects[i]->Name;

    Matrix< 3 > mAux;

    objT.data.push_back( mMap.vpObjects[i]->Tow.inverse().get_translation()[0]);
    objT.data.push_back( mMap.vpObjects[i]->Tow.inverse().get_translation()[1]);
    objT.data.push_back( mMap.vpObjects[i]->Tow.inverse().get_translation()[2]);

    mAux = mMap.vpObjects[i]->Tow.inverse().get_rotation().get_matrix();

    objT.data.push_back(mAux(0,0));
    objT.data.push_back(mAux(0,1));
    objT.data.push_back(mAux(0,2));
    objT.data.push_back(mAux(1,0));
    objT.data.push_back(mAux(1,1));
    objT.data.push_back(mAux(1,2));
    objT.data.push_back(mAux(2,0));
    objT.data.push_back(mAux(2,1));
    objT.data.push_back(mAux(2,2));

    objT.data.push_back(mMap.vpObjects[i]->relativeScale);
    objT.data.push_back(mMap.denseScale);

    res.objects.object.push_back(objT);
  }

  return true;
}


bool MapMaker::pubSemanticInfo(bool success)
{

  c2tam_msgs::SemanticInfo sInfo;
  sInfo.success = success;

  semanticInfo_pub->publish(sInfo);

}

double MapMaker::computeScale(){

  Vector<3> v3KinectCam;
  Vector<3> v3WorldPos;
  Vector<3> v3CamPos;
  std::vector<double> scaleVector;
  double kinectDst = 0;
  double monoDst = 0;    

  scaleVector.clear();

  for(int i=0; i< mMap.vpPoints.size(); i++){

   if (mMap.vpPoints[i]->pPatchSourceKF->rgbdData.size() > 0){

    if(mMap.vpPoints[i]->nSourceLevel == 0 && !std::isnan(mMap.vpPoints[i]->v3WorldPos[0]) &&
	  !std::isnan(mMap.vpPoints[i]->v3WorldPos[1]) && !std::isnan(mMap.vpPoints[i]->v3WorldPos[2]) &&
          !std::isnan(mMap.vpPoints[i]->pPatchSourceKF->rgbdData[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].x) &&
          !std::isnan(mMap.vpPoints[i]->pPatchSourceKF->rgbdData[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].y) &&
          !std::isnan(mMap.vpPoints[i]->pPatchSourceKF->rgbdData[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].z)){

      v3WorldPos[0] = mMap.vpPoints[i]->v3WorldPos[0];
      v3WorldPos[1] = mMap.vpPoints[i]->v3WorldPos[1];
      v3WorldPos[2] = mMap.vpPoints[i]->v3WorldPos[2];
      SE3<> se3CfromW = mMap.vpPoints[i]->pPatchSourceKF->se3CfromW;

      v3KinectCam[0] = mMap.vpPoints[i]->pPatchSourceKF->rgbdData[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].x;  
    
      v3KinectCam[1] = mMap.vpPoints[i]->pPatchSourceKF->rgbdData[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].y; 

      v3KinectCam[2] = mMap.vpPoints[i]->pPatchSourceKF->rgbdData[(640 * mMap.vpPoints[i]->irCenter[1]) + mMap.vpPoints[i]->irCenter[0]].z; 
  
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


bool MapMaker::pubDataPoints()
{
  c2tam_msgs::DataPoints dp;
  bool sendData = false;

  bool pubKF = false;

  dp.initialLoad = 0;

  int cntModPoints = 0;
  indexModPoints.clear();


  KeyFrame &lastKf = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The last keyframe
  vector<KeyFrame*> vClosest = NClosestKeyFramesM(lastKf, numKeyFrames);
  vClosest.push_back(&lastKf);

  for(vector<MapPoint*>::iterator itr = mMap.vpPoints.begin(); itr!=mMap.vpPoints.end(); itr++){
    if ((*itr)->mod == true){
      cntModPoints++;

      if (numKeyFrames > 0){
        for(vector<KeyFrame*>::iterator itrKf = vClosest.begin(); itrKf!=vClosest.end(); itrKf++){
         if((*itr)->pPatchSourceKF->index == (*itrKf)->index){
            indexModPoints.push_back((*itr)->indexPointMap); 
            (*itr)->mod = false;
          }
        }
      }
      else if (numKeyFrames == -1){
          indexModPoints.push_back((*itr)->indexPointMap); 
          (*itr)->mod = false;
      }
    }
  }

 // MOD POINTS

  if (indexModPoints.size() > 0){
    //@VROS_INFO("indexModPoints.size() > 0");
    sendData = true;
    dp.id = mapId;

    for(unsigned int i=0; i<indexModPoints.size(); i++)
    {
      c2tam_msgs::Point p;
      p.bBad = mMap.vpPoints[indexModPoints[i]]->bBad;
      p.nSourceLevel = mMap.vpPoints[indexModPoints[i]]->nSourceLevel;
      p.nMEstimatorOutlierCount = mMap.vpPoints[indexModPoints[i]]->nMEstimatorOutlierCount;
      p.nMEstimatorInlierCount = mMap.vpPoints[indexModPoints[i]]->nMEstimatorInlierCount;
      p.indexPointMap = mMap.vpPoints[indexModPoints[i]]->indexPointMap;
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3WorldPos[0]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3WorldPos[1]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3WorldPos[2]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelDown_W[0]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelDown_W[1]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelDown_W[2]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelRight_W[0]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelRight_W[1]);
      p.v3.push_back(mMap.vpPoints[indexModPoints[i]]->v3PixelRight_W[2]);

      dp.modPoints.push_back(p);
    }
    indexModPoints.clear();
  }

// MOD KF

   if (modKeyFrameInfo){
    modKeyFrameInfo = false;

    sendData = true;

    KeyFrame &lastKf = *(mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]); // The last keyframe
    vector<KeyFrame*> vClosest = NClosestKeyFramesM(lastKf, numKeyFrames);
    vClosest.push_back(&lastKf);

    if (numKeyFrames > 0){
      for(unsigned int i=0; i< vClosest.size(); i++){
        c2tam_msgs::PoseKeyFrame kf;
 
        Matrix< 3 > mAux;

        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[0]);
        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[1]);
        kf.pose.push_back( mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_translation()[2]);

        mAux = mMap.vpKeyFrames[vClosest[i]->index]->se3CfromW.get_rotation().get_matrix();

        kf.pose.push_back(mAux(0,0));
        kf.pose.push_back(mAux(0,1));
        kf.pose.push_back(mAux(0,2));
        kf.pose.push_back(mAux(1,0));
        kf.pose.push_back(mAux(1,1));
        kf.pose.push_back(mAux(1,2));
        kf.pose.push_back(mAux(2,0));
        kf.pose.push_back(mAux(2,1));
        kf.pose.push_back(mAux(2,2));
        kf.color = mMap.vpKeyFrames[vClosest[i]->index]->color;
        dp.modKeyframes.push_back(kf);
      }
    }
    else if (numKeyFrames == -1){
      for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
        c2tam_msgs::PoseKeyFrame kf;
 
        Matrix< 3 > mAux;

        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
        kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

        mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

        kf.pose.push_back(mAux(0,0));
        kf.pose.push_back(mAux(0,1));
        kf.pose.push_back(mAux(0,2));
        kf.pose.push_back(mAux(1,0));
        kf.pose.push_back(mAux(1,1));
        kf.pose.push_back(mAux(1,2));
        kf.pose.push_back(mAux(2,0));
        kf.pose.push_back(mAux(2,1));
        kf.pose.push_back(mAux(2,2));
        kf.color = mMap.vpKeyFrames[i]->color;

        dp.modKeyframes.push_back(kf);
      }
    }
  }
// BAD POINTS

  if (indexBadPoints.size() > 0){
    sendData = true;

    pthread_mutex_lock(&pointsMutex);
    if (!pointsBool){
      pthread_mutex_unlock(&pointsMutex);

      for(unsigned int i=0; i<indexBadPoints.size(); i++)
      {
        dp.badPoints.push_back(indexBadPoints[i]);
      }
      indexBadPoints.clear();
    }
    else 
      pthread_mutex_unlock(&pointsMutex);
  }


// NEW INFO

  c2tam_msgs::NewInfo ni;

  if (newKeyFrameInfo){
    newKeyFrameInfo = false;
    sendData = true;
    pubKF = true;

    dp.queueSize = mvpKeyFrameQueue.size();

    c2tam_msgs::PoseKeyFrame kf;
    Matrix< 3 > mAux;

    kf.pose.push_back( mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->se3CfromW.get_translation()[0]);
    kf.pose.push_back( mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->se3CfromW.get_translation()[1]);
    kf.pose.push_back( mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->se3CfromW.get_translation()[2]);

    mAux = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->se3CfromW.get_rotation().get_matrix();

    kf.pose.push_back(mAux(0,0));
    kf.pose.push_back(mAux(0,1));
    kf.pose.push_back(mAux(0,2));
    kf.pose.push_back(mAux(1,0));
    kf.pose.push_back(mAux(1,1));
    kf.pose.push_back(mAux(1,2));
    kf.pose.push_back(mAux(2,0));
    kf.pose.push_back(mAux(2,1));
    kf.pose.push_back(mAux(2,2));
    kf.color = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->color;
    dp.newKeyframes.push_back(kf);

    for(unsigned int i=0; i<pubPoints.size(); i++)
    {
      c2tam_msgs::Point p;

      p.bBad = pubPoints[i]->bBad;
      p.nSourceLevel = pubPoints[i]->nSourceLevel;
      p.nMEstimatorOutlierCount = pubPoints[i]->nMEstimatorOutlierCount;
      p.nMEstimatorInlierCount = pubPoints[i]->nMEstimatorInlierCount;
      p.indexPointMap = pubPoints[i]->indexPointMap;
      p.v3.push_back(pubPoints[i]->v3WorldPos[0]);
      p.v3.push_back(pubPoints[i]->v3WorldPos[1]);
      p.v3.push_back(pubPoints[i]->v3WorldPos[2]);
      p.v3.push_back(pubPoints[i]->v3PixelDown_W[0]);
      p.v3.push_back(pubPoints[i]->v3PixelDown_W[1]);
      p.v3.push_back(pubPoints[i]->v3PixelDown_W[2]);
      p.v3.push_back(pubPoints[i]->v3PixelRight_W[0]);
      p.v3.push_back(pubPoints[i]->v3PixelRight_W[1]);
      p.v3.push_back(pubPoints[i]->v3PixelRight_W[2]);
      p.irCenter.push_back(pubPoints[i]->irCenter.x);
      p.irCenter.push_back(pubPoints[i]->irCenter.y);

      dp.newPoints.push_back(p);
    }
  }


  if(sendData){
    dataPoints_pub->publish(dp);
  }

/* DATA TO VISUALIZER*/
  if(sendData){
    c2tam_msgs::MapInfo mi;

    for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
      Matrix< 3 > mAux;

      mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
      mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
      mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

      mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

      mi.KFse3CfromW.push_back(mAux(0,0));
      mi.KFse3CfromW.push_back(mAux(0,1));
      mi.KFse3CfromW.push_back(mAux(0,2));
      mi.KFse3CfromW.push_back(mAux(1,0));
      mi.KFse3CfromW.push_back(mAux(1,1));
      mi.KFse3CfromW.push_back(mAux(1,2));
      mi.KFse3CfromW.push_back(mAux(2,0));
      mi.KFse3CfromW.push_back(mAux(2,1));
      mi.KFse3CfromW.push_back(mAux(2,2));
    }

    for(unsigned int i=0; i<mMap.vpPoints.size(); i++){
      mi.Pointv3WorldPos.push_back(mMap.vpPoints[i]->v3WorldPos[0]);
      mi.Pointv3WorldPos.push_back(mMap.vpPoints[i]->v3WorldPos[1]);
      mi.Pointv3WorldPos.push_back(mMap.vpPoints[i]->v3WorldPos[2]);
      mi.nSourceLevel.push_back(mMap.vpPoints[i]->nSourceLevel);
    }
    //fprintf(stderr,"mapInfo_pub->publish\n");



    if (pubKF){
      double scaleComputed = computeScale();
      //fprintf(stderr,"scaleComputed %f\n",scaleComputed);
      mi.scaleDense = scaleComputed;

      if (mapping_visualizer){
        //PUBLISH DATA DENSE TO VISUALIZER
        c2tam_msgs::DenseKf dkf;
        dkf.idKf = mMap.vpKeyFrames.size()-1;
        for(unsigned int j=0; j< mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->rgbdData.size(); j++)
        {
          c2tam_msgs::RgbdInfo rgbdData;
          rgbdData.r = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->rgbdData[j].r;
          rgbdData.g = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->rgbdData[j].g;
          rgbdData.b = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->rgbdData[j].b;
          rgbdData.z = mMap.vpKeyFrames[mMap.vpKeyFrames.size()-1]->rgbdData[j].z;
          dkf.denseCloud.push_back(rgbdData);
        }
        mi.denseKF.push_back(dkf);
      }
    }
    mapInfo_pub->publish(mi);
  }
}


bool MapMaker::pubMergePoints(int firstKF)
{
  c2tam_msgs::DataPoints dp;

  dp.initialLoad = 0;

  int cntModPoints = 0;
  indexModPoints.clear();

//  NEW KEYFRAMES

  dp.id = mapId;
  dp.queueSize = mvpKeyFrameQueue.size();

dp.newKeyframes.reserve(50);



  sensor_msgs::Image::Ptr ros_img_ptr;
  Image<CVD::byte> imFrame;

  for(int i=firstKF ;i < mMap.vpKeyFrames.size(); i++){

    c2tam_msgs::PoseKeyFrame kf;
    Matrix< 3 > mAux;

    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

    mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

    kf.pose.push_back(mAux(0,0));
    kf.pose.push_back(mAux(0,1));
    kf.pose.push_back(mAux(0,2));
    kf.pose.push_back(mAux(1,0));
    kf.pose.push_back(mAux(1,1));
    kf.pose.push_back(mAux(1,2));
    kf.pose.push_back(mAux(2,0));
    kf.pose.push_back(mAux(2,1));
    kf.pose.push_back(mAux(2,2));
    kf.color = mMap.vpKeyFrames[i]->color;

    imFrame = mMap.vpKeyFrames[i]->aLevels[0].im;
    Vector<2> dim = vec(imFrame.size());
    IplImage* greyImage     = cvCreateImage(cvSize((int)dim[0],(int)dim[1]) , IPL_DEPTH_8U, 1);
    cvSetImageData(greyImage,  imFrame.data(),(int)dim[0]);
    ros_img_ptr = bridge_.cvToImgMsg(greyImage, "mono8");
    dp.imgkf.push_back(sensor_msgs::Image(*ros_img_ptr));

//DENSE PUB
   kf.rgbd.clear();
   kf.rgbd.reserve(640*480);
      //fprintf(stderr,"IN %d\n",i);
    for(unsigned int j=0; j< mMap.vpKeyFrames[i]->rgbdData.size(); j++)
    {
      c2tam_msgs::RgbdInfo rgbdData;
      rgbdData.r = mMap.vpKeyFrames[i]->rgbdData[j].r;
      rgbdData.g = mMap.vpKeyFrames[i]->rgbdData[j].g;
      rgbdData.b = mMap.vpKeyFrames[i]->rgbdData[j].b;
      rgbdData.z = mMap.vpKeyFrames[i]->rgbdData[j].z;

      kf.rgbd.push_back(rgbdData);
    }

    dp.newKeyframes.push_back(kf);

  }

  dp.loadPoints.clear();
  dp.loadPoints.reserve(5000);

  for(unsigned int i=0; i<pubPoints.size(); i++){
    c2tam_msgs::LoadPoint lp;
    c2tam_msgs::Point p;

    p.bBad = pubPoints[i]->bBad;
    p.nSourceLevel =pubPoints[i]->nSourceLevel;
    p.nMEstimatorOutlierCount = pubPoints[i]->nMEstimatorOutlierCount;
    p.nMEstimatorInlierCount = pubPoints[i]->nMEstimatorInlierCount;
    p.indexPointMap = pubPoints[i]->indexPointMap;
    p.v3.push_back(pubPoints[i]->v3WorldPos[0]);
    p.v3.push_back(pubPoints[i]->v3WorldPos[1]);
    p.v3.push_back(pubPoints[i]->v3WorldPos[2]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[0]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[1]);
    p.v3.push_back(pubPoints[i]->v3PixelDown_W[2]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[0]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[1]);
    p.v3.push_back(pubPoints[i]->v3PixelRight_W[2]);
    p.irCenter.push_back(pubPoints[i]->irCenter.x);
    p.irCenter.push_back(pubPoints[i]->irCenter.y);
    lp.indexKeyFrame = pubPoints[i]->pPatchSourceKF->index;
    lp.p = p;

    dp.loadPoints.push_back(lp);
  }

  dataPoints_pub->publish(dp);
  pubPoints.clear(); 

}






bool MapMaker::pubLoadPoints()
{
  c2tam_msgs::DataPoints dp;

   dp.id = mapId;
  dp.queueSize = mvpKeyFrameQueue.size();
  dp.initialLoad = 1;
  dp.denseScale = mMap.denseScale;

//KF
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
    c2tam_msgs::PoseKeyFrame kf;
    Matrix< 3 > mAux;

    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
    kf.pose.push_back( mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

    mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

    kf.pose.push_back(mAux(0,0));
    kf.pose.push_back(mAux(0,1));
    kf.pose.push_back(mAux(0,2));
    kf.pose.push_back(mAux(1,0));
    kf.pose.push_back(mAux(1,1));
    kf.pose.push_back(mAux(1,2));
    kf.pose.push_back(mAux(2,0));
    kf.pose.push_back(mAux(2,1));
    kf.pose.push_back(mAux(2,2));
    kf.color = mMap.vpKeyFrames[i]->color;

// NO PUBUBLISH DATA DENSE TO TRACKING
    for(unsigned int j=0; j< mMap.vpKeyFrames[i]->rgbdData.size(); j++)
    {
      c2tam_msgs::RgbdInfo rgbdData;
      rgbdData.r = mMap.vpKeyFrames[i]->rgbdData[j].r;
      rgbdData.g = mMap.vpKeyFrames[i]->rgbdData[j].g;
      rgbdData.b = mMap.vpKeyFrames[i]->rgbdData[j].b;
      rgbdData.z = mMap.vpKeyFrames[i]->rgbdData[j].z;

      kf.rgbd.push_back(rgbdData);
    }
//// DCTAM


    dp.newKeyframes.push_back(kf);
  }

//POINTS

  for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
  {
    c2tam_msgs::LoadPoint lp;
    c2tam_msgs::Point p;

    p.bBad = mMap.vpPoints[i]->bBad;
    p.nSourceLevel = mMap.vpPoints[i]->nSourceLevel;
    p.nMEstimatorOutlierCount = mMap.vpPoints[i]->nMEstimatorOutlierCount;
    p.nMEstimatorInlierCount = mMap.vpPoints[i]->nMEstimatorInlierCount;
    p.indexPointMap = mMap.vpPoints[i]->indexPointMap;
    p.v3.push_back(mMap.vpPoints[i]->v3WorldPos[0]);
    p.v3.push_back(mMap.vpPoints[i]->v3WorldPos[1]);
    p.v3.push_back(mMap.vpPoints[i]->v3WorldPos[2]);
    p.v3.push_back(mMap.vpPoints[i]->v3PixelDown_W[0]);
    p.v3.push_back(mMap.vpPoints[i]->v3PixelDown_W[1]);
    p.v3.push_back(mMap.vpPoints[i]->v3PixelDown_W[2]);
    p.v3.push_back(mMap.vpPoints[i]->v3PixelRight_W[0]);
    p.v3.push_back(mMap.vpPoints[i]->v3PixelRight_W[1]);
    p.v3.push_back(mMap.vpPoints[i]->v3PixelRight_W[2]);
    p.irCenter.push_back(mMap.vpPoints[i]->irCenter.x);
    p.irCenter.push_back(mMap.vpPoints[i]->irCenter.y);
    lp.indexKeyFrame = mMap.vpPoints[i]->pPatchSourceKF->index;
    lp.p = p;

    dp.loadPoints.push_back(lp);
  }


  std::string path_maps;
  FILE * find_file = popen("rospack find c2tam_mapping", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;

  path_maps = command_find;
  path_maps = path_maps + "/maps";

//IMAGES

  sensor_msgs::Image::Ptr ros_img_ptr;
  Image<CVD::byte> imFrame;
  for(unsigned int indexKF=0; indexKF< mMap.vpKeyFrames.size(); indexKF++)
  {
     imFrame = mMap.vpKeyFrames[indexKF]->aLevels[0].im;
     Vector<2> dim = vec(imFrame.size());
     IplImage* greyImage     = cvCreateImage(cvSize((int)dim[0],(int)dim[1]) , IPL_DEPTH_8U, 1);
     cvSetImageData(greyImage,  imFrame.data(),(int)dim[0]);
     ros_img_ptr = bridge_.cvToImgMsg(greyImage, "mono8");
     dp.imgkf.push_back(sensor_msgs::Image(*ros_img_ptr));
  }

  dataPoints_pub->publish(dp);

  c2tam_msgs::MapInfo mi;

  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
    Matrix< 3 > mAux;

    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[0]);
    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[1]);
    mi.KFse3CfromW.push_back(mMap.vpKeyFrames[i]->se3CfromW.get_translation()[2]);

    mAux = mMap.vpKeyFrames[i]->se3CfromW.get_rotation().get_matrix();

    mi.KFse3CfromW.push_back(mAux(0,0));
    mi.KFse3CfromW.push_back(mAux(0,1));
    mi.KFse3CfromW.push_back(mAux(0,2));
    mi.KFse3CfromW.push_back(mAux(1,0));
    mi.KFse3CfromW.push_back(mAux(1,1));
    mi.KFse3CfromW.push_back(mAux(1,2));
    mi.KFse3CfromW.push_back(mAux(2,0));
    mi.KFse3CfromW.push_back(mAux(2,1));
    mi.KFse3CfromW.push_back(mAux(2,2));
  }

  for(unsigned int i=0; i<mMap.vpPoints.size(); i++){
    mi.Pointv3WorldPos.push_back(mMap.vpPoints[i]->v3WorldPos[0]);
    mi.Pointv3WorldPos.push_back(mMap.vpPoints[i]->v3WorldPos[1]);
    mi.Pointv3WorldPos.push_back(mMap.vpPoints[i]->v3WorldPos[2]);
    mi.nSourceLevel.push_back(mMap.vpPoints[i]->nSourceLevel);
  }

  //PUBLISH DATA DENSE TO VISUALIZER
  for(unsigned int i=0; i<mMap.vpKeyFrames.size(); i++){
/* 
   c2tam_msgs::DenseKf dkf;
    dkf.idKf = i;
    for(unsigned int j=0; j< mMap.vpKeyFrames[i]->rgbdData.size(); j++)
    {
      c2tam_msgs::RgbdInfo rgbdData;
      rgbdData.r = mMap.vpKeyFrames[i]->rgbdData[j].r;
      rgbdData.g = mMap.vpKeyFrames[i]->rgbdData[j].g;
      rgbdData.b = mMap.vpKeyFrames[i]->rgbdData[j].b;
      rgbdData.z = mMap.vpKeyFrames[i]->rgbdData[j].z;
      dkf.denseCloud.push_back(rgbdData);
    }
    fprintf(stderr,"denseCloud size %d\n",dkf.denseCloud.size());
*/
    double scaleComputed = computeScale();
    //fprintf(stderr,"scaleComputed %f\n",scaleComputed);
    mi.scaleDense = scaleComputed;

//   mi.denseKF.push_back(dkf);
  }

  mapInfo_pub->publish(mi);

}

bool MapMaker::checkIndexModPoints(int index){         

  bool ret = false;

  for(unsigned int i=0; i<indexModPoints.size(); i++)
  {
    if (index == indexModPoints[i]){
      ret = true;
      break;
    }

  }
  return ret;
}


void MapMaker::TrackMap(Image<CVD::byte> &imFrame)
{

  // Some accounting which will be used for tracking quality assessment:
  for(int i=0; i<LEVELS; i++)
      manMeasAttempted[i] = manMeasFound[i] = 0;

  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  vector<TrackerData*> avPVS[LEVELS];
  for(int i=0; i<LEVELS; i++)
      avPVS[i].reserve(500);

  // For all points in the map..

 for(unsigned int i=0; i<mMap.vpPoints.size(); i++)
  {
      MapPoint &p= *(mMap.vpPoints[i]);
      // Ensure that this map point has an associated TrackerData struct.
      if(!p.pTData){
        p.pTData = new TrackerData(&p);
      }
      TrackerData &TData = *p.pTData;
      
      TData.Project(mse3CamFromWorld, *mCamera);
      if(!TData.bInImage){
        continue;
      }
      // Calculate camera projection derivatives of this point.
      TData.GetDerivsUnsafe(*mCamera);

      // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
      TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
      if(TData.nSearchLevel == -1){
        continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
      }
      // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
      TData.bSearched = false;
      TData.bFound = false;
      avPVS[TData.nSearchLevel].push_back(&TData);
  }

  // Next: A large degree of faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for(int i=0; i<LEVELS; i++)
      random_shuffle(avPVS[i].begin(), avPVS[i].end());

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

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool bTryCoarse = true;
  if(*gvnCoarseDisabled ||
     mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
     nCoarseMax == 0)
      bTryCoarse = false;
  if(mbJustRecoveredSoUseCoarse)
  { 
      bTryCoarse = true;
      nCoarseMax *=2;
      nCoarseRange *=2;
      mbJustRecoveredSoUseCoarse = false;
  };

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
                        vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, *mCamera);
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
          avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, *mCamera);
      }
      SearchForPoints(avPVS[l], nFineRange, 8);
      for(unsigned int i=0; i<avPVS[l].size(); i++)
          vIterationSet.push_back(avPVS[l][i]); // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
  };

  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  vNextToSearch.clear();
  for(int l=LEVELS - 2; l>=0; l--)
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      vNextToSearch.push_back(avPVS[l][i]);

  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
  // ourselves to 1000, and choose these randomly.
  static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);
  int nFinePatchesToUse = *gvnMaxPatchesPerFrame - vIterationSet.size();
  if((int) vNextToSearch.size() > nFinePatchesToUse)
  {
      random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
      vNextToSearch.resize(nFinePatchesToUse); // Chop!
  };

  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(mbDidCoarse)
    for(unsigned int i=0; i<vNextToSearch.size(); i++)
        vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, *mCamera);

  // Find fine points in image:
  SearchForPoints(vNextToSearch, nFineRange, 0);
  // And attach them all to the end of the optimisation-set.
  for(unsigned int i=0; i<vNextToSearch.size(); i++)
      vIterationSet.push_back(vNextToSearch[i]);

  // Again, ten gauss-newton pose update iterations.
  Vector<6> v6LastUpdate;
  v6LastUpdate = Zeros;
  for(int iter = 0; iter<10; iter++)
  {
      bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
                                // reprojection at every iteration - it really isn't necessary!
      if(iter == 0 || iter == 4 || iter == 9)
          bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
      else                            // iterations is for M-Estimator convergence rather than
          bNonLinearIteration = false;  // linearisation effects.

      if(iter != 0)   // Either way: first iteration doesn't need projection update.
      {
          if(bNonLinearIteration)
          {
              for(unsigned int i=0; i<vIterationSet.size(); i++)
                  if(vIterationSet[i]->bFound)
                      vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, *mCamera);
          }
          else
          {
              for(unsigned int i=0; i<vIterationSet.size(); i++)
                  if(vIterationSet[i]->bFound)
                      vIterationSet[i]->LinearUpdate(v6LastUpdate);
          };
      }

      if(bNonLinearIteration)
          for(unsigned int i=0; i<vIterationSet.size(); i++)
              if(vIterationSet[i]->bFound)
                  vIterationSet[i]->CalcJacobian();

      // Again, an M-Estimator hack beyond the fifth iteration.
      double dOverrideSigma = 0.0;
      if(iter > 5)
      dOverrideSigma = 16.0;

      // Calculate and update pose; also store update vector for linear iteration updates.
      Vector<6> v6Update =
        CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
      mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      v6LastUpdate = v6Update;
  };



  // Update the current keyframe with info on what was found in the frame.
  // Strictly speaking this is unnecessary to do every frame, it'll only be
  // needed if the KF gets added to MapMaker. Do it anyway.
  // Export pose to current keyframe:
  mCurrentKF.se3CfromW = mse3CamFromWorld;
  // Record successful measurements. Use the KeyFrame-Measurement struct for this.
  mCurrentKF.mMeasurements.clear();
  for(vector<TrackerData*>::iterator it = vIterationSet.begin();
      it!= vIterationSet.end();
      it++)
  {
      if(! (*it)->bFound)
          continue;
      Measurement m;
       m.v2RootPos = (*it)->v2Found;
      m.nLevel = (*it)->nSearchLevel;
      m.bSubPix = (*it)->bDidSubPix;
      mCurrentKF.mMeasurements[& ((*it)->Point)] = m;

   }

  // Finally, find the mean scene depth from tracked features
  {
    double dSum = 0;
    double dSumSq = 0;
    int nNum = 0;
    for(vector<TrackerData*>::iterator it = vIterationSet.begin();
        it!= vIterationSet.end(); it++)
        if((*it)->bFound)
        {
            double z = (*it)->v3Cam[2];
            dSum+= z;
            dSumSq+= z*z;
            nNum++;
        };
    if(nNum > 20)
    {
        mCurrentKF.dSceneDepthMean = dSum/nNum; //depths[depths.size()/2];//trying to use the median instead of mean(dSum/nNum);
        mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
    }
  }

}


// Find points in the image. Uses the PatchFiner struct stored in TrackerData
int MapMaker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
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


Vector<6> MapMaker::CalcPoseUpdate(vector<TrackerData*> vTD, double dOverrideSigma, bool bMarkOutliers)
{
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

  // No valid measurements? Return null update.
  if(vdErrorSquared.size() == 0)
      return makeVector( 0,0,0,0,0,0);

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

  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior
  for(unsigned int f=0; f<vTD.size(); f++)
  {
      TrackerData &TD = *vTD[f];
      if(!TD.bFound)
          continue;
      Vector<2> &v2 = TD.v2Error_CovScaled;
      double dErrorSq = v2 * v2;
      double dWeight;

      if(nEstimator == 0)
          dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
      else if(nEstimator == 1)
          dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
      else
          dWeight= Huber::Weight(dErrorSq, dSigmaSquared);

      // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
      if(dWeight == 0.0)
      {
          if(bMarkOutliers)
              TD.Point.nMEstimatorOutlierCount++;
              continue;
      }
      else if(bMarkOutliers)
          TD.Point.nMEstimatorInlierCount++;

      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
  }

  wls.compute();
  return wls.get_mu();
}


void MapMaker::MergeMaps(Map& mNew, double dSceneDepthMean, double dSceneDepthSigma,std::vector<int> iOld, std::vector<int> iACtual){

pthread_mutex_lock(&mergeMutex);


  pubPoints.clear();
  int numKFActual = mMap.vpKeyFrames.size();
  int numPointActual = mMap.vpPoints.size();

  std::vector<int> vect_it;
  std::vector<int> vect_first;
  std::vector<Measurement> vect_second;

  vect_it.clear();
  vect_first.clear();
  vect_second.clear();


  for(unsigned int i=0; i < mNew.vpKeyFrames.size(); i++)
  {
    //////////////////
    //   CANGE DUPLICATE POINTS
    for(meas_it it = mNew.vpKeyFrames[i]->mMeasurements.begin(); it != mNew.vpKeyFrames[i]->mMeasurements.end(); it++)
    {
      for(unsigned int j=0; j < iOld.size(); j++)
      {
        if (it->first->indexPointMap == iOld[j])
        {
         vect_it.push_back(iOld[j]);
         vect_first.push_back(iACtual[j]);
         vect_second.push_back(it->second);
        }
      }
    }

    for(unsigned int j=0; j < vect_it.size(); j++)
    {
      mNew.vpKeyFrames[i]->mMeasurements.erase(mNew.vpPoints[vect_it[j]]); 
      mNew.vpKeyFrames[i]->mMeasurements[mMap.vpPoints[vect_first[j]]] = vect_second[j];
    }

    //
    ////////////////////////////////////////////////////////////////

    KeyFrame *k = mNew.vpKeyFrames[i];
    k->index = mMap.vpKeyFrames.size();
    k->dSceneDepthMean = dSceneDepthMean;
    k->dSceneDepthSigma = dSceneDepthSigma;
    //k->color = 10;
    k->color =  mNew.vpKeyFrames[i]->color;
    mMap.vpKeyFrames.push_back(k);
  }

    //////////////////
    //   DELETE DUPLICATE POINTS

    sort(iOld.begin(),iOld.end());

    for(int indexOld = iOld.size()-1; indexOld>=0; indexOld--)
    {
      mNew.vpPoints.erase(mNew.vpPoints.begin() + iOld[indexOld]);
      for(int k = iOld[indexOld]; k <= mNew.vpPoints.size()-1; k++)
      {
          mNew.vpPoints[k]->indexPointMap--;
      }
    }

    //
    ////////////////////////////////////////////////////////////////

  for(unsigned int i=0; i < mNew.vpPoints.size(); i++)
  {
    MapPoint *p = mNew.vpPoints[i];
    p->indexPointMap = mMap.vpPoints.size();
    mMap.vpPoints.push_back(p);
    pubPoints.push_back(p);
  }

  pubMergePoints(numKFActual);

  pub1 = true;

  pthread_mutex_unlock(&mergeMutex);

}


bool MapMaker::AssessTrackingQuality()
{

  bool quality = false;

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
      quality = false;
  }else
  {

      double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
      double dLargeFracFound;
      if(nLargeAttempted > 10)
          dLargeFracFound = (double) nLargeFound / nLargeAttempted;
      else
          dLargeFracFound = dTotalFracFound;

      static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);//0.3  //0.45
      static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);


      if(dTotalFracFound > *gvdQualityGood)
          quality = true;
      else if(dLargeFracFound < *gvdQualityLost)
          quality = false;
      else
          quality = false;
  }

  return quality;
}

void MapMaker::cvd2opencv(const CVD::Image<CVD::byte> &cvd_Im, IplImage & openCv_Im)
{
  int width = cvd_Im.size().x;
  int height = cvd_Im.size().y;
  int channels = 1;

  openCv_Im.nSize = sizeof(IplImage);
  openCv_Im.ID = 0;
  openCv_Im.nChannels = channels;
  openCv_Im.depth = sizeof(CVD::byte)*8;
  openCv_Im.dataOrder = 0;
  openCv_Im.origin = 0;
  openCv_Im.width = width;
  openCv_Im.height = height;
  openCv_Im.roi = NULL;
  openCv_Im.maskROI = NULL;
  openCv_Im.imageId = NULL;
  openCv_Im.tileInfo = NULL;
  openCv_Im.imageSize = width*height*channels;
  openCv_Im.imageData = (char *)(cvd_Im.data());
  openCv_Im.widthStep = width*channels;
  openCv_Im.imageDataOrigin = NULL;
}


